#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from autominy_msgs.msg import (
    NormalizedSpeedCommand,
    NormalizedSteeringCommand,
    SteeringAngle,
)
from geometry_msgs.msg import Point, PointStamped, Pose2D, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import Trigger


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutoMinyPurePursuit(Node):
    """
    AutoMiny Pure Pursuit with:
      - (x, y, theta) goals through /practica5/goal_pose
      - legacy (x, y) goals through /practica5/goal_point
      - Bezier path generation
      - steering feedback
      - forward/reverse
      - automatic replanning
      - automatic direction switching when a maneuver fails
      - final-pose recovery maneuver when position is reached but yaw is not
      - progress watchdog as last-resort safety

    IMPORTANT:
    This replaces ONLY:
      ~/ros2_ws/src/autominy_pure_pursuit/autominy_pure_pursuit/controller.py

    The existing YAML/launch/package files remain compatible because all new
    parameters below have defaults.
    """

    FORWARD = 1
    REVERSE = -1

    def __init__(self):
        super().__init__("autominy_pure_pursuit")

        # ============================================================
        # Existing topics
        # ============================================================
        self.declare_parameter("odom_topic", "/sensors/odom")
        self.declare_parameter(
            "steering_feedback_topic", "/sensors/steering"
        )
        self.declare_parameter(
            "goal_pose_topic", "/practica5/goal_pose"
        )
        self.declare_parameter(
            "goal_point_topic", "/practica5/goal_point"
        )
        self.declare_parameter(
            "speed_topic", "/actuators/speed_normalized"
        )
        self.declare_parameter(
            "steering_topic", "/actuators/steering_normalized"
        )

        # ============================================================
        # Vehicle model / calibration
        # ============================================================
        self.declare_parameter("wheelbase", 0.25)
        self.declare_parameter(
            "max_left_steering_angle_rad", 0.512
        )
        self.declare_parameter(
            "max_right_steering_angle_rad", 0.498
        )

        # ============================================================
        # Speed / steering
        # ============================================================
        self.declare_parameter("max_speed", 0.25)
        self.declare_parameter("cruise_speed", 0.12)
        self.declare_parameter("min_speed", 0.05)
        self.declare_parameter("max_reverse_speed", 0.25)
        self.declare_parameter(
            "max_steering_normalized", 0.80
        )
        self.declare_parameter("speed_sign", 1.0)
        self.declare_parameter("steering_sign", 1.0)

        # ============================================================
        # Steering feedback
        # ============================================================
        self.declare_parameter(
            "steering_feedback_enabled", True
        )
        self.declare_parameter(
            "steering_feedback_sign", 1.0
        )
        self.declare_parameter(
            "steering_feedback_kp", 0.70
        )
        self.declare_parameter(
            "steering_feedback_deadband_rad", 0.008
        )
        self.declare_parameter(
            "steering_feedback_timeout", 0.30
        )
        self.declare_parameter(
            "positive_steering_ff_gain", 1.0
        )
        self.declare_parameter(
            "negative_steering_ff_gain", 1.0
        )
        self.declare_parameter(
            "steering_command_rate_limit", 2.5
        )

        # ============================================================
        # Reverse
        # ============================================================
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter(
            "reverse_activation_x", -0.10
        )
        self.declare_parameter(
            "reverse_min_goal_distance", 0.25
        )

        # ============================================================
        # Path generation
        # ============================================================
        self.declare_parameter("path_resolution", 0.02)
        self.declare_parameter(
            "bezier_tangent_ratio", 0.35
        )
        self.declare_parameter(
            "bezier_min_tangent", 0.25
        )
        self.declare_parameter(
            "bezier_max_tangent", 0.80
        )

        # Prevent nearest-point search from jumping directly to the end
        # of a recovery path that comes back close to the starting point.
        self.declare_parameter(
            "nearest_search_window", 30
        )

        # ============================================================
        # Pure Pursuit / arrival
        # ============================================================
        self.declare_parameter("lookahead", 0.25)
        self.declare_parameter(
            "position_tolerance", 0.08
        )
        self.declare_parameter(
            "yaw_tolerance_rad", math.radians(10.0)
        )
        self.declare_parameter("slow_radius", 0.55)
        self.declare_parameter(
            "curvature_speed_gain", 0.70
        )
        self.declare_parameter(
            "minimum_curve_speed_factor", 0.35
        )

        # ============================================================
        # Progress watchdog
        # ============================================================
        self.declare_parameter(
            "progress_timeout", 5.0
        )
        self.declare_parameter(
            "progress_index_step", 2
        )
        self.declare_parameter(
            "progress_distance_epsilon", 0.02
        )

        # ============================================================
        # NEW: automatic replanning / accommodation
        # ============================================================
        self.declare_parameter(
            "auto_replan_enabled", True
        )

        # Maximum number of replans for one commanded goal.
        self.declare_parameter(
            "max_replans", 5
        )

        # Stop briefly before changing maneuver/direction.
        self.declare_parameter(
            "replan_stop_duration", 0.30
        )

        # If the vehicle is near the target, but the target has moved
        # clearly behind its current direction of motion, replan and
        # allow a direction switch.
        self.declare_parameter(
            "behind_replan_distance", 0.55
        )
        self.declare_parameter(
            "behind_replan_x", -0.12
        )

        # If the vehicle reaches the end of the generated path but the
        # requested pose is not achieved, replan.
        self.declare_parameter(
            "path_end_margin_points", 3
        )

        # When x,y are already close but theta is wrong, create a
        # staging maneuver before approaching the final pose again.
        self.declare_parameter(
            "orientation_recovery_distance", 0.40
        )

        # Avoid repeatedly replanning from tiny measurement changes.
        self.declare_parameter(
            "minimum_replan_interval", 0.60
        )

        # Direction selection hysteresis. Reverse is selected only if
        # its initial alignment is meaningfully better.
        self.declare_parameter(
            "direction_hysteresis_rad", math.radians(12.0)
        )

        # ============================================================
        # Safety / execution
        # ============================================================
        self.declare_parameter("odom_timeout", 0.50)
        self.declare_parameter(
            "control_rate_hz", 20.0
        )

        gp = self.get_parameter

        # Topics
        self.odom_topic = str(gp("odom_topic").value)
        self.steering_feedback_topic = str(
            gp("steering_feedback_topic").value
        )
        self.goal_pose_topic = str(
            gp("goal_pose_topic").value
        )
        self.goal_point_topic = str(
            gp("goal_point_topic").value
        )
        self.speed_topic = str(
            gp("speed_topic").value
        )
        self.steering_topic = str(
            gp("steering_topic").value
        )

        # Vehicle
        self.wheelbase = max(
            0.01, abs(float(gp("wheelbase").value))
        )
        self.max_left_angle = max(
            1e-3,
            abs(
                float(
                    gp("max_left_steering_angle_rad").value
                )
            ),
        )
        self.max_right_angle = max(
            1e-3,
            abs(
                float(
                    gp("max_right_steering_angle_rad").value
                )
            ),
        )

        # Limits
        self.max_speed = min(
            0.25,
            abs(float(gp("max_speed").value)),
        )
        self.cruise_speed = min(
            self.max_speed,
            abs(float(gp("cruise_speed").value)),
        )
        self.min_speed = min(
            self.cruise_speed,
            abs(float(gp("min_speed").value)),
        )
        self.max_reverse_speed = min(
            self.max_speed,
            abs(float(gp("max_reverse_speed").value)),
        )
        self.max_steering_norm = clamp(
            abs(
                float(
                    gp("max_steering_normalized").value
                )
            ),
            0.0,
            1.0,
        )

        self.speed_sign = float(
            gp("speed_sign").value
        )
        self.steering_sign = float(
            gp("steering_sign").value
        )

        # Steering feedback
        self.steering_feedback_enabled = bool(
            gp("steering_feedback_enabled").value
        )
        self.steering_feedback_sign = float(
            gp("steering_feedback_sign").value
        )
        self.steering_feedback_kp = max(
            0.0,
            float(gp("steering_feedback_kp").value),
        )
        self.steering_feedback_deadband = max(
            0.0,
            abs(
                float(
                    gp(
                        "steering_feedback_deadband_rad"
                    ).value
                )
            ),
        )
        self.steering_feedback_timeout = max(
            0.05,
            float(
                gp("steering_feedback_timeout").value
            ),
        )
        self.positive_steering_ff_gain = max(
            0.0,
            float(
                gp("positive_steering_ff_gain").value
            ),
        )
        self.negative_steering_ff_gain = max(
            0.0,
            float(
                gp("negative_steering_ff_gain").value
            ),
        )
        self.steering_rate_limit = max(
            0.0,
            float(
                gp("steering_command_rate_limit").value
            ),
        )

        # Reverse
        self.allow_reverse = bool(
            gp("allow_reverse").value
        )
        self.reverse_activation_x = float(
            gp("reverse_activation_x").value
        )
        self.reverse_min_goal_distance = max(
            0.0,
            float(
                gp("reverse_min_goal_distance").value
            ),
        )

        # Path
        self.path_resolution = max(
            0.005,
            abs(float(gp("path_resolution").value)),
        )
        self.bezier_tangent_ratio = max(
            0.05,
            float(gp("bezier_tangent_ratio").value),
        )
        self.bezier_min_tangent = max(
            0.05,
            abs(float(gp("bezier_min_tangent").value)),
        )
        self.bezier_max_tangent = max(
            self.bezier_min_tangent,
            abs(float(gp("bezier_max_tangent").value)),
        )
        self.nearest_search_window = max(
            5,
            int(gp("nearest_search_window").value),
        )

        # Pure Pursuit
        self.lookahead = max(
            0.08,
            abs(float(gp("lookahead").value)),
        )
        self.position_tolerance = max(
            0.02,
            abs(
                float(gp("position_tolerance").value)
            ),
        )
        self.yaw_tolerance = max(
            math.radians(1.0),
            abs(
                float(gp("yaw_tolerance_rad").value)
            ),
        )
        self.slow_radius = max(
            self.position_tolerance,
            abs(float(gp("slow_radius").value)),
        )
        self.curvature_speed_gain = max(
            0.0,
            float(gp("curvature_speed_gain").value),
        )
        self.min_curve_speed_factor = clamp(
            float(
                gp("minimum_curve_speed_factor").value
            ),
            0.1,
            1.0,
        )

        # Watchdog
        self.progress_timeout = max(
            0.5,
            float(gp("progress_timeout").value),
        )
        self.progress_index_step = max(
            1,
            int(gp("progress_index_step").value),
        )
        self.progress_distance_epsilon = max(
            0.001,
            float(
                gp("progress_distance_epsilon").value
            ),
        )

        # Replanning
        self.auto_replan_enabled = bool(
            gp("auto_replan_enabled").value
        )
        self.max_replans = max(
            0,
            int(gp("max_replans").value),
        )
        self.replan_stop_duration = max(
            0.0,
            float(gp("replan_stop_duration").value),
        )
        self.behind_replan_distance = max(
            self.position_tolerance,
            float(gp("behind_replan_distance").value),
        )
        self.behind_replan_x = float(
            gp("behind_replan_x").value
        )
        self.path_end_margin_points = max(
            1,
            int(gp("path_end_margin_points").value),
        )
        self.orientation_recovery_distance = max(
            0.20,
            float(
                gp(
                    "orientation_recovery_distance"
                ).value
            ),
        )
        self.minimum_replan_interval = max(
            0.0,
            float(gp("minimum_replan_interval").value),
        )
        self.direction_hysteresis = max(
            0.0,
            float(gp("direction_hysteresis_rad").value),
        )

        # Safety
        self.odom_timeout = max(
            0.05,
            float(gp("odom_timeout").value),
        )
        self.control_rate_hz = max(
            1.0,
            float(gp("control_rate_hz").value),
        )

        # ============================================================
        # Runtime state
        # ============================================================
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        self.local_x = 0.0
        self.local_y = 0.0
        self.local_yaw = 0.0
        self.last_odom_time = None

        self.measured_steering_angle = 0.0
        self.last_steering_feedback_time = None

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None
        self.orientation_required = False

        self.goal_active = False
        self.goal_reached = False
        self.goal_blocked = False

        self.drive_direction = self.FORWARD

        self.path_points: List[
            Tuple[float, float]
        ] = []
        self.nearest_index = 0
        self.carrot_index = 0

        self.progress_reference_index = 0
        self.best_goal_distance = float("inf")
        self.last_progress_time = None

        self.last_steering_command = 0.0
        self.last_control_time = None
        self.last_status_time = None
        self.last_path_publish_time = None

        # NEW replan state
        self.replan_count = 0
        self.last_replan_time_sec = -1e9
        self.replan_hold_until_sec = -1e9
        self.last_replan_reason = "none"

        # ============================================================
        # Publishers
        # ============================================================
        self.speed_pub = self.create_publisher(
            NormalizedSpeedCommand,
            self.speed_topic,
            10,
        )
        self.steering_pub = self.create_publisher(
            NormalizedSteeringCommand,
            self.steering_topic,
            10,
        )

        self.local_pose_pub = self.create_publisher(
            Pose2D,
            "/practica5/local_pose",
            10,
        )
        self.path_pub = self.create_publisher(
            Path,
            "/practica5/path",
            10,
        )
        self.carrot_pub = self.create_publisher(
            PointStamped,
            "/practica5/carrot_point",
            10,
        )

        self.distance_pub = self.create_publisher(
            Float64,
            "/practica5/distance_error",
            10,
        )
        self.yaw_error_pub = self.create_publisher(
            Float64,
            "/practica5/yaw_error",
            10,
        )
        self.curvature_pub = self.create_publisher(
            Float64,
            "/practica5/curvature",
            10,
        )

        self.desired_angle_pub = self.create_publisher(
            Float64,
            "/practica5/desired_steering_angle",
            10,
        )
        self.measured_angle_pub = self.create_publisher(
            Float64,
            "/practica5/measured_steering_angle",
            10,
        )
        self.steering_error_pub = self.create_publisher(
            Float64,
            "/practica5/steering_angle_error",
            10,
        )
        self.steering_ff_pub = self.create_publisher(
            Float64,
            "/practica5/steering_feedforward_normalized",
            10,
        )
        self.steering_fb_pub = self.create_publisher(
            Float64,
            "/practica5/steering_feedback_correction",
            10,
        )
        self.steering_command_pub = self.create_publisher(
            Float64,
            "/practica5/steering_command_normalized",
            10,
        )

        self.path_index_pub = self.create_publisher(
            Int32,
            "/practica5/path_index",
            10,
        )
        self.drive_direction_pub = self.create_publisher(
            Int32,
            "/practica5/drive_direction",
            10,
        )
        self.replan_count_pub = self.create_publisher(
            Int32,
            "/practica5/replan_count",
            10,
        )

        self.steer_saturated_pub = self.create_publisher(
            Bool,
            "/practica5/steer_saturated",
            10,
        )
        self.goal_reached_pub = self.create_publisher(
            Bool,
            "/practica5/goal_reached",
            10,
        )
        self.goal_blocked_pub = self.create_publisher(
            Bool,
            "/practica5/goal_blocked",
            10,
        )

        # ============================================================
        # Subscribers / services
        # ============================================================
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            SteeringAngle,
            self.steering_feedback_topic,
            self.steering_feedback_callback,
            10,
        )
        self.create_subscription(
            Pose2D,
            self.goal_pose_topic,
            self.goal_pose_callback,
            10,
        )
        self.create_subscription(
            Point,
            self.goal_point_topic,
            self.goal_point_callback,
            10,
        )

        self.create_service(
            Trigger,
            "/practica5/reset_origin",
            self.reset_origin_callback,
        )
        self.create_service(
            Trigger,
            "/practica5/cancel_goal",
            self.cancel_goal_callback,
        )

        self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info(
            "AutoMiny Pure Pursuit v6.1: reverse_activation_x + automatic replan + "
            "forward/reverse accommodation enabled."
        )
        self.get_logger().info(
            f"auto_replan={self.auto_replan_enabled}, "
            f"max_replans={self.max_replans}, "
            f"recovery_distance="
            f"{self.orientation_recovery_distance:.2f}m"
        )

    # ================================================================
    # Time helpers
    # ================================================================

    def now_sec(self) -> float:
        return (
            self.get_clock().now().nanoseconds
            / 1e9
        )

    # ================================================================
    # Sensors / local frame
    # ================================================================

    def odom_callback(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            q.x, q.y, q.z, q.w
        )

        if not self.origin_set:
            self.origin_x = x
            self.origin_y = y
            self.origin_yaw = yaw
            self.origin_set = True

            self.get_logger().info(
                f"Local origin: x0={x:.3f}, "
                f"y0={y:.3f}, "
                f"yaw0={math.degrees(yaw):.1f}deg"
            )

        dx = x - self.origin_x
        dy = y - self.origin_y

        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        self.local_x = c * dx + s * dy
        self.local_y = -s * dx + c * dy
        self.local_yaw = wrap_to_pi(
            yaw - self.origin_yaw
        )

        self.last_odom_time = (
            self.get_clock().now()
        )

        p = Pose2D()
        p.x = self.local_x
        p.y = self.local_y
        p.theta = self.local_yaw
        self.local_pose_pub.publish(p)

    def steering_feedback_callback(
        self, msg: SteeringAngle
    ):
        self.measured_steering_angle = (
            float(msg.value)
            * self.steering_feedback_sign
        )
        self.last_steering_feedback_time = (
            self.get_clock().now()
        )

        m = Float64()
        m.data = self.measured_steering_angle
        self.measured_angle_pub.publish(m)

    # ================================================================
    # Geometry helpers
    # ================================================================

    def point_in_frame(
        self,
        x: float,
        y: float,
        frame_yaw: float,
    ) -> Tuple[float, float]:
        dx = x - self.local_x
        dy = y - self.local_y

        c = math.cos(frame_yaw)
        s = math.sin(frame_yaw)

        return (
            c * dx + s * dy,
            -s * dx + c * dy,
        )

    def motion_yaw_for_direction(
        self, direction: int
    ) -> float:
        if direction == self.REVERSE:
            return wrap_to_pi(
                self.local_yaw + math.pi
            )
        return self.local_yaw

    def motion_yaw(self) -> float:
        return self.motion_yaw_for_direction(
            self.drive_direction
        )

    def goal_distance(self) -> float:
        if self.goal_x is None or self.goal_y is None:
            return float("inf")

        return math.hypot(
            self.goal_x - self.local_x,
            self.goal_y - self.local_y,
        )

    def goal_yaw_error(self) -> float:
        if (
            not self.orientation_required
            or self.goal_yaw is None
        ):
            return 0.0

        return wrap_to_pi(
            self.goal_yaw - self.local_yaw
        )

    # ================================================================
    # Direction selection
    # ================================================================

    def direction_alignment_cost(
        self,
        direction: int,
        target_x: float,
        target_y: float,
    ) -> float:
        dx = target_x - self.local_x
        dy = target_y - self.local_y

        if math.hypot(dx, dy) < 1e-6:
            return 0.0

        target_heading = math.atan2(dy, dx)
        motion_heading = (
            self.local_yaw
            if direction == self.FORWARD
            else wrap_to_pi(
                self.local_yaw + math.pi
            )
        )

        return abs(
            wrap_to_pi(
                target_heading - motion_heading
            )
        )

    def choose_drive_direction(
        self,
        force_switch: bool = False,
        orientation_recovery: bool = False,
    ) -> int:
        """
        Select FORWARD/REVERSE.

        v6.1 correction:
        `reverse_activation_x` is now actually used.

        For an ordinary goal:
          - transform the final goal into the CURRENT VEHICLE frame;
          - if goal_body_x <= reverse_activation_x, the goal is clearly
            behind the car and REVERSE is selected directly;
          - otherwise use the previous angular-cost comparison.

        For final-orientation recovery, the staging-point comparison is
        retained because the best direction depends on the requested
        final yaw as well as the goal position.

        `force_switch=True` still forces the opposite maneuver when a
        previous attempt failed and replanning is required.
        """
        if not self.allow_reverse:
            return self.FORWARD

        old_direction = self.drive_direction
        goal_dist = self.goal_distance()

        direct_reverse = False

        if not orientation_recovery:
            goal_body_x, goal_body_y = self.point_in_frame(
                self.goal_x,
                self.goal_y,
                self.local_yaw,
            )

            direct_reverse = (
                goal_dist >= self.reverse_min_goal_distance
                and goal_body_x <= self.reverse_activation_x
            )

            if direct_reverse:
                chosen = self.REVERSE
                f_cost = self.direction_alignment_cost(
                    self.FORWARD,
                    self.goal_x,
                    self.goal_y,
                )
                r_cost = self.direction_alignment_cost(
                    self.REVERSE,
                    self.goal_x,
                    self.goal_y,
                )
            else:
                f_cost = self.direction_alignment_cost(
                    self.FORWARD,
                    self.goal_x,
                    self.goal_y,
                )
                r_cost = self.direction_alignment_cost(
                    self.REVERSE,
                    self.goal_x,
                    self.goal_y,
                )

                if (
                    r_cost + self.direction_hysteresis
                    < f_cost
                ):
                    chosen = self.REVERSE
                else:
                    chosen = self.FORWARD

        else:
            # Forward final-approach staging point.
            f_motion_yaw = self.goal_yaw
            f_stage_x = (
                self.goal_x
                - self.orientation_recovery_distance
                * math.cos(f_motion_yaw)
            )
            f_stage_y = (
                self.goal_y
                - self.orientation_recovery_distance
                * math.sin(f_motion_yaw)
            )

            # Reverse final-approach staging point.
            r_motion_yaw = wrap_to_pi(
                self.goal_yaw + math.pi
            )
            r_stage_x = (
                self.goal_x
                - self.orientation_recovery_distance
                * math.cos(r_motion_yaw)
            )
            r_stage_y = (
                self.goal_y
                - self.orientation_recovery_distance
                * math.sin(r_motion_yaw)
            )

            f_cost = self.direction_alignment_cost(
                self.FORWARD,
                f_stage_x,
                f_stage_y,
            )
            r_cost = self.direction_alignment_cost(
                self.REVERSE,
                r_stage_x,
                r_stage_y,
            )

            if (
                r_cost + self.direction_hysteresis
                < f_cost
            ):
                chosen = self.REVERSE
            else:
                chosen = self.FORWARD

        # If a maneuver failed and replanning explicitly asks for a switch,
        # do not repeat the same direction.
        if force_switch and chosen == old_direction:
            chosen = (
                self.REVERSE
                if old_direction == self.FORWARD
                else self.FORWARD
            )

        chosen_name = (
            "REV" if chosen == self.REVERSE else "FWD"
        )

        self.get_logger().info(
            "Direction decision: "
            f"goal_dist={goal_dist:.2f}m, "
            f"reverse_activation_x={self.reverse_activation_x:.2f}, "
            f"f_cost={math.degrees(f_cost):.1f}deg, "
            f"r_cost={math.degrees(r_cost):.1f}deg, "
            f"direct_reverse={direct_reverse}, "
            f"force_switch={force_switch}, "
            f"chosen={chosen_name}"
        )

        return chosen

    # ================================================================
    # Bezier path generation
    # ================================================================

    def generate_bezier_segment(
        self,
        start_x: float,
        start_y: float,
        start_motion_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_motion_yaw: float,
    ) -> List[Tuple[float, float]]:
        distance = math.hypot(
            goal_x - start_x,
            goal_y - start_y,
        )

        if distance < 1e-4:
            return [
                (start_x, start_y),
                (goal_x, goal_y),
            ]

        tangent = clamp(
            self.bezier_tangent_ratio * distance,
            self.bezier_min_tangent,
            self.bezier_max_tangent,
        )

        p0 = (start_x, start_y)

        p1 = (
            start_x
            + tangent
            * math.cos(start_motion_yaw),
            start_y
            + tangent
            * math.sin(start_motion_yaw),
        )

        p3 = (goal_x, goal_y)

        p2 = (
            goal_x
            - tangent
            * math.cos(goal_motion_yaw),
            goal_y
            - tangent
            * math.sin(goal_motion_yaw),
        )

        count = max(
            30,
            int(
                max(distance, tangent)
                * 2.0
                / self.path_resolution
            ),
        )

        path = []

        for i in range(count + 1):
            t = i / count
            u = 1.0 - t

            x = (
                (u ** 3) * p0[0]
                + 3.0
                * (u ** 2)
                * t
                * p1[0]
                + 3.0
                * u
                * (t ** 2)
                * p2[0]
                + (t ** 3) * p3[0]
            )

            y = (
                (u ** 3) * p0[1]
                + 3.0
                * (u ** 2)
                * t
                * p1[1]
                + 3.0
                * u
                * (t ** 2)
                * p2[1]
                + (t ** 3) * p3[1]
            )

            path.append((x, y))

        return path

    def goal_vehicle_yaw_for_path(self) -> float:
        if (
            self.orientation_required
            and self.goal_yaw is not None
        ):
            return self.goal_yaw

        # Position-only legacy mode.
        dx = self.goal_x - self.local_x
        dy = self.goal_y - self.local_y

        if math.hypot(dx, dy) < 1e-6:
            return self.local_yaw

        return math.atan2(dy, dx)

    def build_path(
        self,
        direction: int,
        orientation_recovery: bool = False,
    ) -> List[Tuple[float, float]]:
        """
        Normal mode:
            one cubic segment current pose -> final goal.

        Orientation recovery mode:
            current pose -> staging point -> final pose

        The staging point gives an Ackermann vehicle room to correct yaw;
        it does NOT attempt to rotate in place.
        """
        start_motion_yaw = (
            self.local_yaw
            if direction == self.FORWARD
            else wrap_to_pi(
                self.local_yaw + math.pi
            )
        )

        goal_vehicle_yaw = (
            self.goal_vehicle_yaw_for_path()
        )

        goal_motion_yaw = (
            goal_vehicle_yaw
            if direction == self.FORWARD
            else wrap_to_pi(
                goal_vehicle_yaw + math.pi
            )
        )

        if (
            orientation_recovery
            and self.orientation_required
            and self.goal_yaw is not None
        ):
            # The staging point lies "before" the final pose in the
            # selected direction of motion.
            stage_x = (
                self.goal_x
                - self.orientation_recovery_distance
                * math.cos(goal_motion_yaw)
            )
            stage_y = (
                self.goal_y
                - self.orientation_recovery_distance
                * math.sin(goal_motion_yaw)
            )

            first = self.generate_bezier_segment(
                self.local_x,
                self.local_y,
                start_motion_yaw,
                stage_x,
                stage_y,
                goal_motion_yaw,
            )

            second = self.generate_bezier_segment(
                stage_x,
                stage_y,
                goal_motion_yaw,
                self.goal_x,
                self.goal_y,
                goal_motion_yaw,
            )

            if first and second:
                return first + second[1:]

            return first or second

        return self.generate_bezier_segment(
            self.local_x,
            self.local_y,
            start_motion_yaw,
            self.goal_x,
            self.goal_y,
            goal_motion_yaw,
        )

    def publish_path(self):
        if not self.path_points:
            return

        msg = Path()
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        msg.header.frame_id = (
            "practica5_local"
        )

        for x, y in self.path_points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.path_pub.publish(msg)

    # ================================================================
    # Goals
    # ================================================================

    def goal_pose_callback(self, msg: Pose2D):
        self.set_goal(
            float(msg.x),
            float(msg.y),
            wrap_to_pi(float(msg.theta)),
            orientation_required=True,
        )

    def goal_point_callback(self, msg: Point):
        self.set_goal(
            float(msg.x),
            float(msg.y),
            None,
            orientation_required=False,
        )

    def set_goal(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: Optional[float],
        orientation_required: bool,
    ):
        if not self.origin_set:
            self.get_logger().warn(
                "Goal ignored: odometry/local "
                "origin not ready."
            )
            return

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.orientation_required = (
            orientation_required
        )

        self.replan_count = 0
        self.last_replan_reason = "new_goal"
        self.last_replan_time_sec = -1e9
        self.replan_hold_until_sec = -1e9

        self.drive_direction = (
            self.choose_drive_direction(
                force_switch=False,
                orientation_recovery=False,
            )
        )

        self.path_points = self.build_path(
            self.drive_direction,
            orientation_recovery=False,
        )

        self.reset_path_tracking_state()

        self.goal_active = True
        self.goal_reached = False
        self.goal_blocked = False

        self.publish_bool(
            self.goal_reached_pub, False
        )
        self.publish_bool(
            self.goal_blocked_pub, False
        )
        self.publish_drive_direction()
        self.publish_replan_count()
        self.publish_path()

        self.last_path_publish_time = (
            self.get_clock().now()
        )

        direction = (
            "FORWARD"
            if self.drive_direction
            == self.FORWARD
            else "REVERSE"
        )

        yaw_text = (
            f"{math.degrees(goal_yaw):.1f}deg"
            if orientation_required
            and goal_yaw is not None
            else "free"
        )

        self.get_logger().info(
            f"NEW GOAL: ({goal_x:.3f}, "
            f"{goal_y:.3f}, yaw={yaw_text}), "
            f"direction={direction}, "
            f"path_points={len(self.path_points)}"
        )

    def reset_path_tracking_state(self):
        self.nearest_index = 0
        self.carrot_index = 0

        self.progress_reference_index = 0
        self.best_goal_distance = (
            self.goal_distance()
        )
        self.last_progress_time = (
            self.get_clock().now()
        )

    # ================================================================
    # Automatic replanning
    # ================================================================

    def publish_replan_count(self):
        msg = Int32()
        msg.data = int(self.replan_count)
        self.replan_count_pub.publish(msg)

    def block_goal(self, reason: str):
        self.stop_vehicle()
        self.goal_active = False
        self.goal_reached = False
        self.goal_blocked = True

        self.publish_bool(
            self.goal_reached_pub, False
        )
        self.publish_bool(
            self.goal_blocked_pub, True
        )

        self.get_logger().error(
            f"GOAL BLOCKED: {reason}. "
            f"replans={self.replan_count}/"
            f"{self.max_replans}"
        )

    def attempt_replan(
        self,
        reason: str,
        force_switch: bool = False,
        orientation_recovery: bool = False,
    ) -> bool:
        """
        Stop -> choose forward/reverse -> generate a new path from the
        CURRENT pose -> continue.

        Returns True when a new path was created.
        """
        if not self.auto_replan_enabled:
            self.block_goal(
                f"{reason}; auto_replan disabled"
            )
            return False

        now = self.now_sec()

        if (
            now - self.last_replan_time_sec
            < self.minimum_replan_interval
        ):
            return False

        if self.replan_count >= self.max_replans:
            self.block_goal(
                f"{reason}; maximum replans reached"
            )
            return False

        old_direction = self.drive_direction

        new_direction = (
            self.choose_drive_direction(
                force_switch=force_switch,
                orientation_recovery=(
                    orientation_recovery
                ),
            )
        )

        new_path = self.build_path(
            new_direction,
            orientation_recovery=(
                orientation_recovery
            ),
        )

        if len(new_path) < 2:
            self.block_goal(
                f"{reason}; could not generate "
                "a valid recovery path"
            )
            return False

        # Stop before changing maneuver.
        self.stop_vehicle()

        self.drive_direction = new_direction
        self.path_points = new_path

        self.replan_count += 1
        self.last_replan_reason = reason
        self.last_replan_time_sec = now

        self.replan_hold_until_sec = (
            now + self.replan_stop_duration
        )

        self.reset_path_tracking_state()

        self.goal_active = True
        self.goal_blocked = False

        self.publish_bool(
            self.goal_blocked_pub, False
        )
        self.publish_drive_direction()
        self.publish_replan_count()
        self.publish_path()
        self.last_path_publish_time = (
            self.get_clock().now()
        )

        old_name = (
            "FWD"
            if old_direction == self.FORWARD
            else "REV"
        )
        new_name = (
            "FWD"
            if new_direction == self.FORWARD
            else "REV"
        )

        self.get_logger().warn(
            f"REPLAN #{self.replan_count}: "
            f"{reason}; "
            f"{old_name}->{new_name}; "
            f"recovery="
            f"{orientation_recovery}; "
            f"new_path_points="
            f"{len(self.path_points)}"
        )

        return True

    # ================================================================
    # Services
    # ================================================================

    def reset_origin_callback(
        self, request, response
    ):
        del request

        self.stop_vehicle()
        self.clear_goal()

        self.origin_set = False
        self.last_odom_time = None

        response.success = True
        response.message = (
            "Origin cleared. Next odometry "
            "sample becomes local (0,0,0)."
        )
        return response

    def cancel_goal_callback(
        self, request, response
    ):
        del request

        self.stop_vehicle()
        self.clear_goal()

        response.success = True
        response.message = (
            "Goal cancelled. STOP sent."
        )
        return response

    def clear_goal(self):
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.orientation_required = False

        self.goal_active = False
        self.goal_reached = False
        self.goal_blocked = False

        self.drive_direction = self.FORWARD

        self.path_points = []
        self.nearest_index = 0
        self.carrot_index = 0

        self.progress_reference_index = 0
        self.best_goal_distance = float("inf")
        self.last_progress_time = None

        self.replan_count = 0
        self.last_replan_reason = "none"
        self.last_replan_time_sec = -1e9
        self.replan_hold_until_sec = -1e9

    # ================================================================
    # Path tracking
    # ================================================================

    def find_nearest_path_index(self) -> int:
        if not self.path_points:
            return 0

        start = min(
            self.nearest_index,
            len(self.path_points) - 1,
        )

        # IMPORTANT:
        # Search only a limited forward window. Recovery paths can come
        # back close to their start; searching the whole path could jump
        # directly to the final point.
        end = min(
            len(self.path_points),
            start + self.nearest_search_window,
        )

        best_index = start
        best_d2 = float("inf")

        for i in range(start, end):
            px, py = self.path_points[i]

            d2 = (
                (px - self.local_x) ** 2
                + (py - self.local_y) ** 2
            )

            if d2 < best_d2:
                best_d2 = d2
                best_index = i

        return best_index

    def find_carrot_index(
        self, nearest_index: int
    ) -> int:
        frame_yaw = self.motion_yaw()
        final = len(self.path_points) - 1
        fallback = final

        for i in range(
            nearest_index,
            len(self.path_points),
        ):
            px, py = self.path_points[i]

            distance = math.hypot(
                px - self.local_x,
                py - self.local_y,
            )

            if distance < self.lookahead:
                continue

            body_x, _ = self.point_in_frame(
                px,
                py,
                frame_yaw,
            )

            if body_x > 0.02:
                return i

            fallback = i

        return fallback

    # ================================================================
    # Steering control
    # ================================================================

    def steering_feedback_is_fresh(
        self,
    ) -> bool:
        if self.last_steering_feedback_time is None:
            return False

        age = (
            self.get_clock().now()
            - self.last_steering_feedback_time
        ).nanoseconds / 1e9

        return (
            age <= self.steering_feedback_timeout
        )

    def steering_feedforward(
        self, desired_angle: float
    ) -> float:
        if desired_angle >= 0.0:
            denom = self.max_left_angle
            side_gain = (
                self.positive_steering_ff_gain
            )
        else:
            denom = self.max_right_angle
            side_gain = (
                self.negative_steering_ff_gain
            )

        return (
            desired_angle
            / max(denom, 1e-6)
            * self.steering_sign
            * side_gain
        )

    def compute_steering_command(
        self,
        desired_angle: float,
    ) -> Tuple[
        float, float, float, bool
    ]:
        ff = self.steering_feedforward(
            desired_angle
        )

        correction = 0.0

        steering_error = (
            desired_angle
            - self.measured_steering_angle
        )

        if (
            self.steering_feedback_enabled
            and self.steering_feedback_is_fresh()
            and abs(steering_error)
            > self.steering_feedback_deadband
        ):
            correction = (
                self.steering_feedback_kp
                * steering_error
                * self.steering_sign
            )

        raw = ff + correction

        saturated = (
            abs(raw)
            > self.max_steering_norm
        )

        command = clamp(
            raw,
            -self.max_steering_norm,
            self.max_steering_norm,
        )

        now = self.get_clock().now()

        if (
            self.last_control_time is not None
            and self.steering_rate_limit > 0.0
        ):
            dt = (
                now - self.last_control_time
            ).nanoseconds / 1e9

            if dt > 0.0:
                max_delta = (
                    self.steering_rate_limit
                    * dt
                )

                command = clamp(
                    command,
                    self.last_steering_command
                    - max_delta,
                    self.last_steering_command
                    + max_delta,
                )

        self.last_control_time = now
        self.last_steering_command = command

        return (
            command,
            ff,
            correction,
            saturated,
        )

    # ================================================================
    # Commands / diagnostics
    # ================================================================

    def publish_commands(
        self,
        speed: float,
        steering: float,
    ):
        speed = clamp(
            speed,
            -self.max_speed,
            self.max_speed,
        )

        steering = clamp(
            steering,
            -self.max_steering_norm,
            self.max_steering_norm,
        )

        stamp = (
            self.get_clock().now().to_msg()
        )

        sm = NormalizedSpeedCommand()
        sm.header.stamp = stamp
        sm.header.frame_id = "base_link"
        sm.value = float(speed)

        stm = NormalizedSteeringCommand()
        stm.header.stamp = stamp
        stm.header.frame_id = "base_link"
        stm.value = float(steering)

        self.speed_pub.publish(sm)
        self.steering_pub.publish(stm)

        dm = Float64()
        dm.data = float(steering)
        self.steering_command_pub.publish(dm)

    def stop_vehicle(self):
        self.publish_commands(0.0, 0.0)

    @staticmethod
    def publish_bool(pub, value: bool):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def publish_drive_direction(self):
        msg = Int32()
        msg.data = int(self.drive_direction)
        self.drive_direction_pub.publish(msg)

    # ================================================================
    # Progress watchdog
    # ================================================================

    def update_progress_watchdog(
        self, goal_distance: float
    ) -> bool:
        now = self.get_clock().now()

        if self.last_progress_time is None:
            self.last_progress_time = now
            self.progress_reference_index = (
                self.nearest_index
            )
            self.best_goal_distance = (
                goal_distance
            )
            return True

        progressed = False

        if (
            self.nearest_index
            >= self.progress_reference_index
            + self.progress_index_step
        ):
            self.progress_reference_index = (
                self.nearest_index
            )
            progressed = True

        if (
            goal_distance
            <= self.best_goal_distance
            - self.progress_distance_epsilon
        ):
            self.best_goal_distance = (
                goal_distance
            )
            progressed = True

        if progressed:
            self.last_progress_time = now
            return True

        elapsed = (
            now - self.last_progress_time
        ).nanoseconds / 1e9

        return elapsed <= self.progress_timeout

    # ================================================================
    # Main control loop
    # ================================================================

    def control_loop(self):
        if (
            not self.origin_set
            or self.last_odom_time is None
        ):
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        odom_age = (
            now - self.last_odom_time
        ).nanoseconds / 1e9

        if odom_age > self.odom_timeout:
            if self.goal_active:
                self.get_logger().error(
                    f"Odometry timeout "
                    f"({odom_age:.2f}s). STOP."
                )

            self.stop_vehicle()
            self.goal_active = False
            self.goal_blocked = True
            self.publish_bool(
                self.goal_blocked_pub, True
            )
            return

        if (
            not self.goal_active
            or self.goal_x is None
            or self.goal_y is None
            or not self.path_points
        ):
            self.stop_vehicle()
            return

        # Brief stationary period after every replan.
        if now_sec < self.replan_hold_until_sec:
            self.stop_vehicle()
            return

        self.publish_drive_direction()
        self.publish_replan_count()

        if (
            self.last_path_publish_time is None
            or (
                now - self.last_path_publish_time
            ).nanoseconds / 1e9
            >= 1.0
        ):
            self.publish_path()
            self.last_path_publish_time = now

        goal_distance = self.goal_distance()
        yaw_error = self.goal_yaw_error()

        dm = Float64()
        dm.data = float(goal_distance)
        self.distance_pub.publish(dm)

        ym = Float64()
        ym.data = float(yaw_error)
        self.yaw_error_pub.publish(ym)

        position_ok = (
            goal_distance
            <= self.position_tolerance
        )

        yaw_ok = (
            not self.orientation_required
            or abs(yaw_error)
            <= self.yaw_tolerance
        )

        # ------------------------------------------------------------
        # DONE
        # ------------------------------------------------------------
        if position_ok and yaw_ok:
            self.stop_vehicle()
            self.goal_active = False
            self.goal_reached = True
            self.goal_blocked = False

            self.publish_bool(
                self.goal_reached_pub, True
            )
            self.publish_bool(
                self.goal_blocked_pub, False
            )

            self.get_logger().info(
                f"GOAL REACHED: "
                f"pos_err={goal_distance:.3f}m, "
                f"yaw_err="
                f"{math.degrees(yaw_error):.1f}deg, "
                f"replans={self.replan_count}"
            )
            return

        # ------------------------------------------------------------
        # POSITION reached but ORIENTATION not reached:
        # Ackermann cannot rotate in place, so create room and return.
        # ------------------------------------------------------------
        if (
            position_ok
            and self.orientation_required
            and not yaw_ok
        ):
            replanned = self.attempt_replan(
                reason=(
                    "position reached but final "
                    "orientation is incorrect"
                ),
                force_switch=False,
                orientation_recovery=True,
            )

            if replanned:
                return

            # If the replan was suppressed only by cooldown,
            # remain stopped rather than driving unpredictably.
            self.stop_vehicle()
            return

        # Update nearest path point before deciding if a replan is needed.
        self.nearest_index = (
            self.find_nearest_path_index()
        )

        im = Int32()
        im.data = int(self.nearest_index)
        self.path_index_pub.publish(im)

        # ------------------------------------------------------------
        # Near goal + target behind current direction of motion.
        #
        # This is deliberately limited to a small near-goal region so
        # lateral targets such as (0,1) are NOT falsely rejected at the
        # beginning of a valid curved maneuver.
        # ------------------------------------------------------------
        motion_frame_yaw = self.motion_yaw()

        goal_body_x, _ = self.point_in_frame(
            self.goal_x,
            self.goal_y,
            motion_frame_yaw,
        )

        if (
            goal_distance
            <= self.behind_replan_distance
            and goal_body_x
            < self.behind_replan_x
        ):
            if self.attempt_replan(
                reason=(
                    "goal is now behind the "
                    "current direction of motion"
                ),
                force_switch=True,
                orientation_recovery=(
                    self.orientation_required
                ),
            ):
                return

        # ------------------------------------------------------------
        # Path ended but pose was not achieved.
        # ------------------------------------------------------------
        path_end_threshold = max(
            0,
            len(self.path_points)
            - 1
            - self.path_end_margin_points,
        )

        if (
            self.nearest_index
            >= path_end_threshold
            and not (position_ok and yaw_ok)
        ):
            if self.attempt_replan(
                reason=(
                    "end of path reached before "
                    "requested pose"
                ),
                force_switch=True,
                orientation_recovery=(
                    self.orientation_required
                    and (
                        goal_distance
                        <= self.behind_replan_distance
                        or not yaw_ok
                    )
                ),
            ):
                return

        # ------------------------------------------------------------
        # Watchdog:
        # Instead of immediately declaring failure, first replan from
        # the CURRENT pose and try the opposite drive direction.
        # ------------------------------------------------------------
        if not self.update_progress_watchdog(
            goal_distance
        ):
            if self.attempt_replan(
                reason=(
                    "progress watchdog timeout"
                ),
                force_switch=True,
                orientation_recovery=(
                    self.orientation_required
                    and goal_distance
                    <= max(
                        self.behind_replan_distance,
                        2.0
                        * self.position_tolerance,
                    )
                ),
            ):
                return

            # attempt_replan may have already blocked the goal if the
            # maximum replan count was reached.
            if self.goal_active:
                self.block_goal(
                    "progress watchdog timeout"
                )
            return

        # ------------------------------------------------------------
        # Dynamic carrot
        # ------------------------------------------------------------
        self.carrot_index = (
            self.find_carrot_index(
                self.nearest_index
            )
        )

        carrot_x, carrot_y = (
            self.path_points[
                self.carrot_index
            ]
        )

        frame_yaw = self.motion_yaw()

        body_x, body_y = self.point_in_frame(
            carrot_x,
            carrot_y,
            frame_yaw,
        )

        ld = max(
            math.hypot(body_x, body_y),
            0.05,
        )

        carrot = PointStamped()
        carrot.header.stamp = now.to_msg()
        carrot.header.frame_id = (
            "practica5_local"
        )
        carrot.point.x = float(carrot_x)
        carrot.point.y = float(carrot_y)
        self.carrot_pub.publish(carrot)

        # ------------------------------------------------------------
        # Pure Pursuit
        # ------------------------------------------------------------
        curvature = (
            2.0 * body_y / (ld * ld)
        )

        desired_steering_angle = math.atan(
            self.wheelbase * curvature
        )

        # Same geometric path curvature while reversing requires the
        # opposite physical steering angle because v < 0.
        if (
            self.drive_direction
            == self.REVERSE
        ):
            desired_steering_angle *= -1.0

        (
            steering_cmd,
            steering_ff,
            steering_fb,
            steer_saturated,
        ) = self.compute_steering_command(
            desired_steering_angle
        )

        # Diagnostics
        for pub, value in [
            (
                self.curvature_pub,
                curvature,
            ),
            (
                self.desired_angle_pub,
                desired_steering_angle,
            ),
            (
                self.steering_error_pub,
                desired_steering_angle
                - self.measured_steering_angle,
            ),
            (
                self.steering_ff_pub,
                steering_ff,
            ),
            (
                self.steering_fb_pub,
                steering_fb,
            ),
        ]:
            m = Float64()
            m.data = float(value)
            pub.publish(m)

        self.publish_bool(
            self.steer_saturated_pub,
            steer_saturated,
        )

        # ------------------------------------------------------------
        # Speed adaptation
        # ------------------------------------------------------------
        if goal_distance < self.slow_radius:
            goal_factor = clamp(
                goal_distance
                / self.slow_radius,
                self.min_speed
                / max(
                    self.cruise_speed,
                    1e-6,
                ),
                1.0,
            )
        else:
            goal_factor = 1.0

        curve_factor = 1.0 / (
            1.0
            + self.curvature_speed_gain
            * abs(curvature)
        )

        curve_factor = max(
            self.min_curve_speed_factor,
            curve_factor,
        )

        base_speed = self.cruise_speed

        if (
            self.drive_direction
            == self.REVERSE
        ):
            base_speed = min(
                base_speed,
                self.max_reverse_speed,
            )

        speed_mag = max(
            self.min_speed,
            base_speed
            * goal_factor
            * curve_factor,
        )

        speed_cmd = (
            speed_mag * self.speed_sign
        )

        if (
            self.drive_direction
            == self.REVERSE
        ):
            speed_cmd *= -1.0

        self.publish_commands(
            speed_cmd,
            steering_cmd,
        )

        # ------------------------------------------------------------
        # Human-readable status
        # ------------------------------------------------------------
        if (
            self.last_status_time is None
            or (
                now - self.last_status_time
            ).nanoseconds / 1e9
            >= 0.75
        ):
            self.last_status_time = now

            direction = (
                "FWD"
                if self.drive_direction
                == self.FORWARD
                else "REV"
            )

            self.get_logger().info(
                f"[{direction}] "
                f"pose=({self.local_x:.2f},"
                f"{self.local_y:.2f},"
                f"{math.degrees(self.local_yaw):.1f}deg) "
                f"goal=({self.goal_x:.2f},"
                f"{self.goal_y:.2f}) "
                f"pos_err={goal_distance:.2f}m "
                f"yaw_err="
                f"{math.degrees(yaw_error):.1f}deg "
                f"idx={self.nearest_index}/"
                f"{len(self.path_points)-1} "
                f"cmd=(v={speed_cmd:.3f},"
                f" steer={steering_cmd:.3f}) "
                f"replans={self.replan_count}/"
                f"{self.max_replans} "
                f"last_replan="
                f"{self.last_replan_reason}"
            )

    def destroy_node(self):
        try:
            self.stop_vehicle()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = AutoMinyPurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_vehicle()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
