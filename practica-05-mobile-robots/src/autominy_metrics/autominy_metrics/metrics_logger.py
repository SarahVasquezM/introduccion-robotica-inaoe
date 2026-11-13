#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from autominy_msgs.msg import (
    NormalizedSpeedCommand,
    NormalizedSteeringCommand,
    SteeringAngle,
)
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import Trigger


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def safe_mean(values):
    return sum(values) / len(values) if values else float("nan")


def safe_rmse(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


class AutoMinyMetrics(Node):
    """
    Passive metrics logger. Does NOT command actuators.

    Supports:
      /practica5/goal_pose  Pose2D  -> x, y, theta metrics
      /practica5/goal_point Point   -> legacy position-only metrics
    """

    def __init__(self):
        super().__init__("autominy_metrics")

        self.declare_parameter(
            "output_dir",
            os.path.expanduser("~/autominy_metrics"),
        )
        self.declare_parameter(
            "goal_pose_topic", "/practica5/goal_pose"
        )
        self.declare_parameter(
            "goal_point_topic", "/practica5/goal_point"
        )
        self.declare_parameter(
            "pose_topic", "/practica5/local_pose"
        )
        self.declare_parameter(
            "path_topic", "/practica5/path"
        )
        self.declare_parameter(
            "desired_steering_topic",
            "/practica5/desired_steering_angle",
        )
        self.declare_parameter(
            "measured_steering_topic",
            "/sensors/steering",
        )
        self.declare_parameter(
            "steering_command_topic",
            "/practica5/steering_command_normalized",
        )
        self.declare_parameter(
            "actuator_steering_topic",
            "/actuators/steering_normalized",
        )
        self.declare_parameter(
            "actuator_speed_topic",
            "/actuators/speed_normalized",
        )
        self.declare_parameter(
            "curvature_topic", "/practica5/curvature"
        )
        self.declare_parameter(
            "steer_saturated_topic",
            "/practica5/steer_saturated",
        )
        self.declare_parameter(
            "drive_direction_topic",
            "/practica5/drive_direction",
        )
        self.declare_parameter(
            "goal_reached_topic",
            "/practica5/goal_reached",
        )
        self.declare_parameter(
            "goal_blocked_topic",
            "/practica5/goal_blocked",
        )

        self.output_dir = Path(
            os.path.expanduser(
                str(self.get_parameter("output_dir").value)
            )
        )
        self.output_dir.mkdir(
            parents=True, exist_ok=True
        )

        # Latest signals
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.goal_yaw: Optional[float] = None
        self.orientation_required = False

        self.pose: Optional[
            Tuple[float, float, float]
        ] = None
        self.path_points: List[Tuple[float, float]] = []

        self.desired_steering = float("nan")
        self.measured_steering = float("nan")
        self.controller_steering_cmd = float("nan")
        self.actuator_steering_cmd = float("nan")
        self.actuator_speed_cmd = float("nan")
        self.curvature = float("nan")
        self.steer_saturated = False
        self.drive_direction = 1

        # Trial state
        self.active = False
        self.start_time = None
        self.start_pose = None
        self.previous_pose = None

        self.csv_path = None
        self.csv_file = None
        self.writer = None

        self.sample_count = 0
        self.path_length = 0.0
        self.cross_track_errors = []
        self.steering_errors = []
        self.yaw_errors = []
        self.saturation_samples = 0

        # Publishers for live metrics
        self.error_x_pub = self.create_publisher(
            Float64, "/practica5/metrics/error_x", 10
        )
        self.error_y_pub = self.create_publisher(
            Float64, "/practica5/metrics/error_y", 10
        )
        self.distance_error_pub = self.create_publisher(
            Float64, "/practica5/metrics/distance_error", 10
        )
        self.yaw_error_pub = self.create_publisher(
            Float64, "/practica5/metrics/yaw_error", 10
        )
        self.cross_track_pub = self.create_publisher(
            Float64, "/practica5/metrics/cross_track_error", 10
        )
        self.steering_error_pub = self.create_publisher(
            Float64, "/practica5/metrics/steering_error", 10
        )

        # Subscribers
        self.create_subscription(
            Pose2D,
            str(
                self.get_parameter(
                    "goal_pose_topic"
                ).value
            ),
            self.goal_pose_callback,
            10,
        )
        self.create_subscription(
            Point,
            str(
                self.get_parameter(
                    "goal_point_topic"
                ).value
            ),
            self.goal_point_callback,
            10,
        )
        self.create_subscription(
            Pose2D,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("path_topic").value),
            self.path_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(
                self.get_parameter(
                    "desired_steering_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "desired_steering",
                float(m.data),
            ),
            10,
        )
        self.create_subscription(
            SteeringAngle,
            str(
                self.get_parameter(
                    "measured_steering_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "measured_steering",
                float(m.value),
            ),
            10,
        )
        self.create_subscription(
            Float64,
            str(
                self.get_parameter(
                    "steering_command_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "controller_steering_cmd",
                float(m.data),
            ),
            10,
        )
        self.create_subscription(
            NormalizedSteeringCommand,
            str(
                self.get_parameter(
                    "actuator_steering_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "actuator_steering_cmd",
                float(m.value),
            ),
            10,
        )
        self.create_subscription(
            NormalizedSpeedCommand,
            str(
                self.get_parameter(
                    "actuator_speed_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "actuator_speed_cmd",
                float(m.value),
            ),
            10,
        )
        self.create_subscription(
            Float64,
            str(
                self.get_parameter(
                    "curvature_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "curvature",
                float(m.data),
            ),
            10,
        )
        self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    "steer_saturated_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "steer_saturated",
                bool(m.data),
            ),
            10,
        )
        self.create_subscription(
            Int32,
            str(
                self.get_parameter(
                    "drive_direction_topic"
                ).value
            ),
            lambda m: setattr(
                self,
                "drive_direction",
                int(m.data),
            ),
            10,
        )
        self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    "goal_reached_topic"
                ).value
            ),
            self.goal_reached_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    "goal_blocked_topic"
                ).value
            ),
            self.goal_blocked_callback,
            10,
        )

        self.create_service(
            Trigger,
            "/practica5/metrics/finish_trial",
            self.finish_trial_service,
        )

        self.create_timer(0.05, self.sample_timer)

        self.get_logger().info(
            f"AutoMiny metrics ready: {self.output_dir}"
        )
        self.get_logger().info(
            "Passive observer only; no actuator publishers."
        )

    # --------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------

    def goal_pose_callback(self, msg: Pose2D):
        if self.active:
            self.finish_trial("replaced_by_new_goal")

        self.goal_xy = (float(msg.x), float(msg.y))
        self.goal_yaw = wrap_to_pi(float(msg.theta))
        self.orientation_required = True
        self.start_trial()

    def goal_point_callback(self, msg: Point):
        if self.active:
            self.finish_trial("replaced_by_new_goal")

        self.goal_xy = (float(msg.x), float(msg.y))
        self.goal_yaw = None
        self.orientation_required = False
        self.start_trial()

    def pose_callback(self, msg: Pose2D):
        self.pose = (
            float(msg.x),
            float(msg.y),
            float(msg.theta),
        )

    def path_callback(self, msg: NavPath):
        self.path_points = [
            (
                float(p.pose.position.x),
                float(p.pose.position.y),
            )
            for p in msg.poses
        ]

    def goal_reached_callback(self, msg: Bool):
        if self.active and bool(msg.data):
            self.finish_trial("goal_reached")

    def goal_blocked_callback(self, msg: Bool):
        if self.active and bool(msg.data):
            self.finish_trial("goal_blocked")

    # --------------------------------------------------------------
    # Trial
    # --------------------------------------------------------------

    def start_trial(self):
        if self.goal_xy is None:
            return

        gx, gy = self.goal_xy
        stamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        if self.orientation_required:
            theta_deg = math.degrees(self.goal_yaw)
            name = (
                f"trial_{stamp}_goal_"
                f"{gx:+.2f}_{gy:+.2f}_"
                f"theta_{theta_deg:+.1f}deg.csv"
            )
        else:
            name = (
                f"trial_{stamp}_goal_"
                f"{gx:+.2f}_{gy:+.2f}.csv"
            )

        self.csv_path = self.output_dir / name
        self.csv_file = open(
            self.csv_path,
            "w",
            newline="",
            encoding="utf-8",
        )

        fields = [
            "time_s",
            "x",
            "y",
            "yaw_rad",
            "goal_x",
            "goal_y",
            "goal_yaw_rad",
            "orientation_required",
            "error_x_goal_minus_actual",
            "error_y_goal_minus_actual",
            "distance_error",
            "yaw_error_rad",
            "yaw_error_deg",
            "cross_track_error",
            "desired_steering_rad",
            "measured_steering_rad",
            "steering_error_rad",
            "controller_steering_normalized",
            "actuator_steering_normalized",
            "actuator_speed_normalized",
            "curvature",
            "steer_saturated",
            "drive_direction",
            "path_length_travelled",
        ]

        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=fields,
        )
        self.writer.writeheader()

        self.active = True
        self.start_time = self.get_clock().now()
        self.start_pose = self.pose
        self.previous_pose = (
            (self.pose[0], self.pose[1])
            if self.pose
            else None
        )

        self.sample_count = 0
        self.path_length = 0.0
        self.cross_track_errors = []
        self.steering_errors = []
        self.yaw_errors = []
        self.saturation_samples = 0

        extra = (
            f", theta={math.degrees(self.goal_yaw):.1f}deg"
            if self.orientation_required
            else ""
        )
        self.get_logger().info(
            f"TRIAL START goal=({gx:.2f},{gy:.2f}{extra}) "
            f"file={self.csv_path}"
        )

    def finish_trial_service(
        self, request, response
    ):
        del request
        if not self.active:
            response.success = False
            response.message = "No active trial."
            return response

        response.success = True
        response.message = self.finish_trial(
            "manual_finish"
        )
        return response

    def finish_trial(self, reason: str):
        if not self.active:
            return "No active trial."

        self.sample_timer()

        gx, gy = self.goal_xy
        if self.pose:
            fx, fy, fyaw = self.pose
        else:
            fx = fy = fyaw = float("nan")

        ex = gx - fx
        ey = gy - fy
        final_dist = math.hypot(ex, ey)

        if (
            self.orientation_required
            and self.goal_yaw is not None
            and math.isfinite(fyaw)
        ):
            final_yaw_error = wrap_to_pi(
                self.goal_yaw - fyaw
            )
        else:
            final_yaw_error = float("nan")

        duration = (
            (
                self.get_clock().now()
                - self.start_time
            ).nanoseconds / 1e9
            if self.start_time is not None
            else float("nan")
        )

        sat_pct = (
            100.0
            * self.saturation_samples
            / self.sample_count
            if self.sample_count
            else float("nan")
        )

        summary = {
            "timestamp": datetime.now().isoformat(
                timespec="seconds"
            ),
            "trial_file": (
                self.csv_path.name
                if self.csv_path
                else ""
            ),
            "reason": reason,
            "orientation_required": int(
                self.orientation_required
            ),
            "goal_x": gx,
            "goal_y": gy,
            "goal_yaw_rad": (
                self.goal_yaw
                if self.goal_yaw is not None
                else float("nan")
            ),
            "goal_yaw_deg": (
                math.degrees(self.goal_yaw)
                if self.goal_yaw is not None
                else float("nan")
            ),
            "final_x": fx,
            "final_y": fy,
            "final_yaw_rad": fyaw,
            "final_yaw_deg": (
                math.degrees(fyaw)
                if math.isfinite(fyaw)
                else float("nan")
            ),
            "final_error_x": ex,
            "final_error_y": ey,
            "final_distance_error": final_dist,
            "final_yaw_error_rad": final_yaw_error,
            "final_yaw_error_deg": (
                math.degrees(final_yaw_error)
                if math.isfinite(final_yaw_error)
                else float("nan")
            ),
            "yaw_error_rmse_rad": safe_rmse(
                self.yaw_errors
            ),
            "yaw_error_mae_rad": safe_mean(
                [abs(v) for v in self.yaw_errors]
            ),
            "cross_track_rmse": safe_rmse(
                self.cross_track_errors
            ),
            "cross_track_mae": safe_mean(
                [abs(v) for v in self.cross_track_errors]
            ),
            "steering_error_bias_rad": safe_mean(
                self.steering_errors
            ),
            "steering_error_rmse_rad": safe_rmse(
                self.steering_errors
            ),
            "steering_error_mae_rad": safe_mean(
                [abs(v) for v in self.steering_errors]
            ),
            "steering_saturation_pct": sat_pct,
            "path_length_travelled": self.path_length,
            "duration_s": duration,
            "samples": self.sample_count,
        }

        summary_path = self.output_dir / "summary.csv"
        new_file = not summary_path.exists()

        with open(
            summary_path,
            "a",
            newline="",
            encoding="utf-8",
        ) as f:
            writer = csv.DictWriter(
                f,
                fieldnames=list(summary.keys()),
            )
            if new_file:
                writer.writeheader()
            writer.writerow(summary)

        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()

        self.csv_file = None
        self.writer = None
        self.active = False

        text = (
            f"TRIAL END [{reason}] "
            f"goal=({gx:.2f},{gy:.2f}) "
            f"final=({fx:.2f},{fy:.2f}) "
            f"pos_err={final_dist:.3f}m"
        )
        if math.isfinite(final_yaw_error):
            text += (
                f" yaw_err="
                f"{math.degrees(final_yaw_error):.1f}deg"
            )

        self.get_logger().info(text)
        self.get_logger().info(
            f"Summary: {summary_path}"
        )
        return text

    # --------------------------------------------------------------
    # Metrics
    # --------------------------------------------------------------

    def nearest_path_distance(
        self, x: float, y: float
    ) -> float:
        if not self.path_points:
            return float("nan")
        return min(
            math.hypot(px - x, py - y)
            for px, py in self.path_points
        )

    def sample_timer(self):
        if (
            not self.active
            or self.goal_xy is None
            or self.pose is None
        ):
            return

        x, y, yaw = self.pose
        gx, gy = self.goal_xy

        ex = gx - x
        ey = gy - y
        dist_error = math.hypot(ex, ey)

        yaw_error = float("nan")
        if (
            self.orientation_required
            and self.goal_yaw is not None
        ):
            yaw_error = wrap_to_pi(
                self.goal_yaw - yaw
            )
            self.yaw_errors.append(yaw_error)

        cross_track = self.nearest_path_distance(
            x, y
        )
        if math.isfinite(cross_track):
            self.cross_track_errors.append(
                cross_track
            )

        steering_error = float("nan")
        if (
            math.isfinite(self.desired_steering)
            and math.isfinite(self.measured_steering)
        ):
            steering_error = (
                self.desired_steering
                - self.measured_steering
            )
            self.steering_errors.append(
                steering_error
            )

        if self.previous_pose is not None:
            self.path_length += math.hypot(
                x - self.previous_pose[0],
                y - self.previous_pose[1],
            )
        self.previous_pose = (x, y)

        if self.steer_saturated:
            self.saturation_samples += 1

        self.sample_count += 1

        live_values = [
            (self.error_x_pub, ex),
            (self.error_y_pub, ey),
            (
                self.distance_error_pub,
                dist_error,
            ),
            (self.yaw_error_pub, yaw_error),
            (
                self.cross_track_pub,
                cross_track,
            ),
            (
                self.steering_error_pub,
                steering_error,
            ),
        ]
        for pub, value in live_values:
            if math.isfinite(value):
                m = Float64()
                m.data = float(value)
                pub.publish(m)

        elapsed = (
            (
                self.get_clock().now()
                - self.start_time
            ).nanoseconds / 1e9
            if self.start_time
            else 0.0
        )

        if self.writer is not None:
            self.writer.writerow(
                {
                    "time_s": elapsed,
                    "x": x,
                    "y": y,
                    "yaw_rad": yaw,
                    "goal_x": gx,
                    "goal_y": gy,
                    "goal_yaw_rad": (
                        self.goal_yaw
                        if self.goal_yaw is not None
                        else float("nan")
                    ),
                    "orientation_required": int(
                        self.orientation_required
                    ),
                    "error_x_goal_minus_actual": ex,
                    "error_y_goal_minus_actual": ey,
                    "distance_error": dist_error,
                    "yaw_error_rad": yaw_error,
                    "yaw_error_deg": (
                        math.degrees(yaw_error)
                        if math.isfinite(yaw_error)
                        else float("nan")
                    ),
                    "cross_track_error": cross_track,
                    "desired_steering_rad": self.desired_steering,
                    "measured_steering_rad": self.measured_steering,
                    "steering_error_rad": steering_error,
                    "controller_steering_normalized": self.controller_steering_cmd,
                    "actuator_steering_normalized": self.actuator_steering_cmd,
                    "actuator_speed_normalized": self.actuator_speed_cmd,
                    "curvature": self.curvature,
                    "steer_saturated": int(
                        self.steer_saturated
                    ),
                    "drive_direction": self.drive_direction,
                    "path_length_travelled": self.path_length,
                }
            )

            if self.sample_count % 20 == 0:
                self.csv_file.flush()

    def destroy_node(self):
        if self.active:
            self.finish_trial(
                "metrics_node_shutdown"
            )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoMinyMetrics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
