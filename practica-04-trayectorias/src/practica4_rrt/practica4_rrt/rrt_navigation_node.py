"""Plan each PIDcontrol goal with RRT and follow one continuous trajectory."""
import csv
from datetime import datetime
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_action.action import PIDcontrol
from gazebo_msgs.msg import ContactsState, ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .evidence import plan, save
from .map_utils import OccupancyMap
from .plotting import plot
from .trajectory_utils import ArcLengthPath, path_length, rounded_path


class RRTNavigationNode(Node):
    def __init__(self):
        super().__init__('rrt_navigation')
        defaults = dict(
            execute=True, seed=22, step_size=1.0, goal_bias=0.15,
            goal_tolerance=0.8, max_iterations=10000,
            smoothing_attempts=200, corner_distance=2.0,
            corner_samples=20, seconds_per_meter=2.0,
            tracking_kp=3.5, vertical_kp=2.0, yaw_kp=0.8,
            max_horizontal_speed=0.9, max_vertical_speed=0.8,
            max_yaw_speed=1.0, final_position_tolerance=0.03,
            final_yaw_tolerance=0.05, max_tilt=0.75, pose_timeout=1.0,
            progress_log_period=1.0, safety_margin=0.15,
            world_config=(get_package_share_directory('practica4_rrt') +
                          '/config/world_config.yaml'),
            results_dir=str(Path.home()/'ros2_ws/results/practica4'),
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        positive = (
            'step_size', 'goal_tolerance', 'max_iterations',
            'corner_distance', 'corner_samples', 'seconds_per_meter',
            'tracking_kp', 'vertical_kp', 'yaw_kp',
            'max_horizontal_speed', 'max_vertical_speed', 'max_yaw_speed',
            'final_position_tolerance', 'final_yaw_tolerance', 'max_tilt',
            'pose_timeout', 'progress_log_period', 'safety_margin',
        )
        if any(not math.isfinite(float(self.p[key])) or self.p[key] <= 0
               for key in positive):
            raise ValueError('Controller, planner and safety parameters must be positive')
        if not 0.0 <= self.p['goal_bias'] <= 1.0:
            raise ValueError('goal_bias must be between zero and one')

        self.grid = OccupancyMap.load(self.p['world_config'])
        safety_config = dict(self.grid.config)
        safety_config['margin'] = self.p['safety_margin']
        self.safety_grid = OccupancyMap(safety_config)
        self.pose = None
        self.pose_wall = 0.0
        self.model_states = {}
        self.model_states_wall = 0.0
        self.contact_detected = False
        self.contact_names = []
        self.state = 'idle'
        self.goal_reserved = False
        self.current_directory = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/rrt_map', qos)
        self.raw_pub = self.create_publisher(RosPath, '/rrt/path_raw', qos)
        self.smooth_pub = self.create_publisher(RosPath, '/rrt/path_smoothed', qos)
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.posctrl_pub = self.create_publisher(Bool, '/simple_drone/posctrl', 10)
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_callback, 10)
        self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)
        self.create_subscription(ContactsState, '/simple_drone/bumper_states',
                                 self.contact_callback, 10)
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            self, PIDcontrol, '/pidcontrol', execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=lambda request: CancelResponse.ACCEPT,
            callback_group=self.action_group)
        self.publish_map()
        self.get_logger().info(
            'RRT PIDcontrol server ready. Send any collision-free world '
            'coordinate to /pidcontrol; no restart is required.')

    def pose_callback(self, msg):
        self.pose = msg
        self.pose_wall = time.monotonic()

    def model_states_callback(self, msg):
        self.model_states = dict(zip(msg.name, msg.pose))
        self.model_states_wall = time.monotonic()

    def contact_callback(self, msg):
        if self.state == 'running' and msg.states:
            self.contact_detected = True
            self.contact_names = sorted({state.collision1_name for state in msg.states} |
                                        {state.collision2_name for state in msg.states})

    def goal_callback(self, goal):
        values = (goal.xd, goal.yd, goal.zd, goal.wd, goal.tf)
        if (self.goal_reserved or self.state == 'running' or
                not all(math.isfinite(value) for value in values) or goal.tf <= 0.0):
            return GoalResponse.REJECT
        self.goal_reserved = True
        return GoalResponse.ACCEPT

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width, msg.info.height = self.grid.width, self.grid.height
        msg.info.origin.position.x, msg.info.origin.position.y = (
            self.grid.bounds[0], self.grid.bounds[2])
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.data
        self.map_pub.publish(msg)

    def publish_path(self, result, path):
        for points, publisher in ((result['raw'], self.raw_pub),
                                  (path, self.smooth_pub)):
            msg = RosPath()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.get_clock().now().to_msg()
            for x, y in points:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.position.z = float(result['goal_z'])
                pose.pose.orientation.z = math.sin(result['goal_yaw']/2.0)
                pose.pose.orientation.w = math.cos(result['goal_yaw']/2.0)
                msg.poses.append(pose)
            publisher.publish(msg)

    def world_check(self):
        now = time.monotonic()
        if self.pose is None or now-self.pose_wall > self.p['pose_timeout']:
            return 'missing or stale /simple_drone/gt_pose'
        if not self.model_states or now-self.model_states_wall > self.p['pose_timeout']:
            return 'missing or stale /model_states; Gazebo world cannot be verified'
        for obstacle in self.grid.config['obstacles']:
            gazebo_pose = self.model_states.get(obstacle['name'])
            if gazebo_pose is None:
                return f'Gazebo world does not match map: missing {obstacle["name"]}'
            error = math.dist((gazebo_pose.position.x, gazebo_pose.position.y),
                              (obstacle['x'], obstacle['y']))
            if error > self.grid.resolution/2.0:
                return f'Gazebo world does not match map at {obstacle["name"]}'
        publishers = self.get_publishers_info_by_topic('/simple_drone/cmd_vel')
        if len(publishers) != 1 or publishers[0].node_name != self.get_name():
            return 'another cmd_vel controller is active; stop it before navigating'
        return None

    def stop_drone(self):
        self.cmd_pub.publish(Twist())

    def abort(self, goal_handle, message):
        self.stop_drone()
        self.state = 'idle'
        self.goal_reserved = False
        goal_handle.abort()
        result = PIDcontrol.Result()
        result.result = 'Goal failed: ' + message
        self.get_logger().error(result.result)
        return result

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0-2.0*(q.y*q.y + q.z*q.z))

    @staticmethod
    def attitude_from_quaternion(q):
        roll = math.atan2(2.0*(q.w*q.x + q.y*q.z),
                          1.0-2.0*(q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0*(q.w*q.y-q.z*q.x))))
        return roll, pitch

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        if not self.p['execute']:
            return self.abort(goal_handle, 'execution is disabled')
        reason = self.world_check()
        if reason:
            return self.abort(goal_handle, reason)

        start_pose = self.pose
        start_xy = (start_pose.position.x, start_pose.position.y)
        goal_xy = (goal.xd, goal.yd)
        if not self.grid.point_free(start_xy):
            return self.abort(goal_handle, 'current position is inside inflated occupancy')
        if not self.grid.point_free(goal_xy):
            return self.abort(goal_handle, 'requested XY is outside free space')
        maximum_z = min(obstacle['height'] for obstacle in
                        self.grid.config['obstacles'])-self.grid.config['drone_radius']
        if not 0.5 <= goal.zd <= maximum_z:
            return self.abort(goal_handle,
                              f'zd must be between 0.5 and {maximum_z:.2f} m')

        try:
            if math.dist(start_xy, goal_xy) < 1e-6:
                result_data = dict(
                    world_config=self.grid.config, start=list(start_xy),
                    goal=list(goal_xy), seed=self.p['seed'], raw=[start_xy],
                    smoothed=[start_xy], nodes=[start_xy], parents=[-1],
                    iterations=0, raw_length=0.0, smoothed_length=0.0,
                    planning_wall_seconds=0.0, parameters={})
                continuous_path = [start_xy]
            else:
                result_data = plan(
                    self.grid, start_xy, goal_xy, goal.tf, int(self.p['seed']),
                    float(self.p['step_size']), float(self.p['goal_bias']),
                    float(self.p['goal_tolerance']), int(self.p['max_iterations']),
                    int(self.p['smoothing_attempts']))
                result_data['shortcut_smoothed'] = result_data['smoothed']
                continuous_path = rounded_path(
                    result_data['smoothed'], self.grid,
                    float(self.p['corner_distance']), int(self.p['corner_samples']))
                result_data['smoothed'] = continuous_path
                result_data['smoothed_length'] = path_length(continuous_path)
        except (ValueError, RuntimeError) as exc:
            return self.abort(goal_handle, f'planning failed: {exc}')

        arc_path = ArcLengthPath(continuous_path)
        delta_z = goal.zd-start_pose.position.z
        spatial_length = math.hypot(arc_path.length, delta_z)
        minimum_time = self.p['seconds_per_meter']*spatial_length
        if goal.tf+1e-9 < minimum_time:
            return self.abort(
                goal_handle,
                'requested tf={:.3f} s is too short for the {:.3f} m route; '
                'use at least tf={:.3f} s (2 s/m)'.format(
                    goal.tf, spatial_length, minimum_time))

        result_data.update(
            goal_z=goal.zd, goal_yaw=goal.wd, requested_time=goal.tf,
            minimum_time=minimum_time, spatial_length=spatial_length,
            execution_status='running', navigation_parameters=self.p)
        self.current_directory = (Path(self.p['results_dir']).expanduser() /
                                  datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        save(result_data, self.current_directory)
        plot(self.current_directory/'results.json')
        self.publish_path(result_data, continuous_path)

        csv_file = (self.current_directory/'flight.csv').open('w', newline='')
        writer = csv.writer(csv_file)
        writer.writerow(['wall_elapsed', 'x', 'y', 'z', 'ref_x', 'ref_y',
                         'ref_z', 'err_x', 'err_y', 'err_z', 'err_yaw'])
        posctrl = Bool()
        posctrl.data = False
        for _ in range(5):
            self.posctrl_pub.publish(posctrl)
            time.sleep(0.02)

        self.contact_detected = False
        self.contact_names = []
        self.state = 'running'
        feedback = PIDcontrol.Feedback()
        begun = time.monotonic()
        next_log = 0.0

        try:
            while True:
                elapsed = time.monotonic()-begun
                u = min(1.0, elapsed/goal.tf)
                progress = 3.0*u*u-2.0*u*u*u
                progress_rate = (6.0*u-6.0*u*u)/goal.tf if u < 1.0 else 0.0
                ref_xy, tangent = arc_path.sample(progress*arc_path.length)
                xy_speed = progress_rate*arc_path.length
                ref_vx, ref_vy = tangent[0]*xy_speed, tangent[1]*xy_speed
                ref_z = start_pose.position.z+progress*delta_z
                ref_vz = progress_rate*delta_z

                pose = self.pose
                ex, ey, ez = (ref_xy[0]-pose.position.x,
                              ref_xy[1]-pose.position.y,
                              ref_z-pose.position.z)
                yaw = self.yaw_from_quaternion(pose.orientation)
                ew = math.atan2(math.sin(goal.wd-yaw), math.cos(goal.wd-yaw))
                feedback.err_x, feedback.err_y = ex, ey
                feedback.err_z, feedback.err_w = ez, ew
                goal_handle.publish_feedback(feedback)
                writer.writerow([elapsed, pose.position.x, pose.position.y,
                                 pose.position.z, ref_xy[0], ref_xy[1], ref_z,
                                 ex, ey, ez, ew])

                if goal_handle.is_cancel_requested:
                    self.stop_drone()
                    self.state, self.goal_reserved = 'idle', False
                    goal_handle.canceled()
                    result = PIDcontrol.Result()
                    result.result = (
                        'Goal canceled at {:.3f}/{:.3f} s; final error '
                        'x={:.4f}, y={:.4f}, z={:.4f} m, yaw={:.4f} rad'.format(
                            elapsed, goal.tf, ex, ey, ez, ew))
                    result_data.update(execution_status='canceled',
                                       result=result.result, real_time=elapsed)
                    save(result_data, self.current_directory)
                    return result

                failure = ''
                if time.monotonic()-self.pose_wall > self.p['pose_timeout']:
                    failure = 'pose stream stopped'
                elif self.contact_detected:
                    failure = 'collision contact detected: '+', '.join(self.contact_names)
                elif not self.safety_grid.point_free((pose.position.x, pose.position.y)):
                    failure = 'drone entered the physical safety envelope'
                else:
                    roll, pitch = self.attitude_from_quaternion(pose.orientation)
                    if abs(roll) > self.p['max_tilt'] or abs(pitch) > self.p['max_tilt']:
                        failure = f'unsafe attitude roll={roll:.4f}, pitch={pitch:.4f} rad'
                if failure:
                    result_data.update(execution_status='failed',
                                       real_time=elapsed, result=failure)
                    save(result_data, self.current_directory)
                    return self.abort(goal_handle, failure)
                if elapsed >= goal.tf:
                    break

                command_vx = ref_vx+self.p['tracking_kp']*ex
                command_vy = ref_vy+self.p['tracking_kp']*ey
                horizontal = math.hypot(command_vx, command_vy)
                if horizontal > self.p['max_horizontal_speed']:
                    scale = self.p['max_horizontal_speed']/horizontal
                    command_vx, command_vy = command_vx*scale, command_vy*scale
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                command = Twist()
                command.linear.x = cos_yaw*command_vx+sin_yaw*command_vy
                command.linear.y = -sin_yaw*command_vx+cos_yaw*command_vy
                command.linear.z = float(np.clip(
                    ref_vz+self.p['vertical_kp']*ez,
                    -self.p['max_vertical_speed'], self.p['max_vertical_speed']))
                command.angular.z = float(np.clip(
                    self.p['yaw_kp']*ew,
                    -self.p['max_yaw_speed'], self.p['max_yaw_speed']))
                self.cmd_pub.publish(command)
                if elapsed >= next_log:
                    self.get_logger().info(
                        'trajectory t={:.2f}/{:.2f} s, error=({:.4f}, {:.4f}, '
                        '{:.4f}) m, yaw={:.4f} rad'.format(
                            elapsed, goal.tf, ex, ey, ez, ew))
                    next_log += self.p['progress_log_period']
                time.sleep(0.01)
        finally:
            csv_file.close()

        self.stop_drone()
        elapsed = time.monotonic()-begun
        pose = self.pose
        ex, ey, ez = (goal.xd-pose.position.x, goal.yd-pose.position.y,
                      goal.zd-pose.position.z)
        yaw = self.yaw_from_quaternion(pose.orientation)
        ew = math.atan2(math.sin(goal.wd-yaw), math.cos(goal.wd-yaw))
        position_error = math.sqrt(ex*ex+ey*ey+ez*ez)
        success = (position_error <= self.p['final_position_tolerance'] and
                   abs(ew) <= self.p['final_yaw_tolerance'])
        status = 'succeeded' if success else 'failed'
        message = (
            'Goal {}: desired time {:.3f} s; wall time {:.3f} s; route '
            '{:.3f} m; final error x={:.4f}, y={:.4f}, z={:.4f} m, '
            'norm={:.4f} m, yaw={:.4f} rad'.format(
                status, goal.tf, elapsed, spatial_length,
                ex, ey, ez, position_error, ew))
        result_data.update(
            execution_status=status, real_time=elapsed,
            final_error=dict(x=ex, y=ey, z=ez, norm=position_error, yaw=ew),
            result=message)
        save(result_data, self.current_directory)
        self.state, self.goal_reserved = 'idle', False
        result = PIDcontrol.Result()
        result.result = message
        if success:
            goal_handle.succeed()
            self.get_logger().info(message)
        else:
            goal_handle.abort()
            self.get_logger().error(message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RRTNavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_drone()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
