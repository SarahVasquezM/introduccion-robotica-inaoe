# AutoMiny Pure Pursuit v5 — goal pose (x, y, theta)

## New goal interface

Use:

    /practica5/goal_pose
    geometry_msgs/msg/Pose2D

`theta` is the desired FINAL VEHICLE orientation in radians in the local frame.

Examples:

Forward orientation:

    ros2 topic pub --once \
      /practica5/goal_pose \
      geometry_msgs/msg/Pose2D \
      "{x: 2.0, y: 1.0, theta: 0.0}"

90 degrees:

    ros2 topic pub --once \
      /practica5/goal_pose \
      geometry_msgs/msg/Pose2D \
      "{x: 2.0, y: 1.0, theta: 1.5707963}"

-90 degrees:

    ros2 topic pub --once \
      /practica5/goal_pose \
      geometry_msgs/msg/Pose2D \
      "{x: 2.0, y: -1.0, theta: -1.5707963}"

180 degrees:

    ros2 topic pub --once \
      /practica5/goal_pose \
      geometry_msgs/msg/Pose2D \
      "{x: 1.0, y: 0.0, theta: 3.1415927}"

## How orientation is enforced

The cubic Bezier path uses:

    P0 = current vehicle position
    P1 = current direction of motion
    P2 = chosen from requested final orientation
    P3 = requested final position

Therefore the path tangent at P3 matches the requested final orientation
(for reverse, the direction of motion is theta + pi while vehicle yaw is theta).

The goal is complete only when:

    position_error <= 0.08 m

AND:

    abs(yaw_error) <= 10 degrees

## Legacy position-only command

Still supported:

    /practica5/goal_point
    geometry_msgs/msg/Point

That command does not enforce final yaw.

## Build

    rm -rf ~/ros2_ws/src/autominy_pure_pursuit
    cp -r autominy_pure_pursuit ~/ros2_ws/src/

    cd ~/ros2_ws
    rm -rf build/autominy_pure_pursuit
    rm -rf install/autominy_pure_pursuit

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    colcon build --symlink-install \
      --packages-select autominy_pure_pursuit

    source ~/ros2_ws/install/setup.bash

## Run

    export ROS_DOMAIN_ID=61
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_LOCALHOST_ONLY=0

    ros2 launch autominy_pure_pursuit pure_pursuit.launch.py

Do not run another actuator controller at the same time.

## Reset before independent tests

    ros2 service call \
      /practica5/reset_origin \
      std_srvs/srv/Trigger \
      "{}"

## Orientation error

    ros2 topic echo /practica5/yaw_error --no-daemon

This is in radians.
