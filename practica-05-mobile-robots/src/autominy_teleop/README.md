# AutoMiny Xbox Teleoperation — ROS 2 Humble

This package runs entirely on the external laptop. It does not require modifying the AutoMiny onboard computer.

## Topics

Input:
- `/joy` (`sensor_msgs/msg/Joy`)

Outputs:
- `/actuators/speed_normalized` (`autominy_msgs/msg/NormalizedSpeedCommand`)
- `/actuators/steering_normalized` (`autominy_msgs/msg/NormalizedSteeringCommand`)

## Safety

The vehicle only moves while the configured dead-man button is held.
Default:
- button 0: enable
- left stick vertical (axis 1): speed
- left stick horizontal (axis 0): steering
- speed limited to 25% normalized command
- steering limited to 50%
- joystick timeout: 0.5 s -> stop

Verify the actual Xbox mapping with:

    ros2 topic echo /joy

Edit `config/xbox.yaml` if needed.

## Install joystick driver

    sudo apt update
    sudo apt install ros-humble-joy joystick

## Build

Copy this folder into `~/ros2_ws/src/`, then:

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select autominy_teleop
    source install/setup.bash

## Run

Connect to the AutoMiny Wi-Fi and confirm:

    echo $ROS_DOMAIN_ID
    echo $RMW_IMPLEMENTATION
    ros2 topic list --no-daemon

Connect the Xbox controller via USB, then:

    ros2 launch autominy_teleop xbox_teleop.launch.py

In another terminal:

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 topic echo /joy

Start with the car lifted off the floor or with clear space.
