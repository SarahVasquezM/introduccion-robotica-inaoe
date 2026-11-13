# AutoMiny Metrics v2 — position + orientation

Passive logger. It does NOT command the car.

For `goal_pose`, it records:

- final X error
- final Y error
- final Euclidean position error
- final yaw error in rad/deg
- yaw-error RMSE / MAE
- cross-track RMSE / MAE
- steering error bias / RMSE / MAE
- steering saturation percentage
- path length and duration

## Build

    rm -rf ~/ros2_ws/src/autominy_metrics
    cp -r autominy_metrics ~/ros2_ws/src/

    cd ~/ros2_ws
    rm -rf build/autominy_metrics
    rm -rf install/autominy_metrics

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    colcon build --symlink-install \
      --packages-select autominy_metrics

    source ~/ros2_ws/install/setup.bash

## Run

Keep it running in a second terminal:

    ros2 launch autominy_metrics metrics.launch.py

It starts a trial automatically when either:

    /practica5/goal_pose

or:

    /practica5/goal_point

receives a goal.

Files are written to:

    ~/autominy_metrics/

Summary:

    ~/autominy_metrics/summary.csv

## Live orientation error

    ros2 topic echo \
      /practica5/metrics/yaw_error \
      --no-daemon

## Manual trial finish

    ros2 service call \
      /practica5/metrics/finish_trial \
      std_srvs/srv/Trigger \
      "{}"
