# ROS 2
![humble badge](https://github.com/JosefGst/lingao_ros2/actions/workflows/humble.yaml/badge.svg)
![iron badge](https://github.com/JosefGst/lingao_ros2/actions/workflows/iron.yaml/badge.svg)
![rolling badge](https://github.com/JosefGst/lingao_ros2/actions/workflows/rolling.yaml/badge.svg)

## Dependencies
Install the [aws warehouse](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) in **aws_ws** and source it.

    mkdir -p aws_ws/src
    cd aws_ws/src/
    git clone https://github.com/JosefGst/aws-robomaker-small-warehouse-world.git -b ros2
    cd aws_ws
    # build for ROS
    rosdep install --from-paths . --ignore-src -r -y
    colcon build

    # run in ROS
    source install/setup.sh
    roslaunch aws_robomaker_small_warehouse_world view_small_warehouse.launch


## Installation

    sudo apt install libboost-all-dev
    mkdir ros2/lingao_ws/src -p
    cd ros2/lingao_ros2/src
    git clone https://github.com/JosefGst/lingao_ros2.git
    sudo apt install python3-vcstool
    vcs import .. < my.repos
    cd ~/ros2/lingao_ws
    rosdep update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

and build the workspace.
## :computer: Description

    ros2 launch urdf_launch display.launch.py urdf_package:=lingao_description urdf_package_path:=urdf/MiniUGV_10A.xacro

For Gazebo simulation

    ros2 launch lingao_description gazebo_launch.py open_rviz:=true

![urdf](https://github.com/JosefGst/lingao_ros2/blob/humble/images/urdf.png)
## :computer: Sim

    ros2 launch lingao_sim sim_launch.py

![sim](https://github.com/JosefGst/lingao_ros2/blob/humble/images/sim.png)

## :computer: SLAM

    ros2 launch lingao_slam slam_launch.py use_sim_time:=true open_rviz:=true

for localization only

    ros2 launch lingao_slam slam_launch.py slam_params_file:=src/lingao_ros2/lingao_slam/config/localizer_params_online_async.yaml use_sim_time:=true

![slam](https://github.com/JosefGst/lingao_ros2/blob/humble/images/slam.png)

## :computer: Navigation

    ros2 launch lingao_nav lingao_nav_launch.py use_sim_time:=true

![nav](https://github.com/JosefGst/lingao_ros2/blob/humble/images/nav.png)
    
## :robot: Bringup

    ros2 launch lingao_bringup bringup_launch.py

## :robot: SLAM

    ros2 launch lingao_slam slam_launch.py

:computer:

    ros2 run rviz2 rviz2 -d ros2/lingao_ws/src/lingao_ros2/lingao_slam/config/slam.rviz

![nav](https://github.com/JosefGst/lingao_ros2/blob/humble/images/home.png)

## :robot: Navigation

    ros2 launch lingao_nav lingao_nav_launch.py

:computer:
    
    ros2 run rviz2 rviz2 -d ros2/lingao_ws/src/lingao_ros2/lingao_nav/config/nav.rviz 
