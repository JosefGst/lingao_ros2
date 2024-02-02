![humble badge](https://github.com/JosefGst/lingao_ros2/actions/workflows/humble.yaml/badge.svg)
![iron badge](https://github.com/JosefGst/lingao_ros2/actions/workflows/iron.yaml/badge.svg)
![rolling badge](https://github.com/JosefGst/lingao_ros2/actions/workflows/rolling.yaml/badge.svg)
Developed in ROS Iron. Some features may not work in ROS Humble.

# Table of Content

<details>
<summary>Click to expand</summary>

- [Table of Content](#table-of-content)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
- [:computer: Simulation](#computer-simulation)
  - [:computer: Description](#computer-description)
  - [:computer: Sim](#computer-sim)
  - [:computer: SLAM](#computer-slam)
  - [:computer: Navigation](#computer-navigation)
    - [:computer: Navigate outdoors with GPS](#computer-navigate-outdoors-with-gps)
- [:robot: Real Robot](#robot-real-robot)
  - [:robot: Bringup](#robot-bringup)
  - [:robot: SLAM](#robot-slam)
  - [:robot: Navigation](#robot-navigation)
- [TODO:](#todo)
</details>

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

# :computer: Simulation

## :computer: Description

    ros2 launch urdf_launch display.launch.py urdf_package:=lingao_description urdf_package_path:=urdf/MiniUGV_10A.xacro

For Gazebo simulation

    ros2 launch lingao_description gazebo_launch.py open_rviz:=true

![urdf](https://github.com/JosefGst/lingao_ros2/blob/humble/images/urdf.png)
## :computer: Sim

    ros2 launch lingao_sim sim_launch.py gui:=false

![sim](https://github.com/JosefGst/lingao_ros2/blob/humble/images/sim.png)

## :computer: SLAM

    ros2 launch lingao_slam slam_launch.py use_sim_time:=true open_rviz:=true

for localization only

    ros2 launch lingao_slam slam_launch.py slam_params_file:=src/lingao_ros2/lingao_slam/config/localizer_params_online_async.yaml use_sim_time:=true

![slam](https://github.com/JosefGst/lingao_ros2/blob/humble/images/slam.png)

## :computer: Navigation

    ros2 launch lingao_nav lingao_nav_launch.py use_sim_time:=true open_rviz:=true

![nav](https://github.com/JosefGst/lingao_ros2/blob/humble/images/nav.png)

### :computer: Navigate outdoors with GPS

    ros2 launch lingao_sim emptyfarm_launch.py headless:=False

![emptyfarm](https://github.com/JosefGst/lingao_ros2/blob/humble/images/emptyfarm.png)

    ros2 launch lingao_bringup mapviz.launch.py 

![emptyfarm](https://github.com/JosefGst/lingao_ros2/blob/humble/images/mapviz.png)

    ros2 launch lingao_nav gps_waypoint_follower.launch.py

    ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower
    ros2 run nav2_gps_waypoint_follower_demo gps_waypoint_logger
    ros2 run nav2_gps_waypoint_follower_demo logged_waypoint_follower </path/to/yaml/file.yaml>
    
![gps_navigation](https://github.com/JosefGst/lingao_ros2/blob/humble/gifs/gps_navigation.gif)

# :robot: Real Robot
    
## :robot: Bringup

    ros2 launch lingao_bringup bringup_launch.py

## :robot: SLAM

    ros2 launch lingao_slam slam_launch.py

:computer:

    ros2 run rviz2 rviz2 -d ros2/lingao_ws/src/lingao_ros2/lingao_slam/config/slam.rviz

![nav](https://github.com/JosefGst/lingao_ros2/blob/humble/images/home.png)

## :robot: Navigation

    ros2 launch lingao_nav lingao_nav_launch.py slam_params_file:=src/lingao_ros2/lingao_slam/config/localizer_params_online_async_home.yaml 

:computer:
    
    ros2 run rviz2 rviz2 -d ros2/lingao_ws/src/lingao_ros2/lingao_nav/config/nav.rviz 

or open Foxglove web https://studio.foxglove.dev/josef-gstoettner/view

![foxglove](https://github.com/JosefGst/lingao_ros2/blob/humble/images/foxglove.png)

# TODO:
- [ ] set init pose with SLAM Toolbox
- [ ] use in bringup
- [ ] use GPS
- [ ] enable Collision Monitor
- [ ] Dockerize
- [ ] generally improve performance

