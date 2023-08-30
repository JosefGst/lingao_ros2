/*
 * Copyright (C) 2022, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2021-08-06 17:08:41
 * @LastEditTime 2022-03-08 21:26:35
 * @LastEditors owen
 * @Description 
 * @FilePath /lingao_ws/src/lingaoRobot/lingao_ros/lingao_base/src/lingao_base_node.cpp
 */

#include "lingao_base_ros2/base_driver.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseDriver>());
    rclcpp::shutdown();
    return 0;
}