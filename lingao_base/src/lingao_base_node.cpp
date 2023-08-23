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

#include "base_driver.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "lingao_base_driver");

    Base_Driver base_driver;
    base_driver.base_Loop();

    return 0;
}
