/*
 *  Copyright (C) 2022, LingAo Robotics, INC.
 *  author: owen <keaa@keaa.net>
 *  maintainer: owen <keaa@keaa.net>
 *
 *  Serial Port Async server
 */

#include <sys/ioctl.h> 

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include "lingao_base_ros2/Serial_Async.hpp"

Serial_Async::Serial_Async() : Node("Serial_Async")
{
    std::string some_string = "hiya";
}

void Serial_Async::DoSomething()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Serial do something ");
}
