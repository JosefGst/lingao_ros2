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

#if BOOST_VERSION >= 107000
    io_sev_ = boost::make_shared<boost::asio::io_context>();
#else
    io_sev_ = boost::make_shared<boost::asio::io_service>();
#endif
    //  serial_port_ = "/dev/lingao";
    //
}

Serial_Async::~Serial_Async()
{
    if (isOpen_)
    {
        try
        {
            doClose();
        }
        catch (...)
        {
        }
    }
}

void Serial_Async::doClose()
{
    if (isOpen_)
    {
        isOpen_ = false;
        //        boost::system::error_code ec;
        port_->cancel();
        port_->close();
    }
}

void Serial_Async::DoSomething()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Serial do something ");
}
