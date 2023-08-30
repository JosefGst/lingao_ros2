
#include "lingao_base_ros2/base_driver.hpp"
#include "lingao_base_ros2/data_stream.h"
#include "lingao_base_ros2/Serial_Async.h"
#include "lingao_base_ros2/TCP_Async.h"
#include "lingao_base_ros2/UDP_Async.h"

BaseDriver::BaseDriver()
    : Node("lingao_base_driver"), count_(0)
{
    InitParams();

    // serial = boost::make_shared<Serial_Async>();
    // stream = new Data_Stream(serial.get());

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&BaseDriver::timer_callback, this));
}

void BaseDriver::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void BaseDriver::InitParams()
{
    // Serial Port Params
    this->declare_parameter("port_name", std::string("/dev/lingao"));
    this->declare_parameter("port_baud", 230400);
    this->declare_parameter("freq", 100);

    this->get_parameter("port_name", serial_port_);
    this->get_parameter("port_baud", serial_baud_rate);
    this->get_parameter("freq", loop_rate_);

    // Topic Params
    this->declare_parameter("topic_cmd_vel_name", std::string("/cmd_vel"));
    this->declare_parameter("publish_odom_name", std::string("raw_odom"));
    this->declare_parameter("odom_frame_id", std::string("odom"));
    this->declare_parameter("base_frame_id", std::string("base_footprint"));
    this->declare_parameter("cmd_vel_sub_timeout", 1.0);
    this->declare_parameter("pub_odom_tf", false);

    this->get_parameter("topic_cmd_vel_name", topic_cmd_vel_name_);
    this->get_parameter("publish_odom_name", publish_odom_name_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_frame_id", base_frame_id_);
    this->get_parameter("cmd_vel_sub_timeout", cmd_vel_sub_timeout_vel_);
    this->get_parameter("pub_odom_tf", publish_odom_transform_);

    // Scale Params
    this->declare_parameter("linear_scale", 1.0);
    this->declare_parameter("angular_scale", 1.0);

    this->get_parameter("linear_scale", linear_scale_);
    this->get_parameter("angular_scale", angular_scale_);

    // IMU Params
    this->declare_parameter("topic_imu", std::string("/imu/onboard_imu"));
    this->declare_parameter("imu_frame_id", std::string("imu_link"));
    this->declare_parameter("use_imu", false);
    this->declare_parameter("imu_calibrate_gyro", true);
    this->declare_parameter("imu_calib_samples", 300);

    this->get_parameter("topic_imu", topic_imu_);
    this->get_parameter("imu_frame_id", imu_frame_id_);
    this->get_parameter("use_imu", use_imu_);
    this->get_parameter("imu_calibrate_gyro", imu_calibrate_gyro_);
    this->get_parameter("imu_calib_samples", imu_calib_samples_);
}