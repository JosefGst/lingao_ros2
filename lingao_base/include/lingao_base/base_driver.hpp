#ifndef BASE_DRIVER_H
#define BASE_DRIVER_H

#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <functional>
#include <memory>
#include <stdio.h>
#include <string>
#include "lingao_base/data_format.hpp"
#include <lingao_msgs/msg/ling_ao_bms_status.hpp>
#include <lingao_msgs/msg/ling_ao_rc_status.hpp>
#include <boost/shared_ptr.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;
class Data_Stream;
class Serial_Async;
class TCP_Async;
class UDP_Async;
class MyClass;

class BaseDriver : public rclcpp::Node
{
public:
    BaseDriver();

private:
    void InitParams();

    // Timer
    rclcpp::TimerBase::SharedPtr main_timer_cb_;
    void MainTimerCallback();
    rclcpp::TimerBase::SharedPtr timer_10hz_cb_;
    void Timer10HzCallbackCallback();
    bool timer10HzTimeOut;
    rclcpp::TimerBase::SharedPtr timer_1hz_cb_;
    void Timer1HzCallbackCallback();
    bool timer1HzTimeOut;
    rclcpp::TimerBase::SharedPtr cmd_vel_cb_timer_;
    void CmdVelTimeout();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    // Boost
    boost::shared_ptr<TCP_Async> tcp;
    boost::shared_ptr<UDP_Async> udp;
    boost::shared_ptr<Serial_Async> serial;
    Data_Stream *stream;

    boost::shared_ptr<MyClass> myObject;

    // serial port
    std::string serial_port_;
    int serial_baud_rate;
    bool active;

    // vel_cmd
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    void cmd_vel_CallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
    Data_Format_Liner liner_tx_;

    // odom
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Time last_odom_vel_time_ = BaseDriver::get_clock()->now();
    rclcpp::Time current_time;
    std::string publish_odom_name_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    nav_msgs::msg::Odometry odom_msg;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    geometry_msgs::msg::TransformStamped odom_tf;
    Data_Format_Liner liner_rx_;
    double x_pos_;
    double y_pos_;
    double th_;
    int loop_rate_;
    bool publish_odom_transform_;
    void init_odom();
    void calc_odom();
    void publish_odom();
    void setCovariance(bool isMove);

    // Update speed to board
    std::string topic_cmd_vel_name_;
    int cmd_vel_sub_timeout_vel_;

    // CALIB
    double linear_scale_;
    double angular_scale_;

    // IMU
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    sensor_msgs::msg::Imu imu_msg;
    std::string topic_imu_;
    std::string imu_frame_id_;
    Data_Format_IMU imu_data;
    bool use_imu_;
    bool imuStreamActive;
    bool imu_calibrate_gyro_;
    int imu_calib_samples_;
    void init_imu();
    void publish_imu();

    // BMS
    rclcpp::Publisher<lingao_msgs::msg::LingAoBmsStatus>::SharedPtr bat_publisher_;
    lingao_msgs::msg::LingAoBmsStatus bat_msg;
    Data_Format_BAT rxData_battery;

    // RC
    rclcpp::Publisher<lingao_msgs::msg::LingAoRCStatus>::SharedPtr rc_publisher_;
    lingao_msgs::msg::LingAoRCStatus rc_msg;
    Data_Format_RC rxData_rc;
    bool rcStreamActive;
    void init_robot_stream();

};

#endif // BASE_DRIVER_H
