
#include "lingao_base_ros2/base_driver.hpp"
#include "lingao_base_ros2/data_stream.hpp"
#include "lingao_base_ros2/Serial_Async.hpp"
// #include "lingao_base_ros2/TCP_Async.hpp"
// #include "lingao_base_ros2/UDP_Async.hpp"
#include "lingao_base_ros2/calibrate_gyro.hpp"
#include "lingao_base_ros2/myObject.hpp"

BaseDriver::BaseDriver()
    : Node("lingao_base_driver"), count_(0)
{
    InitParams();
    // // learning boost shared ptr
    // boost::shared_ptr<std::string> x = boost::make_shared<std::string>("hello, world!");
    // std::cout << *x;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Init: " << *x);

    // // learning my object
    // myObject = boost::make_shared<MyClass>();
    // myObject->DoSomething();

    serial = boost::make_shared<Serial_Async>();
    stream = new Data_Stream(serial.get());

    if (serial->init(serial_port_, serial_baud_rate))
    {
        RCLCPP_INFO(this->get_logger(), "Main board Serial Port open success, com_port_name= '%s'", serial_port_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Main board Serial Port open failed... com_port_name= '%s'", serial_port_.c_str());
        return;
    }

    if (stream->version_detection())
    {
        Data_Format_VER version = stream->get_data_version();
        RCLCPP_INFO(this->get_logger(), "The version matches successfully, current version: [ %d ]", (int)version.protoVer);
        RCLCPP_INFO(this->get_logger(), "GET Equipment Identity: %d", version.equipmentIdentity);
    }
    else
    {
        Data_Format_VER version = stream->get_data_version();
        RCLCPP_INFO(this->get_logger(), "GET Equipment Identity: %d", version.equipmentIdentity);
        RCLCPP_INFO(this->get_logger(), "The driver version does not match,  Main control board driver version:[ %d ] Current driver version:[ %d ]", (int)version.protoVer, LA_PROTO_VER_0310);
        return;
    }

    init_imu();

    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(100), std::bind(&BaseDriver::timer_callback, this));
}

void BaseDriver::timer_callback()
{
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);

    bool isRead = false;
    isRead = stream->get_Message(MSG_ID_GET_IMU);
    if (isRead)
    {
        imu_data = stream->get_data_imu();
        publish_imu();
    }
    else
        RCLCPP_WARN(this->get_logger(), "Get IMU Data Time Out!");
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
    this->declare_parameter("use_imu", true);
    this->declare_parameter("imu_calibrate_gyro", true);
    this->declare_parameter("imu_calib_samples", 300);

    this->get_parameter("topic_imu", topic_imu_);
    this->get_parameter("imu_frame_id", imu_frame_id_);
    this->get_parameter("use_imu", use_imu_);
    this->get_parameter("imu_calibrate_gyro", imu_calibrate_gyro_);
    this->get_parameter("imu_calib_samples", imu_calib_samples_);
}

// IMU
void BaseDriver::init_imu()
{
    imuStreamActive = false;

    if (use_imu_ == true)
    {

        if (!stream->onBoardImuAvailable())
        {
            RCLCPP_WARN(this->get_logger(), "onboard imu unavailable!");
            return;
        }

        imu_msg.header.frame_id = imu_frame_id_;

        // https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
        imu_msg.orientation_covariance[0] = 0.0025;
        imu_msg.orientation_covariance[4] = 0.0025;
        imu_msg.orientation_covariance[8] = 0.0025;

        imu_msg.angular_velocity_covariance[0] = 0.000015;
        imu_msg.angular_velocity_covariance[4] = 0.000015;
        imu_msg.angular_velocity_covariance[8] = 0.000015;

        imu_msg.linear_acceleration_covariance[0] = 0.0001;
        imu_msg.linear_acceleration_covariance[4] = 0.0001;
        imu_msg.linear_acceleration_covariance[8] = 0.0001;

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_imu_, 10);
    }
}

// IMU数据发布
void BaseDriver::publish_imu()
{
    imu_msg.header.stamp = rclcpp::Time();
    imu_msg.linear_acceleration.x = imu_data.accx * 9.80665; // 加速度应以 m/s^2（原单位 g ）
    imu_msg.linear_acceleration.y = imu_data.accy * 9.80665;
    imu_msg.linear_acceleration.z = imu_data.accz * 9.80665;

    if (imu_calibrate_gyro_)
    {
        static calibrate_gyro calibGyro(imu_calib_samples_);
        bool isCailb = calibGyro.calib(imu_data.angx, imu_data.angy, imu_data.angz);
        if (isCailb == false)
            return;

        imu_msg.angular_velocity.x = calibGyro.calib_x;
        imu_msg.angular_velocity.y = calibGyro.calib_y;
        imu_msg.angular_velocity.z = calibGyro.calib_z;
    }
    else
    {
        imu_msg.angular_velocity.x = imu_data.angx;
        imu_msg.angular_velocity.y = imu_data.angy;
        imu_msg.angular_velocity.z = imu_data.angz;
    }

    tf2::Quaternion goal_quat;
    goal_quat.setRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);
    imu_msg.orientation.x = goal_quat.x();
    imu_msg.orientation.y = goal_quat.y();
    imu_msg.orientation.z = goal_quat.z();
    imu_msg.orientation.w = goal_quat.w();
    imu_publisher_->publish(imu_msg);
}