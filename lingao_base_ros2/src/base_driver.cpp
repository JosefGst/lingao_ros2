// #define BOOST_BIND_NO_PLACEHOLDERS

#include "lingao_base_ros2/base_driver.hpp"
#include "lingao_base_ros2/data_stream.hpp"
#include "lingao_base_ros2/Serial_Async.hpp"
// #include "lingao_base_ros2/TCP_Async.hpp"
// #include "lingao_base_ros2/UDP_Async.hpp"
#include "lingao_base_ros2/calibrate_gyro.hpp"
#include "lingao_base_ros2/myObject.hpp"
using std::placeholders::_1;

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

    init_odom();
    init_imu();
    init_robot_stream();

    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&BaseDriver::timer_callback, this));
}

void BaseDriver::timer_callback()
{
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);

    
    if (use_imu_ == true)
    {
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

/// 设备数据流初始化
void BaseDriver::init_robot_stream()
{
    // init Battery Management  stream
    bmsStreamActive = false;

        // if (pub_bat_.getNumSubscribers() > 0)
        // {
        //     bmsStreamActive = true;
        //     ROS_INFO_STREAM("Starting battery data stream.");
        // }
        // else 
        // {
        //     bmsStreamActive = false;
        //     ROS_INFO_STREAM("Stopping battery data stream.");
        // }
    bat_publisher_ = this->create_publisher<lingao_msgs::msg::LingAoBmsStatus>("battery_state", 1);
    RCLCPP_INFO(this->get_logger(), "advertise to the battery state topic on [ %s ]", bat_publisher_.get()->get_topic_name());


    // init remote control stream
    rcStreamActive = false;
    if(stream->rcAvailable() == true)
    {
        // ros::SubscriberStatusCallback status_cb = std::bind( [&]()
        // {
        //     if (pub_rc_.getNumSubscribers() > 0)
        //     {
        //         rcStreamActive = true;
        //         ROS_INFO_STREAM("Starting RC data stream.");
        //     }
        //     else 
        //     {
        //         rcStreamActive = false;
        //         ROS_INFO_STREAM("Stopping RC data stream.");
        //     }
        // });
        rc_publisher_ = this->create_publisher<lingao_msgs::msg::LingAoRCStatus>("rc_state", 1);
        RCLCPP_INFO(this->get_logger(), "advertise to the rc state topic on [ %s ]", rc_publisher_.get()->get_topic_name());
    }
}

// ODOM初始化
void BaseDriver::init_odom()
{
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(publish_odom_name_, 1);
    RCLCPP_INFO(this->get_logger(), "advertise to the odom topic on [ %s ]", publish_odom_name_.c_str());

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(topic_cmd_vel_name_, 1, std::bind(&BaseDriver::cmd_vel_CallBack, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribe to the cmd topic on [ %s ]", topic_cmd_vel_name_.c_str());

    // 初始化odom_trans
    odom_tf.header.frame_id         = odom_frame_id_;
    odom_tf.child_frame_id          = base_frame_id_;
    odom_tf.transform.translation.z = 0.0;

    //初始化odom 里程计消息
    odom_msg.header.frame_id      = odom_frame_id_;
    odom_msg.child_frame_id       = base_frame_id_;
    odom_msg.pose.pose.position.z = 0.0;

    setCovariance(false);

    x_pos_ = 0;
    y_pos_ = 0;
    th_ = 0;
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
        RCLCPP_INFO(this->get_logger(), "advertise to the imu topic on [ %s ]", topic_imu_.c_str());
    }
}

// IMU数据发布
void BaseDriver::publish_imu()
{
    imu_msg.header.stamp = BaseDriver::get_clock()->now();
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

/// 订阅回调速度控制命令
void BaseDriver::cmd_vel_CallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    liner_tx_.set(msg->linear.x, msg->linear.y, msg->angular.z);
    // cmd_vel_cb_timer.setPeriod(ros::Duration(cmd_vel_sub_timeout_vel_), true);
}

/// 运动协方差配置
void BaseDriver::setCovariance(bool isMove)
{
    if (isMove == true)
    {
        odom_msg.pose.covariance[0]   = 1e-3;
        odom_msg.pose.covariance[7]   = 1e-3;
        odom_msg.pose.covariance[14]  = 1e6;
        odom_msg.pose.covariance[21]  = 1e6;
        odom_msg.pose.covariance[28]  = 1e6;
        odom_msg.pose.covariance[35]  = 1e-2;
        
        odom_msg.twist.covariance[0]  = 1e-3;
        odom_msg.twist.covariance[7]  = 1e-3;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-2;
    }
    else
    {
        odom_msg.pose.covariance[0]   = 1e-9;
        odom_msg.pose.covariance[7]   = 1e-9;
        odom_msg.pose.covariance[14]  = 1e6;
        odom_msg.pose.covariance[21]  = 1e6;
        odom_msg.pose.covariance[28]  = 1e6;
        odom_msg.pose.covariance[35]  = 1e-9;

        odom_msg.twist.covariance[0]  = 1e-9;
        odom_msg.twist.covariance[7]  = 1e-9;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-9;
    }
}