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

    liner_tx_.set(.0, .0, .0);

    main_timer_cb_ = this->create_wall_timer(std::chrono::milliseconds(1000 / loop_rate_), std::bind(&BaseDriver::MainTimerCallback, this));
    timer_10hz_cb_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BaseDriver::Timer10HzCallbackCallback, this));
    timer_1hz_cb_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BaseDriver::Timer1HzCallbackCallback, this));
}

void BaseDriver::MainTimerCallback()
{
    bool isRead = false;

    if (serial->isOpen() == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial closes unexpectedly!");
        return;
    }

    if (timer1HzTimeOut)
    {

        isRead = stream->get_Message(MSG_ID_GET_VOLTAGE);
        if (isRead)
        {
            // 成功读取后数据处理
            rxData_battery = stream->get_data_battery();
            bat_msg.header.stamp = BaseDriver::get_clock()->now();
            bat_msg.voltage = rxData_battery.voltage / 100.0;
            bat_msg.current = rxData_battery.current / 100.0;
            bat_msg.percentage = rxData_battery.percentage;
            bat_msg.temperature = rxData_battery.temperature / 10.0;

            bat_publisher_->publish(bat_msg);
        }
        else
            RCLCPP_WARN(this->get_logger(), "Get VOLTAGE Data Time Out!");

        timer1HzTimeOut = false;
    }

    if (timer10HzTimeOut)
    {
        // RC遥控数据流
        if (rcStreamActive)
        {
            isRead = stream->get_Message(MSG_ID_GET_RC);
            if (isRead)
            {
                rxData_rc = stream->get_data_rc();
                

                rc_msg.header.stamp = BaseDriver::get_clock()->now();
                rc_msg.connect = rxData_rc.connect;
                rc_msg.ch1 = rxData_rc.ch1;
                rc_msg.ch2 = rxData_rc.ch2;
                rc_msg.ch3 = rxData_rc.ch3;
                rc_msg.ch4 = rxData_rc.ch4;
                rc_msg.ch5 = rxData_rc.ch5;
                rc_msg.ch6 = rxData_rc.ch6;
                rc_msg.ch7 = rxData_rc.ch7;
                rc_msg.ch8 = rxData_rc.ch8;
                rc_msg.ch9 = rxData_rc.ch9;
                rc_msg.ch10 = rxData_rc.ch10;

                rc_publisher_->publish(rc_msg);
            }
            else
                RCLCPP_WARN(this->get_logger(), "Get Remote Control Data Time Out!");
        }

        timer10HzTimeOut = false;
    }

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

    // 速度反馈数据流
    isRead = stream->get_Message(MSG_ID_GET_VELOCITY);
    if (isRead)
    {
        liner_rx_ = stream->get_data_liner();
        if (liner_rx_.v_liner_x == 0 && liner_rx_.v_angular_z == 0)
        {
            setCovariance(false);
        }
        else
            setCovariance(true);

        calc_odom();
        publish_odom();
    }
    else
        RCLCPP_WARN(this->get_logger(), "Get VELOCITY Data Time Out!");

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
    bat_publisher_ = this->create_publisher<lingao_msgs::msg::LingAoBmsStatus>("battery_state", 1);
    RCLCPP_INFO(this->get_logger(), "advertise to the battery state topic on [ %s ]", bat_publisher_.get()->get_topic_name());

    // init remote control stream
    rcStreamActive = false;
    if (stream->rcAvailable() == true)
    {
        rcStreamActive = true;
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
    odom_tf.header.frame_id = odom_frame_id_;
    odom_tf.child_frame_id = base_frame_id_;
    odom_tf.transform.translation.z = 0.0;

    // 初始化odom 里程计消息
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
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

// 函数功能：上传ODOM信息
void BaseDriver::publish_odom()
{
    current_time = BaseDriver::get_clock()->now();

    tf2::Quaternion odom_quat;

    //计算机器人在四元数角下的航向，ROS具有计算四元数角偏航的功能
    odom_quat.setRPY(0, 0, th_);

    // 发布TF
    if (publish_odom_transform_)
    {
        // robot's position in x,y, and z
        odom_tf.transform.translation.x = x_pos_;
        odom_tf.transform.translation.y = y_pos_;
        odom_tf.transform.translation.z = 0.0;

        // robot's heading in quaternion
        odom_tf.transform.rotation.x = odom_quat.x();
        odom_tf.transform.rotation.y = odom_quat.y();
        odom_tf.transform.rotation.z = odom_quat.z();
        odom_tf.transform.rotation.w = odom_quat.w();

        odom_tf.header.stamp = current_time;
        //使用odom_trans对象发布机器人的tf
        odom_broadcaster_->sendTransform(odom_tf);
    }

    //发布里程计消息
    odom_msg.header.stamp         = current_time;
    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    odom_msg.pose.pose.position.z = 0.0;

    //四元数机器人的航向
    odom_msg.pose.pose.orientation.x = odom_quat.x();
    odom_msg.pose.pose.orientation.y = odom_quat.y();
    odom_msg.pose.pose.orientation.z = odom_quat.z();
    odom_msg.pose.pose.orientation.w = odom_quat.w();

    //编码器的线速度
    odom_msg.twist.twist.linear.x = liner_rx_.v_liner_x * linear_scale_;
    odom_msg.twist.twist.linear.y = liner_rx_.v_liner_y * linear_scale_;
    odom_msg.twist.twist.linear.z = 0.0;

    //编码器的角速度
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = liner_rx_.v_angular_z * angular_scale_;

    odom_publisher_->publish(odom_msg);
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
        odom_msg.pose.covariance[0] = 1e-3;
        odom_msg.pose.covariance[7] = 1e-3;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 1e-2;

        odom_msg.twist.covariance[0] = 1e-3;
        odom_msg.twist.covariance[7] = 1e-3;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-2;
    }
    else
    {
        odom_msg.pose.covariance[0] = 1e-9;
        odom_msg.pose.covariance[7] = 1e-9;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 1e-9;

        odom_msg.twist.covariance[0] = 1e-9;
        odom_msg.twist.covariance[7] = 1e-9;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-9;
    }
}

/// 函数功能：根据机器人线速度和角度计算机器人里程计
void BaseDriver::calc_odom()
{
    rclcpp::Time current_time = BaseDriver::get_clock()->now();

    float linear_velocity_x_  = liner_rx_.v_liner_x * linear_scale_;
    float linear_velocity_y_  = liner_rx_.v_liner_y * linear_scale_;
    float angular_velocity_z_ = liner_rx_.v_angular_z * angular_scale_;

    double vel_dt_      = (current_time - last_odom_vel_time_).seconds();
    last_odom_vel_time_ = current_time;

    double delta_x  = (linear_velocity_x_ * cos(th_) - linear_velocity_y_ * sin(th_)) * vel_dt_; // m
    double delta_y  = (linear_velocity_x_ * sin(th_) + linear_velocity_y_ * cos(th_)) * vel_dt_; // m
    double delta_th = angular_velocity_z_ * vel_dt_;                                             // radians

    //计算机器人的当前位置
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    th_ += delta_th; //实时角度信息,如果这里不使用IMU，也可以通过这种方式计算得出
}


void BaseDriver::Timer10HzCallbackCallback()
{
    timer10HzTimeOut = true;
}

void BaseDriver::Timer1HzCallbackCallback()
{
    timer1HzTimeOut = true;
}