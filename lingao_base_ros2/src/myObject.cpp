#include "lingao_base_ros2/myObject.hpp"

MyClass::MyClass() : Node("myObject")
{
    std::string some_string = "hiya";
}

void MyClass::DoSomething()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Do something ");
}
