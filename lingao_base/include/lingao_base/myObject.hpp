#ifndef MY_OBJECT_H
#define MY_OBJECT_H

#include "rclcpp/rclcpp.hpp"

class MyClass : public rclcpp::Node
{
public:
    MyClass();
    void DoSomething();
};

#endif