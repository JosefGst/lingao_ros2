#ifndef LINGAO_SERIAL_ASYNC_H
#define LINGAO_SERIAL_ASYNC_H

#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>

#include <iostream>
#include <queue>
#include <deque>
#include <inttypes.h>
#include <vector>
#include <mutex>

class Serial_Async : public rclcpp::Node
{
public:
  Serial_Async();
  void DoSomething();
};

#endif