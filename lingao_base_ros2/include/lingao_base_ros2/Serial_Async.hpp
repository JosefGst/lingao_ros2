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
  ~Serial_Async();
  void DoSomething();

private:
  boost::shared_ptr<boost::asio::serial_port> port_;

#if BOOST_VERSION >= 107000
  boost::shared_ptr<boost::asio::io_context> io_sev_;
#else
  boost::shared_ptr<boost::asio::io_service> io_sev_;
#endif

  bool isOpen_;

  void doClose();
};

#endif