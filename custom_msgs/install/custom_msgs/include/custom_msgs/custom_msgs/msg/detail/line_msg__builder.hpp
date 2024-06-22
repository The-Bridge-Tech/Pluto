// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/LineMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/line_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_LineMsg_angle
{
public:
  explicit Init_LineMsg_angle(::custom_msgs::msg::LineMsg & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::LineMsg angle(::custom_msgs::msg::LineMsg::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::LineMsg msg_;
};

class Init_LineMsg_point2
{
public:
  explicit Init_LineMsg_point2(::custom_msgs::msg::LineMsg & msg)
  : msg_(msg)
  {}
  Init_LineMsg_angle point2(::custom_msgs::msg::LineMsg::_point2_type arg)
  {
    msg_.point2 = std::move(arg);
    return Init_LineMsg_angle(msg_);
  }

private:
  ::custom_msgs::msg::LineMsg msg_;
};

class Init_LineMsg_point1
{
public:
  Init_LineMsg_point1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LineMsg_point2 point1(::custom_msgs::msg::LineMsg::_point1_type arg)
  {
    msg_.point1 = std::move(arg);
    return Init_LineMsg_point2(msg_);
  }

private:
  ::custom_msgs::msg::LineMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::LineMsg>()
{
  return custom_msgs::msg::builder::Init_LineMsg_point1();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__BUILDER_HPP_
