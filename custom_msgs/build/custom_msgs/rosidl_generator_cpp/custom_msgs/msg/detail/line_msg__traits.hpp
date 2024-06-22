// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/LineMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/line_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'point1'
// Member 'point2'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LineMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: point1
  {
    out << "point1: ";
    to_flow_style_yaml(msg.point1, out);
    out << ", ";
  }

  // member: point2
  {
    out << "point2: ";
    to_flow_style_yaml(msg.point2, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LineMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: point1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point1:\n";
    to_block_style_yaml(msg.point1, out, indentation + 2);
  }

  // member: point2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point2:\n";
    to_block_style_yaml(msg.point2, out, indentation + 2);
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LineMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::LineMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::LineMsg & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::LineMsg>()
{
  return "custom_msgs::msg::LineMsg";
}

template<>
inline const char * name<custom_msgs::msg::LineMsg>()
{
  return "custom_msgs/msg/LineMsg";
}

template<>
struct has_fixed_size<custom_msgs::msg::LineMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::LineMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<custom_msgs::msg::LineMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__TRAITS_HPP_
