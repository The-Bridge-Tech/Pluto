// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/LineMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'point1'
// Member 'point2'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__LineMsg __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__LineMsg __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LineMsg_
{
  using Type = LineMsg_<ContainerAllocator>;

  explicit LineMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point1(_init),
    point2(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0;
    }
  }

  explicit LineMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point1(_alloc, _init),
    point2(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0;
    }
  }

  // field types and members
  using _point1_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _point1_type point1;
  using _point2_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _point2_type point2;
  using _angle_type =
    double;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__point1(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->point1 = _arg;
    return *this;
  }
  Type & set__point2(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->point2 = _arg;
    return *this;
  }
  Type & set__angle(
    const double & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::LineMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::LineMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::LineMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::LineMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__LineMsg
    std::shared_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__LineMsg
    std::shared_ptr<custom_msgs::msg::LineMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LineMsg_ & other) const
  {
    if (this->point1 != other.point1) {
      return false;
    }
    if (this->point2 != other.point2) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const LineMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LineMsg_

// alias to use template instance with default allocator
using LineMsg =
  custom_msgs::msg::LineMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__STRUCT_HPP_
