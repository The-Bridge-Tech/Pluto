// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/LineMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'point1'
// Member 'point2'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/LineMsg in the package custom_msgs.
typedef struct custom_msgs__msg__LineMsg
{
  geometry_msgs__msg__Point point1;
  geometry_msgs__msg__Point point2;
  double angle;
} custom_msgs__msg__LineMsg;

// Struct for a sequence of custom_msgs__msg__LineMsg.
typedef struct custom_msgs__msg__LineMsg__Sequence
{
  custom_msgs__msg__LineMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__LineMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__LINE_MSG__STRUCT_H_
