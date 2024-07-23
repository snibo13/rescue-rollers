// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rr_custom_messages:msg/FilteredTeleop.idl
// generated code does not contain a copyright notice

#ifndef RR_CUSTOM_MESSAGES__MSG__DETAIL__FILTERED_TELEOP__STRUCT_H_
#define RR_CUSTOM_MESSAGES__MSG__DETAIL__FILTERED_TELEOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/FilteredTeleop in the package rr_custom_messages.
typedef struct rr_custom_messages__msg__FilteredTeleop
{
  uint8_t id;
  float v;
  float w;
} rr_custom_messages__msg__FilteredTeleop;

// Struct for a sequence of rr_custom_messages__msg__FilteredTeleop.
typedef struct rr_custom_messages__msg__FilteredTeleop__Sequence
{
  rr_custom_messages__msg__FilteredTeleop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rr_custom_messages__msg__FilteredTeleop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RR_CUSTOM_MESSAGES__MSG__DETAIL__FILTERED_TELEOP__STRUCT_H_
