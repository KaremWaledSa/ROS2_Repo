// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/Age.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__AGE__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__AGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Age in the package custom_msg.
typedef struct custom_msg__msg__Age
{
  int16_t year;
  int16_t month;
  int16_t day;
} custom_msg__msg__Age;

// Struct for a sequence of custom_msg__msg__Age.
typedef struct custom_msg__msg__Age__Sequence
{
  custom_msg__msg__Age * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__Age__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__AGE__STRUCT_H_
