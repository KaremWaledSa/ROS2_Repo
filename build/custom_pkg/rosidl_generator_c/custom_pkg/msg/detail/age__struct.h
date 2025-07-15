// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_pkg:msg/Age.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_PKG__MSG__DETAIL__AGE__STRUCT_H_
#define CUSTOM_PKG__MSG__DETAIL__AGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Age in the package custom_pkg.
typedef struct custom_pkg__msg__Age
{
  float year;
  float month;
  float day;
} custom_pkg__msg__Age;

// Struct for a sequence of custom_pkg__msg__Age.
typedef struct custom_pkg__msg__Age__Sequence
{
  custom_pkg__msg__Age * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_pkg__msg__Age__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_PKG__MSG__DETAIL__AGE__STRUCT_H_
