// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aruco_interface:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'corners'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BoundingBox in the package aruco_interface.
typedef struct aruco_interface__msg__BoundingBox
{
  rosidl_runtime_c__float__Sequence corners;
} aruco_interface__msg__BoundingBox;

// Struct for a sequence of aruco_interface__msg__BoundingBox.
typedef struct aruco_interface__msg__BoundingBox__Sequence
{
  aruco_interface__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aruco_interface__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
