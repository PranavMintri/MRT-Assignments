// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aruco_interface:srv/SendImage.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__STRUCT_H_
#define ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image_data'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in srv/SendImage in the package aruco_interface.
typedef struct aruco_interface__srv__SendImage_Request
{
  sensor_msgs__msg__Image image_data;
} aruco_interface__srv__SendImage_Request;

// Struct for a sequence of aruco_interface__srv__SendImage_Request.
typedef struct aruco_interface__srv__SendImage_Request__Sequence
{
  aruco_interface__srv__SendImage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aruco_interface__srv__SendImage_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'ids'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'bboxs'
#include "aruco_interface/msg/detail/bounding_box__struct.h"

/// Struct defined in srv/SendImage in the package aruco_interface.
typedef struct aruco_interface__srv__SendImage_Response
{
  rosidl_runtime_c__int32__Sequence ids;
  aruco_interface__msg__BoundingBox__Sequence bboxs;
} aruco_interface__srv__SendImage_Response;

// Struct for a sequence of aruco_interface__srv__SendImage_Response.
typedef struct aruco_interface__srv__SendImage_Response__Sequence
{
  aruco_interface__srv__SendImage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aruco_interface__srv__SendImage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__STRUCT_H_
