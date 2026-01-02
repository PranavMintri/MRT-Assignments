// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aruco_interface:srv/SendImage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aruco_interface/srv/detail/send_image__rosidl_typesupport_introspection_c.h"
#include "aruco_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aruco_interface/srv/detail/send_image__functions.h"
#include "aruco_interface/srv/detail/send_image__struct.h"


// Include directives for member types
// Member `image_data`
#include "sensor_msgs/msg/image.h"
// Member `image_data`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_interface__srv__SendImage_Request__init(message_memory);
}

void aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_fini_function(void * message_memory)
{
  aruco_interface__srv__SendImage_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_member_array[1] = {
  {
    "image_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface__srv__SendImage_Request, image_data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_members = {
  "aruco_interface__srv",  // message namespace
  "SendImage_Request",  // message name
  1,  // number of fields
  sizeof(aruco_interface__srv__SendImage_Request),
  aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_member_array,  // message members
  aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_type_support_handle = {
  0,
  &aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage_Request)() {
  aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_type_support_handle.typesupport_identifier) {
    aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aruco_interface__srv__SendImage_Request__rosidl_typesupport_introspection_c__SendImage_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "aruco_interface/srv/detail/send_image__rosidl_typesupport_introspection_c.h"
// already included above
// #include "aruco_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "aruco_interface/srv/detail/send_image__functions.h"
// already included above
// #include "aruco_interface/srv/detail/send_image__struct.h"


// Include directives for member types
// Member `ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `bboxs`
#include "aruco_interface/msg/bounding_box.h"
// Member `bboxs`
#include "aruco_interface/msg/detail/bounding_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_interface__srv__SendImage_Response__init(message_memory);
}

void aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_fini_function(void * message_memory)
{
  aruco_interface__srv__SendImage_Response__fini(message_memory);
}

size_t aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__size_function__SendImage_Response__ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_const_function__SendImage_Response__ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_function__SendImage_Response__ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__fetch_function__SendImage_Response__ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_const_function__SendImage_Response__ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__assign_function__SendImage_Response__ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_function__SendImage_Response__ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__resize_function__SendImage_Response__ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__size_function__SendImage_Response__bboxs(
  const void * untyped_member)
{
  const aruco_interface__msg__BoundingBox__Sequence * member =
    (const aruco_interface__msg__BoundingBox__Sequence *)(untyped_member);
  return member->size;
}

const void * aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_const_function__SendImage_Response__bboxs(
  const void * untyped_member, size_t index)
{
  const aruco_interface__msg__BoundingBox__Sequence * member =
    (const aruco_interface__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_function__SendImage_Response__bboxs(
  void * untyped_member, size_t index)
{
  aruco_interface__msg__BoundingBox__Sequence * member =
    (aruco_interface__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__fetch_function__SendImage_Response__bboxs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const aruco_interface__msg__BoundingBox * item =
    ((const aruco_interface__msg__BoundingBox *)
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_const_function__SendImage_Response__bboxs(untyped_member, index));
  aruco_interface__msg__BoundingBox * value =
    (aruco_interface__msg__BoundingBox *)(untyped_value);
  *value = *item;
}

void aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__assign_function__SendImage_Response__bboxs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  aruco_interface__msg__BoundingBox * item =
    ((aruco_interface__msg__BoundingBox *)
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_function__SendImage_Response__bboxs(untyped_member, index));
  const aruco_interface__msg__BoundingBox * value =
    (const aruco_interface__msg__BoundingBox *)(untyped_value);
  *item = *value;
}

bool aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__resize_function__SendImage_Response__bboxs(
  void * untyped_member, size_t size)
{
  aruco_interface__msg__BoundingBox__Sequence * member =
    (aruco_interface__msg__BoundingBox__Sequence *)(untyped_member);
  aruco_interface__msg__BoundingBox__Sequence__fini(member);
  return aruco_interface__msg__BoundingBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_member_array[2] = {
  {
    "ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface__srv__SendImage_Response, ids),  // bytes offset in struct
    NULL,  // default value
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__size_function__SendImage_Response__ids,  // size() function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_const_function__SendImage_Response__ids,  // get_const(index) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_function__SendImage_Response__ids,  // get(index) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__fetch_function__SendImage_Response__ids,  // fetch(index, &value) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__assign_function__SendImage_Response__ids,  // assign(index, value) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__resize_function__SendImage_Response__ids  // resize(index) function pointer
  },
  {
    "bboxs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface__srv__SendImage_Response, bboxs),  // bytes offset in struct
    NULL,  // default value
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__size_function__SendImage_Response__bboxs,  // size() function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_const_function__SendImage_Response__bboxs,  // get_const(index) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__get_function__SendImage_Response__bboxs,  // get(index) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__fetch_function__SendImage_Response__bboxs,  // fetch(index, &value) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__assign_function__SendImage_Response__bboxs,  // assign(index, value) function pointer
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__resize_function__SendImage_Response__bboxs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_members = {
  "aruco_interface__srv",  // message namespace
  "SendImage_Response",  // message name
  2,  // number of fields
  sizeof(aruco_interface__srv__SendImage_Response),
  aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_member_array,  // message members
  aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_type_support_handle = {
  0,
  &aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage_Response)() {
  aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, msg, BoundingBox)();
  if (!aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_type_support_handle.typesupport_identifier) {
    aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aruco_interface__srv__SendImage_Response__rosidl_typesupport_introspection_c__SendImage_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "aruco_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "aruco_interface/srv/detail/send_image__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_members = {
  "aruco_interface__srv",  // service namespace
  "SendImage",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_Request_message_type_support_handle,
  NULL  // response message
  // aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_Response_message_type_support_handle
};

static rosidl_service_type_support_t aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_type_support_handle = {
  0,
  &aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage)() {
  if (!aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_type_support_handle.typesupport_identifier) {
    aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, srv, SendImage_Response)()->data;
  }

  return &aruco_interface__srv__detail__send_image__rosidl_typesupport_introspection_c__SendImage_service_type_support_handle;
}
