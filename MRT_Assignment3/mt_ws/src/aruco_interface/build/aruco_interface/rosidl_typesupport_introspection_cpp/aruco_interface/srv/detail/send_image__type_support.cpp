// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from aruco_interface:srv/SendImage.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "aruco_interface/srv/detail/send_image__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace aruco_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SendImage_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) aruco_interface::srv::SendImage_Request(_init);
}

void SendImage_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<aruco_interface::srv::SendImage_Request *>(message_memory);
  typed_message->~SendImage_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SendImage_Request_message_member_array[1] = {
  {
    "image_data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface::srv::SendImage_Request, image_data),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SendImage_Request_message_members = {
  "aruco_interface::srv",  // message namespace
  "SendImage_Request",  // message name
  1,  // number of fields
  sizeof(aruco_interface::srv::SendImage_Request),
  SendImage_Request_message_member_array,  // message members
  SendImage_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SendImage_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SendImage_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SendImage_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace aruco_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<aruco_interface::srv::SendImage_Request>()
{
  return &::aruco_interface::srv::rosidl_typesupport_introspection_cpp::SendImage_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, aruco_interface, srv, SendImage_Request)() {
  return &::aruco_interface::srv::rosidl_typesupport_introspection_cpp::SendImage_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "aruco_interface/srv/detail/send_image__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace aruco_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SendImage_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) aruco_interface::srv::SendImage_Response(_init);
}

void SendImage_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<aruco_interface::srv::SendImage_Response *>(message_memory);
  typed_message->~SendImage_Response();
}

size_t size_function__SendImage_Response__ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SendImage_Response__ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SendImage_Response__ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SendImage_Response__ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__SendImage_Response__ids(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__SendImage_Response__ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__SendImage_Response__ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__SendImage_Response__ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SendImage_Response__bboxs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<aruco_interface::msg::BoundingBox> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SendImage_Response__bboxs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<aruco_interface::msg::BoundingBox> *>(untyped_member);
  return &member[index];
}

void * get_function__SendImage_Response__bboxs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<aruco_interface::msg::BoundingBox> *>(untyped_member);
  return &member[index];
}

void fetch_function__SendImage_Response__bboxs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const aruco_interface::msg::BoundingBox *>(
    get_const_function__SendImage_Response__bboxs(untyped_member, index));
  auto & value = *reinterpret_cast<aruco_interface::msg::BoundingBox *>(untyped_value);
  value = item;
}

void assign_function__SendImage_Response__bboxs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<aruco_interface::msg::BoundingBox *>(
    get_function__SendImage_Response__bboxs(untyped_member, index));
  const auto & value = *reinterpret_cast<const aruco_interface::msg::BoundingBox *>(untyped_value);
  item = value;
}

void resize_function__SendImage_Response__bboxs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<aruco_interface::msg::BoundingBox> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SendImage_Response_message_member_array[2] = {
  {
    "ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface::srv::SendImage_Response, ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__SendImage_Response__ids,  // size() function pointer
    get_const_function__SendImage_Response__ids,  // get_const(index) function pointer
    get_function__SendImage_Response__ids,  // get(index) function pointer
    fetch_function__SendImage_Response__ids,  // fetch(index, &value) function pointer
    assign_function__SendImage_Response__ids,  // assign(index, value) function pointer
    resize_function__SendImage_Response__ids  // resize(index) function pointer
  },
  {
    "bboxs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<aruco_interface::msg::BoundingBox>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface::srv::SendImage_Response, bboxs),  // bytes offset in struct
    nullptr,  // default value
    size_function__SendImage_Response__bboxs,  // size() function pointer
    get_const_function__SendImage_Response__bboxs,  // get_const(index) function pointer
    get_function__SendImage_Response__bboxs,  // get(index) function pointer
    fetch_function__SendImage_Response__bboxs,  // fetch(index, &value) function pointer
    assign_function__SendImage_Response__bboxs,  // assign(index, value) function pointer
    resize_function__SendImage_Response__bboxs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SendImage_Response_message_members = {
  "aruco_interface::srv",  // message namespace
  "SendImage_Response",  // message name
  2,  // number of fields
  sizeof(aruco_interface::srv::SendImage_Response),
  SendImage_Response_message_member_array,  // message members
  SendImage_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SendImage_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SendImage_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SendImage_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace aruco_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<aruco_interface::srv::SendImage_Response>()
{
  return &::aruco_interface::srv::rosidl_typesupport_introspection_cpp::SendImage_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, aruco_interface, srv, SendImage_Response)() {
  return &::aruco_interface::srv::rosidl_typesupport_introspection_cpp::SendImage_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "aruco_interface/srv/detail/send_image__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace aruco_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SendImage_service_members = {
  "aruco_interface::srv",  // service namespace
  "SendImage",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<aruco_interface::srv::SendImage>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SendImage_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SendImage_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace aruco_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<aruco_interface::srv::SendImage>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::aruco_interface::srv::rosidl_typesupport_introspection_cpp::SendImage_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::aruco_interface::srv::SendImage_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::aruco_interface::srv::SendImage_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, aruco_interface, srv, SendImage)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<aruco_interface::srv::SendImage>();
}

#ifdef __cplusplus
}
#endif
