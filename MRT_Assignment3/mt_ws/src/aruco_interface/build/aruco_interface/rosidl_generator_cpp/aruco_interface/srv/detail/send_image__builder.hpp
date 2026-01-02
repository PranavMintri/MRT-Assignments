// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aruco_interface:srv/SendImage.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__BUILDER_HPP_
#define ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aruco_interface/srv/detail/send_image__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aruco_interface
{

namespace srv
{

namespace builder
{

class Init_SendImage_Request_image_data
{
public:
  Init_SendImage_Request_image_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aruco_interface::srv::SendImage_Request image_data(::aruco_interface::srv::SendImage_Request::_image_data_type arg)
  {
    msg_.image_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aruco_interface::srv::SendImage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aruco_interface::srv::SendImage_Request>()
{
  return aruco_interface::srv::builder::Init_SendImage_Request_image_data();
}

}  // namespace aruco_interface


namespace aruco_interface
{

namespace srv
{

namespace builder
{

class Init_SendImage_Response_bboxs
{
public:
  explicit Init_SendImage_Response_bboxs(::aruco_interface::srv::SendImage_Response & msg)
  : msg_(msg)
  {}
  ::aruco_interface::srv::SendImage_Response bboxs(::aruco_interface::srv::SendImage_Response::_bboxs_type arg)
  {
    msg_.bboxs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aruco_interface::srv::SendImage_Response msg_;
};

class Init_SendImage_Response_ids
{
public:
  Init_SendImage_Response_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendImage_Response_bboxs ids(::aruco_interface::srv::SendImage_Response::_ids_type arg)
  {
    msg_.ids = std::move(arg);
    return Init_SendImage_Response_bboxs(msg_);
  }

private:
  ::aruco_interface::srv::SendImage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aruco_interface::srv::SendImage_Response>()
{
  return aruco_interface::srv::builder::Init_SendImage_Response_ids();
}

}  // namespace aruco_interface

#endif  // ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__BUILDER_HPP_
