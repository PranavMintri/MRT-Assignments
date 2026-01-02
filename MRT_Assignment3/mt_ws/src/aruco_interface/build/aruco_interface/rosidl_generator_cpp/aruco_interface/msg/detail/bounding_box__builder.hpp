// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aruco_interface:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aruco_interface/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aruco_interface
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_corners
{
public:
  Init_BoundingBox_corners()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aruco_interface::msg::BoundingBox corners(::aruco_interface::msg::BoundingBox::_corners_type arg)
  {
    msg_.corners = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aruco_interface::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aruco_interface::msg::BoundingBox>()
{
  return aruco_interface::msg::builder::Init_BoundingBox_corners();
}

}  // namespace aruco_interface

#endif  // ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
