// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aruco_interface:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aruco_interface/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace aruco_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoundingBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: corners
  {
    if (msg.corners.size() == 0) {
      out << "corners: []";
    } else {
      out << "corners: [";
      size_t pending_items = msg.corners.size();
      for (auto item : msg.corners) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: corners
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.corners.size() == 0) {
      out << "corners: []\n";
    } else {
      out << "corners:\n";
      for (auto item : msg.corners) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoundingBox & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace aruco_interface

namespace rosidl_generator_traits
{

[[deprecated("use aruco_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aruco_interface::msg::BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  aruco_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aruco_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const aruco_interface::msg::BoundingBox & msg)
{
  return aruco_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<aruco_interface::msg::BoundingBox>()
{
  return "aruco_interface::msg::BoundingBox";
}

template<>
inline const char * name<aruco_interface::msg::BoundingBox>()
{
  return "aruco_interface/msg/BoundingBox";
}

template<>
struct has_fixed_size<aruco_interface::msg::BoundingBox>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aruco_interface::msg::BoundingBox>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aruco_interface::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUCO_INTERFACE__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
