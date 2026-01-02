// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aruco_interface:srv/SendImage.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__TRAITS_HPP_
#define ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aruco_interface/srv/detail/send_image__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'image_data'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace aruco_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SendImage_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: image_data
  {
    out << "image_data: ";
    to_flow_style_yaml(msg.image_data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendImage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: image_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image_data:\n";
    to_block_style_yaml(msg.image_data, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendImage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace aruco_interface

namespace rosidl_generator_traits
{

[[deprecated("use aruco_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aruco_interface::srv::SendImage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  aruco_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aruco_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const aruco_interface::srv::SendImage_Request & msg)
{
  return aruco_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<aruco_interface::srv::SendImage_Request>()
{
  return "aruco_interface::srv::SendImage_Request";
}

template<>
inline const char * name<aruco_interface::srv::SendImage_Request>()
{
  return "aruco_interface/srv/SendImage_Request";
}

template<>
struct has_fixed_size<aruco_interface::srv::SendImage_Request>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<aruco_interface::srv::SendImage_Request>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<aruco_interface::srv::SendImage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'bboxs'
#include "aruco_interface/msg/detail/bounding_box__traits.hpp"

namespace aruco_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SendImage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ids
  {
    if (msg.ids.size() == 0) {
      out << "ids: []";
    } else {
      out << "ids: [";
      size_t pending_items = msg.ids.size();
      for (auto item : msg.ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bboxs
  {
    if (msg.bboxs.size() == 0) {
      out << "bboxs: []";
    } else {
      out << "bboxs: [";
      size_t pending_items = msg.bboxs.size();
      for (auto item : msg.bboxs) {
        to_flow_style_yaml(item, out);
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
  const SendImage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ids.size() == 0) {
      out << "ids: []\n";
    } else {
      out << "ids:\n";
      for (auto item : msg.ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: bboxs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bboxs.size() == 0) {
      out << "bboxs: []\n";
    } else {
      out << "bboxs:\n";
      for (auto item : msg.bboxs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendImage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace aruco_interface

namespace rosidl_generator_traits
{

[[deprecated("use aruco_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aruco_interface::srv::SendImage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  aruco_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aruco_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const aruco_interface::srv::SendImage_Response & msg)
{
  return aruco_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<aruco_interface::srv::SendImage_Response>()
{
  return "aruco_interface::srv::SendImage_Response";
}

template<>
inline const char * name<aruco_interface::srv::SendImage_Response>()
{
  return "aruco_interface/srv/SendImage_Response";
}

template<>
struct has_fixed_size<aruco_interface::srv::SendImage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aruco_interface::srv::SendImage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aruco_interface::srv::SendImage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aruco_interface::srv::SendImage>()
{
  return "aruco_interface::srv::SendImage";
}

template<>
inline const char * name<aruco_interface::srv::SendImage>()
{
  return "aruco_interface/srv/SendImage";
}

template<>
struct has_fixed_size<aruco_interface::srv::SendImage>
  : std::integral_constant<
    bool,
    has_fixed_size<aruco_interface::srv::SendImage_Request>::value &&
    has_fixed_size<aruco_interface::srv::SendImage_Response>::value
  >
{
};

template<>
struct has_bounded_size<aruco_interface::srv::SendImage>
  : std::integral_constant<
    bool,
    has_bounded_size<aruco_interface::srv::SendImage_Request>::value &&
    has_bounded_size<aruco_interface::srv::SendImage_Response>::value
  >
{
};

template<>
struct is_service<aruco_interface::srv::SendImage>
  : std::true_type
{
};

template<>
struct is_service_request<aruco_interface::srv::SendImage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aruco_interface::srv::SendImage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__TRAITS_HPP_
