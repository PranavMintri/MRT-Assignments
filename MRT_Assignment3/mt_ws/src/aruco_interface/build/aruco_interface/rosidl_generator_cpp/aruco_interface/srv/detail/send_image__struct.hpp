// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aruco_interface:srv/SendImage.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__STRUCT_HPP_
#define ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'image_data'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__aruco_interface__srv__SendImage_Request __attribute__((deprecated))
#else
# define DEPRECATED__aruco_interface__srv__SendImage_Request __declspec(deprecated)
#endif

namespace aruco_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendImage_Request_
{
  using Type = SendImage_Request_<ContainerAllocator>;

  explicit SendImage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image_data(_init)
  {
    (void)_init;
  }

  explicit SendImage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image_data(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _image_data_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_data_type image_data;

  // setters for named parameter idiom
  Type & set__image_data(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aruco_interface::srv::SendImage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const aruco_interface::srv::SendImage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aruco_interface::srv::SendImage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aruco_interface::srv::SendImage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aruco_interface__srv__SendImage_Request
    std::shared_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aruco_interface__srv__SendImage_Request
    std::shared_ptr<aruco_interface::srv::SendImage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendImage_Request_ & other) const
  {
    if (this->image_data != other.image_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendImage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendImage_Request_

// alias to use template instance with default allocator
using SendImage_Request =
  aruco_interface::srv::SendImage_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace aruco_interface


// Include directives for member types
// Member 'bboxs'
#include "aruco_interface/msg/detail/bounding_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__aruco_interface__srv__SendImage_Response __attribute__((deprecated))
#else
# define DEPRECATED__aruco_interface__srv__SendImage_Response __declspec(deprecated)
#endif

namespace aruco_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendImage_Response_
{
  using Type = SendImage_Response_<ContainerAllocator>;

  explicit SendImage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SendImage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _ids_type ids;
  using _bboxs_type =
    std::vector<aruco_interface::msg::BoundingBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<aruco_interface::msg::BoundingBox_<ContainerAllocator>>>;
  _bboxs_type bboxs;

  // setters for named parameter idiom
  Type & set__ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->ids = _arg;
    return *this;
  }
  Type & set__bboxs(
    const std::vector<aruco_interface::msg::BoundingBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<aruco_interface::msg::BoundingBox_<ContainerAllocator>>> & _arg)
  {
    this->bboxs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aruco_interface::srv::SendImage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const aruco_interface::srv::SendImage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aruco_interface::srv::SendImage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aruco_interface::srv::SendImage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aruco_interface__srv__SendImage_Response
    std::shared_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aruco_interface__srv__SendImage_Response
    std::shared_ptr<aruco_interface::srv::SendImage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendImage_Response_ & other) const
  {
    if (this->ids != other.ids) {
      return false;
    }
    if (this->bboxs != other.bboxs) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendImage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendImage_Response_

// alias to use template instance with default allocator
using SendImage_Response =
  aruco_interface::srv::SendImage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace aruco_interface

namespace aruco_interface
{

namespace srv
{

struct SendImage
{
  using Request = aruco_interface::srv::SendImage_Request;
  using Response = aruco_interface::srv::SendImage_Response;
};

}  // namespace srv

}  // namespace aruco_interface

#endif  // ARUCO_INTERFACE__SRV__DETAIL__SEND_IMAGE__STRUCT_HPP_
