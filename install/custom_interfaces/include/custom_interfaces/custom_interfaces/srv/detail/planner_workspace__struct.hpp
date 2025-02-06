// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/PlannerWorkspace.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlannerWorkspace_Request_
{
  using Type = PlannerWorkspace_Request_<ContainerAllocator>;

  explicit PlannerWorkspace_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->upper = 0.0f;
      this->lower = 0.0f;
    }
  }

  explicit PlannerWorkspace_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->upper = 0.0f;
      this->lower = 0.0f;
    }
  }

  // field types and members
  using _upper_type =
    float;
  _upper_type upper;
  using _lower_type =
    float;
  _lower_type lower;

  // setters for named parameter idiom
  Type & set__upper(
    const float & _arg)
  {
    this->upper = _arg;
    return *this;
  }
  Type & set__lower(
    const float & _arg)
  {
    this->lower = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Request
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Request
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlannerWorkspace_Request_ & other) const
  {
    if (this->upper != other.upper) {
      return false;
    }
    if (this->lower != other.lower) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlannerWorkspace_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlannerWorkspace_Request_

// alias to use template instance with default allocator
using PlannerWorkspace_Request =
  custom_interfaces::srv::PlannerWorkspace_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlannerWorkspace_Response_
{
  using Type = PlannerWorkspace_Response_<ContainerAllocator>;

  explicit PlannerWorkspace_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit PlannerWorkspace_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Response
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__PlannerWorkspace_Response
    std::shared_ptr<custom_interfaces::srv::PlannerWorkspace_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlannerWorkspace_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlannerWorkspace_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlannerWorkspace_Response_

// alias to use template instance with default allocator
using PlannerWorkspace_Response =
  custom_interfaces::srv::PlannerWorkspace_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct PlannerWorkspace
{
  using Request = custom_interfaces::srv::PlannerWorkspace_Request;
  using Response = custom_interfaces::srv::PlannerWorkspace_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__STRUCT_HPP_
