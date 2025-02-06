// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/PlannerWorkspace.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/planner_workspace__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlannerWorkspace_Request_lower
{
public:
  explicit Init_PlannerWorkspace_Request_lower(::custom_interfaces::srv::PlannerWorkspace_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::PlannerWorkspace_Request lower(::custom_interfaces::srv::PlannerWorkspace_Request::_lower_type arg)
  {
    msg_.lower = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::PlannerWorkspace_Request msg_;
};

class Init_PlannerWorkspace_Request_upper
{
public:
  Init_PlannerWorkspace_Request_upper()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlannerWorkspace_Request_lower upper(::custom_interfaces::srv::PlannerWorkspace_Request::_upper_type arg)
  {
    msg_.upper = std::move(arg);
    return Init_PlannerWorkspace_Request_lower(msg_);
  }

private:
  ::custom_interfaces::srv::PlannerWorkspace_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::PlannerWorkspace_Request>()
{
  return custom_interfaces::srv::builder::Init_PlannerWorkspace_Request_upper();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlannerWorkspace_Response_success
{
public:
  Init_PlannerWorkspace_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::PlannerWorkspace_Response success(::custom_interfaces::srv::PlannerWorkspace_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::PlannerWorkspace_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::PlannerWorkspace_Response>()
{
  return custom_interfaces::srv::builder::Init_PlannerWorkspace_Response_success();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__BUILDER_HPP_
