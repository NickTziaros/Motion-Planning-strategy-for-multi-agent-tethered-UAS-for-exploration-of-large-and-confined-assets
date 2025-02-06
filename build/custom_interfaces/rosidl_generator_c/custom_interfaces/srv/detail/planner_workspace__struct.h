// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/PlannerWorkspace.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PlannerWorkspace in the package custom_interfaces.
typedef struct custom_interfaces__srv__PlannerWorkspace_Request
{
  /// ompl planner workspace dimentions
  float upper;
  float lower;
} custom_interfaces__srv__PlannerWorkspace_Request;

// Struct for a sequence of custom_interfaces__srv__PlannerWorkspace_Request.
typedef struct custom_interfaces__srv__PlannerWorkspace_Request__Sequence
{
  custom_interfaces__srv__PlannerWorkspace_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__PlannerWorkspace_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/PlannerWorkspace in the package custom_interfaces.
typedef struct custom_interfaces__srv__PlannerWorkspace_Response
{
  bool success;
} custom_interfaces__srv__PlannerWorkspace_Response;

// Struct for a sequence of custom_interfaces__srv__PlannerWorkspace_Response.
typedef struct custom_interfaces__srv__PlannerWorkspace_Response__Sequence
{
  custom_interfaces__srv__PlannerWorkspace_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__PlannerWorkspace_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__PLANNER_WORKSPACE__STRUCT_H_
