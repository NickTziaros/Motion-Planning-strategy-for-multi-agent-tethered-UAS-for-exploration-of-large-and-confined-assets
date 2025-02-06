// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace pick_ik {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::string mode = "global";
        double gd_step_size = 0.0001;
        int64_t gd_max_iters = 100;
        double gd_min_cost_delta = 1e-12;
        double position_threshold = 0.001;
        double orientation_threshold = 0.001;
        double approximate_solution_position_threshold = 0.05;
        double approximate_solution_orientation_threshold = 0.05;
        double approximate_solution_joint_threshold = 0.0;
        double approximate_solution_cost_threshold = 0.0;
        double cost_threshold = 0.001;
        double position_scale = 1.0;
        double rotation_scale = 0.5;
        double center_joints_weight = 0.0;
        double avoid_joint_limits_weight = 0.0;
        double minimal_displacement_weight = 0.0;
        bool stop_optimization_on_valid_solution = true;
        int64_t memetic_num_threads = 5;
        bool memetic_stop_on_first_solution = true;
        int64_t memetic_population_size = 16;
        int64_t memetic_elite_size = 4;
        double memetic_wipeout_fitness_tol = 1e-05;
        int64_t memetic_max_generations = 100;
        int64_t memetic_gd_max_iters = 25;
        double memetic_gd_max_time = 0.005;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double gd_step_size = 0.0001;
        int64_t gd_max_iters = 100;
        double gd_min_cost_delta = 1e-12;
        double position_threshold = 0.001;
        double orientation_threshold = 0.001;
        double approximate_solution_position_threshold = 0.05;
        double approximate_solution_orientation_threshold = 0.05;
        double approximate_solution_joint_threshold = 0.0;
        double approximate_solution_cost_threshold = 0.0;
        double cost_threshold = 0.001;
        double position_scale = 1.0;
        double rotation_scale = 0.5;
        double center_joints_weight = 0.0;
        double avoid_joint_limits_weight = 0.0;
        double minimal_displacement_weight = 0.0;
        bool stop_optimization_on_valid_solution = true;
        int64_t memetic_num_threads = 5;
        bool memetic_stop_on_first_solution = true;
        int64_t memetic_population_size = 16;
        int64_t memetic_elite_size = 4;
        double memetic_wipeout_fitness_tol = 1e-05;
        int64_t memetic_max_generations = 100;
        int64_t memetic_gd_max_iters = 25;
        double memetic_gd_max_time = 0.005;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("pick_ik"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "") {
      logger_ = logger;
      prefix_ = prefix;
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.gd_step_size = params.gd_step_size;
      output.gd_max_iters = params.gd_max_iters;
      output.gd_min_cost_delta = params.gd_min_cost_delta;
      output.position_threshold = params.position_threshold;
      output.orientation_threshold = params.orientation_threshold;
      output.approximate_solution_position_threshold = params.approximate_solution_position_threshold;
      output.approximate_solution_orientation_threshold = params.approximate_solution_orientation_threshold;
      output.approximate_solution_joint_threshold = params.approximate_solution_joint_threshold;
      output.approximate_solution_cost_threshold = params.approximate_solution_cost_threshold;
      output.cost_threshold = params.cost_threshold;
      output.position_scale = params.position_scale;
      output.rotation_scale = params.rotation_scale;
      output.center_joints_weight = params.center_joints_weight;
      output.avoid_joint_limits_weight = params.avoid_joint_limits_weight;
      output.minimal_displacement_weight = params.minimal_displacement_weight;
      output.stop_optimization_on_valid_solution = params.stop_optimization_on_valid_solution;
      output.memetic_num_threads = params.memetic_num_threads;
      output.memetic_stop_on_first_solution = params.memetic_stop_on_first_solution;
      output.memetic_population_size = params.memetic_population_size;
      output.memetic_elite_size = params.memetic_elite_size;
      output.memetic_wipeout_fitness_tol = params.memetic_wipeout_fitness_tol;
      output.memetic_max_generations = params.memetic_max_generations;
      output.memetic_gd_max_iters = params.memetic_gd_max_iters;
      output.memetic_gd_max_time = params.memetic_gd_max_time;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "mode")) {
            if(auto validation_result = one_of<std::string>(param, {"global", "local"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.mode = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "gd_step_size")) {
            if(auto validation_result = gt_eq<double>(param, 1e-12);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.gd_step_size = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "gd_max_iters")) {
            if(auto validation_result = gt_eq<int64_t>(param, 1);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.gd_max_iters = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "gd_min_cost_delta")) {
            if(auto validation_result = gt_eq<double>(param, 1e-64);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.gd_min_cost_delta = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "position_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.position_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "orientation_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.orientation_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "approximate_solution_position_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.approximate_solution_position_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "approximate_solution_orientation_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.approximate_solution_orientation_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "approximate_solution_joint_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.approximate_solution_joint_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "approximate_solution_cost_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.approximate_solution_cost_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cost_threshold")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.cost_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "position_scale")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.position_scale = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "rotation_scale")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.rotation_scale = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "center_joints_weight")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.center_joints_weight = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "avoid_joint_limits_weight")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.avoid_joint_limits_weight = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "minimal_displacement_weight")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.minimal_displacement_weight = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "stop_optimization_on_valid_solution")) {
            updated_params.stop_optimization_on_valid_solution = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_num_threads")) {
            if(auto validation_result = gt_eq<int64_t>(param, 1);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_num_threads = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_stop_on_first_solution")) {
            updated_params.memetic_stop_on_first_solution = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_population_size")) {
            if(auto validation_result = gt_eq<int64_t>(param, 1);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_population_size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_elite_size")) {
            if(auto validation_result = gt_eq<int64_t>(param, 1);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_elite_size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_wipeout_fitness_tol")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_wipeout_fitness_tol = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_max_generations")) {
            if(auto validation_result = gt_eq<int64_t>(param, 1);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_max_generations = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_gd_max_iters")) {
            if(auto validation_result = gt_eq<int64_t>(param, 1);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_gd_max_iters = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "memetic_gd_max_time")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.memetic_gd_max_time = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "mode")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "IK solver mode. Set to global to allow the initial guess to be a long distance from the goal, or local if the initial guess is near the goal.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.mode);
          parameters_interface_->declare_parameter(prefix_ + "mode", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "gd_step_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Gradient descent step size for joint perturbation";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 1e-12;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.gd_step_size);
          parameters_interface_->declare_parameter(prefix_ + "gd_step_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "gd_max_iters")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum iterations for local gradient descent";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 1;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int>::max();
          auto parameter = to_parameter_value(updated_params.gd_max_iters);
          parameters_interface_->declare_parameter(prefix_ + "gd_max_iters", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "gd_min_cost_delta")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum change in cost function value for gradient descent";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 1e-64;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.gd_min_cost_delta);
          parameters_interface_->declare_parameter(prefix_ + "gd_min_cost_delta", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "position_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Position threshold for solving IK, in meters";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.position_threshold);
          parameters_interface_->declare_parameter(prefix_ + "position_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "orientation_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Orientation threshold for solving IK, in radians";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.orientation_threshold);
          parameters_interface_->declare_parameter(prefix_ + "orientation_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "approximate_solution_position_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Position threshold for approximate IK solutions, in meters. If displacement is larger than this, the approximate solution will fall back to the initial guess";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.approximate_solution_position_threshold);
          parameters_interface_->declare_parameter(prefix_ + "approximate_solution_position_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "approximate_solution_orientation_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Orientation threshold for approximate IK solutions, in radians. If displacement is larger than this, the approximate solution will fall back to the initial guess";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.approximate_solution_orientation_threshold);
          parameters_interface_->declare_parameter(prefix_ + "approximate_solution_orientation_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "approximate_solution_joint_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Joint threshold for approximate IK solutions, in radians. If displacement is larger than this, the approximate solution will fall back to the initial guess";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.approximate_solution_joint_threshold);
          parameters_interface_->declare_parameter(prefix_ + "approximate_solution_joint_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "approximate_solution_cost_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Cost threshold for approximate IK solutions. If the result of the cost function is larger than this, the approximate solution will fall back to the initial guess.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.approximate_solution_cost_threshold);
          parameters_interface_->declare_parameter(prefix_ + "approximate_solution_cost_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cost_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Scalar value for comparing to result of cost functions. IK is considered solved when all position/rotation/twist thresholds are satisfied and all cost functions return a value lower than this value.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.cost_threshold);
          parameters_interface_->declare_parameter(prefix_ + "cost_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "position_scale")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The position scale for the pose cost function. Set to 0.0 to solve for only rotation or other goals";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.position_scale);
          parameters_interface_->declare_parameter(prefix_ + "position_scale", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "rotation_scale")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The rotation scale for the pose cost function. Set to 0.0 to solve for only position";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.rotation_scale);
          parameters_interface_->declare_parameter(prefix_ + "rotation_scale", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "center_joints_weight")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Weight for centering cost function, >0.0 enables const function";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.center_joints_weight);
          parameters_interface_->declare_parameter(prefix_ + "center_joints_weight", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "avoid_joint_limits_weight")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Weight for avoiding joint limits cost function, >0.0 enables const function";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.avoid_joint_limits_weight);
          parameters_interface_->declare_parameter(prefix_ + "avoid_joint_limits_weight", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "minimal_displacement_weight")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Weight for minimal displacement cost function, >0.0 enables const function";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.minimal_displacement_weight);
          parameters_interface_->declare_parameter(prefix_ + "minimal_displacement_weight", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "stop_optimization_on_valid_solution")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If false, keeps running after finding a solution to further optimize the solution until a time or iteration limit is reached";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.stop_optimization_on_valid_solution);
          parameters_interface_->declare_parameter(prefix_ + "stop_optimization_on_valid_solution", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_num_threads")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Number of threads for memetic IK";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 1;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int>::max();
          auto parameter = to_parameter_value(updated_params.memetic_num_threads);
          parameters_interface_->declare_parameter(prefix_ + "memetic_num_threads", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_stop_on_first_solution")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true, stops on first solution and terminates other threads";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.memetic_stop_on_first_solution);
          parameters_interface_->declare_parameter(prefix_ + "memetic_stop_on_first_solution", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_population_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Population size for memetic IK";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 1;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int>::max();
          auto parameter = to_parameter_value(updated_params.memetic_population_size);
          parameters_interface_->declare_parameter(prefix_ + "memetic_population_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_elite_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Number of elite members of memetic IK population";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 1;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int>::max();
          auto parameter = to_parameter_value(updated_params.memetic_elite_size);
          parameters_interface_->declare_parameter(prefix_ + "memetic_elite_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_wipeout_fitness_tol")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Minimum fitness must improve by this value or population will be wiped out";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.memetic_wipeout_fitness_tol);
          parameters_interface_->declare_parameter(prefix_ + "memetic_wipeout_fitness_tol", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_max_generations")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum iterations of evolutionary algorithm";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 1;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int>::max();
          auto parameter = to_parameter_value(updated_params.memetic_max_generations);
          parameters_interface_->declare_parameter(prefix_ + "memetic_max_generations", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_gd_max_iters")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum iterations of gradient descent during memetic exploitation";
          descriptor.read_only = false;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 1;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int>::max();
          auto parameter = to_parameter_value(updated_params.memetic_gd_max_iters);
          parameters_interface_->declare_parameter(prefix_ + "memetic_gd_max_iters", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "memetic_gd_max_time")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum time spent on gradient descent during memetic exploitation";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.memetic_gd_max_time);
          parameters_interface_->declare_parameter(prefix_ + "memetic_gd_max_time", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "mode");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"global", "local"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'mode': {}", validation_result.error()));
      }
      updated_params.mode = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "gd_step_size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 1e-12);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'gd_step_size': {}", validation_result.error()));
      }
      updated_params.gd_step_size = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "gd_max_iters");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 1);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'gd_max_iters': {}", validation_result.error()));
      }
      updated_params.gd_max_iters = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "gd_min_cost_delta");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 1e-64);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'gd_min_cost_delta': {}", validation_result.error()));
      }
      updated_params.gd_min_cost_delta = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "position_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'position_threshold': {}", validation_result.error()));
      }
      updated_params.position_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "orientation_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'orientation_threshold': {}", validation_result.error()));
      }
      updated_params.orientation_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "approximate_solution_position_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'approximate_solution_position_threshold': {}", validation_result.error()));
      }
      updated_params.approximate_solution_position_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "approximate_solution_orientation_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'approximate_solution_orientation_threshold': {}", validation_result.error()));
      }
      updated_params.approximate_solution_orientation_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "approximate_solution_joint_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'approximate_solution_joint_threshold': {}", validation_result.error()));
      }
      updated_params.approximate_solution_joint_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "approximate_solution_cost_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'approximate_solution_cost_threshold': {}", validation_result.error()));
      }
      updated_params.approximate_solution_cost_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cost_threshold");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'cost_threshold': {}", validation_result.error()));
      }
      updated_params.cost_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "position_scale");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'position_scale': {}", validation_result.error()));
      }
      updated_params.position_scale = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "rotation_scale");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'rotation_scale': {}", validation_result.error()));
      }
      updated_params.rotation_scale = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "center_joints_weight");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'center_joints_weight': {}", validation_result.error()));
      }
      updated_params.center_joints_weight = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "avoid_joint_limits_weight");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'avoid_joint_limits_weight': {}", validation_result.error()));
      }
      updated_params.avoid_joint_limits_weight = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "minimal_displacement_weight");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'minimal_displacement_weight': {}", validation_result.error()));
      }
      updated_params.minimal_displacement_weight = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "stop_optimization_on_valid_solution");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.stop_optimization_on_valid_solution = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_num_threads");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 1);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_num_threads': {}", validation_result.error()));
      }
      updated_params.memetic_num_threads = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_stop_on_first_solution");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.memetic_stop_on_first_solution = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_population_size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 1);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_population_size': {}", validation_result.error()));
      }
      updated_params.memetic_population_size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_elite_size");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 1);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_elite_size': {}", validation_result.error()));
      }
      updated_params.memetic_elite_size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_wipeout_fitness_tol");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_wipeout_fitness_tol': {}", validation_result.error()));
      }
      updated_params.memetic_wipeout_fitness_tol = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_max_generations");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 1);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_max_generations': {}", validation_result.error()));
      }
      updated_params.memetic_max_generations = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_gd_max_iters");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 1);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_gd_max_iters': {}", validation_result.error()));
      }
      updated_params.memetic_gd_max_iters = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "memetic_gd_max_time");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'memetic_gd_max_time': {}", validation_result.error()));
      }
      updated_params.memetic_gd_max_time = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = updated_params;
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

      // rclcpp::Logger cannot be default-constructed
      // so we must provide a initialization here even though
      // every one of our constructors initializes logger_
      rclcpp::Logger logger_ = rclcpp::get_logger("pick_ik");
      std::mutex mutable mutex_;
  };

} // namespace pick_ik
