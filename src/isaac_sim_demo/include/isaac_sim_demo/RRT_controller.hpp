#pragma once

// ROS

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// #include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
 #include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <iostream>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include "custom_interfaces/srv/planner_workspace.hpp"
#include <rclcpp/rclcpp.hpp>


// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace RRT_controller {



class ompl_controller
{
public:

    /*!
   * Constructor.
   */
    ompl_controller(void);

    /*!
   * Destructor.
   */
    virtual ~ompl_controller();

    /*!
   * plan path
   */
    ompl::geometric::PathGeometric planPath(ompl::geometric::SimpleSetup ss);
    ompl::geometric::SimpleSetup configure(std::vector<double> quad_poses, std::vector<double> goal_pose);

    /// extract path
    nav_msgs::msg::Path  extractPath(ompl::geometric::PathGeometric* path_);

private:

    // /// problem dim
    // int dim;

    // /// max step length
    // double maxStepLength;

    // /// bounds for the x axis
    // std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    // /// bounds for the y axis
    // std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    // /// start position
    // std::shared_ptr<ompl::base::ScopedState<>> start;

    // /// goal position
    // std::shared_ptr<ompl::base::ScopedState<>> goal;

    // /// search space
    // std::shared_ptr<ompl::base::StateSpace> space;

    /// configure node
    // ompl::geometric::SimpleSetup* configure(void);

    // /// extract path
    // nav_msgs::msg::Path extractPath(ompl::geometric::SimpleSetup* ss);
};


} //namespace