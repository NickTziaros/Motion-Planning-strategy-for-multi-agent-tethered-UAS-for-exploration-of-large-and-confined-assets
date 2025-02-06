

#include <rclcpp/rclcpp.hpp>
#include "isaac_sim_demo/RRT_controller.hpp"

// STL
#include <string>
#include <math.h>
#include <limits>

using namespace std;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#pragma GCC diagnostic ignored "-Wunused-parameter"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace RRT_controller {


/// occupancy map used for planning

ompl_controller::ompl_controller(void)
{
    // std::cout << "Planner started\n";
    // configure();
}

ompl_controller::~ompl_controller()
{
}

/// check if the current state is valid
bool isStateValid(const ob::State *state){

    return true;
}



//  extract path
nav_msgs::msg::Path  ompl_controller::extractPath(ompl::geometric::PathGeometric* path_){
    nav_msgs::msg::Path plannedPath;
    plannedPath.header.frame_id = "/World";

    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        
        const ob::State* state = path_->getState(i);



        // get x coord of the robot
        auto coordX = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // // get y coord of the robot
        auto coordY = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);


        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordX->values[0];
        poseMsg.pose.position.y = coordY->values[1];
        poseMsg.pose.position.z = coordY->values[2];
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/World";
        poseMsg.header.stamp = rclcpp::Clock().now();
        // ... and add the pose to the path
        plannedPath.poses.push_back(poseMsg);
        // std::cout << "Paath published.\n";

    }
    std::cout << "planned path size: " << plannedPath.poses.size() << "\n";
    return plannedPath;
}

/*!
 * plan path
 */
ompl::geometric::PathGeometric ompl_controller::planPath(ompl::geometric::SimpleSetup ss){
    


    ob::PlannerStatus solved = ss.solve(1.0);
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.simplifySolution();
    ompl::geometric::PathGeometric path = ss.getSolutionPath();
    // std::cout << path.getStateCount();

    return path;

    
}

/// configure planner
ompl::geometric::SimpleSetup ompl_controller::configure(std::vector<double> quad_poses, std::vector<double> goal_pose){

    auto space(std::make_shared<ob::SE3StateSpace>());
    std::vector<double> start_pose = quad_poses;
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-20);
    bounds.setHigh(20);
    space->setBounds(bounds);
    
    auto request = std::make_shared<custom_interfaces::srv::PlannerWorkspace::Request>();
    request->upper = 10;
    request->lower = -10;
    
    // auto result = client_->async_send_request(request);
    og::SimpleSetup ss(space);
     // define start state
    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setX(start_pose[0]);
    start->setY(start_pose[1]);
    start->setZ(start_pose[2]);
    start->rotation().setIdentity();

     // define goal state
    ob::ScopedState<ob::SE3StateSpace> goal(space);

  
    goal->setX(goal_pose[0]);                                                                                                                                                                                                                                                                                                              
    goal->setY(goal_pose[1]);
    goal->setZ(goal_pose[2]);
    goal->rotation().setIdentity();







  
    // ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    ss.setStateValidityChecker(isStateValid);
    ss.setStartAndGoalStates(start, goal);

    ompl::geometric::SimpleSetup* ss_pointer = &ss;
    return ss;
    
}

} //namespace

