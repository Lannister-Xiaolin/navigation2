// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SINGLE_NODE__PLANNER_SERVER_HPP_
#define NAV2_SINGLE_NODE__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace nav2_single_node_navigator {

enum class NavToPoseStatus {
  GOAL_UPDATED,
  PATH_UPDATED,
  PLAN_PATH_FAILED,
  GOAL_COLLIDED,
  FOLLOWING,
  ACTION_FAILED,
  FOLLOW_PATH_ABORTED,
  ODOM_NO_MOVE,
  FOLLOW_PATH_COLLIDED,
  STUCK_RECOVER_FAIL,
  CURRENT_STUCK_RECOVERY,
  CANCELLED,
  TF_FAILED,
  SUCCESS
};

class ProgressChecker;
/**
 * @class nav2_single_node_navigator::Nav2SingleNodeNavigator
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class Nav2SingleNodeNavigator : public nav2_util::LifecycleNode {
 public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;

  /**
   * @brief A constructor for nav2_single_node_navigator::Nav2SingleNodeNavigator
   */
  Nav2SingleNodeNavigator();
  /**
   * @brief A destructor for nav2_single_node_navigator::Nav2SingleNodeNavigator
   */
  ~Nav2SingleNodeNavigator();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      const std::string &planner_id);

 protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server);

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param action_server Action server to terminate if required
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  template<typename T>
  bool getStartPose(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      typename std::shared_ptr<const typename T::Goal> goal,
      geometry_msgs::msg::PoseStamped &start);

  /**
   * @brief Transform start and goal poses into the costmap
   * global frame for path planning plugins to utilize
   * @param action_server Action server to terminate if required
   * @param start The starting pose to transform
   * @param goal Goal pose to transform
   * @return bool If successful in transforming poses
   */
  template<typename T>
  bool transformPosesToGlobalFrame(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      geometry_msgs::msg::PoseStamped &curr_start,
      geometry_msgs::msg::PoseStamped &curr_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * @param action_server Action server to terminate if required
   * @param goal Goal Current goal
   * @param path Current path
   * @param planner_id The planner ID used to generate the path
   * @return bool If path is valid
   */
  template<typename T>
  bool validatePath(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      const geometry_msgs::msg::PoseStamped &curr_goal,
      const nav_msgs::msg::Path &path,
      const std::string &planner_id);

  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
  using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;
  using ActionFollowPath = nav2_msgs::action::FollowPath;
  using ActionServerFollowPath = nav2_util::SimpleActionServer<ActionFollowPath>;
  using ActionNavToPose = nav2_msgs::action::NavigateToPose;
  using ActionServerNavToPose = nav2_util::SimpleActionServer<ActionNavToPose>;
  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServerFollowPath> action_server_follow_path_;
  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_path_to_pose_;
  std::unique_ptr<ActionServerThroughPoses> action_server_path_to_poses_;
  std::unique_ptr<ActionServerNavToPose> action_server_nav_to_pos_;
  rclcpp_action::Client<ActionNavToPose>::SharedPtr action_client_nav_to_pos_;
  rclcpp_action::Client<ActionFollowPath>::SharedPtr action_client_follow_path_;

  typedef std::function<bool(typename ActionNavToPose::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
  typedef std::function<void()> OnLoopCallback;
  typedef std::function<void(typename ActionNavToPose::Goal::ConstSharedPtr)> OnPreemptCallback;
  typedef std::function<void(typename ActionNavToPose::Result::SharedPtr)> OnCompletionCallback;
  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathToPose
   */
  void computePlan();
  bool computePathForNavToPose(const std::string &planner_id,
                               const geometry_msgs::msg::PoseStamped &goal_pose,
                               nav_msgs::msg::Path &path);
  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathThroughPoses
   */
  void computePlanThroughPoses();

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path &path);

  // ---------------controller----------------
  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string &c_name, std::string &name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string &c_name, std::string &name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path &path);
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped &velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped &pose);

  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold) {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D &twist) {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }
  bool isPathCollisionWithObstacle(nav_msgs::msg::Path &path);

  // ---------------controller----------------

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> global_costmap_thread_;
  nav2_costmap_2d::Costmap2D *global_costmap_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  //============controller==========
  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> local_costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> controller_default_ids_;
  std::vector<std::string> controller_default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::PoseStamped end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;

  //---------------nav to pose ---------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  // -------------callback group--------------
 private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
  void navToPoseCallback();
//  void onNavToPoseLoop();
//  void onNavToPosePreempt(ActionNavToPose::Goal::ConstSharedPtr goal);
//  void goalNavToPoseCompleted(ActionNavToPose::Result::SharedPtr result);
  bool OnNavToPoseGoalReceivedCallback(ActionNavToPose::Goal::ConstSharedPtr goal);
  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void followPathResultCallback(const rclcpp_action::ClientGoalHandle<ActionFollowPath>::WrappedResult &result);
  bool startFollowPath(const nav_msgs::msg::Path &path);
  bool startNavToPose(const geometry_msgs::msg::PoseStamped &pose);
  //-------------------nav to goal -----------------------------
  std::array<uint8_t, UUID_SIZE> follow_path_client_goal_id_, nav_to_pose_client_goal_id_;
  rclcpp_action::ClientGoalHandle<ActionFollowPath>::WrappedResult follow_path_client_result_;
  rclcpp_action::Client<ActionFollowPath>::SendGoalOptions follow_path_send_goal_options_;
  rclcpp_action::Client<ActionFollowPath>::SendGoalOptions nav_to_pose_send_goal_options_;
  bool follow_path_working_;
  NavToPoseStatus nav_to_pose_status_, prev_nav_to_pose_status_;
  geometry_msgs::msg::PoseStamped global_pose_, local_pose_;
  rclcpp::Time last_check_time_{0, 0}, current_time_{0, 0},
  current_path_time_{0, 0},task_start_time_{0,0};
  nav_msgs::msg::Path current_planned_path_;
  bool can_try_recover_;
  int re_plan_count_;
  double max_back_dis_,max_back_vel_,max_back_angular_vel_,path_fail_stuck_confirm_range_,follow_fail_stuck_confirm_range_;
  std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> navigate_to_pose_goal_;
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Result> navigate_to_pose_result_;
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Feedback> navigate_to_pose_feedback_;
  std::shared_ptr<rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>> clear_local_around_client_;
  void updateStatus(NavToPoseStatus nav_to_pose_status);
  bool updateGlobalPose();
  void followingDeal();
  void pathUpdatedDeal();
  void goalUpdatedDeal();
  void planPathFailedDeal();
  bool isCurrentStuck(double search_range);
  bool isCurrentLocalStuck(double search_range);
  bool isGoalCollided();
  void currentStuckRecoveryDeal();
};

}  // namespace nav2_single_node_navigator

#endif  // nav2_single_node_navigator__PLANNER_SERVER_HPP_
