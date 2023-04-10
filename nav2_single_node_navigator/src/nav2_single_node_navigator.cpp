// Copyright (c) 2018 Intel Corporation
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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_single_node_navigator.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_single_node_navigator {

std::string StringNavToPoseStatus(NavToPoseStatus task_status) {
  switch (task_status) {
    case NavToPoseStatus::GOAL_UPDATED:return "GOAL_UPDATED";
    case NavToPoseStatus::PATH_UPDATED:return "PATH_UPDATED";
    case NavToPoseStatus::PLAN_PATH_FAILED:return "PLAN_PATH_FAILED";
    case NavToPoseStatus::GOAL_COLLIDED:return "GOAL_COLLIDED";
    case NavToPoseStatus::FOLLOWING:return "FOLLOWING";
    case NavToPoseStatus::ACTION_FAILED:return "ACTION_FAILED";
    case NavToPoseStatus::FOLLOW_PATH_ABORTED:return "FOLLOW_PATH_ABORTED";
    case NavToPoseStatus::ODOM_NO_MOVE:return "ODOM_NO_MOVE";
    case NavToPoseStatus::FOLLOW_PATH_COLLIDED:return "FOLLOW_PATH_COLLIDED";
    case NavToPoseStatus::STUCK_RECOVER_FAIL:return "STUCK_RECOVER_FAIL";
    case NavToPoseStatus::CURRENT_STUCK_RECOVERY:return "CURRENT_STUCK_RECOVERY";
    case NavToPoseStatus::CANCELLED:return "CANCELLED";
    case NavToPoseStatus::TF_FAILED:return "TF_FAILED";
    case NavToPoseStatus::SUCCESS:return "SUCCESS";

    default:return "ERROR STATUS";
  }
}

Nav2SingleNodeNavigator::Nav2SingleNodeNavigator()
    : nav2_util::LifecycleNode("nav2_single_node_navigator",
                               "",
                               true,
                               rclcpp::NodeOptions().use_intra_process_comms(true)),
      gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
      default_ids_{"GridBased"},
      default_types_{"nav2_navfn_planner/NavfnPlanner"},
      global_costmap_(nullptr),
      progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
      default_progress_checker_id_{"progress_checker"},
      default_progress_checker_type_{"nav2_controller::SimpleProgressChecker"},
      goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
      default_goal_checker_ids_{"goal_checker"},
      default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
      lp_loader_("nav2_core", "nav2_core::Controller"),
      controller_default_ids_{"FollowPath"},
      controller_default_types_{"dwb_core::DWBLocalPlanner"} {
  RCLCPP_INFO(get_logger(), "Creating");
  //=========planner==================
  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap
  global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{get_namespace()}, "global_costmap");

  // Launch a thread to run the costmap node
  global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);
  //============controller==========
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);
  declare_parameter("max_back_dis", 0.36);
  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", controller_default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));

  // The costmap node is used in the implementation of the controller
  local_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, "local_costmap");

  // Launch a thread to run the costmap node
  local_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(local_costmap_ros_);

}

Nav2SingleNodeNavigator::~Nav2SingleNodeNavigator() {
  planners_.clear();
  global_costmap_thread_.reset();
}

nav2_util::CallbackReturn
Nav2SingleNodeNavigator::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Configuring");

  global_costmap_ros_->on_configure(state);
  global_costmap_ = global_costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
      get_logger(), "Costmap size: %d,%d",
      global_costmap_->getSizeInCellsX(), global_costmap_->getSizeInCellsY());

  tf_ = global_costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
          node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
          gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
          get_logger(), "Created global planner plugin %s of type %s",
          planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, global_costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_FATAL(
          get_logger(), "Failed to create global planner. Exception: %s",
          ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
      get_logger(),
      "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
        get_logger(),
        "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
        " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_path_to_pose_ = std::make_unique<ActionServerToPose>(
      rclcpp_node_,
      "compute_path_to_pose",
      std::bind(&Nav2SingleNodeNavigator::computePlan, this));

  action_server_path_to_poses_ = std::make_unique<ActionServerThroughPoses>(
      rclcpp_node_,
      "compute_path_through_poses",
      std::bind(&Nav2SingleNodeNavigator::computePlanThroughPoses, this));

  //========================controller============================

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
        node, default_progress_checker_id_ + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_type_));
  }

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
          node, default_goal_checker_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_goal_checker_types_[i]));
      RCLCPP_INFO(get_logger(),
                  "getting goal checker plugins..  ......      ........%s",
                  default_goal_checker_ids_.at(i).c_str());

    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
          node, default_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);

  local_costmap_ros_->on_configure(state);

  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
        get_logger(), "Created progress_checker : %s of type %s",
        progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException &ex) {
    RCLCPP_FATAL(
        get_logger(),
        "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
          goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
          get_logger(), "Created goal checker : %s of type %s",
          goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i]);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_FATAL(
          get_logger(),
          "Failed to create goal checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
      get_logger(),
      "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
          lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
          get_logger(), "Created controller : %s of type %s",
          controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
          node, controller_ids_[i],
          local_costmap_ros_->getTfBuffer(), local_costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_FATAL(
          get_logger(),
          "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
      get_logger(),
      "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Create the action server that we implement with our followPath method
  action_server_follow_path_ = std::make_unique<ActionServerFollowPath>(
      rclcpp_node_, "follow_path",
      std::bind(&Nav2SingleNodeNavigator::computeControl, this));

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic, rclcpp::QoS(10),
      std::bind(&Nav2SingleNodeNavigator::speedLimitCallback, this, std::placeholders::_1));

  //--------------------nav to pose ----------------------------

  action_server_nav_to_pos_ = std::make_unique<ActionServerNavToPose>(
      rclcpp_node_,
      "navigate_to_pose",
      std::bind(&Nav2SingleNodeNavigator::navToPoseCallback, this));
  action_client_nav_to_pos_ = rclcpp_action::create_client<ActionNavToPose>(node, "navigate_to_pose");
  action_client_follow_path_ = rclcpp_action::create_client<ActionFollowPath>(node, "follow_path");
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      10,
      std::bind(&Nav2SingleNodeNavigator::callback_updated_goal, this, _1));
  follow_path_send_goal_options_.result_callback =
      std::bind(&Nav2SingleNodeNavigator::followPathResultCallback, this, _1);

  clear_local_around_client_ = node->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>(
      "/local_costmap/clear_around_local_costmap");
  //-------to be parameterized params
  max_back_angular_vel_ = 0.35; // stuck recover max back angular vel
  get_parameter("max_back_dis", max_back_dis_);
  max_back_vel_ = 0.1;
  path_fail_stuck_confirm_range_ = 0.06;
  follow_fail_stuck_confirm_range_ = 0.11;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Nav2SingleNodeNavigator::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_path_to_pose_->activate();
  action_server_path_to_poses_->activate();
  global_costmap_ros_->on_activate(state);

  PlannerMap::iterator it;

  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }
  //========================controller============================
  local_costmap_ros_->on_activate(state);
  ControllerMap::iterator controller_it;
  for (controller_it = controllers_.begin(); controller_it != controllers_.end(); ++controller_it) {
    controller_it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_follow_path_->activate();
  action_server_nav_to_pos_->activate();
  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Nav2SingleNodeNavigator::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_path_to_pose_->deactivate();
  action_server_path_to_poses_->deactivate();
  plan_publisher_->on_deactivate();
  global_costmap_ros_->on_deactivate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }
  //========================controller============================
  action_server_follow_path_->deactivate();
  ControllerMap::iterator controller_it;
  for (controller_it = controllers_.begin(); controller_it != controllers_.end(); ++controller_it) {
    controller_it->second->deactivate();
  }
  local_costmap_ros_->on_deactivate(state);

  publishZeroVelocity();
  vel_publisher_->on_deactivate();

  action_server_nav_to_pos_->deactivate();
  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Nav2SingleNodeNavigator::on_cleanup(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_path_to_pose_.reset();
  action_server_path_to_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();
  global_costmap_ros_->on_cleanup(state);
  clear_local_around_client_.reset();
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();
  global_costmap_ = nullptr;
  //========================controller============================
  // Cleanup the helper classes
  ControllerMap::iterator controller_it;
  for (controller_it = controllers_.begin(); controller_it != controllers_.end(); ++controller_it) {
    controller_it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();
  local_costmap_ros_->on_cleanup(state);

  // Release any allocated resources
  action_server_follow_path_.reset();
  odom_sub_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();
  action_server_follow_path_.reset();
  //---------------nav to pose -------------------
  goal_sub_.reset();
  action_server_nav_to_pos_.reset();
  action_client_nav_to_pos_.reset();
  action_client_follow_path_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Nav2SingleNodeNavigator::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool Nav2SingleNodeNavigator::isServerInactive(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server) {
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void Nav2SingleNodeNavigator::waitForCostmap() {
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!global_costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool Nav2SingleNodeNavigator::isCancelRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server) {
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void Nav2SingleNodeNavigator::getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
    typename std::shared_ptr<const typename T::Goal> goal) {
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool Nav2SingleNodeNavigator::getStartPose(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped &start) {
  if (goal->use_start) {
    start = goal->start;
  } else if (!global_costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool Nav2SingleNodeNavigator::transformPosesToGlobalFrame(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
    geometry_msgs::msg::PoseStamped &curr_start,
    geometry_msgs::msg::PoseStamped &curr_goal) {
  if (!global_costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
      !global_costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal)) {
    RCLCPP_WARN(
        get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool Nav2SingleNodeNavigator::validatePath(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
    const geometry_msgs::msg::PoseStamped &goal,
    const nav_msgs::msg::Path &path,
    const std::string &planner_id) {
  if (path.poses.empty()) {
    RCLCPP_WARN(
        get_logger(), "Planning algorithm %s failed to generate a valid"
                      " path to (%.2f, %.2f)", planner_id.c_str(),
        goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
      get_logger(),
      "Found valid path of size %lu to (%.2f, %.2f)",
      path.poses.size(), goal.pose.position.x,
      goal.pose.position.y);

  return true;
}

void
Nav2SingleNodeNavigator::computePlanThroughPoses() {
  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_path_to_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  try {
    if (isServerInactive(action_server_path_to_poses_) || isCancelRequested(action_server_path_to_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_path_to_poses_, goal);

    if (goal->goals.empty()) {
      RCLCPP_WARN(
          get_logger(),
          "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_path_to_poses_->terminate_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_path_to_poses_, goal, start)) {
      return;
    }

    // Get consecutive paths through these points
    std::vector<geometry_msgs::msg::PoseStamped>::iterator goal_iter;
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        curr_start = goal->goals[i - 1];
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_path_to_poses_, curr_start, curr_goal)) {
        return;
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      // check path for validity
      if (!validatePath(action_server_path_to_poses_, curr_goal, curr_path, goal->planner_id)) {
        return;
      }

      // Concatenate paths together
      concat_path.poses.insert(
          concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
          get_logger(),
          "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_path_to_poses_->succeeded_current(result);
  } catch (std::exception &ex) {
    RCLCPP_WARN(
        get_logger(),
        "%s plugin failed to plan through %li points with final goal (%.2f, %.2f): \"%s\"",
        goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
        goal->goals.back().pose.position.y, ex.what());
    action_server_path_to_poses_->terminate_current();
  }
}

void
Nav2SingleNodeNavigator::computePlan() {
  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_path_to_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    if (isServerInactive(action_server_path_to_pose_) || isCancelRequested(action_server_path_to_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_path_to_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_path_to_pose_, goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_path_to_pose_, start, goal_pose)) {
      return;
    }

    result->path = getPlan(start, goal_pose, goal->planner_id);

    if (!validatePath(action_server_path_to_pose_, goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
          get_logger(),
          "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_path_to_pose_->succeeded_current(result);
  } catch (std::exception &ex) {
    RCLCPP_WARN(
        get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
        goal->planner_id.c_str(), goal->goal.pose.position.x,
        goal->goal.pose.position.y, ex.what());
    action_server_path_to_pose_->terminate_current();
  }
}

nav_msgs::msg::Path
Nav2SingleNodeNavigator::getPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    const std::string &planner_id) {
  RCLCPP_DEBUG(
      get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
                    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
          get_logger(), "No planners specified in action call. "
                        "Server will use only plugin %s in server."
                        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
          get_logger(), "planner %s is not a valid planner. "
                        "Planner names are: %s", planner_id.c_str(),
          planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
}

void
Nav2SingleNodeNavigator::publishPlan(const nav_msgs::msg::Path &path) {
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

//============controller==========

bool Nav2SingleNodeNavigator::findControllerId(
    const std::string &c_name,
    std::string &current_controller) {
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
          get_logger(), "No controller was specified in action call."
                        " Server will use only plugin loaded %s. "
                        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
          get_logger(), "FollowPath called with controller name %s, "
                        "which does not exist. Available controllers are: %s.",
          c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool Nav2SingleNodeNavigator::findGoalCheckerId(
    const std::string &c_name,
    std::string &current_goal_checker) {
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
          get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
                        " Server will use only plugin loaded %s. "
                        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
          get_logger(), "FollowPath called with goal_checker name %s in parameter"
                        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
          c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void Nav2SingleNodeNavigator::computeControl() {
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    std::string c_name = action_server_follow_path_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_follow_path_->terminate_current();
      return;
    }

    std::string gc_name = action_server_follow_path_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_follow_path_->terminate_current();
      return;
    }

    setPlannerPath(action_server_follow_path_->get_current_goal()->path);
    progress_checker_->reset();

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_follow_path_ == nullptr || !action_server_follow_path_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_follow_path_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_follow_path_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!local_costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
            get_logger(), "Control loop missed its desired rate of %.4fHz",
            controller_frequency_);
      }
    }
  } catch (nav2_core::PlannerException &e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    action_server_follow_path_->terminate_current();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_follow_path_->succeeded_current();
}

void Nav2SingleNodeNavigator::setPlannerPath(const nav_msgs::msg::Path &path) {
  RCLCPP_DEBUG(
      get_logger(),
      "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
      get_logger(), "Path end point is (%.2f, %.2f)",
      end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
}

void Nav2SingleNodeNavigator::computeAndPublishVelocity() {
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    throw nav2_core::PlannerException("Failed to make progress");
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d =
        controllers_[current_controller_]->computeVelocityCommands(
            pose,
            nav_2d_utils::twist2Dto3D(twist),
            goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
  } catch (nav2_core::PlannerException &e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = local_costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
          failure_tolerance_ != -1.0) {
        throw nav2_core::PlannerException("Controller patience exceeded");
      }
    } else {
      throw nav2_core::PlannerException(e.what());
    }
  }

  std::shared_ptr<ActionFollowPath::Feedback> feedback = std::make_shared<ActionFollowPath::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  nav_msgs::msg::Path &current_path = current_path_;
  auto find_closest_pose_idx =
      [&pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
              pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

  feedback->distance_to_goal =
      nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  action_server_follow_path_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void Nav2SingleNodeNavigator::updateGlobalPath() {
  if (action_server_follow_path_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_follow_path_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(
          get_logger(), "Terminating action, invalid controller %s requested.",
          goal->controller_id.c_str());
      action_server_follow_path_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void Nav2SingleNodeNavigator::publishVelocity(const geometry_msgs::msg::TwistStamped &velocity) {
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void Nav2SingleNodeNavigator::publishZeroVelocity() {
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = local_costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool Nav2SingleNodeNavigator::isGoalReached() {
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(local_costmap_ros_->getTransformTolerance()));
  nav_2d_utils::transformPose(
      local_costmap_ros_->getTfBuffer(), local_costmap_ros_->getGlobalFrameID(),
      end_pose_, transformed_end_pose, tolerance);

  return goal_checkers_[current_goal_checker_]->isGoalReached(
      pose.pose, transformed_end_pose.pose,
      velocity);
}

bool Nav2SingleNodeNavigator::getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
  geometry_msgs::msg::PoseStamped current_pose;
  if (!local_costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void Nav2SingleNodeNavigator::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}
void Nav2SingleNodeNavigator::navToPoseCallback() {
  RCLCPP_INFO(
      get_logger(),
      "Begin navigating from current location to (%.2f, %.2f)",
      action_server_nav_to_pos_->get_current_goal()->pose.pose.position.x,
      action_server_nav_to_pos_->get_current_goal()->pose.pose.position.y);
  auto is_canceling = [&]() {
    if (action_server_nav_to_pos_ == nullptr) {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable. Canceling.");
      return true;
    }
    if (!action_server_nav_to_pos_->is_server_active()) {
      RCLCPP_DEBUG(get_logger(), "Action server is inactive. Canceling.");
      return true;
    }
    return action_server_nav_to_pos_->is_cancel_requested();
  };

  rclcpp::WallRate loopRate(5);
  navigate_to_pose_goal_ = action_server_nav_to_pos_->get_current_goal();
  navigate_to_pose_result_ = std::make_shared<ActionNavToPose::Result>();
  task_start_time_ = get_clock()->now();
  // Loop until something happens with ROS or the node completes
  nav_to_pose_status_ = NavToPoseStatus::GOAL_UPDATED;
  prev_nav_to_pose_status_ = NavToPoseStatus::GOAL_UPDATED;
  current_path_time_ = now();
  last_check_time_ = now();
  follow_path_working_ = false;
  int tf_failed_retry_count = 0;
  can_try_recover_ = true;
  current_planned_path_ = nav_msgs::msg::Path();
  int unknown_follow_path_fail_retry = 0;
  fail_re_plan_count_ = 0;
  try {
    while (rclcpp::ok()) {
      current_time_ = now();
      if (nav_to_pose_status_ == NavToPoseStatus::GOAL_UPDATED) {
        goalUpdatedDeal();
        unknown_follow_path_fail_retry = std::max(unknown_follow_path_fail_retry - 1, 0);
      } else if (nav_to_pose_status_ == NavToPoseStatus::PATH_UPDATED) {
        pathUpdatedDeal();
      } else if (nav_to_pose_status_ == NavToPoseStatus::PLAN_PATH_FAILED) {
        planPathFailedDeal();
        if (!can_try_recover_) break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::ACTION_FAILED) {
        RCLCPP_WARN(get_logger(), "Follow path failed!!!");
        break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::TF_FAILED) {
        tf_failed_retry_count += 1;
        if (updateGlobalPose()) updateStatus(NavToPoseStatus::GOAL_UPDATED);
        if (tf_failed_retry_count > 5) break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::FOLLOW_PATH_COLLIDED
          || nav_to_pose_status_ == NavToPoseStatus::FOLLOW_PATH_ABORTED) {
        if (isCurrentStuck(follow_fail_stuck_confirm_range_)) {
          updateStatus(NavToPoseStatus::CURRENT_STUCK_RECOVERY);
        } else if (isCurrentLocalStuck(path_fail_stuck_confirm_range_)) {
          RCLCPP_WARN(get_logger(),
                      "Local cost map and global cost map is different, some obstacle is not in global cost map!!!");
          if (clear_local_around_client_->wait_for_service(200ms)) {
            RCLCPP_WARN(get_logger(), "Just clear around costmap and move very small distance!!!");
            nav2_msgs::srv::ClearCostmapAroundRobot::Request::SharedPtr
                request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
            request->reset_distance = 0.6;
            clear_local_around_client_->async_send_request(request);
            std::this_thread::sleep_for(500ms);
            geometry_msgs::msg::Twist twist;
            twist.angular.z = -0.1;
            twist.angular.x = -0.03;
            vel_publisher_->publish(twist);
            std::this_thread::sleep_for(100ms);
            twist.angular.z = 0.0;
            twist.angular.x = 0.0;
            vel_publisher_->publish(twist);
            updateStatus(NavToPoseStatus::GOAL_UPDATED);
          } else {
            RCLCPP_ERROR(get_logger(), "/local_costmap/clear_around_local_costmap service is not ready, stop task!!!");
            break;
          }
        } else {
          // only continual fail with same reason will lead to unknown_follow_path_fail_retry increase;
          //todo optimize
          unknown_follow_path_fail_retry += 2;
          if (unknown_follow_path_fail_retry < 3) {
            RCLCPP_WARN(get_logger(),
                        "Current position is not stuck. Maybe controller is not able to compute path retry!!!!!!");
            updateStatus(NavToPoseStatus::GOAL_UPDATED);
          } else {
            RCLCPP_WARN(get_logger(), "Current position is not stuck. reach max retry times, abort!!!!!!");
            break;
          }
        }
      } else if (nav_to_pose_status_ == NavToPoseStatus::ODOM_NO_MOVE) {
        RCLCPP_WARN(get_logger(), "Look like robot is not moving!!!");
        break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::GOAL_COLLIDED) {
        action_client_follow_path_->async_cancel_all_goals();
        RCLCPP_WARN(get_logger(), "Look like goal is occupied by obstacle!!!");
        break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::CANCELLED) {
        RCLCPP_INFO(get_logger(), "Goal cancelled!!!");
        break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::STUCK_RECOVER_FAIL) {
        RCLCPP_INFO(get_logger(), "Can't recover from stuck!!!");
        break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::CURRENT_STUCK_RECOVERY) {
        currentStuckRecoveryDeal();
      } else if (nav_to_pose_status_ == NavToPoseStatus::SUCCESS) {
        break;
      } else if (nav_to_pose_status_ == NavToPoseStatus::FOLLOWING) {
        followingDeal();
      }
      if (is_canceling()) {
        updateStatus(NavToPoseStatus::CANCELLED);
      } else if (action_server_nav_to_pos_->is_preempt_requested()) {
        navigate_to_pose_goal_ = action_server_nav_to_pos_->accept_pending_goal();
        updateStatus(NavToPoseStatus::GOAL_UPDATED);
      }
      loopRate.sleep();
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(get_logger(), "Unknown Error!!!! %s", ex.what());
    action_server_nav_to_pos_->terminate_current(navigate_to_pose_result_);
  }
  if (follow_path_working_) {
    action_client_follow_path_->async_cancel_all_goals();
  }
  if (nav_to_pose_status_ == NavToPoseStatus::SUCCESS) {
    // Give server an opportunity to populate the result message or simple give
    // an indication that the action is complete.
    action_server_nav_to_pos_->succeeded_current(navigate_to_pose_result_);
  } else {
    action_server_nav_to_pos_->terminate_current(navigate_to_pose_result_);
  }
  navigate_to_pose_goal_.reset();
  navigate_to_pose_result_.reset();
  current_planned_path_ = nav_msgs::msg::Path();
}

//bool Nav2SingleNodeNavigator::OnNavToPoseGoalReceivedCallback(ActionNavToPose::Goal::ConstSharedPtr goal) {
//
//  RCLCPP_INFO(
//      get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
//      goal->pose.pose.position.x, goal->pose.pose.position.y);
//  //navigator.hpp 239 line means there are two action, need to
//  // check for avoid running two different action in the same time
//  return true;
//}
bool Nav2SingleNodeNavigator::computePathForNavToPose(const std::string &planner_id,
                                                      const geometry_msgs::msg::PoseStamped &goal_pose,
                                                      nav_msgs::msg::Path &path) {

  waitForCostmap();
  // Use start pose if provided otherwise use current robot pose
//  geometry_msgs::msg::PoseStamped start;
  if (!global_costmap_ros_->getRobotPose(global_pose_)) {
    return false;
  }
  auto end = goal_pose;
  if (!global_costmap_ros_->transformPoseToGlobalFrame(global_pose_, global_pose_) ||
      !global_costmap_ros_->transformPoseToGlobalFrame(end, end)) {
    return false;

  }
  path = getPlan(global_pose_, end, planner_id);
  if (!validatePath(action_server_path_to_pose_, end, path, planner_id)) {
    return false;
  }
  // Publish the plan for visualization purposes
  publishPlan(path);
//  RCLCPP_INFO(get_logger(), "---Path size %zu  frame: %s", path.poses.size(), path.header.frame_id.c_str());
  return true;
}
void
Nav2SingleNodeNavigator::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  startNavToPose(*msg);
  //todo
}
void Nav2SingleNodeNavigator::followPathResultCallback(const rclcpp_action::ClientGoalHandle<ActionFollowPath>::WrappedResult &result) {
  if (follow_path_client_goal_id_ != result.goal_id) return;
  RCLCPP_DEBUG(get_logger(), "         follow_path_client_goal_id_:  %s  result.goal_id:  %s",
               rclcpp_action::to_string(follow_path_client_goal_id_).c_str(),
               rclcpp_action::to_string(result.goal_id).c_str());
  follow_path_client_result_ = result;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:RCLCPP_INFO(get_logger(), ">>>>>>>>>>  Follow path was success");
      break;
    case rclcpp_action::ResultCode::ABORTED:RCLCPP_WARN(get_logger(), ">>>>>>>>>>  Follow path was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:RCLCPP_WARN(get_logger(), ">>>>>>>>>>  Follow path was canceled");
      break;
    default:RCLCPP_WARN(get_logger(), ">>>>>>>>>>  Follow path: Unknown result code");
      break;
  }
  follow_path_working_ = false;
}
bool Nav2SingleNodeNavigator::startFollowPath(const nav_msgs::msg::Path &path) {
//  auto cancel_handle = action_client_follow_path_->async_cancel_all_goals();
//  cancel_handle.wait();
  if (!action_client_follow_path_->wait_for_action_server(std::chrono::milliseconds(100))) return false;
  ActionFollowPath::Goal goal;
  goal.path = path;
  goal.goal_checker_id = "general_goal_checker";
  goal.controller_id = "FollowPath";
  auto goal_handle = action_client_follow_path_->async_send_goal(goal, follow_path_send_goal_options_).get();
  goal_handle->is_result_aware();
  if (goal_handle == nullptr) {
    return false;
  } else {
    follow_path_client_goal_id_ = goal_handle->get_goal_id();
    follow_path_working_ = true;
    RCLCPP_DEBUG(get_logger(),
                 "         follow_path_client_goal_id_:  %s",
                 rclcpp_action::to_string(follow_path_client_goal_id_).c_str());
  }
  return true;
}
bool Nav2SingleNodeNavigator::startNavToPose(const geometry_msgs::msg::PoseStamped &pose) {
  action_client_nav_to_pos_->async_cancel_all_goals();
  if (!action_client_nav_to_pos_->wait_for_action_server(std::chrono::milliseconds(100))) return false;
  ActionNavToPose::Goal goal;
  goal.pose = pose;
  action_client_nav_to_pos_->async_send_goal(goal);
  return true;
}
bool Nav2SingleNodeNavigator::isPathCollisionWithObstacle(nav_msgs::msg::Path &path) {
  if (!updateGlobalPose()) return false;
  auto closest = std::min_element(path.poses.begin(), path.poses.end(), [this](geometry_msgs::msg::PoseStamped &a,
                                                                               geometry_msgs::msg::PoseStamped &b) {
    auto a_dis = std::abs(global_pose_.pose.position.x - a.pose.position.x)
        + std::abs(global_pose_.pose.position.y - a.pose.position.y);
    auto b_dis = std::abs(global_pose_.pose.position.x - b.pose.position.x)
        + std::abs(global_pose_.pose.position.y - b.pose.position.y);
    return a_dis < b_dis;
  });
  auto start = std::distance(path.poses.begin(), closest);
  for (unsigned int i = start; i < path.poses.size(); ++i) {
    unsigned int mx, my;
    if (global_costmap_->worldToMap(path.poses[i].pose.position.x, path.poses[i].pose.position.y, mx, my)) {
      auto value = global_costmap_->getCost(mx, my);
      if (value == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || value == nav2_costmap_2d::LETHAL_OBSTACLE) {
        RCLCPP_WARN(get_logger(), "Path may be collision with obstacle!!!!");
        return true;
      }
    }
  }
  return false;
}

void Nav2SingleNodeNavigator::updateStatus(NavToPoseStatus nav_to_pose_status) {
  if (nav_to_pose_status == nav_to_pose_status_) return;
  prev_nav_to_pose_status_ = nav_to_pose_status_;
  nav_to_pose_status_ = nav_to_pose_status;
  RCLCPP_INFO(get_logger(), "--------------------Current  status: %s  prev  status: %s--------------------",
              StringNavToPoseStatus(nav_to_pose_status_).c_str(),
              StringNavToPoseStatus(prev_nav_to_pose_status_).c_str());
}
bool Nav2SingleNodeNavigator::updateGlobalPose() {
  if (!global_costmap_ros_->getRobotPose(global_pose_)) {
    updateStatus(NavToPoseStatus::TF_FAILED);
    return false;
  }
  if (!global_costmap_ros_->transformPoseToGlobalFrame(global_pose_, global_pose_)) {
    updateStatus(NavToPoseStatus::TF_FAILED);
    return false;
  }
  return true;
}
void Nav2SingleNodeNavigator::followingDeal() {
  if (!follow_path_working_) {
    if (follow_path_client_result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      updateStatus(NavToPoseStatus::SUCCESS);
    } else {
      updateStatus(NavToPoseStatus::FOLLOW_PATH_COLLIDED);
    }
  } else {
    auto diff = current_time_ - last_check_time_;
    if (diff > tf2::durationFromSec(0.8)) {
      last_check_time_ = current_time_;
      auto path_diff_time = current_time_ - current_path_time_;
      if (isPathCollisionWithObstacle(current_planned_path_) || path_diff_time > tf2::durationFromSec(120.0)) {
        if (isGoalCollided()) {
          updateStatus(NavToPoseStatus::GOAL_COLLIDED);
        }else{
          updateStatus(NavToPoseStatus::GOAL_UPDATED);
        }
      }
    }
  }
}
void Nav2SingleNodeNavigator::pathUpdatedDeal() {
  if (startFollowPath(current_planned_path_)) {
    current_path_time_ = current_time_;
    last_check_time_ = current_time_;
    updateStatus(NavToPoseStatus::FOLLOWING);
  } else updateStatus(NavToPoseStatus::ACTION_FAILED);
}
void Nav2SingleNodeNavigator::goalUpdatedDeal() {
  if (computePathForNavToPose("GridBased", navigate_to_pose_goal_->pose, current_planned_path_)) {
    fail_re_plan_count_ = 0;
    updateStatus(NavToPoseStatus::PATH_UPDATED);
  } else updateStatus(NavToPoseStatus::PLAN_PATH_FAILED);
}
void Nav2SingleNodeNavigator::planPathFailedDeal() {
  if (isCurrentStuck(path_fail_stuck_confirm_range_)) {
    updateStatus(NavToPoseStatus::CURRENT_STUCK_RECOVERY);
    can_try_recover_ = true;
  } else if (isGoalCollided()) {
    updateStatus(NavToPoseStatus::GOAL_COLLIDED);
    can_try_recover_ = true;
  } else {
    can_try_recover_ = false;
  };
}
bool Nav2SingleNodeNavigator::isCurrentStuck(double search_range) {
  if (!updateGlobalPose()) return false;
  unsigned int mx, my;
  int grid_search_range = std::max(static_cast<int>(search_range / global_costmap_->getResolution()), 1);
  if (global_costmap_->worldToMap(global_pose_.pose.position.x, global_pose_.pose.position.y, mx, my)) {
    for (int i = 1; i <= grid_search_range; ++i) {
      std::vector<int> x_offset = {0, 0, 0, -i, i, i, -i, i, -i};
      std::vector<int> y_offset = {0, i, -i, 0, 0, i, -i, -i, i};
      for (int j = 0; j < 9; ++j) {
        unsigned int x = static_cast<int>(mx) + x_offset[j];
        unsigned int y = static_cast<int>(my) + y_offset[j];
        auto value1 = global_costmap_->getCost(x, y);
        auto condition1 =
            ((value1 == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || (value1 == nav2_costmap_2d::LETHAL_OBSTACLE));
        if (condition1) {
          return true;
        }
      }
    }
  }
  return false;
}
bool Nav2SingleNodeNavigator::isCurrentLocalStuck(double search_range) {
  geometry_msgs::msg::PoseStamped local_pose;
  if (!local_costmap_ros_->getRobotPose(local_pose)) {
    return false;
  }
  unsigned int mx, my;
  int grid_search_range =
      std::max(static_cast<int>(search_range / local_costmap_ros_->getCostmap()->getResolution()), 1);
  if (local_costmap_ros_->getCostmap()->worldToMap(local_pose.pose.position.x, local_pose.pose.position.y, mx, my)) {
    for (int i = 1; i <= grid_search_range; ++i) {
      std::vector<int> x_offset = {0, 0, 0, -i, i, i, -i, i, -i};
      std::vector<int> y_offset = {0, i, -i, 0, 0, i, -i, -i, i};
      for (int j = 0; j < 9; ++j) {
        unsigned int x = static_cast<int>(mx) + x_offset[j];
        unsigned int y = static_cast<int>(my) + y_offset[j];
        auto value1 = local_costmap_ros_->getCostmap()->getCost(x, y);
        auto condition1 =
            ((value1 == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || (value1 == nav2_costmap_2d::LETHAL_OBSTACLE));
        if (condition1) {
          return true;
        }
      }
    }
  }
  return false;
}

bool Nav2SingleNodeNavigator::isGoalCollided() {
  auto goal_pose = navigate_to_pose_goal_->pose;
  global_costmap_ros_->transformPoseToGlobalFrame(goal_pose, goal_pose);
  unsigned int mx, my;
  if (global_costmap_->worldToMap(goal_pose.pose.position.x, goal_pose.pose.position.y, mx, my)) {
    auto value1 = global_costmap_->getCost(mx, my - 1);
    auto value2 = global_costmap_->getCost(mx + 1, my);
    auto value3 = global_costmap_->getCost(mx - 1, my);
    auto value4 = global_costmap_->getCost(mx, my + 1);
    auto condition1 =
        ((value1 == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || (value1 == nav2_costmap_2d::LETHAL_OBSTACLE));
    auto condition2 =
        ((value2 == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || (value2 == nav2_costmap_2d::LETHAL_OBSTACLE));
    auto condition3 =
        ((value3== nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || (value3 == nav2_costmap_2d::LETHAL_OBSTACLE));
    auto condition4 =
        ((value4 == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || (value4 == nav2_costmap_2d::LETHAL_OBSTACLE));
    if (condition1 || condition2 || condition3 || condition4) {
      return true;
    }
  }
  return false;
}
void Nav2SingleNodeNavigator::currentStuckRecoveryDeal() {
  if (fail_re_plan_count_ > 2) {
    updateStatus(NavToPoseStatus::STUCK_RECOVER_FAIL);
    return;
  }
  if (!updateGlobalPose()) return;
  action_client_follow_path_->async_cancel_all_goals();
  unsigned int mx = 0, my = 0, final_x = 0, final_y = 0;
  if (!global_costmap_->worldToMap(global_pose_.pose.position.x, global_pose_.pose.position.y, mx, my)) {
    updateStatus(NavToPoseStatus::STUCK_RECOVER_FAIL);
    return;
  };
  if (mx < 6 || my < 6 || (my > (global_costmap_->getSizeInCellsY() - 6))
      || (mx > (global_costmap_->getSizeInCellsX() - 6))) {
    RCLCPP_WARN(get_logger(), "Robot is very close to the edge of map");
    updateStatus(NavToPoseStatus::STUCK_RECOVER_FAIL);
    return;
  }
  int search_range = static_cast<int>(max_back_dis_ / 0.05);
//  unsigned char thresh = 80;
  bool final_recover_pos = false;
  for (int i = 1; i < search_range; ++i) {
    std::vector<int> x_offset = {0, 0, 0, -i, i, i, -i, i, -i};
    std::vector<int> y_offset = {0, i, -i, 0, 0, i, -i, -i, i};
    unsigned char value = 0;
    for (int j = 0; j < 9; ++j) {
      final_x = static_cast<int>(mx) + x_offset[j];
      final_y = static_cast<int>(my) + y_offset[j];
      for (int k = -2; k < 2; ++k) {
        for (int l = -2; l < 2; ++l) {
          unsigned int sample_x = static_cast<int>(final_x)+l;
          unsigned int sample_y = static_cast<int>(final_y)+k;
          value = std::max(value,global_costmap_->getCost(sample_x, sample_y));
        }
      }
//      RCLCPP_INFO(get_logger(),
//                  "cur: %.2f %.2f    final_x: %u  final_y:  %u value:%u",global_pose_.pose.position.x, global_pose_.pose.position.y,
//                  final_x,
//                  final_y,value);
      if (value < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || value == nav2_costmap_2d::NO_INFORMATION) {
//        RCLCPP_INFO(get_logger(),
//            "cur: %.2f %.2f  mx: %u  my:  %u  final_x: %u  final_y:  %u value:%u",global_pose_.pose.position.x, global_pose_.pose.position.y,
//            mx,
//            my,
//            final_x,
//            final_y,value);
        final_recover_pos = true;
        break;
      }
    }
    if (final_recover_pos) break;
  }
  if (!final_recover_pos) {
    RCLCPP_WARN(get_logger(), "Can't find any movable area for stuck recover!!!");
    updateStatus(NavToPoseStatus::STUCK_RECOVER_FAIL);
    return;
  }
  geometry_msgs::msg::PoseStamped pose_stamped = global_pose_;
  global_costmap_->mapToWorld(final_x, final_y, pose_stamped.pose.position.x, pose_stamped.pose.position.y);
  nav2_util::transformPoseInTargetFrame(pose_stamped, pose_stamped, *tf_, "base_link", 0.2);
  auto dis = std::hypot(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
  auto angle = std::atan2(pose_stamped.pose.position.y, pose_stamped.pose.position.x);
  angle = tf2NormalizeAngle(angle);
  double vel = 0.1;
  double angular_vel = 0;
  auto abs_angle = abs(angle);
  if (abs_angle < 1.5) {
    vel = max_back_vel_;
  } else {
    vel = -max_back_vel_;
  }
  if (dis<0.05) vel=0;
  if (abs_angle > 2.5 || abs_angle < 0.5) {
    angular_vel = 0.;
  } else {
    if (angle < 1.5 && angle > 0) {
      angular_vel = max_back_angular_vel_;
    } else if (angular_vel > -1.5 && angle < 0) {
      angular_vel = -max_back_angular_vel_;
    } else if (angle > 0 && angle > 1.5) {
      angular_vel = -max_back_angular_vel_;
    } else if (angular_vel < -1.5 && angle < 0) {
      angular_vel = max_back_angular_vel_;
    }
  }
  auto count = static_cast<int>(((dis)) * 10 / max_back_vel_);
  geometry_msgs::msg::Twist twist;
  twist.angular.z = angular_vel;
  twist.linear.x = vel;
  int odom_no_move_count = 0;
  RCLCPP_INFO(get_logger(),"Recover pos in base link: x: %.2f  y: %.2f target vel: %.2f  angular: %.2f",
              pose_stamped.pose.position.x,pose_stamped.pose.position.y,vel,angular_vel);
//  RCLCPP_INFO(get_logger(),
//              "Find relative recover position:x:%.2f  y: %.2f  mx: %u  my:  %u  final_x: %u  final_y:  %u angular_vel: %.2f vel: %.2f  angle: %.2f",
//              pose_stamped.pose.position.x,
//              pose_stamped.pose.position.y,
//              mx,
//              my,
//              final_x,
//              final_y,
//              angular_vel,
//              vel,
//              angle);
  for (int i = 0; i < count; ++i) {
    if (rclcpp::ok()) {
      vel_publisher_->publish(twist);
      std::this_thread::sleep_for(100ms);
      if (abs(odom_sub_->getTwist().x) <= 0.001) {
        odom_no_move_count++;
      }
    }
  }
  twist.angular.z = 0;
  twist.linear.x = 0;
  vel_publisher_->publish(twist);
  fail_re_plan_count_ += 1;
  if (std::abs(odom_no_move_count - count) < 2 && count > 10) {
    updateStatus(NavToPoseStatus::ODOM_NO_MOVE);
  } else {
    RCLCPP_INFO(get_logger(), "Try recover behavior successfully!!!");
    updateStatus(NavToPoseStatus::GOAL_UPDATED);
  }
}

}  // namespace nav2_non_bt_navigator
