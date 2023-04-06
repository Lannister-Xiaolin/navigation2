/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "nav2_costmap_2d/copy_layer.hpp"

#include <algorithm>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/geometry_utils.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::CopyLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d {

CopyLayer::CopyLayer()
    : map_buffer_(nullptr) {
}

CopyLayer::~CopyLayer() {
}

void
CopyLayer::onInitialize() {
  global_frame_ = layered_costmap_->getGlobalFrameID();
  getParameters();
  if (track_unknown_space_) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }
  rclcpp::QoS map_qos(10);  // initialize to default
  if (map_subscribe_transient_local_) {
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);
  }

  RCLCPP_INFO(
      logger_,
      "Subscribing to the map topic (%s) with %s durability",
      map_topic_.c_str(),
      map_subscribe_transient_local_ ? "transient local" : "volatile");

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  matchSize();

  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&CopyLayer::incomingMap, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "Copy layer subscribe to topic:  %s", map_sub_->get_topic_name());
}

void
CopyLayer::activate() {
}

void
CopyLayer::deactivate() {
}

void
CopyLayer::reset() {
//  has_updated_data_ = true;
  current_ = false;
  map_received_ = false;
}

void
CopyLayer::getParameters() {
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("subscribe_to_updates", rclcpp::ParameterValue(false));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  declareParameter("copy_costmap_topic", rclcpp::ParameterValue("/local_costmap/costmap"));
  declareParameter("only_keep_copy_window", rclcpp::ParameterValue(false));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
  std::string private_map_topic, global_map_topic;
  node->get_parameter(name_ + "." + "copy_costmap_topic", private_map_topic);
  node->get_parameter("copy_costmap_topic", global_map_topic);
  if (!private_map_topic.empty()) {
    map_topic_ = private_map_topic;
  } else {
    map_topic_ = global_map_topic;
  }
  node->get_parameter(
      name_ + "." + "map_subscribe_transient_local", map_subscribe_transient_local_);
  node->get_parameter("track_unknown_space", track_unknown_space_);
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
  node->get_parameter("unknown_cost_value", unknown_cost_value_);
  node->get_parameter("trinary_costmap", trinary_costmap_);
  node->get_parameter("transform_tolerance", temp_tf_tol);
  node->get_parameter("only_keep_copy_window", only_keep_copy_window_);
  // Enforce bounds
//  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  map_received_ = false;

  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

}

void
CopyLayer::processMap(const nav_msgs::msg::OccupancyGrid &new_map, double *min_x,
                      double *min_y,
                      double *max_x,
                      double *max_y) {
//  RCLCPP_ERROR(logger_, "start----------------------CopyLayer: Process map   %.2f  %.2f  %.2f  %.2f ",*min_x,*min_y,*max_x,*max_y);

  if (size_x_ == 0 || size_y_ == 0) return;
//  if (new_map.header.frame_id==global_frame_) {
//    RCLCPP_WARN(logger_,"N")
//  }
  useExtraBounds(min_x, min_y, max_x, max_y);
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
        tf_->lookupTransform(global_frame_, new_map.header.frame_id, new_map.header.stamp, transform_tolerance_);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_, "StaticLayer: %s", ex.what());
//    update_in_progress_.store(false);
    return;
  }
  if (only_keep_copy_window_) {
    resetMaps();
  }
  // Copy map data given proper transformations
  tf2::Transform tf2_transform;
  tf2::fromMsg(transform.transform, tf2_transform);

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  for (unsigned int kY = 0; kY < new_map.info.height; ++kY) {
    for (unsigned int kX = 0; kX < new_map.info.width; ++kX) {
      auto wx = new_map.info.origin.position.x + (kX + 0.5) * resolution_;
      auto wy = new_map.info.origin.position.y + (kY + 0.5) * resolution_;
      tf2::Vector3 p(wx, wy, 0);
      p = tf2_transform * p;
      unsigned int current_mx, current_my;
      if (worldToMap(p.x(), p.y(), current_mx, current_my)) {
        //todo just copy all value from incoming map and for old value, we can using max or mean to update, but rewrite
        // for current value
        costmap_[getIndex(current_mx, current_my)] =
            new_map.data[kX + kY * new_map.info.width] == 100 ? LETHAL_OBSTACLE : FREE_SPACE;
        touch(p.x(), p.y(), min_x, min_y, max_x, max_y);
      }
    }
  }
//  map_frame_ = new_map.header.frame_id;
//  has_updated_data_ = true;
  current_ = true;
//  RCLCPP_ERROR(logger_, "end----------------------CopyLayer: Process map   %.2f  %.2f  %.2f  %.2f ",*min_x,*min_y,*max_x,*max_y);

}


void
CopyLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
//  if (update_in_progress_.load()) {
  map_buffer_ = new_map;
  map_received_ = true;
//  }
}

void
CopyLayer::updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/, double *min_x,
    double *min_y,
    double *max_x,
    double *max_y) {
  if (!map_received_) {
    return;
  }
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // If there is a new available map, load it.
  if (map_buffer_) {
    processMap(*map_buffer_, min_x, min_y, max_x, max_y);
    map_buffer_ = nullptr;
  }

}

void
CopyLayer::updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid,
    int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_) {
    return;
  }
  if (!map_received_) {
    static int count = 0;
    // throttle warning down to only 1/10 message rate
    if (++count == 10) {
      RCLCPP_WARN(logger_, "Can't update static costmap layer, no map received");
      count = 0;
    }
    return;
  }
//  if (!use_maximum_) {
//    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
//  } else {
  //todo whether need to using different combination method ?
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

}

}  // namespace nav2_costmap_2d
