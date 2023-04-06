/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
#ifndef NAV2_COSTMAP_2D__COPY_LAYER_HPP_
#define NAV2_COSTMAP_2D__COPY_LAYER_HPP_

#include <mutex>
#include <string>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d {

/**
 * @class CopyLayer
 * @brief Takes in a costmap(local from global or global from local), and just copy all valid value into current costmap
 * it's used to avoid duplicate calculating (such as density pointcloud)
 */
class CopyLayer : public CostmapLayer {
 public:
  /**
    * @brief Static Layer constructor
    */
  CopyLayer();
  /**
    * @brief Static Layer destructor
    */
  virtual ~CopyLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Activate this layer
   */
  virtual void activate();
  /**
   * @brief Deactivate this layer
   */
  virtual void deactivate();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() { return true; }

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
      double robot_x, double robot_y, double robot_yaw, double *min_x,
      double *min_y, double *max_x, double *max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid,
      int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Match the size of the master costmap
   */
//  virtual void matchSize();

 private:
  /**
   * @brief Get parameters of layer
   */
  void getParameters();

  /**
   * @brief Process a new map coming from a topic
   */
  void processMap(const nav_msgs::msg::OccupancyGrid &new_map, double *min_x,
                  double *min_y,
                  double *max_x,
                  double *max_y);

  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);
//  /**
//   * @brief Callback to update the costmap's map from the map_server (or SLAM)
//   * with an update in a particular area of the map
//   */
//  void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

//  /**
//   * @brief Interpret the value in the static map given on the topic to
//   * convert into costs for the costmap to utilize
//   */
//  unsigned char interpretValue(unsigned char value);

  std::string global_frame_;  ///< @brief The global frame for the costmap
//  std::string map_frame_;  /// @brief frame that map is located in
//
//  bool has_updated_data_{false};
//
//  unsigned int x_{0};
//  unsigned int y_{0};
//  unsigned int width_{0};
//  unsigned int height_{0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_update_sub_;

  // Parameters
  std::string map_topic_;
  bool map_subscribe_transient_local_;
  bool subscribe_to_updates_;
  bool track_unknown_space_;
  //only keep value from incoming topic map area, and set area outside of incoming map area to default value
  bool only_keep_copy_window_;
//  bool use_maximum_;
  unsigned char lethal_threshold_;
  unsigned char unknown_cost_value_;
  bool trinary_costmap_;
  bool map_received_{false};
  tf2::Duration transform_tolerance_;
//  std::atomic<bool> update_in_progress_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_buffer_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__STATIC_LAYER_HPP_
