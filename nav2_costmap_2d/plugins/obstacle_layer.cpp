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
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;

namespace nav2_costmap_2d {

ObstacleLayer::~ObstacleLayer() {
  for (auto &notifier : observation_notifiers_) {
    notifier.reset();
  }
  if (count_costmap_ != nullptr) {
    delete[] count_costmap_;
    count_costmap_ = NULL;
  }

}

void ObstacleLayer::onInitialize() {
  bool track_unknown_space;
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  RCLCPP_INFO(
      logger_,
      "Subscribed to Topics: %s", topics_string.c_str());

  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  ObstacleLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
    declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
    declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
    declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
    declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "enable_label_cloud", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "enable_lower_clear", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "only_obstacle_label", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "label_with_min_height_mode", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "hit_count_mode", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "enable_small_area_restrict", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "small_area_edge_len", rclcpp::ParameterValue(2.5));
    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(
        name_ + "." + source + "." + "observation_persistence",
        observation_keep_time);
    node->get_parameter(
        name_ + "." + source + "." + "expected_update_rate",
        expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "marking", marking);
    node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      RCLCPP_FATAL(
          logger_,
          "Only topics that use point cloud2s or laser scans are currently supported");
      throw std::runtime_error(
          "Only topics that use point cloud2s or laser scans are currently supported");
    }

    // get the obstacle range for the sensor
    double obstacle_max_range, obstacle_min_range;
    node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
    node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

    // get the raytrace ranges for the sensor
    double raytrace_max_range, raytrace_min_range;
    node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
    node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);
    bool enable_label_cloud, enable_lower_clear, only_obstacle_label, label_with_min_height_mode;
    node->get_parameter(name_ + "." + source + "." + "enable_label_cloud", enable_label_cloud);
    node->get_parameter(name_ + "." + source + "." + "enable_lower_clear", enable_lower_clear);
    node->get_parameter(name_ + "." + source + "." + "only_obstacle_label", only_obstacle_label);
    node->get_parameter(name_ + "." + source + "." + "label_with_min_height_mode", label_with_min_height_mode);
    node->get_parameter(name_ + "." + source + "." + "hit_count_mode", hit_count_mode_);
    node->get_parameter(name_ + "." + source + "." + "enable_small_area_restrict", enable_small_area_restrict_);
    node->get_parameter(name_ + "." + source + "." + "small_area_edge_len", small_area_edge_len_);
    inner_radius_ = static_cast<int>((small_area_edge_len_ - 1) / 0.05);
    outer_radius_ = static_cast<int>(small_area_edge_len_ / 0.05);
    outer_clear_count_ = 0;
    RCLCPP_DEBUG(
        logger_,
        "Creating an observation buffer for source %s, topic %s, frame %s",
        source.c_str(), topic.c_str(),
        sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        std::shared_ptr<ObservationBuffer
        >(
            new ObservationBuffer(
                node,
                topic,
                observation_keep_time,
                expected_update_rate,
                min_obstacle_height,
                max_obstacle_height,
                obstacle_max_range,
                obstacle_min_range,
                raytrace_max_range,
                raytrace_min_range,
                *tf_,
                global_frame_,
                sensor_frame,
                tf2::durationFromSec(transform_tolerance),
                enable_label_cloud,
                enable_lower_clear,
                only_obstacle_label,
                label_with_min_height_mode)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }
    if (hit_count_mode_) RCLCPP_INFO(logger_, "-----------Using hit count mode---------------");
    RCLCPP_DEBUG(
        logger_,
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(),
        global_frame_.c_str(), expected_update_rate, observation_keep_time);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // create a callback for the topic
    if (data_type == "LaserScan") {
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
          sub(
          new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(
              rclcpp_node_, topic, custom_qos_profile));
      sub->unsubscribe();

      std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
          filter(
          new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(
              *sub, *tf_, global_frame_, 50, rclcpp_node_, tf2::durationFromSec(transform_tolerance)));

      if (inf_is_valid) {
        filter->registerCallback(
            std::bind(
                &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
                observation_buffers_.back()));

      } else {
        filter->registerCallback(
            std::bind(
                &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
                observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);

      observation_notifiers_.push_back(filter);
      observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));

    } else {
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
          sub(
          new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
              rclcpp_node_, topic, custom_qos_profile));
      sub->unsubscribe();

      if (inf_is_valid) {
        RCLCPP_WARN(
            logger_,
            "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>
          filter(
          new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
              *sub, *tf_, global_frame_, 50, rclcpp_node_, tf2::durationFromSec(transform_tolerance)));

      filter->registerCallback(
          std::bind(
              &ObstacleLayer::pointCloud2Callback, this, std::placeholders::_1,
              observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }
}

void
ObstacleLayer::laserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> &buffer) {
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(
        logger_,
        "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
        global_frame_.c_str(),
        ex.what());
    projector_.projectLaser(*message, cloud);
  } catch (std::runtime_error &ex) {
    RCLCPP_WARN(
        logger_,
        "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
        " Ignore this message. what(): %s",
        ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::laserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> &buffer) {
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(
        logger_,
        "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
        global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  } catch (std::runtime_error &ex) {
    RCLCPP_WARN(
        logger_,
        "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
        " Ignore this message. what(): %s",
        ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<ObservationBuffer> &buffer) {
  // buffer the point cloudFailed to make progressFailed to make progress
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void
ObstacleLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, double *min_x,
    double *min_y, double *max_x, double *max_y) {
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;
  if (hit_count_mode_) {
    count_costmap_ = new unsigned char[size_x_ * size_y_];
    memset(count_costmap_, FREE_SPACE, size_x_ * size_y_ * sizeof(unsigned char));
  }
  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    emptyFreespace(clearing_observations[i]);
  }
  unsigned int max_inner_x = size_x_, min_inner_x = 0, max_inner_y = size_y_, min_inner_y = 0;
  unsigned int max_outer_x = size_x_, min_outer_x = 0, max_outer_y = size_y_, min_outer_y = 0;
  unsigned int robot_mx, robot_my;
  worldToMap(robot_x, robot_y, robot_mx, robot_my);
  if (enable_small_area_restrict_) {
    max_inner_x = std::min(int(robot_mx) + inner_radius_, int(size_x_));
    min_inner_x = std::max(int(robot_mx) - inner_radius_, 0);
    max_inner_y = std::min(int(robot_my) + inner_radius_, int(size_y_));
    min_inner_y = std::max(int(robot_my) - inner_radius_, 0);
    max_outer_x = std::min(int(robot_mx) + outer_radius_, int(size_x_));
    min_outer_x = std::max(int(robot_mx) - outer_radius_, 0);
    max_outer_y = std::min(int(robot_my) + outer_radius_, int(size_y_));
    min_outer_y = std::max(int(robot_my) - outer_radius_, 0);
    outer_clear_count_ += 1;
  }
  if (hit_count_mode_) {
    RCLCPP_INFO_ONCE(logger_, "-----------Using hit count mode---------------");
    std::vector<unsigned int> indexes;
    for (std::vector<Observation>::const_iterator it = observations.begin();
         it != observations.end(); ++it) {
      const Observation &obs = *it;

      const sensor_msgs::msg::PointCloud2 &cloud = *(obs.cloud_);

      double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
      double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        // if the obstacle is too high or too far away from the robot we won't add it
        if (pz > max_obstacle_height_) {
          RCLCPP_DEBUG(logger_, "The point is too high");
          continue;
        }

        // compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist =
            (px -
                obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
                (pz - obs.origin_.z) * (pz - obs.origin_.z);

        // if the point is far enough away... we won't consider it
        if (sq_dist >= sq_obstacle_max_range) {
          RCLCPP_DEBUG(logger_, "The point is too far away");
          continue;
        }

        // if the point is too close, do not conisder it
        if (sq_dist < sq_obstacle_min_range) {
          RCLCPP_DEBUG(logger_, "The point is too close");
          continue;
        }

        // now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my)) {
          RCLCPP_DEBUG(logger_, "Computing map coords failed");
          continue;
        }

        unsigned int index = getIndex(mx, my);
        if (count_costmap_[index] == CLEAR_COUNT) {
          count_costmap_[index] = HIT_SUB_CLEAR_COUNT;
        } else if (count_costmap_[index] == FREE_SPACE) {
          count_costmap_[index] = HIT_COUNT;
        }
        touch(px, py, min_x, min_y, max_x, max_y);
      }
      for (unsigned int iy = min_inner_y; iy < max_inner_y; ++iy) {
        for (unsigned int ix = min_inner_x; ix < max_inner_x; ++ix) {
          auto index = getIndex(ix, iy);
          if (count_costmap_[index] == FREE_SPACE) continue;
          if (costmap_[index] == NO_INFORMATION) {
            costmap_[index] = count_costmap_[index] == CLEAR_COUNT ? FREE_SPACE : count_costmap_[index];
          } else if (count_costmap_[index] == CLEAR_COUNT) {
            costmap_[index] = costmap_[index] > CLEAR_COUNT ? costmap_[index] - CLEAR_COUNT : FREE_SPACE;
          } else {
            costmap_[index] = std::min(static_cast<unsigned char>(costmap_[index] + count_costmap_[index]), MAX_COUNT);
          }
        }
      }
    }
  } else {
    // place the new obstacles into a priority queue... each with a priority of zero to begin with
    for (std::vector<Observation>::const_iterator it = observations.begin();
         it != observations.end(); ++it) {
      const Observation &obs = *it;

      const sensor_msgs::msg::PointCloud2 &cloud = *(obs.cloud_);

      double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
      double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        // if the obstacle is too high or too far away from the robot we won't add it
        if (pz > max_obstacle_height_) {
          RCLCPP_DEBUG(logger_, "The point is too high");
          continue;
        }

        // compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist =
            (px -
                obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
                (pz - obs.origin_.z) * (pz - obs.origin_.z);

        // if the point is far enough away... we won't consider it
        if (sq_dist >= sq_obstacle_max_range) {
          RCLCPP_DEBUG(logger_, "The point is too far away");
          continue;
        }

        // if the point is too close, do not conisder it
        if (sq_dist < sq_obstacle_min_range) {
          RCLCPP_DEBUG(logger_, "The point is too close");
          continue;
        }

        // now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my)) {
          RCLCPP_DEBUG(logger_, "Computing map coords failed");
          continue;
        }

        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
        touch(px, py, min_x, min_y, max_x, max_y);
      }
    }
  }
  delete[] count_costmap_;
  count_costmap_ = NULL;
  if (enable_small_area_restrict_ && (outer_clear_count_ % 2 == 0)) {
    touch(robot_x + small_area_edge_len_, robot_y + small_area_edge_len_, min_x, min_y, max_x, max_y);
    touch(robot_x - small_area_edge_len_, robot_y - small_area_edge_len_, min_x, min_y, max_x, max_y);
    for (unsigned int iy = min_outer_y; iy < max_outer_y; ++iy) {
      for (unsigned int ix = min_outer_x; ix < max_outer_x; ++ix) {
        auto index = getIndex(ix, iy);
        if (ix < min_inner_x || ix > max_inner_x || iy < min_inner_y || iy > max_inner_y) {
          costmap_[index] = FREE_SPACE;
        }
      }
    }
  }
  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double *min_x, double *min_y,
    double *max_x,
    double *max_y) {
  if (!footprint_clearing_enabled_) { return; }
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
ObstacleLayer::updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
    int max_i,
    int max_j) {
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) {
    case 0:  // Overwrite
      if (hit_count_mode_) {
        updateWithOverwriteByCount(master_grid, min_i, min_j, max_i, max_j);
      } else {
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      }

      break;
    case 1:  // Maximum
      if (hit_count_mode_) { updateWithMaxByCount(master_grid, min_i, min_j, max_i, max_j); }
      else {
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      }
      break;
    default:  // Nothing
      break;
  }
}

void
ObstacleLayer::addStaticObservation(
    nav2_costmap_2d::Observation &obs,
    bool marking, bool clearing) {
  if (marking) {
    static_marking_observations_.push_back(obs);
  }
  if (clearing) {
    static_clearing_observations_.push_back(obs);
  }
}

void
ObstacleLayer::clearStaticObservations(bool marking, bool clearing) {
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

bool
ObstacleLayer::getMarkingObservations(std::vector<Observation> &marking_observations) const {
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(
      marking_observations.end(),
      static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool
ObstacleLayer::getClearingObservations(std::vector<Observation> &clearing_observations) const {
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(
      clearing_observations.end(),
      static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void
ObstacleLayer::raytraceFreespace(
    const Observation &clearing_observation, double *min_x,
    double *min_y,
    double *max_x,
    double *max_y) {
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 &cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(
        logger_,
        "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
        "The costmap cannot raytrace for it.",
        ox, oy,
        origin_x_, origin_y_,
        origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
    if (hit_count_mode_) {
      MarkCell marker(count_costmap_, CLEAR_COUNT);
      // and finally... we can execute our trace to clear obstacles along that line
      raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);
    } else {
      MarkCell marker(costmap_, FREE_SPACE);
      // and finally... we can execute our trace to clear obstacles along that line
      raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);
    }

    updateRaytraceBounds(
        ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
        clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
        max_y);
  }
}

void ObstacleLayer::emptyFreespace(const Observation &clearing_observation) {
//  double ox = clearing_observation.origin_.x;
//  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 &cloud = *(clearing_observation.empty_cloud_);
  RCLCPP_DEBUG(logger_, "-----------Start to empty cloud with valid size:  %u", cloud.width);
  if (cloud.data.size() < 2) return;

  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  unsigned long count = 0;
  double sq_max_range = clearing_observation.raytrace_max_range_ * clearing_observation.raytrace_max_range_;
  double sq_min_range = clearing_observation.raytrace_min_range_ * clearing_observation.raytrace_min_range_;
  if (hit_count_mode_) {
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double wx = *iter_x;
      double wy = *iter_y;
      // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      unsigned int x1, y1;
      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist =
          (wx -
              clearing_observation.origin_.x) * (wx - clearing_observation.origin_.x)
              + (wy - clearing_observation.origin_.y) * (wy - clearing_observation.origin_.y);
      if (sq_dist<sq_min_range ||sq_dist>sq_max_range) continue;
      // check for legality just in case
      if (!worldToMap(wx, wy, x1, y1)) {
        continue;
      }
      unsigned int index = getIndex(x1, y1);
      //todo 直接新建临时变量，不再去改动原始的costmap类
      count_costmap_[index] = CLEAR_COUNT;
      count += 1;
    }
  } else {
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double wx = *iter_x;
      double wy = *iter_y;
      // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      unsigned int x1, y1;
      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist =
          (wx -
              clearing_observation.origin_.x) * (wx - clearing_observation.origin_.x)
              + (wy - clearing_observation.origin_.y) * (wy - clearing_observation.origin_.y);
      if (sq_dist<sq_min_range ||sq_dist>sq_max_range) continue;
      // check for legality just in case
      if (!worldToMap(wx, wy, x1, y1)) {
        continue;
      }
      unsigned int index = getIndex(x1, y1);
      costmap_[index] = FREE_SPACE;
      count += 1;
    }
  }

  RCLCPP_DEBUG(logger_, "-----------Finish to empty cloud with valid size:  %lu", count);
}

void
ObstacleLayer::activate() {
  for (auto &notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
  resetBuffersLastUpdated();
}

void
ObstacleLayer::deactivate() {
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void
ObstacleLayer::updateRaytraceBounds(
    double ox, double oy, double wx, double wy, double max_range, double min_range,
    double *min_x, double *min_y, double *max_x, double *max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  if (full_distance < min_range) {
    return;
  }
  double scale = std::min(1.0, max_range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::reset() {
  resetMaps();
  resetBuffersLastUpdated();
  current_ = false;
  was_reset_ = true;
}

void
ObstacleLayer::resetBuffersLastUpdated() {
  for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i]) {
      observation_buffers_[i]->resetLastUpdated();
    }
  }
}

}  // namespace nav2_costmap_2d
