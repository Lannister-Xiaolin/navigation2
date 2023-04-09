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
 *********************************************************************/
#include "nav2_costmap_2d/observation_buffer.hpp"

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

#include "tf2/convert.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std::chrono_literals;

namespace nav2_costmap_2d {
ObservationBuffer::ObservationBuffer(
    const nav2_util::LifecycleNode::WeakPtr &parent,
    std::string topic_name,
    double observation_keep_time,
    double expected_update_rate,
    double min_obstacle_height,
    double max_obstacle_height,
    double obstacle_max_range,
    double obstacle_min_range,
    double raytrace_max_range,
    double raytrace_min_range,
    tf2_ros::Buffer &tf2_buffer,
    std::string global_frame,
    std::string sensor_frame,
    tf2::Duration tf_tolerance,
    bool enable_label_cloud,
    bool enable_lower_clear,
    bool only_obstacle_label,
    bool label_with_min_height_mode)
    : tf2_buffer_(tf2_buffer),
      observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
      expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
      global_frame_(global_frame),
      sensor_frame_(sensor_frame),
      topic_name_(topic_name),
      min_obstacle_height_(min_obstacle_height),
      max_obstacle_height_(max_obstacle_height),
      obstacle_max_range_(obstacle_max_range),
      obstacle_min_range_(obstacle_min_range),
      raytrace_max_range_(raytrace_max_range),
      raytrace_min_range_(
          raytrace_min_range),
      tf_tolerance_(tf_tolerance),
      enable_label_cloud_(enable_label_cloud),
      enable_lower_clear_(enable_lower_clear),
      only_obstacle_label_(only_obstacle_label),
      label_with_min_height_mode_(label_with_min_height_mode) {
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_updated_ = node->now();
  local_footprint_origin_.header.frame_id = "base_footprint";
  local_footprint_origin_.point.x = 0;
  local_footprint_origin_.point.y = 0;
  local_footprint_origin_.point.z = 0;
  if (enable_lower_clear_) {
    RCLCPP_INFO(logger_, "+++++++++++++++++++++++Enable lower cloud clear++++++++++++++++++++++++++++!!!");
  }
  if (enable_label_cloud_) {
    RCLCPP_INFO(logger_, "+++++++++++++++++++++++Enable label cloud ++++++++++++++++++++++++++++!!!");
  }
  if (only_obstacle_label_) {
    RCLCPP_WARN(logger_, "+++++++++++++++++++++++Only using labeled point cloud ++++++++++++++++++++++++++++!!!");
  }
  if (label_with_min_height_mode_) {
    RCLCPP_WARN(logger_,
                "+++++++++++++++++++++++Using labeled point cloud and height than specified height++++++++++++++++++++++++++++!!!");
  }
}

ObservationBuffer::~ObservationBuffer() {
}

void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 &cloud) {
  geometry_msgs::msg::PointStamped global_origin, global_footprint_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly
  // or whether we should get it from the cloud
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try {
    // given these observations come from sensors...
    // we'll need to store the origin pt of the sensor
    geometry_msgs::msg::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    local_footprint_origin_.header.stamp = cloud.header.stamp;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);
    tf2_buffer_.transform(local_footprint_origin_, global_footprint_origin, global_frame_, tf_tolerance_);
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    // make sure to pass on the raytrace/obstacle range
    // of the observation buffer to the observations
    observation_list_.front().raytrace_max_range_ = raytrace_max_range_;
    observation_list_.front().raytrace_min_range_ = raytrace_min_range_;
    observation_list_.front().obstacle_max_range_ = obstacle_max_range_;
    observation_list_.front().obstacle_min_range_ = obstacle_min_range_;
    observation_list_.front().enable_label_cloud_ = enable_label_cloud_;
    observation_list_.front().enable_lower_clear_ = enable_lower_clear_;
    sensor_msgs::msg::PointCloud2 global_frame_cloud;

    // transform the point cloud
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    //todo 清空处理，以及点云处理
    bool point_has_label = false;
    for (auto &field:global_frame_cloud.fields) {
      if (field.name == "label" && enable_label_cloud_) {
        point_has_label = true;
        break;
      }
    }
    if (point_has_label) {
      RCLCPP_INFO_ONCE(logger_, "+++++++++++++++++++++++Point has label fields++++++++++++++++++++++++++++!!!");
    }


    // now we need to remove observations from the cloud that are below
    // or above our height thresholds
    sensor_msgs::msg::PointCloud2 &observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height * global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);

    sensor_msgs::msg::PointCloud2 &empty_obs_cloud = *(observation_list_.front().empty_cloud_);
    empty_obs_cloud.height = global_frame_cloud.height;
    empty_obs_cloud.width = global_frame_cloud.width;
    empty_obs_cloud.fields = global_frame_cloud.fields;
    empty_obs_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    empty_obs_cloud.point_step = global_frame_cloud.point_step;
    empty_obs_cloud.row_step = global_frame_cloud.row_step;
    empty_obs_cloud.is_dense = global_frame_cloud.is_dense;
    sensor_msgs::PointCloud2Modifier empty_modifier(empty_obs_cloud);
    empty_modifier.resize(cloud_size);

    unsigned int point_count = 0, empty_point_count = 0;
    uint8_t no_lawn_label=1;

    // copy over the points that are within our height bounds
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(),
        iter_global_end = global_frame_cloud.data.end();
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    std::vector<unsigned char>::iterator iter_empty_obs = empty_obs_cloud.data.begin();
    if (!point_has_label && enable_lower_clear_) {
      RCLCPP_INFO_ONCE(logger_, ">>>>>>Lower_clear   %.2f!!!",global_footprint_origin.point.z);
      // 默认肯定开启
      for (; iter_global != iter_global_end; ++iter_z, iter_global +=
                                                           global_frame_cloud.point_step) {
        auto height = (*iter_z) - global_footprint_origin.point.z;
        if (height <= max_obstacle_height_ &&
            height >= min_obstacle_height_) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
          iter_obs += global_frame_cloud.point_step;
          ++point_count;
        } else if (height < min_obstacle_height_) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_empty_obs);
          iter_empty_obs += global_frame_cloud.point_step;
          ++empty_point_count;
        }
      }
    }
      // 原始模式全局点云高度不作处理
    else if (!point_has_label && !enable_lower_clear_) {
      RCLCPP_INFO_ONCE(logger_, ">>>>>>Default mode   %.2f!!!",global_footprint_origin.point.z);
      for (; iter_global != iter_global_end; ++iter_z, iter_global +=
                                                           global_frame_cloud.point_step) {
        auto height = (*iter_z) - global_footprint_origin.point.z;
        if (height <= max_obstacle_height_ && height >= min_obstacle_height_) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
          iter_obs += global_frame_cloud.point_step;
          ++point_count;
        }
      }
    } else if (point_has_label && only_obstacle_label_ && label_with_min_height_mode_) {
      RCLCPP_INFO_ONCE(logger_, ">>>>>>Only_obstacle_label and label_with_min_height_mode    %.2f!!!",global_footprint_origin.point.z);
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(global_frame_cloud, "label");
      for (; iter_global != iter_global_end; ++iter_label, ++iter_z, iter_global +=
                                                                         global_frame_cloud.point_step) {
        auto height = (*iter_z) - global_footprint_origin.point.z;
        // Todo 高度过滤需要去掉，比较大的斜坡不好处理
        if ((*iter_label) >= no_lawn_label || height >= min_obstacle_height_) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
          iter_obs += global_frame_cloud.point_step;
          ++point_count;
        }
      }
    } else if (point_has_label && label_with_min_height_mode_) {
      RCLCPP_INFO_ONCE(logger_, "++++label_with_min_height_mode_+++++++++++++++++++point_has_label+++++++++   %.2f!!!",global_footprint_origin.point.z);
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(global_frame_cloud, "label");
      for (; iter_global != iter_global_end; ++iter_label, ++iter_z, iter_global +=
                                                                         global_frame_cloud.point_step) {
        auto height = (*iter_z) - global_footprint_origin.point.z;
        // Todo 高度过滤需要去掉
        if ((*iter_label) >= no_lawn_label || height >= min_obstacle_height_) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
          iter_obs += global_frame_cloud.point_step;
          ++point_count;
        } else if ((*iter_label) < no_lawn_label) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_empty_obs);
          iter_empty_obs += global_frame_cloud.point_step;
          ++empty_point_count;
        }
      }
    } else if (point_has_label && only_obstacle_label_) {
      RCLCPP_INFO_ONCE(logger_, ">>>>>>Only_obstacle_label     %.2f!!!",global_footprint_origin.point.z);
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(global_frame_cloud, "label");
      for (; iter_global != iter_global_end; ++iter_label, ++iter_z, iter_global +=
                                                                         global_frame_cloud.point_step) {
        if ((*iter_label) >= no_lawn_label) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
          iter_obs += global_frame_cloud.point_step;
          ++point_count;
        }
      }
    } else if (point_has_label) {
      RCLCPP_INFO_ONCE(logger_, ">>>>>>Only labeled cloud     %.2f!!!",global_footprint_origin.point.z);
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(global_frame_cloud, "label");
      for (; iter_global != iter_global_end; ++iter_label, ++iter_z, iter_global +=
                                                                         global_frame_cloud.point_step) {
        if ((*iter_label) >= no_lawn_label) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
          iter_obs += global_frame_cloud.point_step;
          ++point_count;
        } else if ((*iter_label) <no_lawn_label) {
          std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_empty_obs);
          iter_empty_obs += global_frame_cloud.point_step;
          ++empty_point_count;
        }
      }
    }



    // resize the cloud for the number of legal points
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    empty_modifier.resize(empty_point_count);
    empty_obs_cloud.header.stamp = cloud.header.stamp;
    empty_obs_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    RCLCPP_DEBUG(logger_, "point_count: %d empty_point_count:%d", observation_cloud.width, empty_obs_cloud.width);
  } catch (tf2::TransformException &ex) {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    RCLCPP_ERROR(
        logger_,
        "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
        sensor_frame_.c_str(),
        cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = clock_->now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(std::vector<Observation> &observations) {
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  std::list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations() {
  if (!observation_list_.empty()) {
    std::list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == rclcpp::Duration(0.0s)) {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
      Observation &obs = *obs_it;
      // check if the observation is out of date... and if it is,
      // remove it and those that follow from the list
      if ((clock_->now() - obs.cloud_->header.stamp) >
          observation_keep_time_) {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const {
  if (expected_update_rate_ == rclcpp::Duration(0.0s)) {
    return true;
  }

  bool current = (clock_->now() - last_updated_) <=
      expected_update_rate_;
  if (!current) {
    RCLCPP_WARN(
        logger_,
        "The %s observation buffer has not been updated for %.2f seconds, "
        "and it should be updated every %.2f seconds.",
        topic_name_.c_str(),
        (clock_->now() - last_updated_).seconds(),
        expected_update_rate_.seconds());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated() {
  last_updated_ = clock_->now();
}
}  // namespace nav2_costmap_2d
