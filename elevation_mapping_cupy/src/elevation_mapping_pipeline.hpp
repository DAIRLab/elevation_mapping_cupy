//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#pragma once

// STL
#include <iostream>
#include <algorithm>
#include <mutex>
#include <set>

// Eigen
#include <Eigen/Dense>

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include "elevation_mapping_wrapper.hpp"

namespace py = pybind11;

namespace elevation_mapping_cupy {

struct ElevationMappingStatistics {
  uint64_t pointcloud_preprocess_nanoseconds;
  uint64_t gridmap_update_nanoseconds;
  uint64_t total_callback_nanoseconds;
};

class ElevationMappingNode {
 public:
  ElevationMappingNode();

 private:
  void readParameters();
  void stanceFootCallback(const std_msgs::String& msg);
  void applyFootCropBoxCassie(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              const tf::StampedTransform& tf_X_LC,
                              const tf::StampedTransform& tf_X_RC);
  void preprocessPointcloudCassie(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
  void pointcloudCallback(const sensor_msgs::PointCloud2& cloud);
  bool getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
  bool checkSafety(elevation_map_msgs::CheckSafety::Request& request, elevation_map_msgs::CheckSafety::Response& response);
  bool initializeMap(elevation_map_msgs::Initialize::Request& request, elevation_map_msgs::Initialize::Response& response);
  bool clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool clearMapWithInitializer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void updatePose(const ros::TimerEvent&);
  void updateVariance(const ros::TimerEvent&);
  void updateTime(const ros::TimerEvent&);
  void updateGridMap(const ros::TimerEvent&);
  void initializeWithTF();
  void setDynamicFootCropBoxParams(pcl::CropBox<pcl::PointXYZ>* crop_box, const Eigen::Affine3d& X_FC);


  // map topics info
  std::vector<std::vector<std::string>> map_topics_;
  std::vector<std::vector<std::string>> map_layers_;
  std::vector<std::vector<std::string>> map_basic_layers_;
  std::set<std::string> map_layers_all_;
  std::set<std::string> map_layers_sync_;
  std::vector<double> map_fps_;
  std::set<double> map_fps_unique_;

  std::vector<std::string> initialize_frame_id_;
  std::vector<double> initialize_tf_offset_;
  std::string initializeMethod_;
  std::string stanceFrameTopic_;

  Eigen::Vector3d lowpassPosition_;
  Eigen::Vector4d lowpassOrientation_;

  std::string stanceFrameid_{};

  std::mutex mapMutex_;  // protects gridMap_
  grid_map::GridMap gridMap_;
  std::atomic_bool isGridmapUpdated_;  // needs to be atomic (read is not protected by mapMutex_)

  std::mutex errorMutex_; // protects positionError_, and orientationError_
  double positionError_;
  double orientationError_;

  double positionAlpha_;
  double orientationAlpha_;

  /* DAIR filter params */
  double y_min_;
  double foot_mask_x_extent_;
  double foot_mask_y_extent_;
  double depth_min_;
  double depth_max_;
  double stance_foot_drift_thresh_;
  const std::string left_toe_frame_ = "toe_left";
  const std::string right_toe_frame_ = "toe_right";
  const Eigen::Vector3d toe_front_{-0.0457, 0.112, 0.0};
  const Eigen::Vector3d toe_rear_{0.088, 0.0, 0.0};
  const Eigen::Vector3d crop_box_origin_{-0.2, 0.0, 0.2};
  const Eigen::Vector3d toe_mid_ = 0.5 * (toe_front_ + toe_rear_);
  Eigen::Vector3d pointcloud_bias_{0.0, 0.0, 0.0};

  bool useInitializerAtStart_;
  double initializeTfGridSize_;
};

}  // namespace elevation_mapping_cupy
