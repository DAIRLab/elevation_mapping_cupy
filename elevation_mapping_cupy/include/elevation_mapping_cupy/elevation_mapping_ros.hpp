//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#pragma once

// STL
#include <iostream>
#include <mutex>

// Eigen
#include <Eigen/Dense>

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding

// ROS
#include <std_msgs/String.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <elevation_map_msgs/CheckSafety.h>
#include <elevation_map_msgs/Initialize.h>

#include "elevation_mapping_cupy/elevation_mapping_wrapper.hpp"

namespace py = pybind11;

namespace elevation_mapping_cupy {

class ElevationMappingNode {
 public:
  ElevationMappingNode(ros::NodeHandle& nh);

 private:
  void readParameters();
  void setupMapPublishers();
  void stanceFootCallback(const std_msgs::String& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2& cloud);
  void publishAsPointCloud(const grid_map::GridMap& map) const;
  bool getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
  bool checkSafety(elevation_map_msgs::CheckSafety::Request& request, elevation_map_msgs::CheckSafety::Response& response);
  bool initializeMap(elevation_map_msgs::Initialize::Request& request, elevation_map_msgs::Initialize::Response& response);
  bool clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool clearMapWithInitializer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool setPublishPoint(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  void updatePose(const ros::TimerEvent&);
  void updateVariance(const ros::TimerEvent&);
  void updateTime(const ros::TimerEvent&);
  void updateGridMap(const ros::TimerEvent&);
  void publishNormalAsArrow(const grid_map::GridMap& map) const;
  void initializeWithTF();
  void publishMapToOdom(double error);
  void publishStatistics(const ros::TimerEvent&);
  void publishMapOfIndex(int index);
  void setDynamicFootCropBoxParams(pcl::CropBox<pcl::PointXYZ>* crop_box, const Eigen::Affine3d& X_FC);

  visualization_msgs::Marker vectorToArrowMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const int id) const;
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> pointcloudSubs_;
  std::vector<ros::Publisher> mapPubs_;
  ros::Subscriber stanceFootSub_;
  tf::TransformBroadcaster tfBroadcaster_;
  ros::Publisher alivePub_;
  ros::Publisher pointPub_;
  ros::Publisher pointPubFilter_;
  ros::Publisher normalPub_;
  ros::Publisher statisticsPub_;
  ros::ServiceServer rawSubmapService_;
  ros::ServiceServer clearMapService_;
  ros::ServiceServer clearMapWithInitializerService_;
  ros::ServiceServer initializeMapService_;
  ros::ServiceServer setPublishPointService_;
  ros::ServiceServer checkSafetyService_;
  ros::Timer updateVarianceTimer_;
  ros::Timer updateTimeTimer_;
  ros::Timer updatePoseTimer_;
  ros::Timer updateGridMapTimer_;
  ros::Timer publishStatisticsTimer_;
  ros::Time lastStatisticsPublishedTime_;
  tf::TransformListener transformListener_;
  ElevationMappingWrapper map_;
  std::string mapFrameId_;
  std::string correctedMapFrameId_;
  std::string baseFrameId_;

  // map topics info
  std::vector<std::vector<std::string>> map_topics_;
  std::vector<std::vector<std::string>> map_layers_;
  std::vector<std::vector<std::string>> map_basic_layers_;
  std::set<std::string> map_layers_all_;
  std::set<std::string> map_layers_sync_;
  std::vector<double> map_fps_;
  std::set<double> map_fps_unique_;
  std::vector<ros::Timer> mapTimers_;

  std::vector<std::string> initialize_frame_id_;
  std::vector<double> initialize_tf_offset_;
  std::string initializeMethod_;
  std::string stanceFrameTopic_;

  Eigen::Vector3d lowpassPosition_;
  Eigen::Vector4d lowpassOrientation_;

  std::mutex stanceMutex_;
  std::string stanceFrameid_ = "";

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
  const std::string left_toe_frame_ = "toe_left";
  const std::string right_toe_frame_ = "toe_right";
  const Eigen::Vector3d left_hip_yaw_origin_in_pelvis_{0.021, 0.135, 0.0};
  const Eigen::Vector3d right_hip_yaw_origin_in_pelvis_{0.021, -0.135, 0.0};
  const Eigen::Vector3d toe_front_{-0.0457, 0.112, 0.0};
  const Eigen::Vector3d toe_rear_{0.088, 0.0, 0.0};
  const Eigen::Vector3d crop_box_origin_{-0.2, 0.0, 0.2};
  const Eigen::Vector3d toe_mid_ = toe_front_ + toe_rear_;




  double recordableFps_;
  std::atomic_bool enablePointCloudPublishing_;
  bool enableNormalArrowPublishing_;
  bool enableDriftCorrectedTFPublishing_;
  bool useInitializerAtStart_;
  double initializeTfGridSize_;
  std::atomic_int pointCloudProcessCounter_;
};

}  // namespace elevation_mapping_cupy
