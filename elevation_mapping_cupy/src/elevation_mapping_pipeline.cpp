//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "elevation_mapping_pipeline.hpp"
#include "elevation_mapping_wrapper.hpp"


// Pybind
#include <pybind11/eigen.h>

// PCL
#include "pcl/common/projection_matrix.h"

// filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <chrono>

namespace elevation_mapping_cupy {

using Eigen::Vector3d;

ElevationMappingNode::ElevationMappingNode()
    : isGridmapUpdated_(false),
      positionError_(0),
      orientationError_(0),
      positionAlpha_(0.1),
      orientationAlpha_(0.1) {

  std::string pose_topic, map_frame;
  XmlRpc::XmlRpcValue publishers;
  std::vector<std::string> pointcloud_topics;
  std::vector<std::string> map_topics;
  double recordableFps, updateVarianceFps, timeInterval, updatePoseFps, updateGridMapFps, publishStatisticsFps;
  bool enablePointCloudPublishing(false);

  nh.param<std::vector<std::string>>("pointcloud_topics", pointcloud_topics, {"points"});
  nh.getParam("publishers", publishers);
  nh.param<std::vector<std::string>>("initialize_frame_id", initialize_frame_id_, {"base"});
  nh.param<std::vector<double>>("initialize_tf_offset", initialize_tf_offset_, {0.0});
  nh.param<std::string>("pose_topic", pose_topic, "pose");
  nh.param<std::string>("map_frame", mapFrameId_, "map");
  nh.param<std::string>("base_frame", baseFrameId_, "base");
  nh.param<std::string>("initialize_method", initializeMethod_, "cubic");
  nh.param<std::string>("stance_frame_topic", stanceFrameTopic_, "/alip_mpc/stance_foot");
  nh.param<double>("position_lowpass_alpha", positionAlpha_, 0.2);
  nh.param<double>("orientation_lowpass_alpha", orientationAlpha_, 0.2);
  nh.param<double>("recordable_fps", recordableFps, 3.0);
  nh.param<double>("update_variance_fps", updateVarianceFps, 1.0);
  nh.param<double>("time_interval", timeInterval, 0.1);
  nh.param<double>("update_pose_fps", updatePoseFps, 10.0);
  nh.param<double>("initialize_tf_grid_size", initializeTfGridSize_, 0.5);
  nh.param<double>("map_acquire_fps", updateGridMapFps, 5.0);
  nh.param<double>("publish_statistics_fps", publishStatisticsFps, 1.0);
  
  // Custom filter params
  std::vector<double> pointcloud_bias_param;
  nh.param<std::vector<double>>("pointcloud_bias", pointcloud_bias_param, {0.0, 0.0, 0.0});
  pointcloud_bias_ = Eigen::Vector3d::Map(pointcloud_bias_param.data());
  nh.param<double>("y_min", y_min_, -0.45);
  nh.param<double>("foot_mask_x_extent", foot_mask_x_extent_, 0.35);
  nh.param<double>("foot_mask_y_extent", foot_mask_y_extent_, -0.1);
  nh.param<double>("depth_min", depth_min_, 0.75);
  nh.param<double>("depth_max", depth_max_, 2.0);
  nh.param<double>("stance_foot_drift_thresh", stance_foot_drift_thresh_, 1.0);
  
  nh.param<bool>("enable_pointcloud_publishing", enablePointCloudPublishing, false);
  nh.param<bool>("enable_normal_arrow_publishing", enableNormalArrowPublishing_, false);
  nh.param<bool>("enable_drift_corrected_TF_publishing", enableDriftCorrectedTFPublishing_, false);
  nh.param<bool>("use_initializer_at_start", useInitializerAtStart_, false);

  enablePointCloudPublishing_ = enablePointCloudPublishing;

  for (const auto& pointcloud_topic : pointcloud_topics) {
    ros::Subscriber sub = nh_.subscribe(pointcloud_topic, 1, &ElevationMappingNode::pointcloudCallback, this);
    pointcloudSubs_.push_back(sub);
  }
  stanceFootSub_ = nh_.subscribe(stanceFrameTopic_, 1, &ElevationMappingNode::stanceFootCallback, this);

  // register map publishers
  for (auto itr = publishers.begin(); itr != publishers.end(); ++itr) {
    // parse params
    std::string topic_name = itr->first;
    std::vector<std::string> layers_list;
    std::vector<std::string> basic_layers_list;
    auto layers = itr->second["layers"];
    auto basic_layers = itr->second["basic_layers"];
    double fps = itr->second["fps"];

    if (fps > updateGridMapFps) {
      ROS_WARN(
          "[ElevationMappingCupy] fps for topic %s is larger than map_acquire_fps (%f > %f). The topic data will be only updated at %f "
          "fps.",
          topic_name.c_str(), fps, updateGridMapFps, updateGridMapFps);
    }

    for (int32_t i = 0; i < layers.size(); ++i) {
      layers_list.push_back(static_cast<std::string>(layers[i]));
    }

    for (int32_t i = 0; i < basic_layers.size(); ++i) {
      basic_layers_list.push_back(static_cast<std::string>(basic_layers[i]));
    }

    // make publishers
    ros::Publisher pub = nh_.advertise<grid_map_msgs::GridMap>(topic_name, 1);
    mapPubs_.push_back(pub);

    // register map layers
    map_layers_.push_back(layers_list);
    map_basic_layers_.push_back(basic_layers_list);

    // register map fps
    map_fps_.push_back(fps);
    map_fps_unique_.insert(fps);
  }
  setupMapPublishers();

  pointPub_ = nh_.advertise<sensor_msgs::PointCloud2>("elevation_map_points", 1);
  pointPubFilter_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_filtered", 1);
  alivePub_ = nh_.advertise<std_msgs::Empty>("alive", 1);
  normalPub_ = nh_.advertise<visualization_msgs::MarkerArray>("normal", 1);
  statisticsPub_ = nh_.advertise<elevation_map_msgs::Statistics>("statistics", 1);

  gridMap_.setFrameId(mapFrameId_);

}


void ElevationMappingNode::stanceFootCallback(const std_msgs::String& msg) {
  std::lock_guard<std::mutex> lock(stanceMutex_);
  stanceFrameid_ = msg.data;
}


void ElevationMappingNode::setDynamicFootCropBoxParams(
    pcl::CropBox<pcl::PointXYZ>* crop_box, const Eigen::Affine3d& X_FC) {

  Vector3d toe_front_c = X_FC.translation() + X_FC.rotation() * toe_front_;
  Vector3d toe_rear_c = X_FC.translation() + X_FC.rotation() * toe_rear_;

  Vector3d x = (toe_front_c - toe_rear_c).normalized();
  Vector3d y = X_FC.rotation().col(2);
  Vector3d z = x.cross(y).normalized();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;

  Eigen::Affine3d X_BC;
  X_BC.linear() = R;
  X_BC.translation() = toe_front_c + R * crop_box_origin_;

  crop_box->setMin(Eigen::Vector4f(-0.7, -0.075, -0.4, 1.0));
  crop_box->setMax(Eigen::Vector4f(0.25, 0.075, 0.5, 1.0));
  crop_box->setTransform(X_BC.inverse().cast<float>());
}

/*
  * Custom masking filters for cassie to remove the pelvis and
  * other extraneous points
  *
  * Parameters:
  *
  * y_min_: minimum y value (in the camera frame) for which to consider points
  *       meant to cleanup the bottom edge which has stray points from
  *       seeing the pelvis
  *
  * depth_min_: minimum depth to include points from
  * depth_max_: maximum depth to consider points from
  */
void ElevationMappingNode::preprocessPointcloudCassie(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {

  // camera y passthrough for edge cleanup
  pcl::PassThrough<pcl::PointXYZ> camera_y_passthrough;
  camera_y_passthrough.setFilterFieldName ("y");
  camera_y_passthrough.setFilterLimits (y_min_, 10.0);

  // Crop box to set min and max depth
  pcl::CropBox<pcl::PointXYZ> boxFilterDepthMask;
  float x_min_depth = -10, y_min_depth = -10, z_min_depth = depth_min_;
  float x_max_depth = 10,  y_max_depth = 10,  z_max_depth = depth_max_;
  boxFilterDepthMask.setMin(Eigen::Vector4f(x_min_depth, y_min_depth, z_min_depth, 1.0));
  boxFilterDepthMask.setMax(Eigen::Vector4f(x_max_depth, y_max_depth, z_max_depth, 1.0));

  // Voxel grid filter to limit the number of points passed to the GPU
  pcl::VoxelGrid<pcl::PointXYZ>  voxelGrid;
  voxelGrid.setLeafSize(0.02, 0.02, 0.02);

  // Apply filters
  camera_y_passthrough.setInputCloud (cloud);
  camera_y_passthrough.filter (*cloud);
  voxelGrid.setInputCloud(cloud);
  voxelGrid.filter(*cloud);
  boxFilterDepthMask.setInputCloud(cloud);
  boxFilterDepthMask.filter(*cloud);
}

/*
 * Custom cropbox to remove cassie's feet from a pointcloud
 */

void ElevationMappingNode::applyFootCropBoxCassie(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const tf::StampedTransform& tf_X_LC, const tf::StampedTransform& tf_X_RC) {
  pcl::CropBox<pcl::PointXYZ> boxFilterLeft;
  pcl::CropBox<pcl::PointXYZ> boxFilterRight;
  Eigen::Affine3d X_LC, X_RC;

  poseTFToEigen(tf_X_LC, X_LC);
  poseTFToEigen(tf_X_RC, X_RC);
  setDynamicFootCropBoxParams(&boxFilterLeft, X_LC);
  setDynamicFootCropBoxParams(&boxFilterRight, X_RC);
  boxFilterLeft.setNegative(true);
  boxFilterRight.setNegative(true);

  boxFilterLeft.setInputCloud(cloud);
  boxFilterLeft.filter(*cloud);
  boxFilterRight.setInputCloud(cloud);
  boxFilterRight.filter(*cloud);
}

void ElevationMappingNode::pointcloudCallback(const sensor_msgs::PointCloud2& cloud) {
  auto begin_callback = std::chrono::high_resolution_clock::now();
  auto start = ros::Time::now();
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(cloud, pcl_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  auto timeStamp = cloud.header.stamp;
  std::string sensorFrameId = cloud.header.frame_id;

  pcl::fromPCLPointCloud2(pcl_pc, *cloud_filtered);

  /*
   *  Begin dair custom mods
   */

  auto begin_preprocess = std::chrono::high_resolution_clock::now();
  // Pointcloud processing
  preprocessPointcloudCassie(cloud_filtered);
  tf::StampedTransform tf_X_LC, tf_X_RC;
  try {
    transformListener_.waitForTransform(
        left_toe_frame_, sensorFrameId, timeStamp, ros::Duration(0.1));
    transformListener_.lookupTransform(
        sensorFrameId, left_toe_frame_, timeStamp, tf_X_LC);
    transformListener_.lookupTransform(
        sensorFrameId, right_toe_frame_, timeStamp, tf_X_RC);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  applyFootCropBoxCassie(cloud_filtered, tf_X_LC, tf_X_RC);

  auto end_preprocess = std::chrono::high_resolution_clock::now();
  // set updatePose fps to 0 and only update in the pointcloud callback to
  // avoid artifacts
  updatePose(ros::TimerEvent{});

  // shift the map to align with the stance foot
  std::string stanceFrame = "";
  {
    std::lock_guard<std::mutex> lock(stanceMutex_);
    stanceFrame = stanceFrameid_;
  }
  if (!stanceFrame.empty()) {
    tf::StampedTransform tf_X_WF;
    Eigen::Affine3d X_WF;
    try {
      transformListener_.waitForTransform(mapFrameId_, stanceFrame, timeStamp, ros::Duration(0.1));
      transformListener_.lookupTransform(mapFrameId_, stanceFrame, timeStamp, tf_X_WF);
      poseTFToEigen(tf_X_WF, X_WF);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    Eigen::Vector3d stance_pos = X_WF.translation() + X_WF.rotation() * toe_rear_;
    double map_z = 0;
    {
      // Note we're using the gridMap_, not map_, so ideally we should
      // be retrieving the map at least as fast as we receive new pointclouds
      // (i.e. publish_map_fps = 50, pointcloud_fps = 30)
      std::lock_guard<std::mutex> lock(mapMutex_);
      try {
        map_z = gridMap_.atPosition("elevation", stance_pos.head<2>(),
                                  grid_map::InterpolationMethods::INTER_LINEAR);

        // We are trying to get the median for the points around the center of the gridMap
        grid_map::Position center_sub_map = stance_pos.head<2>(); //Position in the foot frame
        grid_map::Length length_sub_map = {0.075, 0.075}; //
        bool success;

        // Getting the submap of where the foot location is
        auto subMap = gridMap_.getSubmap(center_sub_map, length_sub_map, success);
        if (success) {
          // Retrieving the data and making the median of the values
          const auto& mat = subMap.get("elevation");
          std::vector<double> zvals(mat.data(), mat.data() + mat.rows() * mat.cols());
          std::sort(zvals.begin(), zvals.end());
          int n = zvals.size() / 2;
          map_z = (n % 2 == 0) ? 0.5 * (zvals.at(n-1) + zvals.at(n)) : zvals.at(n);
        }

      } catch (std::out_of_range& ex) {
        ROS_ERROR("%s", ex.what());
        initializeWithTF();
      }
    }
    if (!std::isnan(map_z)) {
      double shift = stance_pos(2) - map_z;
      if (abs(shift) < stance_foot_drift_thresh_) {
        map_.shift_map_z(stance_pos(2) - map_z);
      }
    }
  }

  /*
   *  End dair custom mods
   */

  tf::StampedTransform transformTf;
  Eigen::Affine3d transformationSensorToMap;
  try {
    transformListener_.waitForTransform(mapFrameId_, sensorFrameId, timeStamp, ros::Duration(1.0));
    transformListener_.lookupTransform(mapFrameId_, sensorFrameId, timeStamp, transformTf);
    poseTFToEigen(transformTf, transformationSensorToMap);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  double positionError{0.0};
  double orientationError{0.0};
  {
    std::lock_guard<std::mutex> lock(errorMutex_);
    positionError = positionError_;
    orientationError = orientationError_;
  }

  auto begin_map_update = std::chrono::high_resolution_clock::now();

  // ROS_INFO("Frame id", filter_msg.header.frame_id);
  map_.input(cloud_filtered, transformationSensorToMap.rotation(),
             transformationSensorToMap.translation() - pointcloud_bias_,
             positionError,
             orientationError);

  updateGridMap(ros::TimerEvent{});

  auto end_map_update = std::chrono::high_resolution_clock::now();

  if (enablePointCloudPublishing_) {
    pcl::PCLPointCloud2 filtered_pc;
    pcl::toPCLPointCloud2(*cloud_filtered, filtered_pc);
    sensor_msgs::PointCloud2 filter_msg;
    pcl_conversions::fromPCL(filtered_pc, filter_msg);
    pointPubFilter_.publish(filter_msg);
  }

  if (enableDriftCorrectedTFPublishing_) {
    publishMapToOdom(map_.get_additive_mean_error());
  }

  auto end_callback = std::chrono::high_resolution_clock::now();
  // publish profiling info
  elevation_map_msgs::Statistics msg;
  msg.stamp = start;
  msg.pointcloud_preprocess_nanoseconds =  std::chrono::duration_cast<std::chrono::nanoseconds>(end_preprocess - begin_preprocess).count();
  msg.gridmap_update_nanoseconds =  std::chrono::duration_cast<std::chrono::nanoseconds>(end_map_update - begin_map_update).count();
  msg.total_callback_nanoseconds =  std::chrono::duration_cast<std::chrono::nanoseconds>(end_callback - begin_callback).count();
  statisticsPub_.publish(msg);
  pointCloudProcessCounter_++;
}

void ElevationMappingNode::updatePose(const ros::TimerEvent&) {
  tf::StampedTransform transformTf;
  const auto& timeStamp = ros::Time::now();
  try {
    transformListener_.waitForTransform(mapFrameId_, baseFrameId_, timeStamp, ros::Duration(1.0));
    transformListener_.lookupTransform(mapFrameId_, baseFrameId_, timeStamp, transformTf);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  map_.move_to(position);

  if (useInitializerAtStart_) {
    initializeWithTF();
    useInitializerAtStart_ = false;
  }
}

bool ElevationMappingNode::getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response) {
  std::string requestedFrameId = request.frame_id;
  Eigen::Isometry3d transformationOdomToMap;
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  if (requestedFrameId != mapFrameId_) {
    tf::StampedTransform transformTf;
    const auto& timeStamp = ros::Time::now();
    try {
      transformListener_.waitForTransform(requestedFrameId, mapFrameId_, timeStamp, ros::Duration(1.0));
      transformListener_.lookupTransform(requestedFrameId, mapFrameId_, timeStamp, transformTf);
      tf::poseTFToEigen(transformTf, transformationOdomToMap);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    Eigen::Vector3d p(request.position_x, request.position_y, 0);
    Eigen::Vector3d mapP = transformationOdomToMap.inverse() * p;
    requestedSubmapPosition.x() = mapP.x();
    requestedSubmapPosition.y() = mapP.y();
  }
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));

  bool isSuccess;
  grid_map::Index index;
  grid_map::GridMap subMap;
  {
    std::lock_guard<std::mutex> lock(mapMutex_);
    subMap = gridMap_.getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  }
  const auto& length = subMap.getLength();
  if (requestedFrameId != mapFrameId_) {
    subMap = subMap.getTransformedMap(transformationOdomToMap, "elevation", requestedFrameId);
  }

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool ElevationMappingNode::clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  ROS_INFO("Clearing map.");
  map_.clear();
  return true;
}

bool ElevationMappingNode::clearMapWithInitializer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  ROS_INFO("Clearing map with initializer.");
  map_.clear();
  initializeWithTF();
  return true;
}

void ElevationMappingNode::initializeWithTF() {
  std::vector<Eigen::Vector3d> points;
  const auto& timeStamp = ros::Time::now();
  int i = 0;
  Eigen::Vector3d p;
  for (const auto& frame_id : initialize_frame_id_) {
    // Get tf from map frame to tf frame
    Eigen::Affine3d transformationBaseToMap;
    tf::StampedTransform transformTf;
    try {
      transformListener_.waitForTransform(mapFrameId_, frame_id, timeStamp, ros::Duration(1.0));
      transformListener_.lookupTransform(mapFrameId_, frame_id, timeStamp, transformTf);
      poseTFToEigen(transformTf, transformationBaseToMap);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    p = transformationBaseToMap.translation();
    p.z() += initialize_tf_offset_[i];
    points.push_back(p);
    i++;
  }
  if (!points.empty() && points.size() < 3) {
    points.emplace_back(p + Eigen::Vector3d(initializeTfGridSize_, initializeTfGridSize_, 0));
    points.emplace_back(p + Eigen::Vector3d(-initializeTfGridSize_, initializeTfGridSize_, 0));
    points.emplace_back(p + Eigen::Vector3d(initializeTfGridSize_, -initializeTfGridSize_, 0));
    points.emplace_back(p + Eigen::Vector3d(-initializeTfGridSize_, -initializeTfGridSize_, 0));
  }
  ROS_INFO_STREAM("Initializing map with points using " << initializeMethod_);
  map_.initializeWithPoints(points, initializeMethod_);
}


void ElevationMappingNode::updateVariance(const ros::TimerEvent&) {
  map_.update_variance();
}

void ElevationMappingNode::updateTime(const ros::TimerEvent&) {
  map_.update_time();
}

void ElevationMappingNode::updateGridMap(const ros::TimerEvent&) {
  std::vector<std::string> layers(map_layers_all_.begin(), map_layers_all_.end());
  std::lock_guard<std::mutex> lock(mapMutex_);
  map_.get_grid_map(gridMap_, layers);
  const auto timeStamp = ros::Time::now();
  gridMap_.setTimestamp(timeStamp.toNSec());
  isGridmapUpdated_ = true;
}

bool ElevationMappingNode::initializeMap(elevation_map_msgs::Initialize::Request& request,
                                         elevation_map_msgs::Initialize::Response& response) {
  // If initialize method is points
  if (request.type == request.POINTS) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : request.points) {
      const auto& pointFrameId = point.header.frame_id;
      const auto& timeStamp = point.header.stamp;
      const auto& pvector = Eigen::Vector3d(point.point.x, point.point.y, point.point.z);

      // Get tf from map frame to points' frame
      if (mapFrameId_ != pointFrameId) {
        Eigen::Affine3d transformationBaseToMap;
        tf::StampedTransform transformTf;
        try {
          transformListener_.waitForTransform(mapFrameId_, pointFrameId, timeStamp, ros::Duration(1.0));
          transformListener_.lookupTransform(mapFrameId_, pointFrameId, timeStamp, transformTf);
          poseTFToEigen(transformTf, transformationBaseToMap);
        } catch (tf::TransformException& ex) {
          ROS_ERROR("%s", ex.what());
          return false;
        }
        const auto transformed_p = transformationBaseToMap * pvector;
        points.push_back(transformed_p);
      } else {
        points.push_back(pvector);
      }
    }
    std::string method;

    if (request.method == request.NEAREST) {
      method = "nearest";
    } else if (request.method == request.LINEAR) {
      method = "linear";
    } else if (request.method == request.CUBIC) {
      method = "cubic";
    } else {
      throw std::logic_error("invlaid request interpolation method");
    }

    // switch (request.method) {
    //   case request.NEAREST:
    //     method = "nearest";
    //     break;
    //   case request.LINEAR:
    //     method = "linear";
    //     break;
    //   case request.CUBIC:
    //     method = "cubic";
    //     break;
    // }
    ROS_INFO_STREAM("Initializing map with points using " << method);
    map_.initializeWithPoints(points, method);
  }
  response.success = true;
  return true;
}

void ElevationMappingNode::publishNormalAsArrow(const grid_map::GridMap& map) const {
  auto startTime = ros::Time::now();

  const auto& normalX = map["normal_x"];
  const auto& normalY = map["normal_y"];
  const auto& normalZ = map["normal_z"];
  double scale = 0.1;

  visualization_msgs::MarkerArray markerArray;
  // For each cell in map.
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (!map.isValid(*iterator, "elevation")) {
      continue;
    }
    grid_map::Position3 p;
    map.getPosition3("elevation", *iterator, p);
    Eigen::Vector3d start = p;
    const auto i = iterator.getLinearIndex();
    Eigen::Vector3d normal(normalX(i), normalY(i), normalZ(i));
    Eigen::Vector3d end = start + normal * scale;
    if (normal.norm() < 0.1) {
      continue;
    }
    markerArray.markers.push_back(vectorToArrowMarker(start, end, i));
  }
  normalPub_.publish(markerArray);
  ROS_INFO_THROTTLE(1.0, "publish as normal in %f sec.", (ros::Time::now() - startTime).toSec());
}

}  // namespace elevation_mapping_cupy