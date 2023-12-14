/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Stefan Kohlbrecher, TU Darmstadt
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
 *
 */

/*
 * Author: Stefan Kohlbrecher
 */

#include <grid_map_2d_mapper/grid_map_2d_mapper_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace grid_map_2d_mapper
{

  GridMap2DMapperNodelet::GridMap2DMapperNodelet() {}

  void GridMap2DMapperNodelet::onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<std::string>("map_frame", map_frame_, "world");

    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    //private_nh_.param<double>("min_height", min_height_, -0.1);
    //private_nh_.param<double>("max_height", max_height_, 1.3);

    private_nh_.param<double>("angle_min", angle_min_, -M_PI / 1.0);
    private_nh_.param<double>("angle_max", angle_max_, M_PI / 1.0);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
    private_nh_.param<bool>("downsample_cloud", downsample_cloud_, true);
    private_nh_.param<double>("scan_time", scan_time_, 0.0);
    private_nh_.param<double>("range_min", range_min_, 0.45);
    private_nh_.param<double>("range_max", range_max_, 15.0);
    private_nh_.param<double>("probability_occupied", probability_occ_, 0.75);
    private_nh_.param<double>("probability_free", probability_free_, 0.4);
    //private_nh_.param<bool>("no_mapping", no_mapping_, false);

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);

    dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, private_nh_));
    dyn_rec_server_->setCallback(boost::bind(&GridMap2DMapperNodelet::reconfigureCallback, this, _1, _2));

    //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
      nh_ = getNodeHandle();
    }
    else
    {
      nh_ = getMTNodeHandle();
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
      input_queue_size_ = concurrency_level;
    }
    else
    {
      input_queue_size_ = boost::thread::hardware_concurrency();
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, 20, nh_));
      message_filter_->registerCallback(boost::bind(&GridMap2DMapperNodelet::cloudCb, this, _1));
      message_filter_->registerFailureCallback(boost::bind(&GridMap2DMapperNodelet::failureCb, this, _1, _2));
    }
    else // otherwise setup direct subscription
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      sub_.registerCallback(boost::bind(&GridMap2DMapperNodelet::cloudCb, this, _1));
    }

    //pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
    //                                             boost::bind(&GridMap2DMapperNodelet::connectCb, this),
    //                                             boost::bind(&GridMap2DMapperNodelet::disconnectCb, this));

    //map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map",10,false);

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, false);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map",10);
    //                                                  boost::bind(&GridMap2DMapperNodelet::connectCb, this),
    //                                                  boost::bind(&GridMap2DMapperNodelet::disconnectCb, this));


    map_throttled_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_throttled",10);
                                                      //boost::bind(&GridMap2DMapperNodelet::connectCb, this),
                                                      //boost::bind(&GridMap2DMapperNodelet::disconnectCb, this));

    map_service_ = private_nh_.advertiseService("map", &GridMap2DMapperNodelet::mapServiceCallback, this);


    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/debug_map",10,false);

    syscommand_subscriber_ = nh_.subscribe("syscommand", 10, &GridMap2DMapperNodelet::syscommandCallback, this);

    sub_.subscribe(nh_, "/scan_matched_points2", input_queue_size_);

    grid_map_.add("occupancy_log_odds");
    grid_map_.add("occupancy_prob");
    //grid_map_.add("update_time");
    grid_map_.setGeometry(grid_map::Length(2.0, 2.0), 0.05);
    grid_map_.setFrameId(map_frame_);


    log_odds_free_ = probToLogOdds(probability_free_);
    log_odds_occ_  = probToLogOdds(probability_occ_);


    min_log_odds_ = log_odds_free_ * 20;
    max_log_odds_ = log_odds_occ_ * 20;
    ROS_INFO("log odds free: %f log odds occ: %f", log_odds_free_, log_odds_occ_);

    double map_publish_period = private_nh_.param("map_publish_period", 1.0);
    map_throttled_pub_timer_ = nh_.createTimer(ros::Duration(map_publish_period), &GridMap2DMapperNodelet::mapThrottledPubTimer, this);
  }

  void GridMap2DMapperNodelet::connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (map_pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
    {
      NODELET_INFO("Got a subscriber to map, starting subscriber to pointcloud");
      sub_.subscribe(nh_, "/scan_matched_points2", input_queue_size_);
    }
  }

  void GridMap2DMapperNodelet::disconnectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (map_pub_.getNumSubscribers() == 0)
    {
      NODELET_INFO("No subscribers to scan, shutting down subscriber to pointcloud");
      sub_.unsubscribe();
    }
  }

  void GridMap2DMapperNodelet::syscommandCallback(const std_msgs::String::ConstPtr& msg)
  {
    if(msg->data == "reset" || msg->data == "reset_2d_map"){
      grid_map_.clear("occupancy_log_odds");
      grid_map_.clear("occupancy_prob");
      ROS_INFO("Cleared grid_map_2d_mapper map!");
    }
  }

  void GridMap2DMapperNodelet::reconfigureCallback(grid_map_2d_mapper::GridMap2DMapperConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f", config.min_height, config.max_height);

    min_height_ = config.min_height;
    max_height_ = config.max_height;
    no_mapping_ = !config.mapping_active;
  }

  void GridMap2DMapperNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                               tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
        << message_filter_->getTargetFramesString());
  }

  void GridMap2DMapperNodelet::mapThrottledPubTimer(const ros::TimerEvent &event)
  {
    if (map_throttled_pub_.getNumSubscribers() > 0){

      grid_map::Matrix& grid_data = grid_map_["occupancy_log_odds"];

      grid_map::Matrix& grid_data_prob = grid_map_["occupancy_prob"];


      //grid_map::GridMapRosConverter::toOccupancyGrid()
      size_t total_size = grid_data.rows() * grid_data.cols();
      for (size_t i = 0; i < total_size; ++i){
        const float& cell = grid_data.data()[i];

        if (cell != cell){
          grid_data_prob.data()[i] = cell;
        }else if (cell < 0.0){
          grid_data_prob.data()[i] = 0.0;
        }else{
          grid_data_prob.data()[i] = 1.0;
        }
      }

      nav_msgs::OccupancyGrid occ_grid_msg;

      grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, occ_grid_msg);
      map_throttled_pub_.publish(occ_grid_msg);

    }
  }

  bool GridMap2DMapperNodelet::mapServiceCallback(nav_msgs::GetMap::Request  &req,
                                                  nav_msgs::GetMap::Response &res )
  {
    ROS_INFO("grid_mapper_2d map service called");

    grid_map::Matrix& grid_data = grid_map_["occupancy_log_odds"];

    grid_map::Matrix& grid_data_prob = grid_map_["occupancy_prob"];


    //grid_map::GridMapRosConverter::toOccupancyGrid()
    size_t total_size = grid_data.rows() * grid_data.cols();
    for (size_t i = 0; i < total_size; ++i){
      const float& cell = grid_data.data()[i];

      if (cell != cell){
        grid_data_prob.data()[i] = cell;
      }else if (cell < 0.0){
        grid_data_prob.data()[i] = 0.0;
      }else{
        grid_data_prob.data()[i] = 1.0;
      }
    }

    grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, res.map);

    return true;
  }

  void GridMap2DMapperNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // Initialize a new LaserScan message
    // A LaserScan message contains an array of ranges. Each range corresponds to a specific angle.
    sensor_msgs::LaserScanPtr laser_scan_out = boost::make_shared<sensor_msgs::LaserScan>();

    // Set basic properties of the LaserScan message
    laser_scan_out->header = cloud_msg->header;
    if (!target_frame_.empty()) {
      laser_scan_out->header.frame_id = target_frame_;
    }
    laser_scan_out->angle_min = angle_min_;
    laser_scan_out->angle_max = angle_max_;
    laser_scan_out->angle_increment = angle_increment_;
    laser_scan_out->time_increment = 0.0;
    laser_scan_out->scan_time = scan_time_;
    laser_scan_out->range_min = range_min_;
    laser_scan_out->range_max = range_max_;

    // Calculate the number of rays in the LaserScan
    uint32_t ranges_size = std::ceil((laser_scan_out->angle_max - laser_scan_out->angle_min) / laser_scan_out->angle_increment);
    laser_scan_out->ranges.assign(ranges_size, use_inf_ ? std::numeric_limits<double>::infinity() : laser_scan_out->range_max + 1.0);

    // Transform the point cloud if necessary
    sensor_msgs::PointCloud2ConstPtr cloud_out;
    if (!(laser_scan_out->header.frame_id == cloud_msg->header.frame_id)) {
      try {
        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
        tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
        cloud_out = cloud;
      } catch (tf2::TransformException &ex) {
        NODELET_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    } else {
      cloud_out = cloud_msg;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    // After the following loop:
    // cloud_filtered should only contain wall points, but no floor or ceiling points
    // laser_scan_out should only contain wall points, projected onto a plane

    // Process each point in the cloud and filter out invalid points.
    // Project the point cloud onto a plane and generate laserscan data
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // Filter out NaN points
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
        NODELET_DEBUG("Rejected NaN point(%f, %f, %f)", *iter_x, *iter_y, *iter_z);
        continue;
      }
      // Filter points outside of the height bounds
      if (*iter_z > max_height_ || *iter_z < min_height_) {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }
      // Filter points based on range
      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_) {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }
      // Filter points based on angle
      double angle = atan2(*iter_y, *iter_x);
      if (angle < laser_scan_out->angle_min || angle > laser_scan_out->angle_max) {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, laser_scan_out->angle_min, laser_scan_out->angle_max);
        continue;
      }

      // Update the corresponding range in the LaserScan message if the new range is smaller than the old one
      int index = (angle - laser_scan_out->angle_min) / laser_scan_out->angle_increment;
      if (range < laser_scan_out->ranges[index]) {
        laser_scan_out->ranges[index] = range;
      }

      // Optionally accumulate all points for downsampling
      if(!downsample_cloud_) {
        cloud_filtered.emplace_back(*iter_x, *iter_y, *iter_z);
      }
    }

    pub_.publish(laser_scan_out);

    if (no_mapping_)
      return;

    sensor_msgs::PointCloud2 cloud_reduced;
    if(downsample_cloud_) {
      projector_.projectLaser(*laser_scan_out, cloud_reduced);
    }
    else {
      pcl::toROSMsg(cloud_filtered, cloud_reduced);
      cloud_reduced.header = laser_scan_out->header;
    }
    // if downsample_cloud_ is false, cloud_reduced contains all points that are within height,
    // range and angle bounds and is 3D
    // if it is true, cloud_reduced only contains the points that were projected into the laser scan
    // and is 2D

    ros::Duration wait_duration(1.0);
    geometry_msgs::TransformStamped to_world_tf;
    try {
      to_world_tf = tf2_->lookupTransform(map_frame_, cloud_reduced.header.frame_id, cloud_reduced.header.stamp, wait_duration);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Cannot lookup transform, skipping map update!: %s",ex.what());
      return;
    }

    Eigen::Affine3d cloud_to_world_eigen = tf2::transformToEigen(to_world_tf);
    Eigen::Vector3d sensor_frame_world_pos (cloud_to_world_eigen.translation());
    grid_map::Matrix& grid_data = grid_map_["occupancy_log_odds"];
    grid_map::Position sensor_position (sensor_frame_world_pos.x(), sensor_frame_world_pos.y());

    end_points_.clear();
    Eigen::Vector2d min_coords(std::numeric_limits<double>::max(),    std::numeric_limits<double>::max());
    Eigen::Vector2d max_coords(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());

    // Find max and min coords of all points, to determine map size
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(cloud_reduced, "x"), iter_y(cloud_reduced, "y"), iter_z(cloud_reduced, "z");
              iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      Eigen::Vector3d end_point (cloud_to_world_eigen * Eigen::Vector3d(*iter_x, *iter_y, *iter_z));

      if (max_coords.x() < end_point.x())
          max_coords.x() = end_point.x();
      if (max_coords.y() < end_point.y())
          max_coords.y() = end_point.y();
      if (min_coords.x() > end_point.x())
          min_coords.x() = end_point.x();
      if (min_coords.y() > end_point.y())
          min_coords.y() = end_point.y();

      end_points_.push_back(end_point);
    }
    Eigen::Vector2d center  ( (min_coords + max_coords) * 0.5 );
    Eigen::Vector2d lengths (max_coords - min_coords);
    grid_map::GridMap update_area;
    update_area.setGeometry(lengths.array() + 0.5, grid_map_.getResolution(), center);
    grid_map_.extendToInclude(update_area);

    std::vector<grid_map::Index> curr_ray;
    // fill gridmap with data - white for free, black for occupied
    for (const Eigen::Vector3d& end_point : end_points_) {
      grid_map::Position end_point_position(end_point.x(), end_point.y());
      curr_ray.clear();

      for (grid_map::LineIterator iterator(grid_map_, sensor_position, end_point_position); !iterator.isPastEnd(); ++iterator) {
        curr_ray.emplace_back(*iterator);
      }

      // Update log odds for each cell along the ray
      for (size_t i = 0; i < curr_ray.size() - 1; ++i) {
        const grid_map::Index& index = curr_ray[i];
        if (std::isnan(grid_data(index(0), index(1)))) {
          grid_data(index(0), index(1)) = 0.0;
        }
        if (min_log_odds_ < grid_data(index(0), index(1))) {
          grid_data(index(0), index(1)) += log_odds_free_;
        }
      }

      // Update the log odds for the end cell of the ray
      const grid_map::Index& end_index = curr_ray.back();
      if (std::isnan(grid_data(end_index(0), end_index(1)))) {
        grid_data(end_index(0), end_index(1)) = 0.0;
      }
      if (max_log_odds_ > grid_data(end_index(0), end_index(1))) {
        grid_data(end_index(0), end_index(1)) += log_odds_occ_;
      }
    }
    ROS_INFO_STREAM("FCK ODDSSSSS: " << log_odds_free_ << " " << log_odds_occ_ << " " << min_log_odds_ << " " << max_log_odds_);
    // Make new method out of this
    double min_floor_height = -0.1;
    double max_floor_height = 0.1;

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {

      // Filter out NaN points
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
        NODELET_DEBUG("Rejected NaN point(%f, %f, %f)", *iter_x, *iter_y, *iter_z);
        continue;
      }
      // Filter points based on range
      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_) {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }
      // Filter points based on angle
      double angle = atan2(*iter_y, *iter_x);
      if (angle < laser_scan_out->angle_min || angle > laser_scan_out->angle_max) {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, laser_scan_out->angle_min, laser_scan_out->angle_max);
        continue;
      }

      Eigen::Vector3d floor_point (cloud_to_world_eigen * Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
      grid_map::Position floor_point_position(floor_point.x(), floor_point.y());

      // Update the corresponding range in the LaserScan message if the new range is smaller than the old one
      int laser_index = (angle - laser_scan_out->angle_min) / laser_scan_out->angle_increment;
      if (laser_scan_out->ranges[laser_index] == (use_inf_ ? std::numeric_limits<double>::infinity() : laser_scan_out->range_max + 1.0)) {
        // Filter points outside of the height bounds
        if (floor_point.z() > min_floor_height && floor_point.z() < max_floor_height) {
          grid_map::Index index;
          if (grid_map_.getIndex(floor_point_position, index)) {
            if (std::isnan(grid_data(index(0), index(1)))) {
              grid_data(index(0), index(1)) = log_odds_free_;
            }
          }
        }
      }
    }

    if (grid_map_pub_.getNumSubscribers() > 0){
      grid_map_msgs::GridMap grid_map_msg;
      grid_map::GridMapRosConverter::toMessage(grid_map_, grid_map_msg);
      grid_map_pub_.publish(grid_map_msg);
    }

    // Turn map into binary map and publish
    if (map_pub_.getNumSubscribers() > 0){
      grid_map::Matrix& grid_data_prob = grid_map_["occupancy_prob"];

      //grid_map::GridMapRosConverter::toOccupancyGrid()
      size_t total_size = grid_data.rows() * grid_data.cols();
      for (size_t i = 0; i < total_size; ++i) {
        const float& cell = grid_data.data()[i];
        if (cell != cell) {
          grid_data_prob.data()[i] = cell;
        } else if (cell < 0.0) {
          grid_data_prob.data()[i] = 0.0;
        } else {
          grid_data_prob.data()[i] = 1.0;
        }
      }

      nav_msgs::OccupancyGrid occ_grid_msg;
      grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, occ_grid_msg);
      map_pub_.publish(occ_grid_msg);
    }
  }

  float GridMap2DMapperNodelet::probToLogOdds(float prob) {
      float odds = prob / (1.0f - prob);
      return log(odds);
    }

}

PLUGINLIB_EXPORT_CLASS(grid_map_2d_mapper::GridMap2DMapperNodelet, nodelet::Nodelet);
