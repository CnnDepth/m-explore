/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
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
 *   * Neither the name of the Jiri Horner nor the names of its
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
 *********************************************************************/

#include <explore/costmap_client.h>

#include <functional>
#include <mutex>
#include <string>
#include <fstream>

namespace explore
{
// static translation table to speed things up
std::array<unsigned char, 256> init_translation_table();
static const std::array<unsigned char, 256> cost_translation_table__ =
    init_translation_table();

Costmap2DClient::Costmap2DClient(ros::NodeHandle& param_nh,
                                 ros::NodeHandle& subscription_nh,
                                 const tf::TransformListener* tf)
  : tf_(tf)
{
  std::string costmap_topic;
  std::string footprint_topic;
  std::string costmap_updates_topic;
  param_nh.param("costmap_topic", costmap_topic, std::string("costmap"));
  param_nh.param("costmap_updates_topic", costmap_updates_topic,
                 std::string("costmap_updates"));
  param_nh.param("robot_base_frame", robot_base_frame_,
                 std::string("base_link"));
  // transform tolerance is used for all tf transforms here
  param_nh.param("transform_tolerance", transform_tolerance_, 0.3);

  /* initialize costmap */
  costmap_sub_ = subscription_nh.subscribe<nav_msgs::OccupancyGrid>(
      costmap_topic, 1000,
      [this](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        updateFullMap(msg);
      });
  map_pub = subscription_nh.advertise<nav_msgs::OccupancyGrid>("projmap_resized", 10);
  ROS_INFO("Waiting for costmap to become available, topic: %s",
           costmap_topic.c_str());
  auto costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
      costmap_topic, subscription_nh);
  updateFullMap(costmap_msg);

  /* subscribe to map updates */
  costmap_updates_sub_ =
      subscription_nh.subscribe<map_msgs::OccupancyGridUpdate>(
          costmap_updates_topic, 1000,
          [this](const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
            updatePartialMap(msg);
          });

  /* resolve tf prefix for robot_base_frame */
  std::string tf_prefix = tf::getPrefixParam(param_nh);
  robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  // we need to make sure that the transform between the robot base frame and
  // the global frame is available
  /* tf transform is necessary for getRobotPose */
  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  while (ros::ok() &&
         !tf_->waitForTransform(global_frame_, robot_base_frame_, ros::Time(),
                                ros::Duration(0.1), ros::Duration(0.01),
                                &tf_error)) {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now()) {
      ROS_WARN(
          "Timed out waiting for transform from %s to %s to become available "
          "before subscribing to costmap, tf error: %s",
          robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same,
    // so the last
    // will do for the warning above. Reset the string here to avoid
    // accumulation.
    tf_error.clear();
  }
}

void Costmap2DClient::preprocess(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                                 unsigned int& size_in_cells_x,
                                 unsigned int& size_in_cells_y,
                                 double& resolution,
                                 signed char* &data)
{
  size_in_cells_x = int(msg->info.width / 2);
  size_in_cells_y = int(msg->info.height / 2);
  resolution = msg->info.resolution * 2;
  int size = size_in_cells_x * size_in_cells_y;
  data = new signed char[size];
  for (int idx = 0; idx < size; idx++)
  {
    int i = int(idx / size_in_cells_x);
    int j = idx % size_in_cells_x;
    int idx1 = i * 2 * msg->info.width + j * 2;
    int idx2 = i * 2 * msg->info.width + j * 2 + 1;
    int idx3 = (i * 2 + 1) * msg->info.width + j * 2;
    int idx4 = (i * 2 + 1) * msg->info.width + j * 2 + 1;
    int max_value = std::max(std::max(std::max(msg->data[idx1], msg->data[idx2]), msg->data[idx3]), msg->data[idx4]);
    data[idx] = max_value;
    if (data[idx] == 100)
      for (int ii = i - 1; ii <= i + 1; ii++)
        for (int jj = j - 1; jj <= j + 1; jj++)
          if ((ii >= 0) && (ii < size_in_cells_y) && (jj >= 0) && (jj < size_in_cells_x))
          {
            data[ii * size_in_cells_x + jj] = 100;
          }
  }
  nav_msgs::OccupancyGrid map_resized;
  map_resized.info.width = size_in_cells_x;
  map_resized.info.height = size_in_cells_y;
  map_resized.info.resolution = resolution;
  map_resized.info.origin = msg->info.origin;
  for (int i = 0; i < size; i++)
  {
    map_resized.data.push_back(data[i]);
  }
  map_resized.header.stamp = msg->header.stamp;
  map_pub.publish(map_resized);
}

void Costmap2DClient::updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  global_frame_ = msg->header.frame_id;

  unsigned int size_in_cells_x = msg->info.width;
  unsigned int size_in_cells_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  signed char* data;
  preprocess(msg, size_in_cells_x, size_in_cells_y, resolution, data);

  //ROS_DEBUG("received full new map, resizing to: %d, %d", size_in_cells_x,
  //          size_in_cells_y);
  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
                     origin_y);

  // lock as we are accessing raw underlying map
  auto* mutex = costmap_.getMutex();
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // fill map with data
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  //ROS_DEBUG("full map update, %lu values", costmap_size);
  for (size_t i = 0; i < costmap_size; ++i) {
    unsigned char cell_cost = static_cast<unsigned char>(data[i]);
    costmap_data[i] = cost_translation_table__[cell_cost];
  }
  //ROS_DEBUG("map updated, written %lu values", costmap_size);
  delete data;
}

void Costmap2DClient::updatePartialMap(
    const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
  ROS_DEBUG("received partial map update");
  global_frame_ = msg->header.frame_id;

  if (msg->x < 0 || msg->y < 0) {
    ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  // lock as we are accessing raw underlying map
  auto* mutex = costmap_.getMutex();
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  size_t costmap_xn = costmap_.getSizeInCellsX();
  size_t costmap_yn = costmap_.getSizeInCellsY();

  if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
      y0 > costmap_yn) {
    ROS_WARN("received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, costmap_xn, costmap_yn);
  }

  // update map with data
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = cost_translation_table__[cell_cost];
      ++i;
    }
  }
}

geometry_msgs::Pose Costmap2DClient::getRobotPose() const
{
  tf::Stamped<tf::Pose> global_pose;
  global_pose.setIdentity();
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time =
      ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try {
    tf_->transformPose(global_frame_, robot_pose, global_pose);
  } catch (tf::LookupException& ex) {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot "
                            "pose: %s\n",
                       ex.what());
    return {};
  } catch (tf::ConnectivityException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n",
                       ex.what());
    return {};
  } catch (tf::ExtrapolationException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n",
                       ex.what());
    return {};
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() >
      transform_tolerance_) {
    ROS_WARN_THROTTLE(1.0, "Costmap2DClient transform timeout. Current time: "
                           "%.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(),
                      transform_tolerance_);
    return {};
  }

  geometry_msgs::PoseStamped msg;
  tf::poseStampedTFToMsg(global_pose, msg);
  return msg.pose;
}

std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // lineary mapped from [0..100] to [0..255]
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  // special values:
  cost_translation_table[0] = 0;      // NO obstacle
  cost_translation_table[99] = 253;   // INSCRIBED obstacle
  cost_translation_table[100] = 254;  // LETHAL obstacle
  cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

  return cost_translation_table;
}

}  // namespace explore
