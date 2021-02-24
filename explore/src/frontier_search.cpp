#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <bits/stdc++.h>
#include <math.h>

#include <explore/costmap_tools.h>

#include <iostream>
#include <fstream>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale, double orientation_scale,
                               double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , orientation_scale_(orientation_scale)
  , min_frontier_size_(min_frontier_size)
  , private_nh_("~")
{
  path_publisher_ = private_nh_.advertise<nav_msgs::Path>("path_to_zero", 10);
}

double FrontierSearch::getDirectionTo(const unsigned int &nbr, const std::vector<int> &prev)
{
  int cur = int(nbr);
  int from = cur;
  std::vector<int> path;
  while (prev[cur] >= 0)
  {
    from = cur;
    cur = prev[cur];
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  unsigned int cx, cy, fx, fy;
  double cwx, cwy, fwx, fwy;
  double angle = 0, n = 0;
  for (int i = 0; (i < 10) && (i + 1 < path.size()); i++)
  {
    costmap_->indexToCells(path[i], cx, cy);
    costmap_->indexToCells(path[i + 1], fx, fy);
    costmap_->mapToWorld(cx, cy, cwx, cwy);
    costmap_->mapToWorld(fx, fy, fwx, fwy);
    angle += std::atan2(fwy - cwy, fwx - cwx);
    n += 1;
  }
  return angle / n;
}

double normalize(double angle)
{
  double result = angle + 2 * M_PI;
  if (result > M_PI)
    result -= 2 * M_PI;
  if (result > M_PI)
    result -= 2 * M_PI;
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position, geometry_msgs::Quaternion orientation)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // compute current robot's orientation (yaw angle)
  tf::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
  tf::Matrix3x3 quaternion_matrix(quat);
  double roll, pitch, yaw;
  quaternion_matrix.getRPY(roll, pitch, yaw);

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<double> distance(size_x_ * size_y_, std::numeric_limits<double>::infinity());
  std::vector<int> prev(size_x_ * size_y_, -1);

  // initialize breadth first search
  std::priority_queue<std::pair<double, unsigned int> > heap;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    heap.push(std::make_pair(0, clear));
  } else {
    heap.push(std::make_pair(0, pos));
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  distance[heap.top().second] = 0;

  while (!heap.empty()) {
    unsigned int idx = heap.top().second;
    heap.pop();

    // iterate over 8-connected neighbourhood
    for (unsigned nbr : nhood8(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      unsigned int wx, wy, nx, ny;
      costmap_->indexToCells(idx, wx, wy);
      costmap_->indexToCells(nbr, nx, ny);
      //ROS_INFO("wx, wy, nx, ny: %d, %d, %d, %d", wx, wy, nx, ny);
      double dx = double(wx) - double(nx);
      double dy = double(wy) - double(ny);
      //ROS_INFO("dx, dy: %f, %f", dx, dy);
      double step = sqrt(dx * dx + dy * dy) * costmap_->getResolution();
      //ROS_INFO("D[idx]: %f", distance[idx]);
      //ROS_INFO("Step: %f", step);
      if ((map_[nbr] <= map_[idx]) && (distance[nbr] > distance[idx] + step)) {
        distance[nbr] = distance[idx] + step;
        prev[nbr] = idx;
        heap.push(std::make_pair(-distance[nbr], nbr));
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, yaw, frontier_flag);
        new_frontier.min_distance = distance[idx] + step;
        double direction_to_frontier = getDirectionTo(idx, prev);
        new_frontier.angle = std::abs(normalize(direction_to_frontier - yaw));
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  // visualize path to (0, 0) point
  costmap_->worldToMap(0, 0, mx, my);
  unsigned int zero_idx = costmap_->getIndex(mx, my);
  //ROS_INFO("Zero idx: %d", zero_idx);
  std::vector<int> path;
  int cur = zero_idx;
  while (cur >= 0)
  {
    path.push_back(cur);
    cur = prev[cur];
  }
  std::reverse(path.begin(), path.end());
  //ROS_INFO("Path to zero contains %d points", path.size());
  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped pose;
  path_msg.header.frame_id = "map";
  double wx, wy;
  for (auto idx : path)
  {
    costmap_->indexToCells(idx, mx, my);
    costmap_->mapToWorld(mx, my, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    path_msg.poses.push_back(pose);
  }
  path_publisher_.publish(path_msg);
  double direction_to_zero = getDirectionTo(zero_idx, prev);
  //ROS_INFO("Direction to zero: %f", direction_to_zero);
  //ROS_INFO("Angle: %f", std::abs(normalize(direction_to_zero - yaw)));

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          double yaw,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double diff_x = reference_x - wx;
        double diff_y = reference_y - wy;
        double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
        if (distance < output.min_distance) {
          //output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
          double direction_to_frontier = std::atan2(double(wy) - double(reference_y), double(wx) - double(reference_x));
          output.angle = std::abs(normalize(direction_to_frontier - yaw));
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;

  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance) +
         (orientation_scale_ * frontier.angle) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}
