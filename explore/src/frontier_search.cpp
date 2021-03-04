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
  vis_checker_ = LineOfSight(0.01);
}

double FrontierSearch::getDirectionTo(const unsigned int &nbr, const std::vector<int> &prev)
{
  unsigned int cx, cy, fx, fy;
  double cwx, cwy, fwx, fwy;
  costmap_->indexToCells(nbr, cx, cy);
  costmap_->mapToWorld(cx, cy, cwx, cwy);
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

void FrontierSearch::fillPathToCentroid(Frontier &f, const std::vector<int> &prev)
{
  unsigned int mx, my;
  double wx, wy;
  costmap_->worldToMap(f.centroid.x, f.centroid.y, mx, my);
  int centroid_idx = costmap_->getIndex(mx, my);
  std::vector<int> path;
  int cur = centroid_idx;
  // if centroid is unreachable, find nearest reachable point
  if (prev[cur] == -1)
  {
    int nearest_border = std::min(std::min(mx, my),
                                  std::min(costmap_->getSizeInCellsX() - mx, costmap_->getSizeInCellsY() - my));
    int mx_new = -1000, my_new = -1000;
    int idx;
    for (int i = 1; i < nearest_border; i++)
    {
      for (int j = -i; j <= 0; j++)
      {
        idx = costmap_->getIndex(mx + j, my + (j + i));
        if (prev[idx] > -1)
        {
          mx_new = mx + j;
          my_new = my + j + i;
        }
        idx = costmap_->getIndex(mx + j, my - (j + i));
        if (prev[idx] > -1)
        {
          mx_new = mx + j;
          my_new = my - (j + i);
        }
        idx = costmap_->getIndex(mx - j, my + (j + i));
        if (prev[idx] > -1)
        {
          mx_new = mx - j;
          my_new = my + j + i;
        }
        idx = costmap_->getIndex(mx - j, my - (j + i));
        if (prev[idx] > -1)
        {
          mx_new = mx - j;
          my_new = my - (j + i);
        }
      }
      if (mx_new > -1000)
      {
        mx = mx_new;
        my = my_new;
        cur = costmap_->getIndex(mx, my);
        break;
      }
    }
  }
  // restore path to centroid
  while (cur >= 0)
  {
    path.push_back(cur);
    cur = prev[cur];
  }
  std::reverse(path.begin(), path.end());
  // fill path in world coords
  for (size_t i = 0; i < path.size(); ++i)
  {
    geometry_msgs::Point p;
    costmap_->indexToCells(path[i], mx, my);
    costmap_->mapToWorld(mx, my, wx, wy);
    p.x = wx; p.y = wy;
    f.path_to_centroid.push_back(p);
  }
}

void FrontierSearch::smoothPath(frontier_exploration::Frontier &f, Map* map_for_sight_ptr)
{
  if (f.path_to_centroid.size() < 2)
  {
    ROS_WARN("Path to frontier's centroid is empty!!!");
    return;
  }

  // translate path to map coords
  unsigned int i, j;
  std::vector<std::pair<int, int> > path_in_map_coords;
  for (auto pt : f.path_to_centroid)
  {
    costmap_->worldToMap(pt.x, pt.y, i, j);
    path_in_map_coords.push_back(std::make_pair(i, j));
  }

  // smooth path using vis_checker_
  std::vector<std::pair<int, int> > path_smoothed;
  int k = 0;
  path_smoothed.push_back(path_in_map_coords[0]);
  for (size_t i = 1; i + 1 < path_in_map_coords.size(); ++i)
  {
    if (!vis_checker_.checkLine(path_smoothed[k].second, path_smoothed[k].first, 
                                path_in_map_coords[i].second, path_in_map_coords[i].first,
                                *map_for_sight_ptr))
    {
      k++;
      path_smoothed.push_back(path_in_map_coords[i]);
    }
  }
  path_smoothed.push_back(path_in_map_coords[path_in_map_coords.size() - 1]);

  // translate smoothed path into world coords
  f.path_to_centroid.clear();
  double wx, wy;
  for (auto pt : path_smoothed)
  {
    costmap_->mapToWorld(pt.first, pt.second, wx, wy);
    geometry_msgs::Point p;
    p.x = wx; p.y = wy;
    f.path_to_centroid.push_back(p);
  }
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position, geometry_msgs::Quaternion orientation, Map* map_for_sight)
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
        //double direction_to_frontier = getDirectionTo(idx, prev);
        //new_frontier.angle = std::abs(normalize(direction_to_frontier - yaw));
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // smooth path to frontiers and re-estimate distances and directions
  for (size_t i = 0; i < frontier_list.size(); ++i) {
    fillPathToCentroid(frontier_list[i], prev);
    smoothPath(frontier_list[i], map_for_sight);
    if (frontier_list[i].path_to_centroid.size() > 1)
    {
      // re-estimate angle
      double dy = frontier_list[i].path_to_centroid[1].y - frontier_list[i].path_to_centroid[0].y;
      double dx = frontier_list[i].path_to_centroid[1].x - frontier_list[i].path_to_centroid[0].x;
      double direction_to_frontier = std::atan2(dy, dx);
      frontier_list[i].angle = std::abs(normalize(direction_to_frontier - yaw));
      // re-estimate distance
      frontier_list[i].min_distance = 0;
      for (size_t j = 1; j < frontier_list[i].path_to_centroid.size(); ++j)
      {
        dy = frontier_list[i].path_to_centroid[j].y - frontier_list[i].path_to_centroid[j - 1].y;
        dx = frontier_list[i].path_to_centroid[j].x - frontier_list[i].path_to_centroid[j - 1].x;
        frontier_list[i].min_distance += sqrt(dx * dx + dy * dy);
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

  // visualize path to centroid of first frontier
  /*nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped pose;
  path_msg.header.frame_id = "map";
  double wx, wy;
  for (auto pt : frontier_list[0].path_to_centroid)
  {
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    path_msg.poses.push_back(pose);
  }
  path_publisher_.publish(path_msg);*/

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
  output.size = 0;
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
          output.min_distance = distance;
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
