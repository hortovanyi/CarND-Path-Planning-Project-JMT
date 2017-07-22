/*
 * HighwayMap.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "HighwayMap.h"

HighwayMap::HighwayMap(const string& map_file_) {

  cout << "Loading waypoints " << map_file_ << endl;
  // open way point file and start streaming
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  if (!in_map_.is_open()){
    cout << "unable to open map file!" << endl;
    return;
  }

  // save the first and prev waypoints so we can create a forward circular link list
  WayPoint * first_waypoint = nullptr;
  WayPoint * prev_waypoint = nullptr;

  // load up the way points
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    WayPoint * wp = new WayPoint(x, y, s, d_x, d_y);

    // save the first waypoint
    if (first_waypoint == nullptr)
      first_waypoint = wp;

    // if we have a previous then reference this waypoint as its next
    if (prev_waypoint != nullptr)
      prev_waypoint->next = wp;

    way_points_.push_back(wp);
    prev_waypoint = wp;
  }

  // prev_waypoint will be the last here, so its next is the first
  if (prev_waypoint != nullptr)
    prev_waypoint->next = first_waypoint;

  cout << "loaded " << way_points_.size() << " waypoints into highway map" << endl;
}

vector<WayPoint *> HighwayMap::way_points() const {
  return way_points_;
}

WayPoint * HighwayMap::ClosestWaypoint(double x, double y) const {
  double closestDist =  std::numeric_limits<double>::max();
  WayPoint * closestWaypoint = NULL;

  for (auto wp: way_points_) {
    double dist = wp->distance(x,y);
    if (dist < closestDist) {
      closestDist = dist;
      closestWaypoint = wp;
    }
  }

  return closestWaypoint;
}

WayPoint * HighwayMap::NextWaypoint(double x, double y, double theta) const {
  WayPoint * closest_wp = ClosestWaypoint(x,y);

  double map_x = closest_wp->x;
  double map_y = closest_wp->y;

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  WayPoint * next_wp = closest_wp;
  if (angle > M_PI / 4) {
    next_wp = next_wp->next;
  }

  return next_wp;
}

double HighwayMap::distance(const WayPoint & wp0, const WayPoint& wp1) const {
  return wp0.waypoint_distance(wp1);
}



