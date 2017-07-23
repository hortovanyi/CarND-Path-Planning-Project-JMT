/*
 * HighwayMap.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "HighwayMap.h"

HighwayMap::HighwayMap(const string& map_file_) {
  cout << "Loading waypoints " << map_file_ << endl;
  LoadWaypoints(map_file_);
  cout << "loaded " << way_points_.size() << " waypoints into highway map" << endl;
}

void HighwayMap::LoadWaypoints(const string& map_file_) {
  // open way point file and start streaming
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  if (!in_map_.is_open()){
    cout << "unable to open map file!" << endl;
    return;
  }

  // save the first and prev waypoints so we can create a circular link list
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
    if (prev_waypoint != nullptr) {
      prev_waypoint->next = wp;
      wp->prev = prev_waypoint;
    }

    way_points_.push_back(wp);
    prev_waypoint = wp;
  }

  // prev_waypoint will be the last here, so its next is the first
  if (prev_waypoint != nullptr)
    prev_waypoint->next = first_waypoint;

  // first_waypoint needs have its prev as the last to complete the circular reference
  if (first_waypoint != nullptr)
    first_waypoint->prev = prev_waypoint;
}

vector<WayPoint *> HighwayMap::Waypoints() const {
  return way_points_;
}

WayPoint * HighwayMap::ClosestWaypoint(double x, double y) const {
  double closestDist =  std::numeric_limits<double>::max();
  WayPoint * closestWaypoint = NULL;

  for (auto wp: way_points_) {
    double dist = wp->distance_to(x,y);
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
  return wp0.distance(wp1);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> HighwayMap::getFrenet(double x, double y, double theta) const {
  auto next_wp = NextWaypoint(x,y,theta);
  auto prev_wp = next_wp->prev;

  double n_x = next_wp->x - prev_wp->x;
  double n_y = next_wp->y - prev_wp->y;

  double x_x = x - prev_wp->x;
  double x_y = y - prev_wp->y;

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = EuclidianDistance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prev_wp->x;
  double center_y = 2000 - prev_wp->y;
  double centerToPos = EuclidianDistance(center_x, center_y, x_x, x_y);
  double centerToRef = EuclidianDistance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // TODO investigate do I need to accumulate this as waypoints appear to be loaded with s
  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < way_points_.size(); i++) {
    auto wp = way_points_[i];
    frenet_s += EuclidianDistance(wp->x, wp->y, wp->next->x, wp->next->y);

    // if we are now at the waypoint stop accumulating frenet_s
    if (wp == prev_wp)
      break;
  }

  frenet_s += EuclidianDistance(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> HighwayMap::getXY(double s, double d) const {

  // start from the first waypoint
  WayPoint * prev_wp = way_points_[0];
  int i = 0;

  // find the previous waypoint before this s value
  while(s > prev_wp->s && i++ < way_points_.size()) {
    prev_wp = prev_wp->next;
  }

  auto wp2 = prev_wp->next;

  double heading = atan2((wp2->y - prev_wp->y),
                         (wp2->x - prev_wp->x));

  // the x,y,s along the segment
  double seg_s = (s - prev_wp->s);

  double seg_x = prev_wp->x + seg_s * cos(heading);
  double seg_y = prev_wp->y + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};
}

// get the lane based on frenet coordinates
int HighwayMap::LaneFrenet(double s, double d) const {
  return int(ceil(d/lane_wdith));
}

// get the lane based on cartesian coordinates and angle
int HighwayMap::LaneXY(double x, double y, double theta) const {
  WayPoint * next_wp = NextWaypoint(x, y, theta);
  cout << "next_wp x " << next_wp->x << " y " << next_wp->y << endl;

  // setup vectors for spline
  vector<double> X(5), Y(5);

  // start from two way points back
  X[0] = next_wp->prev->prev->x; X[1] = next_wp->prev->x; X[2] = next_wp->x; X[3] = next_wp->next->x; X[4] = next_wp->next->next->x;
  Y[0] = next_wp->prev->prev->y; Y[1] = next_wp->prev->y; Y[2] = next_wp->y; Y[3] = next_wp->next->y; Y[4] = next_wp->next->next->y;

  // need to sort by Y ascending as X,Y switched later
  bool switched = true;
  while(switched){
    switched = false;
    for (unsigned i=0; i < Y.size()-1; i++){
      if (Y[i]>Y[i+1]) {
        double Xx = X[i];
        double Yy = Y[i];
        X[i]=X[i+1];
        Y[i]=Y[i+1];
        X[i+1]=Xx;
        Y[i+1]=Yy;
        switched = true;
      }
    }
  }
  cout << "X ";
      for (auto x: X)
        cout << x << " ";
  cout << "Y ";
      for (auto y: Y)
        cout << y << " ";
//  cout << endl;

  // create spline - note have reversed X,Y as at Y want to know x of road centrel ine
  tk::spline s;
  s.set_points(Y,X);

  // TODO investigate dx,dy vector calc
  //  the distance away from centreline
  double d = EuclidianDistance(x , y, s(y), y);

  cout << " x " << x << " y " << y << " s(y) " << s(y);
  cout << endl;

  return int(ceil(d/lane_wdith));
}

double HighwayMap::EuclidianDistance(double x1, double y1, double x2, double y2) const {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


