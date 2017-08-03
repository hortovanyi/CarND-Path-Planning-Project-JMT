/*
 * WayPoint.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "WayPoint.h"

WayPoint::WayPoint(double x, double y, double s, double max_s, double dx, double dy) {
	this->x = x;
	this->y = y;
	this->s = s;
	this->max_s = max_s;
	this->dx = dx;
	this->dy = dy;
	this->next = nullptr;
	this->prev = nullptr;
}

double WayPoint::distance(const WayPoint& other) const {
  return sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y));
}

double WayPoint::distance_to(double x1, double y1) const {
  return sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
}

bool WayPoint::isFirst() const {
  return this->prev->s > this->s;
}

bool WayPoint::isLast() const {
  return this->next->s < this->s;
}

double WayPoint::delta_s(WayPoint * other) const {
  double other_s  = other->s;
  double this_s = this->s;
  double delta_s;

  if (this->s == other->s)
    return 0.0f;

  // address edge cases of looping around s
  if (this->isFirst()) {
    if(other->isLast()){
      this_s += max_s;
    } else if (other->next->isLast()) {
      this_s += max_s;
    }
  }

  if (this->prev->isFirst()) {
    if (other->isLast()) {
      this_s += max_s;
    }
  }

  if (this->isLast()){
    if (other->isFirst()){
      other_s += max_s;
    } else if (other->prev->isFirst()) {
      other_s += max_s;
    }
  }

  if (this->next->isLast()) {
    if (other->isFirst()) {
      other_s += max_s;
    }
  }

  delta_s = other_s - this_s;

  return delta_s;
}

// create splines using -2, current & + 2 way points
// splines are delta relative to this way points s
void WayPoint::InitSplines() {

  // initialse way points
  vector<WayPoint *> wps;
  wps.push_back(this->prev->prev);
  wps.push_back(this->prev);
  wps.push_back(this);
  wps.push_back(this->next);
  wps.push_back(this->next->next);

  // initalise spline vectors
  vector<double> S, X, Y, DX, DY;

  for (WayPoint * wp: wps){
    // TODO double check delta s calcs if not adjacent waypoints
    S.push_back(this->delta_s(wp));
    X.push_back(wp->x);
    Y.push_back(wp->y);
    DX.push_back(wp->dx);
    DY.push_back(wp->dy);
  }
//  for (auto s: S) {
//    cout << s << endl;
//  }

  splines.sx.set_points(S,X);
  splines.sy.set_points(S,Y);
  splines.dx.set_points(S,DX);
  splines.dy.set_points(S,DY);

  return;
}
