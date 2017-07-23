/*
 * WayPoint.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "WayPoint.h"

WayPoint::WayPoint(double x, double y, double s, double dx, double dy) {
	this->x = x;
	this->y = y;
	this->s = s;
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
