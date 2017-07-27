/*
 * Vehicle.cpp
 *
 *  Created on: 23 Jul. 2017
 *      Author: nick
 */

#include "Vehicle.h"

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->v = vxvy2v(vx,vy);
  this->yaw = atan2(vx,vy);
  this->s = s;
  this->d = d;
  this->speed = v * metersPerSecRatioMilesPerHr;

}

Vehicle::Vehicle(double x, double y, double s, double d, double angle_deg, double speed) {
  this->id = -1;
  this->x = x;
  this->y = y;
  this->yaw = deg2rad(angle_deg);
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->v = speed * (1/metersPerSecRatioMilesPerHr);
  this->vx = cos(yaw) * v;
  this->vy = sin(yaw) * v;
}

double Vehicle::YawDeg() {
  return rad2deg(atan2(vy,vx));
}

double Vehicle::deg2rad(double x) {
  return x * M_PI / 180;
}
double Vehicle::rad2deg(double x) {
  return x * 180 / M_PI;
}

double Vehicle::vxvy2v(double vx, double vy) {
  return sqrt(vx*vx+vy*vy);
}


