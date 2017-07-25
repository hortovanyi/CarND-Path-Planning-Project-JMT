/*
 * Vehicle.h
 *
 *  Created on: 23 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <math.h>

class Vehicle {

 public:
  int id;
  double x;
  double y;
  double s; // frenet s
  double d; // frenet d
  double vx;
  double vy;
  double v; // speed in meters/sec
  double yaw; // radians
  double speed; // mph


  // constructor for sensor fusion
  Vehicle(int id, double x, double y, double xv, double yv, double s, double d);

  // constructor for ego vehicle
  Vehicle(double x, double y, double s, double d, double angle, double speed);
  double YawDeg();

  double deg2rad(double x);
  double rad2deg(double x);

  double vxvy2v(double vx, double vy);

 private:
  // value taken from http://study.com/academy/lesson/how-to-convert-meters-per-second-to-miles-per-hour.html
  constexpr static double meterPerSecRatioMilesPerHr = 2.236936292;
};

#endif /* SRC_VEHICLE_H_ */
