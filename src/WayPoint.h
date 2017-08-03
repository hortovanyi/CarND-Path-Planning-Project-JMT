/*
 * WayPoint.h
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_WAYPOINT_H_
#define SRC_WAYPOINT_H_
#include <math.h>
#include "spline.h"
#include <iostream>

using namespace std;
using namespace tk;

class WayPoint {
private:
  // The max s value before wrapping around the track back to 0
  double max_s;

public:
	double x;
	double y;
	double s;
	double dx;
	double dy;
	WayPoint * next;
	WayPoint * prev;

	struct splines_struct {
	  spline sx;
	  spline sy;
	  spline dx;
	  spline dy;
	} splines;

	WayPoint(double x, double y, double s, double max_s, double dx, double dy);

	double distance(const WayPoint& other) const;
	double distance_to(double x1, double y1) const;

	double delta_s(WayPoint * other) const;

	bool isFirst() const;
	bool isLast() const;


	void InitSplines();

	vector<double> getXY(double s, double d);
};

#endif /* SRC_WAYPOINT_H_ */
