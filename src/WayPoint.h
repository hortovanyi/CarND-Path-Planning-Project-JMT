/*
 * WayPoint.h
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_WAYPOINT_H_
#define SRC_WAYPOINT_H_
#include <math.h>

class WayPoint {

public:
	double x;
	double y;
	double s;
	double dx;
	double dy;
	WayPoint * next;
	WayPoint * prev;

	WayPoint(double x, double y, double s, double dx, double dy);

	double distance(const WayPoint& other) const;
	double distance_to(double x1, double y1) const;
};

#endif /* SRC_WAYPOINT_H_ */
