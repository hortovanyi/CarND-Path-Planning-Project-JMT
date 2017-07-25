/*
 * HighwayMap.h
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_HIGHWAYMAP_H_
#define SRC_HIGHWAYMAP_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "WayPoint.h"
#include "spline.h"
#include <limits>
#include <cmath>


using namespace std;

class HighwayMap {
private:
	vector<WayPoint *> way_points_;

public:
	HighwayMap(const string& map_file_);

	void LoadWaypoints(const string& map_file_);

	vector<WayPoint *> Waypoints() const;

	WayPoint * ClosestWaypoint(double x, double y) const;
	WayPoint * NextWaypoint(double x, double y, double theta) const;

	double distance(const WayPoint & wp0, const WayPoint& wp1) const;

	vector<double> getFrenet(double x, double y, double theta) const;
	vector<double> getXY(double s, double d) const;

	int LaneFrenet(double d) const;

	double EuclidianDistance(double x1, double y1, double x2, double y2) const;

	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	// lane width in meters
	double lane_wdith = 4;
};

#endif /* SRC_HIGHWAYMAP_H_ */
