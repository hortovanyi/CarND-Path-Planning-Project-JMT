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
#include <limits>
#include <cmath>

using namespace std;

class HighwayMap {
private:
	vector<WayPoint *> way_points_;

public:
	HighwayMap(const string& map_file_);

	vector<WayPoint *> way_points() const;

	WayPoint * ClosestWaypoint(double x, double y) const;
	WayPoint * NextWaypoint(double x, double y, double theta) const;

	double distance(const WayPoint & wp0, const WayPoint& wp1) const;

	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
};

#endif /* SRC_HIGHWAYMAP_H_ */
