/*
 * SensorFusion.h
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_SENSORFUSION_H_
#define SRC_SENSORFUSION_H_

#include <vector>
#include "Vehicle.h"
#include "HighwayMap.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class SensorFusion {

 public:
  vector<Vehicle> vehicles;
  HighwayMap * highway_map;
	SensorFusion(json sensor_fusion, HighwayMap * highway_map);
	SensorFusion();

 	vector<Vehicle> Vehicles();
};

#endif /* SRC_SENSORFUSION_H_ */
