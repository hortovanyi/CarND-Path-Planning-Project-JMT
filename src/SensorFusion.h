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
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class SensorFusion {

private:
  vector<Vehicle> vehicles;

public:
	SensorFusion(json sensor_fusion);
	SensorFusion();

 	vector<Vehicle> Vehicles();
};

#endif /* SRC_SENSORFUSION_H_ */
