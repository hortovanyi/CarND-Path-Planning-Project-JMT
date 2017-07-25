/*
 * SensorFusion.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "SensorFusion.h"

SensorFusion::SensorFusion(json sensor_fusion) {
  for (auto v : sensor_fusion) {
    Vehicle vehicle(int(v[0]),v[1],v[2],v[3],v[4],v[5],v[6]);
    vehicles.push_back(vehicle);
  }
}

SensorFusion::SensorFusion() {
}

vector<Vehicle> SensorFusion::Vehicles() {
  return vehicles;
}
