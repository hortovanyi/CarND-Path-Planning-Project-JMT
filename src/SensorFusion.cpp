/*
 * SensorFusion.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "SensorFusion.h"

SensorFusion::SensorFusion(json sensor_fusion, HighwayMap * highway_map) {
  this->highway_map = highway_map;
  for (auto v : sensor_fusion) {
    double lane = highway_map->LaneFrenet(v[6]);
    Vehicle vehicle(int(v[0]),v[1],v[2],v[3],v[4],v[5],v[6],lane);
    vehicles.push_back(vehicle);
  }
}

SensorFusion::SensorFusion() {
  highway_map = nullptr;
}

vector<Vehicle> SensorFusion::Vehicles() {
  return vehicles;
}
