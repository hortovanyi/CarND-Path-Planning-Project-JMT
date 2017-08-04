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
    int id = int(v[0]);
    double x = v[1];
    double y = v[2];
    double vx = v[3];
    double vy = v[4];
    double s = v[5];
    double d = v[6];

    int lane = highway_map->LaneFrenet(d);
    // only add if in lanes near us is positive (bug with data supplied)
    if (lane < 0) {
      continue;
//      cout <<"* d " << d;
//      // let calculate the correct d
//      auto frenet = highway_map->getFrenet(x,y,atan2(vy,vx));
//      d=frenet[1];
//      lane = highway_map->LaneFrenet(d);
//      cout << " new d " << d;
    }

//    cout << "sf id " << id << " x " << x << " y " << y << " vx " << vx << " vy " << vy;
//    cout << " s " << s << " d " << d << " lane " << lane << endl;

    Vehicle vehicle(id,x,y,vx,vy,s,d,lane);
    vehicles.push_back(vehicle);
  }
}

SensorFusion::SensorFusion() {
  highway_map = nullptr;
}

vector<Vehicle> SensorFusion::Vehicles() {
  return vehicles;
}
