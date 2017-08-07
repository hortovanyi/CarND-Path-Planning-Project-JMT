/*
 * Prediction.cpp
 *
 *  Created on: 27 Jul. 2017
 *      Author: nick
 */

#include "Prediction.h"

Prediction::Prediction(SensorFusion * sensor_fusion) {
  this->sensor_fusion = sensor_fusion;
  this->vehicles = &sensor_fusion->vehicles;
}

void Prediction::GeneratePredictions(int horizon){
  predictions.clear();

  for (auto &vehicle: sensor_fusion->vehicles){
    cout << "pred v.id: " << vehicle.id;
    predictions[vehicle.id]=vehicle.GeneratePredictions(horizon);
    for (auto const& location: predictions[vehicle.id]) {
          cout << "  " << location[0] << "," << location[1];
     }
    cout << endl;
  }
}
