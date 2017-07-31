/*
 * Prediction.h
 *
 *  Created on: 27 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_PREDICTION_H_
#define SRC_PREDICTION_H_

#include <map>
#include <string>
#include <vector>
#include "SensorFusion.h"

using namespace std;

class Prediction {
 private:
 public:
  SensorFusion * sensor_fusion;
  map<int, vector<vector<double> > > predictions;

  Prediction(SensorFusion * sensor_fusion);

  void GeneratePredictions(int horizon);
};

#endif /* SRC_PREDICTION_H_ */
