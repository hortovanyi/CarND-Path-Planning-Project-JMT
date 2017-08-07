/*
 * PathPlanner.h
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_PATHPLANNER_H_
#define SRC_PATHPLANNER_H_

#include <vector>
#include <tuple>
#include <math.h>
#include "HighwayMap.h"
#include "WayPoint.h"
#include "SensorFusion.h"
#include "TrajectoryGeneration.h"
#include "Vehicle.h"
#include "Prediction.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class PathPlanner {

public:
  HighwayMap * highway_map;
  TrajectoryGeneration * trajectory_generation;
  SensorFusion sensor_fusion;
  predictionsType predictions;
  Prediction * prediction;
  vector<double>  previous_path_x;
  vector<double>  previous_path_y;
  double end_path_s_prev;
  double end_path_d_prev;

  int prediction_horizon = 5;  // look predictions out
  int prediction_outlook = 10; // prepare predictions

  double revise_behaviour_interval = 1.6; // how many seconds until next behaviour state
  double behaviour_ttl; // time to live of this behaviour



  Vehicle * ego = nullptr;

	PathPlanner(HighwayMap * highway_map);
	void UpdateEgo(Vehicle * ego);
	void UpdateSensorFusion(json sensor_fusion);
	void UpdatePreviousPath(json previous_path_x, json previous_path_y);
	void UpdatePreviousEndPath(double end_path_s, double end_path_d);

	tuple<vector<double>,vector<double>> NewPathPlan();

};

#endif /* SRC_PATHPLANNER_H_ */
