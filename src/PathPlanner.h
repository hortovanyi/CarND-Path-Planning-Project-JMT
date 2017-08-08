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
  vector<double> previous_path_x;
  vector<double> previous_path_y;

  vector<double> last_x_vals;
  vector<double> last_y_vals;

  vector<double> traj_s_vals;
  vector<double> traj_d_vals;

  double end_path_s_prev;
  double end_path_d_prev;

  int prediction_horizon = 5;  // look predictions out
  int prediction_outlook = 10; // prepare predictions

  double revise_behaviour_interval = 1.6f; // how many seconds until next behaviour state
  double behaviour_ttl; // time to live of this behaviour

  double revise_trajectory_interval = 1.0f;
  double trajectory_ttl; // time to live of this trajectory

  constexpr static int n_path_points = 50; // number of trajectory points to return

  constexpr static double point_path_interval = 0.02f; // every 20ms the car will move exactly to the next point;


  Vehicle * ego = nullptr;

	PathPlanner(HighwayMap * highway_map);
	void UpdateEgo(Vehicle * ego);
	void UpdateSensorFusion(json sensor_fusion);
	void UpdateBehaviour();
	double SimulatorTimeElapsed();
	int SimulatorPointsConsumed();
	void PopConsumedTrajectoryPoints(int n);
	void UpdatePreviousPath(json previous_path_x, json previous_path_y);
	void UpdatePreviousEndPath(double end_path_s, double end_path_d);

	tuple<vector<double>,vector<double>> NewPathPlan();
	tuple<vector<double>,vector<double>> NextTrajectory();

};

#endif /* SRC_PATHPLANNER_H_ */
