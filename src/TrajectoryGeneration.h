/*
 * TrajectoryGeneration.h
 *
 *  Created on: 1 Aug. 2017
 *      Author: nick
 */

#ifndef SRC_TRAJECTORYGENERATION_H_
#define SRC_TRAJECTORYGENERATION_H_

#include "HighwayMap.h"
#include <vector>
#include <tuple>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class TrajectoryGeneration {
 public:
  HighwayMap * highway_map;


  // every 20ms the car will move exactly to the next point;
  double point_path_interval_ms = 20;

  TrajectoryGeneration(HighwayMap * highway_map);

  // Jerk Minimising Trajectory
  vector<double> JMT(vector< double> start, vector <double> end, double T);

  double PolynomialEquate(vector<double> coefficients, double T);
  vector<double> DifferentiatePolynominal(vector<double> coefficients);

  tuple<vector<double>,vector<double>> TrajectoryFrenetNext(vector< double> s_initial, vector <double> s_final, double d_initial, double d_final, double T);

};

#endif /* SRC_TRAJECTORYGENERATION_H_ */
