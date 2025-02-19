/*
 * TrajectoryGeneration.h
 *
 *  Created on: 1 Aug. 2017
 *      Author: nick
 */

#ifndef SRC_TRAJECTORYGENERATION_H_
#define SRC_TRAJECTORYGENERATION_H_

#include <random>
#include "HighwayMap.h"
#include "Vehicle.h"
#include "Prediction.h"
#include <vector>
#include <tuple>
#include "Eigen-3.3/Eigen/Dense"
#include <math.h>
#include <sstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using trajectoryType = tuple<vector<double>,vector<double>, double>;

class TrajectoryGeneration {

 public:
  HighwayMap * highway_map;
  const static vector<double> delta;

  vector<double> sigma_s;
  vector<double> sigma_d;

  constexpr static double sigma_t = 2.0f;

  constexpr static double max_jerk = 50.0f; // m/s/s/s
  constexpr static double max_accel = 10.0f; // m/s/s
  constexpr static double max_speed = 22.0f; // m/s - equates to 49.2126 MPH

  constexpr static double vehicle_radius = 1.5;

  constexpr static double expected_jerk_in_one_sec = 10.0f; // m/s/s
  constexpr static double expected_acc_in_one_sec = 2.0f; // m/s

  struct solver_struct {
    vector <double> s_initial;
    vector <double> s_final;
    vector <double> d_initial;
    vector <double> d_final;
    double T;
  } last_trajectory;

  struct vals_struct {
    vector <double> s_vals;
    vector <double> d_vals;
  } last_path;

  trajectoryType best_trajectory;

  // every 20ms the car will move exactly to the next point;
  constexpr static double point_path_interval = 0.02f;

  map<string,double> cost_levels;
  void InitCostLevels();

  TrajectoryGeneration(HighwayMap * highway_map);

  // Jerk Minimising Trajectory
  vector<double> JMT(vector< double> start, vector <double> end, double T);

  tuple<vector<double>,vector<double>, double> BestFinalGoal(vector <double> s_initial, vector <double> d_initial, vector <double> s_goal, vector <double> d_goal, Vehicle * ego, vector<double> delta, Prediction * prediction, double goal_T);
  tuple<vector<double>,vector<double>> PertubedGoal(vector<double> goal_s, vector<normal_distribution<double>> s_sigma_dists,
                                                    vector<double> goal_d, vector<normal_distribution<double>> d_sigma_dists);

  double PolynomialEquate(vector<double> coefficients, double T);

  vector<double> DifferentiateCoefficients(vector<double> coefficients);
  vector<double> StateFromCoefficients(vector<double> coefficients, double T);


  tuple<vector<double>,vector<double>> TrajectoryFrenetNext(vector< double> s_initial, vector <double> s_final, vector <double> d_initial, vector<double> d_final, double T);



  double CalculateCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  string DisplayCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  string DisplayTrajectoryType(trajectoryType trajectory);

  double Normalise(double x) const;
  double NearestApproach(trajectoryType trajectory, Vehicle * vehicle);
  double NearestApproachToAnyVehicle(trajectoryType trajectory, vector<Vehicle> *vehicles);

  double TimeDiffCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double SDiffCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double DDiffCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double CollisionCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double BufferCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double StaysOnRoadCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double ExceedsSpeedLimitCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double EfficiencyCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double MaxAccelCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double TotalAccelCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double MaxJerkCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
  double TotalJerkCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction);
};

#endif /* SRC_TRAJECTORYGENERATION_H_ */
