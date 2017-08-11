/*
 * Vehicle.h
 *
 *  Created on: 23 Jul. 2017
 *      Author: nick
 */

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <sstream>

using namespace std;

using predictionsType = map<int, vector<vector<double>>>;

class Vehicle {

 public:

  struct collider{
    bool collision ; // is there a collision?
    int  time; // time collision happens
  };

  struct trajectory_struct{
    int proposed_lane;
    double avg_speed;
    double max_accel;
    double rms_acceleration;
    int closest_approach;
    int closest_behind;
    bool collides;
    int collides_at;
  } trajectory_data;

  struct state_struct {
    double s;
    double v; // s dot
    double a; // s dot dot
    double d;
    vector<double> state; //  s, s_dot, s_dot_dot, d, d_dot, d_dot_dot
    int lane;
    double t;
  } initial, goal, final;

  int id;
  double x;
  double y;
  double s; // frenet s
  double d; // frenet d
  int lane; // highway lane
  int proposed_lane; // used for prepare lane change
  double vx;
  double vy;
  double v; // speed in meters/sec
  double a; // acceleration



  double yaw; // radians
  double speed; // mph
  int points_consumed; // trajectory values consumed getting ego to here;

  // note these apply for ego vehicle
  vector<double> state; //  s, s_dot, s_dot_dot, d, d_dot, d_dot_dot

  string behaviour_state;
  double target_speed; // miles per hour
  double max_acceleration; // meters per second per second

  int preferred_buffer = 1; // impacts "keep lane" behavior.

  int goal_lane = 2 ; // stay in the middle lane if possible;

  Vehicle * prev_ego = nullptr; // needed to calculate acceleration


  // constructor for sensor fusion
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d, int lane);

  // constructor for ego vehicle
  Vehicle(double x, double y, double s, double d, int lane, double angle, double speed, int poinst_consumed, string state, Vehicle * ego_prev);

  // constructor for copying
  Vehicle(Vehicle *obj);

  double YawDeg();

  double deg2rad(double x);
  double rad2deg(double x);

  double vxvy2v(double vx, double vy);
  double distance(Vehicle * other);

  string display();
  string StateDisplay();

  vector<string> SuccessorStates(string current_state);
  vector<string> PossibleStates(string current_state);
  void UpdateBehaviour(predictionsType predictions);
  string NextBehaviour(predictionsType predictions);

  void increment(int dt);
  vector<double> StateAt(int t);
  bool collides_with(Vehicle other, int at_time);

  vector<double> TrajectoryStateAt(double t);

  collider will_collide_with(Vehicle other, int timesteps);

  vector<vector<double> > GeneratePredictions(int horizon);

  vector<Vehicle> TrajectoryForBehaviour(string state, predictionsType predictions, int horizon);

  // realise behaviour_state transitions
  void RealiseState(predictionsType predictions);
  void RealiseConstantSpeed();
  void RealiseKeepLane(predictionsType predictions);
  void RealiseLaneChange(predictionsType predictions, string direction);
  void RealisePrepLaneChange(predictionsType predictions, string direction);

  double _MaxAccelForLane(predictionsType predictions, int lane, double s);

  // cost functions
  double ChangeLaneCost(vector<Vehicle> trajectory, predictionsType predictions,int proposed_lane);
  double InefficiencyCost(vector<Vehicle> trajectory, predictionsType predictions, double target_speed);
  double CollisionCost(vector<Vehicle> trajectory, predictionsType predictions);
  double BufferCost(vector<Vehicle> trajectory, predictionsType predictions);

  double CalculateCost(vector<Vehicle> trajectory, predictionsType predictions, int horizon);

  void UpdateTrajectoryData(vector<Vehicle> trajectory, predictionsType predictions, int horizon);

  map<string,double> cost_levels;
  void InitCostLevels();

  predictionsType FilterPredictionsByLane(predictionsType predictions, int lane);
  bool CheckCollision(Vehicle * ego_trajected, double s_previous, double s_now);

//  void UpdateState(Prediction * prediction);

 private:
  // value taken from http://study.com/academy/lesson/how-to-convert-meters-per-second-to-miles-per-hour.html
  constexpr static double metersPerSecRatioMilesPerHr = 2.236936292;
  double _CalcAcceleration();

  // should really put this is constants or helpers file
  constexpr static int lane_width=4;

};

#endif /* SRC_VEHICLE_H_ */
