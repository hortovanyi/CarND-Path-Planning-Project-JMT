/*
 * TrajectoryGeneration.cpp
 *
 *  Created on: 1 Aug. 2017
 *      Author: nick
 */

#include "TrajectoryGeneration.h"

const vector<double> TrajectoryGeneration :: delta ({0,0,0,0,0,0});
const vector<double> TrajectoryGeneration :: sigma_s ({10.0,4.0,2.0});
const vector<double> TrajectoryGeneration :: sigma_d ({1.0, 1.0, 1.0});

TrajectoryGeneration::TrajectoryGeneration(HighwayMap * highway_map) {
  this->highway_map = highway_map;
  InitCostLevels();
}

void TrajectoryGeneration::InitCostLevels() {
  cost_levels["time_diff_cost"] = pow(10,2);
  cost_levels["s_diff_cost"] = pow(10,4);
  cost_levels["d_diff_cost"] = pow(10,4);
  cost_levels["efficiency_cost"] = pow(10,2);
  cost_levels["max_jerk_cost"] = pow(10,2);
  cost_levels["total_jerk_cost"] = pow(10,3);
  cost_levels["collision_cost"] = pow(10,6);
  cost_levels["buffer_cost"] = pow(10,1);
  cost_levels["max_accel_cost"] = pow(10,2);
  cost_levels["total_accel_cost"] = pow(10,3);
}

vector<double> TrajectoryGeneration::JMT(vector<double> start,
                                         vector<double> end, double T) {
  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.

   INPUTS

   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]

   end   - the desired end state for vehicle. Like "start" this is a
   length three array.

   T     - The duration, in seconds, over which this maneuver should occur.

   OUTPUT
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

   EXAMPLE

   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  // Quintic Polynomial Solver
  MatrixXd A = MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T,
       3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
       6 * T, 12 * T * T, 20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
       end[1] - (start[1] + start[2] * T),
       end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = { start[0], start[1], .5 * start[2] };
  for (int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result;

}

double TrajectoryGeneration::PolynomialEquate(vector<double> coefficients,
                                              double T) {
  double x = 0.0f;
  for (unsigned i = 0; i < coefficients.size(); i++){
    x += coefficients[i] * pow(T,i);
  }
  return x;
}

vector<double> TrajectoryGeneration::DifferentiatePolynominal(vector<double> coefficients) {
  vector<double> new_coefficients;

  for (int i =1; i < coefficients.size(); i++) {
    new_coefficients.push_back(coefficients[i] * (double)i);
  }
  return new_coefficients;
}

vector<double> TrajectoryGeneration::StateFromPolynominal(vector<double> coefficients, double T) {
  vector<double> state;

  state.push_back(PolynomialEquate(coefficients, T));

  // dot
  coefficients = DifferentiatePolynominal(coefficients);
  state.push_back(PolynomialEquate(coefficients, T));

  // dot dot
  coefficients = DifferentiatePolynominal(coefficients);
  state.push_back(PolynomialEquate(coefficients, T));

  return state;
}

tuple<vector<double>,vector<double>, double> TrajectoryGeneration::BestFinalGoal(vector <double> s_initial, vector <double> d_initial, vector <double> s_goal, vector <double> d_goal, Vehicle * ego, vector<double> delta, Prediction * prediction, double goal_T) {


  cout << "Best Trajectory ";
  cout << "s_initial ";
  for (auto s: s_initial) cout << s << " ";
  cout << "d_initial ";
  for (auto s: d_initial) cout << s << " ";
  cout << " s_goal ";
  for (auto s: s_goal) cout << s << " ";
  cout << " d_goal ";
  for (auto s: d_goal) cout << s << " ";
  cout << " n pred " << prediction->predictions.size();
  cout << " goal_T " << goal_T;
  cout << endl;

  double timestep = 0.5f;
  int n_timestep = 4;
  int n_samples = 10;
  double t = goal_T - n_timestep * timestep;

  Vehicle target(ego);
  target.s = s_goal[0];
  target.d = d_goal[0];
  target.v = s_goal[1];
  target.a = s_goal[2];
  target.lane = highway_map->LaneFrenet(target.d);

  vector<tuple<vector<double>,vector<double>, double>> all_goals;
  while(t <= goal_T + n_timestep * timestep) {
    // TODO add in delta addition
    auto state = target.StateAt(t);
    vector<double> goal_s {state[1],state[2], state[3]};
    vector<double> goal_d {target.d, 0 , 0};

    all_goals.push_back(make_tuple(goal_s, goal_d, t));
    for (int i = 0; i < n_samples; i++) {
      vector<double> new_s_goal, new_d_goal;
      tie(new_s_goal, new_d_goal) = PertubedGoal(goal_s, goal_d);
      all_goals.push_back(make_tuple(new_s_goal, new_d_goal, t));
    }
    t += timestep;
  }

//  for (auto goal: all_goals) {
//    vector<double> s_goal, d_goal;
//    double t_goal;
//    tie(s_goal, d_goal, t_goal) = goal;
//    cout << "sg ";
//    for (auto s:s_goal) cout << s << " ";
//    cout << "dg ";
//    for (auto d:d_goal) cout << d << " ";
//    cout << "t " << t_goal << endl;
//  }
//  cout << endl;

  trajectoryType best_trajectory;
  tuple<vector<double>,vector<double>, double> best_goal;
  double best_cost=numeric_limits<double>::max();

  for (auto goal: all_goals) {
    vector<double> s_goal, d_goal;
    double t;
    tie(s_goal, d_goal, t) = goal;

    cout << "sg ";
    for (auto s:s_goal) cout << s << " ";
    cout << "dg ";
    for (auto d:d_goal) cout << d << " ";
    cout << "t " << t;

    auto s_coefficients = JMT(s_initial, s_goal, t);
    auto d_coefficients = JMT(d_initial, d_goal, t);

    cout << " scoeffs ";
    for (auto c: s_coefficients) cout << c << " ";
    cout << " dcoeffs ";
    for (auto c: d_coefficients) cout << c << " ";

    trajectoryType trajectory = make_tuple(s_coefficients, d_coefficients, t);

    double cost = CalculateCost(trajectory, ego, delta,goal_T,prediction);
    if (cost < best_cost){
      best_trajectory = trajectory;
      best_goal = goal;
      best_cost = cost;
    }

    cout << endl;
  }

  return best_goal;
}

tuple<vector<double>,vector<double>> TrajectoryGeneration::PertubedGoal(vector<double> goal_s, vector<double> goal_d) {

  vector<double> new_s_goal;
  for (int i = 0; i < sigma_s.size(); i++) {
    double s = goal_s[i];
    double sig =  sigma_s[i];
    normal_distribution<double> dist_s(s, sig);
    double s_rand=dist_s(sgen);
    new_s_goal.push_back(s_rand);
  }

  vector<double> new_d_goal;
  for (int i = 0; i <  sigma_d.size(); i++) {
    double d = goal_d[i];
    double sig = sigma_d[i];
    normal_distribution<double> dist_d(d, sig);
    double d_rand=dist_d(dgen);
    new_d_goal.push_back(d_rand);
  }

  return make_tuple(new_s_goal, new_d_goal);
}

double TrajectoryGeneration::CalculateCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction) {
  double cost = 0.0f;

  auto tdcost = TimeDiffCost(trajectory,ego,delta,T,prediction);
  cout << " TimeDiff " << tdcost;
  cost += tdcost;
  auto sdiffcost = SDiffCost(trajectory,ego,delta,T,prediction);
  cout << " SDiff " << sdiffcost;
  cost += sdiffcost;
  auto ddiffcost = DDiffCost(trajectory,ego,delta,T,prediction);
  cout << " DDiff " << ddiffcost;
  cost += ddiffcost;
  auto efficiencycost = EfficiencyCost(trajectory,ego,delta,T,prediction);
  cout << " Efficiency " << efficiencycost;
  cost += efficiencycost;
  auto maxjerkcost = MaxJerkCost(trajectory,ego,delta,T,prediction);
  cout << " MaxJerk " << maxjerkcost;
  cost += maxjerkcost;
  auto totaljerkcost = TotalJerkCost(trajectory,ego,delta,T,prediction);
  cout << " TotalJerk " << totaljerkcost;
  cost += totaljerkcost;
  auto collisioncost  = CollisionCost(trajectory,ego,delta,T,prediction);
  cout << " Collision " << collisioncost;
  cost += collisioncost;
  auto buffercost = BufferCost(trajectory,ego,delta,T,prediction);
  cout << " Buffer " << buffercost;
  cost += buffercost;
  auto maxaccelcost = MaxAccelCost(trajectory,ego,delta,T,prediction);
  cout << " MaxAccel " << maxaccelcost;
  cost += maxaccelcost;
  auto totalaccelcost = TotalAccelCost(trajectory,ego,delta,T,prediction);
  cout << " TotalAccel " << totalaccelcost;
  cost += totalaccelcost;

  cout << " cost " << cost;
  return cost;
}


tuple<vector<double>,vector<double>> TrajectoryGeneration::TrajectoryFrenetNext(vector< double> s_initial, vector <double> s_final, vector <double> d_initial, vector <double> d_final, double T) {

  last_trajectory.s_initial = s_initial;
  last_trajectory.s_final = s_final;
  last_trajectory.d_initial = d_initial;
  last_trajectory.d_final = d_final;
  last_trajectory.T = T;

  vector<double> next_s_vals;
  vector<double> next_d_vals;

  vector <double>s_coefficients = JMT(s_initial, s_final, T);
  // for d, its reasonable to not need velocity and acceleration (for lane changes)
  vector <double>d_coefficients = JMT(d_initial, d_final, T);

  double next_s = s_initial[0];
  double next_d = d_initial[0];
  next_s_vals.push_back(next_s);
  next_d_vals.push_back(next_d);

  double t = 0.0f;
  for (int i = 1; i < (T/(point_path_interval)); i++){
    t+=point_path_interval;
    next_s = PolynomialEquate(s_coefficients,t);
    next_d = PolynomialEquate(d_coefficients,t);

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(next_d);
  }

  last_path.s_vals = next_s_vals;
  last_path.d_vals = next_d_vals;

  return make_tuple(next_s_vals, next_d_vals);
}


double TrajectoryGeneration::Normalise(double x) const{
  return 2.0f / (1.0f + exp(-x)) - 1.0f;
}

double TrajectoryGeneration::NearestApproach(trajectoryType trajectory, Vehicle * vehicle) {
  double closest = 99999;

  vector<double> s_coefficients, d_coefficients;
  double T;
  tie(s_coefficients, d_coefficients, T)=trajectory;

  for (int i = 0; i < 100; i++) {
    double t = double(i) /100 * T;
    double s_cur = PolynomialEquate(s_coefficients, T);
    double d_cur = PolynomialEquate(d_coefficients, T);

    vector<double> state = vehicle->StateAt(t);
    double s_targ = state[0];
    double d_targ = highway_map->FrenetLaneCenter(state[0]);

    double dist = sqrt(pow(s_cur-s_targ,2)+pow(d_cur-d_targ,2));
    if (dist < closest)
      closest = dist;
  }

  return closest;
}

double TrajectoryGeneration::NearestApproachToAnyVehicle(trajectoryType trajectory, vector<Vehicle> * vehicles){
  double closest = 99999;

  for (auto &vehicle: *vehicles) {
    double dist = NearestApproach(trajectory, &vehicle);
    if (dist < closest)
      closest = dist;
  }

  return closest;
}

double TrajectoryGeneration::TimeDiffCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  cost = Normalise(fabs(t-T)/T) ;

  return cost * cost_levels["time_diff_cost"];
}

double TrajectoryGeneration::SDiffCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto target = ego->StateAt(T);

  vector<double> s_targ {target[1],target[2],target[3]};

  vector<double> S = StateFromPolynominal(s_coefficients, T);

  double cost = 0.0f;
  for (int i=0; i < s_targ.size(); i++) {
    double actual = S[i];
    double expected = s_targ[i];
    double sigma = sigma_s[i];

    double diff = fabs(actual-expected);
    cost += Normalise(diff/sigma);
  }


  return cost * cost_levels["s_diff_cost"];
}

double TrajectoryGeneration::DDiffCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto target = ego->StateAt(T);

  vector<double> D = StateFromPolynominal(d_coefficients, T);

  double d = this->highway_map->FrenetLaneCenter(target[0]);
  vector<double> d_targ {d,0.0f,0.0f};

  double cost = 0.0f;
  for (int i=0; i < d_targ.size(); i++) {
    double actual = D[i];
    double expected = d_targ[i];
    double sigma = sigma_d[i];

    double diff = fabs(actual-expected);
    cost += Normalise(diff/sigma);
  }

  return cost * cost_levels["d_diff_cost"];
}

double TrajectoryGeneration::CollisionCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  double nearest = NearestApproachToAnyVehicle(trajectory, prediction->vehicles);
  if (nearest < 2*vehicle_radius)
    cost=1.0f;

  return cost * cost_levels["collision_cost"];
}

double TrajectoryGeneration::BufferCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  double nearest = NearestApproachToAnyVehicle(trajectory, prediction->vehicles);

  cost = Normalise(2*vehicle_radius/nearest);

  return cost * cost_levels["buffer_cost"];
}

double TrajectoryGeneration::StaysOnRoadCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;

  // TODO stays on road cost
  return cost;
}

double TrajectoryGeneration::ExceedsSpeedLimitCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;

  // TODO exceeds speed limit cost
  return cost;
}

double TrajectoryGeneration::EfficiencyCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto target = ego->StateAt(t);

  vector<double> s_targ {target[1],target[2],target[3]};

//  vector<double> S = StateFromPolynominal(s_coefficients, t);
//  double avg_v = S[0]/t;
  double avg_v=PolynomialEquate(s_coefficients,t)/t;

  double targ_s = s_targ[0];
  double targ_v = targ_s/t;

  cost = Normalise(2.0f*(targ_v-avg_v)/avg_v);

  return cost * cost_levels["efficiency_cost"];
}

double TrajectoryGeneration::MaxAccelCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto s_dot = DifferentiatePolynominal(s_coefficients);
  auto s_dot_dot = DifferentiatePolynominal(s_dot);

  double total_acc = 0.0f;

  double dt = T / 100.0f;
  for (int i = 0; i < 100; i++) {
    t = dt * i;
    double acc = PolynomialEquate(s_dot_dot, t);
    total_acc += fabs(acc*dt);
  }
  double acc_per_second = total_acc / T;

  cost = Normalise(acc_per_second/expected_acc_in_one_sec);
  return cost * cost_levels["max_accel_cost"];
}

double TrajectoryGeneration::TotalAccelCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto s_dot = DifferentiatePolynominal(s_coefficients);
  auto s_dot_dot = DifferentiatePolynominal(s_dot);

  double max_acc = 0.0f;

  for (int i=0; i < 100; i++) {
    double acc = fabs(PolynomialEquate(s_dot_dot, T/100*i));
    if (acc>max_acc)
      max_acc=acc;
  }

  if (max_acc > max_accel)
    cost = 1.0f;

  return cost * cost_levels["total_accel_cost"];
}

double TrajectoryGeneration::MaxJerkCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  // TODO what about d jerk??
  auto s_dot = DifferentiatePolynominal(s_coefficients);
  auto s_d_dot = DifferentiatePolynominal(s_dot);
  auto s_jerk = DifferentiatePolynominal(s_d_dot);

  double jerk_max = 0.0f;

  for (int i=0; i < 100; i++) {
    double jerk = fabs(PolynomialEquate(s_jerk, T/100*i));
    if (jerk>jerk_max)
      jerk_max=jerk;
  }

  if (jerk_max > max_jerk)
    cost = 1.0f;

  return cost * cost_levels["max_jerk_cost"];
}

double TrajectoryGeneration::TotalJerkCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  // TODO what about d jerk??
  auto s_dot = DifferentiatePolynominal(s_coefficients);
  auto s_d_dot = DifferentiatePolynominal(s_dot);
  auto s_jerk = DifferentiatePolynominal(s_d_dot);

  double total_jerk = 0.0f;

  double dt = T / 100.0f;
  for (int i = 0; i < 100; i++) {
    t = dt * i;
    double jerk = PolynomialEquate(s_jerk, t);
    total_jerk += fabs(jerk*dt);
  }
  double jerk_per_second = total_jerk / T;

  cost = Normalise(jerk_per_second/expected_jerk_in_one_sec);
  return cost * cost_levels["total_jerk_cost"];
}
