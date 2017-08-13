/*
 * TrajectoryGeneration.cpp
 *
 *  Created on: 1 Aug. 2017
 *      Author: nick
 */

#include "TrajectoryGeneration.h"

const vector<double> TrajectoryGeneration :: delta = {0,0,0,0,0,0};


TrajectoryGeneration::TrajectoryGeneration(HighwayMap * highway_map) {
  this->highway_map = highway_map;
  InitCostLevels();
}

void TrajectoryGeneration::InitCostLevels() {


  sigma_s = {20.0,8.0,4.0};
  sigma_d = {1.0, 1.0, 1.0};

  cout << "sigma_s ";
  for (auto s: sigma_s) cout << s <<" ";
  cout << "sigma_d ";
  for (auto s: sigma_d) cout << s <<" ";
  cout << endl;

  cost_levels["time_diff_cost"] = pow(10,4);
  cost_levels["s_diff_cost"] = pow(10,4);
  cost_levels["d_diff_cost"] = pow(10,3);
  cost_levels["efficiency_cost"] = pow(10,3);
  cost_levels["max_jerk_cost"] = pow(10,2);
  cost_levels["total_jerk_cost"] = pow(10,4);
  cost_levels["collision_cost"] = pow(10,6);
  cost_levels["buffer_cost"] = pow(10,3);
  cost_levels["max_accel_cost"] = pow(10,2);
  cost_levels["total_accel_cost"] = pow(10,4);
  cost_levels["exceeds_speed_cost"] = pow(10,5);
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

  vector<double> result = { start[0], start[1], .5f * start[2] };
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

vector<double> TrajectoryGeneration::DifferentiateCoefficients(vector<double> coefficients) {
  vector<double> new_coefficients;

  for (int i =1; i < coefficients.size(); i++) {
    new_coefficients.push_back(coefficients[i] * (double)i);
  }
  return new_coefficients;
}

vector<double> TrajectoryGeneration::StateFromCoefficients(vector<double> coefficients, double T) {
  vector<double> state;

  auto s = PolynomialEquate(coefficients, T);
  state.push_back(s);

  // dot
  auto dot_coefficients = DifferentiateCoefficients(coefficients);
  auto s_dot = PolynomialEquate(dot_coefficients, T);
  state.push_back(s_dot);

  // dot dot
  auto dot_dot_coefficients = DifferentiateCoefficients(dot_coefficients);
  auto s_dot_dot =PolynomialEquate(dot_dot_coefficients, T);
  state.push_back(s_dot_dot);

  return state;
}

tuple<vector<double>,vector<double>, double> TrajectoryGeneration::BestFinalGoal(vector <double> s_initial, vector <double> d_initial, vector <double> s_goal, vector <double> d_goal, Vehicle * ego, vector<double> delta, Prediction * prediction, double goal_T) {

  cout << "BestFinalGoal begin ";
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

  // create a JMT trajectory for this goal
  vector <double>s_coefficients = JMT(s_initial, s_goal, goal_T);
  vector <double>d_coefficients = JMT(d_initial, d_goal, goal_T);


//  double timestep = 0.25f;
//  double initial_v = s_initial[1];
//  if (initial_v < 8.0f)
//    timestep = 0.75f;
//  else if (initial_v < 16.0f )
//    timestep = 0.5f;

  double timestep = 0.5f;

  int n_timestep = 4;
  int n_samples = 10;
  double t = goal_T - n_timestep * timestep;

  Vehicle * target_vehicle(ego);
  target_vehicle->state={s_initial[0],s_initial[1],s_initial[2], d_initial[0], d_initial[1], d_initial[2]};


  vector<tuple<vector<double>,vector<double>, double>> all_goals;
  while(t <= goal_T + n_timestep * timestep) {

    // establish the goal states for this timestap
    vector<double> goal_s = StateFromCoefficients(s_coefficients,t);
    vector<double> goal_d = StateFromCoefficients(d_coefficients,t);

    // setup gaussian distributions
    vector<normal_distribution<double>> s_sigma_dists;
    for (int i = 0; i < sigma_s.size(); i++) {
      double s = goal_s[i];
      double sig =  sigma_s[i];
      normal_distribution<double> dist_s(s, sig);
      s_sigma_dists.push_back(dist_s);
    }

    vector<normal_distribution<double>> d_sigma_dists;
    for (int i = 0; i < sigma_s.size(); i++) {
      double d = goal_d[i];
      double sig =  sigma_s[i];
      normal_distribution<double> dist_d(d, sig);
      d_sigma_dists.push_back(dist_d);
    }

    // generate samples
    all_goals.push_back(make_tuple(goal_s, goal_d, t));
    for (int i = 0; i < n_samples; i++) {
      vector<double> new_s_goal, new_d_goal;
      tie(new_s_goal, new_d_goal) = PertubedGoal(goal_s, s_sigma_dists, goal_d, d_sigma_dists);
      if (new_s_goal <= goal_s)
        continue;
      all_goals.push_back(make_tuple(new_s_goal, new_d_goal, t));
    }
    t += timestep;
  }

  // find the best goal and trajectory;
  tuple<vector<double>,vector<double>, double> best_goal;
  double best_cost=numeric_limits<double>::max();

  for (auto goal: all_goals) {
    vector<double> s_g, d_g;
    double t;
    tie(s_g, d_g, t) = goal;

    cout << "sg ";
    for (auto s:s_g) cout << s << " ";
    cout << "dg ";
    for (auto d:d_g) cout << d << " ";
    cout << "t " << t;

    auto s_coefficients = JMT(s_initial, s_g, t);
    auto d_coefficients = JMT(d_initial, d_g, t);

    trajectoryType trajectory = make_tuple(s_coefficients, d_coefficients, t);

    cout << DisplayTrajectoryType(trajectory) << endl << "** " << DisplayCost(trajectory, target_vehicle, delta, goal_T,prediction);

    double cost = CalculateCost(trajectory, target_vehicle, delta, goal_T,prediction);
    if (cost < best_cost){
      best_trajectory = trajectory;
      best_goal = goal;
      best_cost = cost;
    }

    cout << endl;
  }

  cout << "Best Goal Costs " << DisplayCost(best_trajectory,ego,delta,goal_T,prediction) << endl;

  return best_goal;
}

tuple<vector<double>,vector<double>> TrajectoryGeneration::PertubedGoal(vector<double> goal_s, vector<normal_distribution<double>> s_sigma_dists,
                                                                        vector<double> goal_d, vector<normal_distribution<double>> d_sigma_dists) {
  random_device rd1,rd2;
  mt19937 e1(rd1()), e2(rd2());

  vector<double> new_s_goal;
  for (int i = 0; i < sigma_s.size(); i++) {
    normal_distribution<double> dist_s = s_sigma_dists[i];
    double s_rand=dist_s(e1);
    if (fabs(s_rand) < numeric_limits<double>::epsilon())
      s_rand =0.0;
    new_s_goal.push_back(s_rand);
  }

  vector<double> new_d_goal;
  for (int i = 0; i <  sigma_d.size(); i++) {
    normal_distribution<double> dist_d = d_sigma_dists[i];
    double d_rand=dist_d(e2);
    if (fabs(d_rand) < numeric_limits<double>::epsilon())
       d_rand =0.0;
    new_d_goal.push_back(d_rand);
  }

  return make_tuple(new_s_goal, new_d_goal);
}

double TrajectoryGeneration::CalculateCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction) {
  double cost = 0.0f;

  auto tdcost = TimeDiffCost(trajectory,ego,delta,T,prediction);
//  cout << " TimeDiff " << tdcost;
  cost += tdcost;
  auto sdiffcost = SDiffCost(trajectory,ego,delta,T,prediction);
//  cout << " SDiff " << sdiffcost;
  cost += sdiffcost;
  auto ddiffcost = DDiffCost(trajectory,ego,delta,T,prediction);
//  cout << " DDiff " << ddiffcost;
  cost += ddiffcost;
  auto efficiencycost = EfficiencyCost(trajectory,ego,delta,T,prediction);
//  cout << " Efficiency " << efficiencycost;
  cost += efficiencycost;
  auto maxjerkcost = MaxJerkCost(trajectory,ego,delta,T,prediction);
//  cout << " MaxJerk " << maxjerkcost;
  cost += maxjerkcost;
  auto totaljerkcost = TotalJerkCost(trajectory,ego,delta,T,prediction);
//  cout << " TotalJerk " << totaljerkcost;
  cost += totaljerkcost;
  auto collisioncost  = CollisionCost(trajectory,ego,delta,T,prediction);
//  cout << " Collision " << collisioncost;
  cost += collisioncost;
  auto buffercost = BufferCost(trajectory,ego,delta,T,prediction);
//  cout << " Buffer " << buffercost;
  cost += buffercost;
  auto maxaccelcost = MaxAccelCost(trajectory,ego,delta,T,prediction);
//  cout << " MaxAccel " << maxaccelcost;
  cost += maxaccelcost;
  auto totalaccelcost = TotalAccelCost(trajectory,ego,delta,T,prediction);
//  cout << " TotalAccel " << totalaccelcost;
  cost += totalaccelcost;
  auto exceedsspeedcost = ExceedsSpeedLimitCost(trajectory,ego,delta,T,prediction);
//  cout << " ExceedsSpeed " << exceedsspeedcost;
  cost += exceedsspeedcost;

//  cout << " cost " << cost;
  return cost;
}

string TrajectoryGeneration::DisplayCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction) {
  ostringstream oss;

  double cost = 0.0f;

  auto tdcost = TimeDiffCost(trajectory, ego, delta, T, prediction);
  oss << " TimeDiff " << tdcost;
  cost += tdcost;
  auto sdiffcost = SDiffCost(trajectory, ego, delta, T, prediction);
  oss << " SDiff " << sdiffcost;
  cost += sdiffcost;
  auto ddiffcost = DDiffCost(trajectory, ego, delta, T, prediction);
  oss << " DDiff " << ddiffcost;
  cost += ddiffcost;
  auto efficiencycost = EfficiencyCost(trajectory, ego, delta, T, prediction);
  oss << " Efficiency " << efficiencycost;
  cost += efficiencycost;
  auto maxjerkcost = MaxJerkCost(trajectory, ego, delta, T, prediction);
  oss << " MaxJerk " << maxjerkcost;
  cost += maxjerkcost;
  auto totaljerkcost = TotalJerkCost(trajectory, ego, delta, T, prediction);
  oss << " TotalJerk " << totaljerkcost;
  cost += totaljerkcost;
  auto collisioncost = CollisionCost(trajectory, ego, delta, T, prediction);
  oss << " Collision " << collisioncost;
  cost += collisioncost;
  auto buffercost = BufferCost(trajectory, ego, delta, T, prediction);
  oss << " Buffer " << buffercost;
  cost += buffercost;
  auto maxaccelcost = MaxAccelCost(trajectory, ego, delta, T, prediction);
  oss << " MaxAccel " << maxaccelcost;
  cost += maxaccelcost;
  auto totalaccelcost = TotalAccelCost(trajectory, ego, delta, T, prediction);
  oss << " TotalAccel " << totalaccelcost;
  cost += totalaccelcost;
  auto exceedsspeedcost = ExceedsSpeedLimitCost(trajectory, ego, delta, T,
                                                prediction);
  oss << " ExceedsSpeed " << exceedsspeedcost;
  cost += exceedsspeedcost;
  oss << " Total Cost " << cost;

  return oss.str();
}

string TrajectoryGeneration::DisplayTrajectoryType(trajectoryType trajectory){
  ostringstream oss;
  vector<double> s_coefficients, d_coefficients;
  double T;
  tie(s_coefficients, d_coefficients, T)=trajectory;

//  auto s_dot_coefficients=DifferentiateCoefficients(s_coefficients);
//  auto s_dot_dot_coefficients=DifferentiateCoefficients(s_dot_coefficients);
//
//  auto d_dot_coefficients=DifferentiateCoefficients(d_coefficients);
//  auto d_dot_dot_coefficients=DifferentiateCoefficients(d_dot_coefficients);

  oss << " s_coeffs ";
  for (auto c: s_coefficients) oss <<" "<< c;
//  for (auto c: s_dot_coefficients) oss <<" . " << c;
//  for (auto c: s_dot_dot_coefficients) oss <<" .. "<< c;

  oss << " d_coeffs ";
  for (auto c: d_coefficients) oss << " " << c;
//  for (auto c: d_dot_coefficients) oss << " . " << c;
//  for (auto c: d_dot_dot_coefficients) oss << " .. " << c;

  oss << " T " << T;
  return oss.str();
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

    double next_s;
    double next_d;
//  next_s = s_initial[0];
//  next_d = d_initial[0];
//  next_s_vals.push_back(next_s);
//  next_d_vals.push_back(next_d);

  double t = 0.0f;
  for (int i = 1; i < (T/point_path_interval)-1; i++){
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

  auto target = ego->TrajectoryStateAt(T);

  vector<double> s_targ {target[0],target[1],target[2]};

  vector<double> S = StateFromCoefficients(s_coefficients, t);

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

  auto target = ego->TrajectoryStateAt(T);
  vector<double> d_targ {target[3],target[4],target[5]};

  vector<double> D = StateFromCoefficients(d_coefficients, t);

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

  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto s_dot = DifferentiateCoefficients(s_coefficients);

  int calc_steps=10;
  double dt = t / calc_steps;
  for (int i = 0; i < calc_steps; i++) {
    t = dt * i;
    double speed = PolynomialEquate(s_dot, t);

    if (speed>max_speed)
      cost=1.0f;
  }

  return cost * cost_levels["exceeds_speed_cost"];
}

double TrajectoryGeneration::EfficiencyCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto target = ego->TrajectoryStateAt(t);;

  vector<double> s_targ {target[0],target[1],target[2]};

  double avg_v=PolynomialEquate(s_coefficients,t)/t;

  double targ_s = s_targ[0];
  double targ_v = targ_s/t;

  cost = Normalise(2.0f*(avg_v-targ_v)/avg_v);

  return cost * cost_levels["efficiency_cost"];
}

double TrajectoryGeneration::MaxAccelCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto s_dot = DifferentiateCoefficients(s_coefficients);
  auto s_dot_dot = DifferentiateCoefficients(s_dot);

  double total_acc = 0.0f;

  int calc_steps=10;
  double dt = T / calc_steps;
  for (int i = 0; i < calc_steps; i++) {
    t = dt * i;
    double acc = PolynomialEquate(s_dot_dot, t);
//    cout << " " << acc;
    total_acc += fabs(acc*dt);
  }
  double acc_per_second = total_acc / T;

//  cout << " total_acc " << total_acc<< " acc_per_second " << acc_per_second << " T " << T;

  cost = Normalise(acc_per_second/expected_acc_in_one_sec);
  return cost * cost_levels["max_accel_cost"];
}

double TrajectoryGeneration::TotalAccelCost(trajectoryType trajectory, Vehicle * ego, vector <double> delta, double T, Prediction * prediction){
  double cost = 0.0f;
  vector<double> s_coefficients, d_coefficients;
  double t;
  tie(s_coefficients, d_coefficients, t)=trajectory;

  auto s_dot = DifferentiateCoefficients(s_coefficients);
  auto s_dot_dot = DifferentiateCoefficients(s_dot);

  double max_acc = 0.0f;
  int calc_steps=10;
  double dt = T / calc_steps;

  for (int i=0; i < calc_steps; i++) {
    double acc = fabs(PolynomialEquate(s_dot_dot, dt*i));
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
  auto s_dot = DifferentiateCoefficients(s_coefficients);
  auto s_d_dot = DifferentiateCoefficients(s_dot);
  auto s_jerk = DifferentiateCoefficients(s_d_dot);

  double jerk_max = 0.0f;
  int calc_steps=10;
  for (int i=0; i < calc_steps; i++) {
    double jerk = fabs(PolynomialEquate(s_jerk, T/calc_steps*i));
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
  auto s_dot = DifferentiateCoefficients(s_coefficients);
  auto s_d_dot = DifferentiateCoefficients(s_dot);
  auto s_jerk = DifferentiateCoefficients(s_d_dot);

  double total_jerk = 0.0f;
  int calc_steps=10;
  double dt = T / calc_steps;
  for (int i = 0; i < calc_steps; i++) {
    t = dt * i;
    double jerk = PolynomialEquate(s_jerk, t);
    total_jerk += fabs(jerk*dt);
  }
  double jerk_per_second = total_jerk / T;

  cost = Normalise(jerk_per_second/expected_jerk_in_one_sec);
  return cost * cost_levels["total_jerk_cost"];
}
