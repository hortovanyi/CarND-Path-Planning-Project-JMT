/*
 * PathPlanner.cpp
 *
 *  Created on: 22 Jul. 2017
 *      Author: nick
 */

#include "PathPlanner.h"

PathPlanner::PathPlanner(HighwayMap * highway_map) {
	this->highway_map = highway_map;
	sensor_fusion = SensorFusion();
	prediction = new Prediction(&sensor_fusion);

	predictions = prediction->predictions;

	previous_path_x.clear();
	previous_path_y.clear();

	// we dont know the initial end path yet
	end_path_s_prev=0.f;
	end_path_d_prev=0.f;

	// we need a new behaviour
	behaviour_ttl =-1.0f;

	// we need a new trajectory
	trajectory_ttl = -1.0f;

	trajectory_generation = new TrajectoryGeneration(highway_map);
}

void PathPlanner::UpdateSensorFusion(json sensor_fusion){
  this->sensor_fusion = SensorFusion(sensor_fusion, highway_map);

  prediction = new Prediction(&this->sensor_fusion);
  prediction->GeneratePredictions(prediction_outlook);
  predictions = prediction->predictions;
}

void PathPlanner::UpdateEgo(Vehicle * ego) {

  this->ego = ego;

  if (this->ego->prev_ego) {
    this->ego->initial = this->ego->prev_ego->initial;
    this->ego->goal = this->ego->prev_ego->goal;
    this->ego->final = this->ego->prev_ego->final;
  }
}

void PathPlanner::UpdatePreviousPath(json previous_path_x, json previous_path_y){

  // clear out values
  this->previous_path_x.clear();
  this->previous_path_y.clear();

  // convert from json to vector<double>
  for(double x: previous_path_x)
    this->previous_path_x.push_back(x);

  for(double y: previous_path_y)
    this->previous_path_y.push_back(y);
}

void PathPlanner::UpdatePreviousEndPath(double end_path_s , double end_path_d){
  this->end_path_s_prev = end_path_s;
  this->end_path_d_prev = end_path_d;
}

double PathPlanner::SimulatorTimeElapsed() {
  return point_path_interval*SimulatorPointsConsumed();
}

int PathPlanner::SimulatorPointsConsumed() {
  int points_consumed =last_x_vals.size()-previous_path_x.size();
//  cout << "last_x_vals.size() " << last_x_vals.size() << " previous_path_x.size() " << previous_path_x.size() << " points_consumed " << points_consumed<<endl;
  return points_consumed;
}

void PathPlanner::PopConsumedTrajectoryPoints(int n=1){
  int n_s = traj_s_vals.size();
  int n_d = traj_d_vals.size();
  assert(n_s == n_d);
  if (n < 1) {
    cout << "Unable to pop " << n << " trajectories." << endl;
    return;
  }

  cout << " pop " << n << " traj_s_vals ("<<n_s<<")  traj_d_vals (" << n_d <<") "<<endl;
  // make sure we have some values in the vectors otherwise return
  if (traj_s_vals.size()-n <= 0 || traj_d_vals.size()-n <= 0)
    return;

  for (int i = 0; i < n; i++){
    traj_s_vals.erase(traj_s_vals.begin());
    traj_d_vals.erase(traj_d_vals.begin());
  }
}

void PathPlanner::UpdateBehaviour() {

  // only update the behaviour if enough time has passed
  double time_elapsed = SimulatorTimeElapsed();
  cout << "behaviour_ttl " << behaviour_ttl;
  cout << " time_elapsed " << time_elapsed;
  behaviour_ttl -= time_elapsed;
  if (behaviour_ttl < 0.0f) {
    cout << " updating ego behaviour!" << endl;

    // start the trajectory based on the final state
    ego->s=ego->final.s;
    ego->d=ego->final.d;
    ego->v=ego->final.v;
    ego->a=ego->final.a;
    ego->lane=ego->final.lane;

    double time_offset = double(traj_s_vals.size()) / point_path_interval;
    ego->UpdateState(predictions, time_offset);

    behaviour_ttl = revise_behaviour_interval;
  } else {
    cout << " not updating behaviour." << endl;
  }
}

//
//  return the new path as a tuple of vectors of the next x & y values
//
tuple<vector<double>,vector<double>> PathPlanner::NewPathPlan(){
  if (!ego) {
    cout << "Error: No ego vehicle set! Terminating!" << endl;
    exit (-1);
  }

  double time_elapsed = SimulatorTimeElapsed();
  int points_consumed = SimulatorPointsConsumed();

  cout << "points consumed " << points_consumed;
  if (points_consumed > 0)
    PopConsumedTrajectoryPoints(points_consumed);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  int path_size = previous_path_x.size();
  for(int i = 0; i < path_size; i++)
  {
     next_x_vals.push_back(previous_path_x[i]);
     next_y_vals.push_back(previous_path_y[i]);
  }

  vector<double> s_vals;
  vector<double> d_vals;

  cout << " trajectory_ttl " << trajectory_ttl;
  cout << " time_elapsed " << time_elapsed;

  trajectory_ttl -= time_elapsed;

  if (trajectory_ttl < 0.0f) {
    cout << " updating trajectory vals!" << endl;
    // NextTrajectory works off of the state structures in ego
    // we want to start from where we left off unless first time through
    if (traj_s_vals.size() >0)
      ego->initial = ego->final;

    tie(s_vals, d_vals)=NextTrajectory();
    trajectory_ttl = revise_trajectory_interval;
  } else {
    cout << " not updating trajectory." << endl;
  }

  traj_s_vals.insert(traj_s_vals.end(), s_vals.begin(), s_vals.end());
  traj_d_vals.insert(traj_d_vals.end(), d_vals.begin(), d_vals.end());

  unsigned traj_size = traj_s_vals.size();

  cout << "traj (" << traj_size<< ") s ";
  for (unsigned i=0; i < traj_size; i++) {
   if (i >0)
     cout << ",";
   cout << traj_s_vals[i];
  }
  cout << endl;

  cout << "traj (" << traj_size<< ") d ";
  for (unsigned i=0; i < traj_size; i++) {
   if (i >0)
     cout << ",";
   cout << traj_d_vals[i];
  }
  cout << endl;

  int n_trajs_needed=traj_size-next_x_vals.size();
  if (n_trajs_needed > n_path_points)
    n_trajs_needed = n_path_points;

  for(unsigned i=path_size; i < n_trajs_needed; i++){
    auto XY = highway_map->getXY(traj_s_vals[i], traj_d_vals[i]);

    next_x_vals.push_back(XY[0]);
    next_y_vals.push_back(XY[1]);
  }

  last_x_vals = next_x_vals;
  last_y_vals = next_y_vals;

  return make_tuple(next_x_vals, next_y_vals);
}

tuple<vector<double>, vector<double>> PathPlanner::NextTrajectory() {

  double pos_x;
  double pos_y;
  double angle;

  pos_x = ego->x;
  pos_y = ego->y;

  angle = ego->yaw;  // always in radians

  auto frenet = highway_map->getFrenet(pos_x, pos_y, angle);
  //  int lane = highway_map->LaneFrenet(frenet[1]);
  cout << "ego frenet " << frenet[0] << "," << frenet[1] << " car " << ego->s
       << "," << ego->d << " x " << pos_x << " y " << pos_y << " angle "
       << angle;
  //  cout << endl;

  // initial state
  double si = ego->initial.s;
  double si_dot = ego->initial.v;
  double si_dot_dot = ego->initial.a;
  double di = ego->initial.d;

  // goal state
  double sg = ego->goal.s;
  double sg_dot = ego->goal.v;
  double sg_dot_dot = ego->goal.a;
  double dg = highway_map->FrenetLaneCenter(ego->goal.lane);

  vector<double> s_initial { si, si_dot, si_dot_dot };
  vector<double> s_goal { sg, sg_dot, sg_dot_dot };
  vector<double> d_initial { di, 0, 0 };
  vector<double> d_goal { dg, 0, 0 };

  double goal_T;

  // TODO create a delta_s function against Highway Map
  //  T=3.f;
//  goal_T = (sg - si) / (si_dot + sg_dot) / 2;
  goal_T = ego->goal.t;
  cout << " si " << si << " si. " << si_dot << " si.. " << si_dot_dot;
  cout << " sg " << sg << " sg. " << sg_dot << " sg.. " << sg_dot_dot;
  cout << " di " << di << " dg " << dg;
  cout << " T " << goal_T;
  cout << endl;

  vector<double> delta { 0, 0, 0, 0, 0, 0 };  // TODO havent coded this

  vector<double> s_final, d_final;
  double T_final;
  tie(s_final, d_final, T_final) = trajectory_generation->BestFinalGoal(
      s_initial, d_initial, s_goal, d_goal, ego, delta, prediction, goal_T);
  cout << "sf " << s_final[0] << " sf. " << s_final[1] << " sf.. "
       << s_final[2];
  cout << " df " << d_final[0] << " df. " << d_final[1] << " df.. "
       << d_final[2];
  cout << " T " << T_final << endl;
  // save final state
  ego->final.s = s_final[0];
  ego->final.v = s_final[1];
  ego->final.a = s_final[2];
  ego->final.d = d_final[0];
  ego->final.lane = highway_map->LaneFrenet(ego->final.d);

  return trajectory_generation->TrajectoryFrenetNext(s_initial, s_final,
                                                     d_initial, d_final,
                                                     T_final);
}

