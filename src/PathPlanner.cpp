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
	behaviour_ttl =0.0f;

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

//
//  return the new path as a tuple of vectors of the next x & y values
//
tuple<vector<double>,vector<double>> PathPlanner::NewPathPlan(){
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  if (!ego) {
    cout << "Error: No ego vehicle set! Terminating!" << endl;
    exit (-1);
  }

  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();

  pos_x = ego->x;
  pos_y = ego->y;

  angle = ego->yaw; // always in radians

//  next_x_vals.push_back(pos_x);
//  next_y_vals.push_back(pos_y);


  int n_previous_path = 6;
  if (path_size < n_previous_path)
    n_previous_path = path_size;

  for(int i = 0; i < n_previous_path; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }



//  if(path_size == 0)
//  {
//    pos_x = ego->x;
//    pos_y = ego->y;
////    angle = deg2rad(car_yaw);
//    angle = ego->yaw; // always in radians
//  }
//  else
//  {
//    pos_x = previous_path_x[path_size-1];
//    pos_y = previous_path_y[path_size-1];
//
//    double pos_x2 = previous_path_x[path_size-2];
//    double pos_y2 = previous_path_y[path_size-2];
//    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
//  }

//  double dist_inc = 0.5;
//  for(int i = 0; i < 50-path_size; i++)
//  {
//    next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(M_PI/100)));
//    next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(M_PI/100)));
//    pos_x += (dist_inc)*cos(angle+(i+1)*(M_PI/100));
//    pos_y += (dist_inc)*sin(angle+(i+1)*(M_PI/100));
//  }

//  WayPoint * wp_next = highway_map->NextWaypoint(pos_x, pos_y, angle);
  auto frenet = highway_map->getFrenet(pos_x, pos_y, angle);
//  int lane = highway_map->LaneFrenet(frenet[1]);
  cout << "ego frenet " << frenet[0] << "," <<frenet[1] << " car " << ego->s<<","<< ego->d <<" x " << pos_x << " y " << pos_y << " angle " << angle;
//  cout << endl;

//  double dist_inc = 0.45;
//  for (unsigned i =0; i < 50-path_size; i++)
//  {
//    frenet[0] += dist_inc;
////    auto XY = highway_map->getXY(frenet[0],frenet[1]);
//    auto XY = highway_map->getXY(frenet[0],6); // just keep in middle of second lane
//    next_x_vals.push_back(XY[0]);
//    next_y_vals.push_back(XY[1]);
//  }

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

  vector<double> s_initial {si, si_dot, si_dot_dot};
  vector<double> s_goal {sg, sg_dot, sg_dot_dot};
  vector<double> d_initial {di,0,0};
  vector<double> d_goal {dg,0,0};

  // TODO work out a better Time calculation
  double T;
//  if (si_dot > 0.001)
//    T = fabs(sf-si)/sf_dot;
//  else
//    T = 1.f;


//  T=3.f;

  T = 100/(si_dot+sg_dot)/2;
  cout << " si " << si << " si. " << si_dot << " si.. " << si_dot_dot;
  cout << " sg " << sg << " sg. " << sg_dot << " sg.. " << sg_dot_dot;
  cout << " di " << di << " dg " << dg;
  cout << " T " << T;
  cout << endl;

  vector<double> traj_s_vals;
  vector<double> traj_d_vals;

  vector<double> delta {0,0,0,0,0,0}; // TODO havent coded this

  vector <double> s_final, d_final;
  double T_final;
  tie(s_final, d_final, T_final)=trajectory_generation->BestFinalGoal(s_initial,d_initial,s_goal,d_goal,ego,delta,prediction,T);
  cout << "sf " << s_final[0] << " sf. " << s_final[1] << " sf.. " << s_final[2];
  cout << " df " << d_final[0] << " df. " << d_final[1] << " df.. " << d_final[2];
  cout << " T " << T_final <<endl;
  // save final state
  ego->final.s=s_final[0];
  ego->final.v=s_final[1];
  ego->final.a=s_final[2];
  ego->final.d=d_final[0];
  ego->final.lane=highway_map->LaneFrenet(ego->final.d);

  tie(traj_s_vals, traj_d_vals)=trajectory_generation->TrajectoryFrenetNext(s_initial, s_final, d_initial, d_final, T_final);

  unsigned traj_size = traj_s_vals.size();

  cout << "traj " << traj_size<< " s ";
  for (unsigned i=0; i < traj_size; i++) {
   if (i >0)
     cout << ",";
   cout << traj_s_vals[i];
  }
  cout << endl;

  cout << "traj " << traj_size<< " d ";
  for (unsigned i=0; i < traj_size; i++) {
   if (i >0)
     cout << ",";
   cout << traj_d_vals[i];
  }
  cout << endl;

  int n_trajs_needed=traj_size-next_x_vals.size();

  for(unsigned i=n_previous_path; i < n_trajs_needed; i++){
    auto XY = highway_map->getXY(traj_s_vals[i], traj_d_vals[i]);

    next_x_vals.push_back(XY[0]);
    next_y_vals.push_back(XY[1]);
  }

  return make_tuple(next_x_vals, next_y_vals);
}


