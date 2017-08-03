/*
 * Vehicle.cpp
 *
 *  Created on: 23 Jul. 2017
 *      Author: nick
 */

#include "Vehicle.h"

// constructor for sensor fusion
Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s,
                 double d, int lane) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->v = vxvy2v(vx, vy);
  this->a = 0.0f;  // assuming vehicles are at a constant stepp
  this->yaw = atan2(vx, vy);
  this->s = s;
  this->d = d;
  this->lane = lane;
  this->proposed_lane=lane;
  this->speed = v * metersPerSecRatioMilesPerHr;

  // save initial
  initial.s = this->s;
  initial.v = this->v;
  initial.a = this->a;
  initial.d = this->d;
  initial.lane = this->lane;

  // defaults - not used for non-ego vehicles
  this->state = "CS";
  this->target_speed = 1.0f;
  this->max_acceleration = 0.0f;

  InitCostLevels();
}

// constructor for ego vehicle
Vehicle::Vehicle(double x, double y, double s, double d, int lane,
                 double angle_deg, double speed, string state,
                 Vehicle * ego_prev) {
  this->prev_ego = ego_prev;

  this->id = -1;
  this->x = x;
  this->y = y;
  this->yaw = deg2rad(angle_deg);
  this->s = s;
  this->d = d;
  this->lane = lane;
  this->proposed_lane=lane;
  this->speed = speed;
  this->v = speed * (1 / metersPerSecRatioMilesPerHr);
  this->a = _CalcAcceleration();
  this->vx = cos(yaw) * v;
  this->vy = sin(yaw) * v;
  this->state = state;

  // save initial
  initial.s = this->s;
  initial.v = this->v;
  initial.a = this->a;
  initial.d = this->d;
  initial.lane = lane;

  // defaults
  this->target_speed = 45.0f * (1 / metersPerSecRatioMilesPerHr);  // 50 MPH
  this->max_acceleration = 10;

  InitCostLevels();
}

// clone vehicle
Vehicle::Vehicle(Vehicle * obj) {
  this->prev_ego = obj->prev_ego;

  this->id = obj->id;
  this->x = obj->x;
  this->y = obj->y;
  this->yaw = obj->yaw;
  this->s = obj->s;
  this->d = obj->d;
  this->lane = obj->lane;
  this->proposed_lane = obj->lane;
  this->speed = obj->speed;
  this->v = obj->v;
  this->a = obj->a;
  this->vx = obj->vx;
  this->vy = obj->vy;
  this->state = obj->state;

  // defaults
  this->target_speed = obj->target_speed;
  this->max_acceleration = obj->max_acceleration;
  this->preferred_buffer = obj->preferred_buffer;

  this->initial = obj->initial;

  InitCostLevels();
}

double Vehicle::YawDeg() {
  return rad2deg(atan2(vy, vx));
}

double Vehicle::deg2rad(double x) {
  return x * M_PI / 180;
}
double Vehicle::rad2deg(double x) {
  return x * 180 / M_PI;
}

double Vehicle::vxvy2v(double vx, double vy) {
  return sqrt(vx * vx + vy * vy);
}

// calculates the new acceleration value between the prev and this ego
double Vehicle::_CalcAcceleration() {

  // if no ego previous vehicle then we cant calculate accelleration
  if (!prev_ego)
    return 0.0f;

  // distance
  double x1 = this->x;
  double y1 = this->y;
  double x = this->prev_ego->x;
  double y = this->prev_ego->y;

  double distance = sqrt(pow(x1 - x,2 ) + pow(y1 - y, 2));
  // calc delta time from last ego to here
  double time = distance / this->v;

  // if there hasnt been any change in time since the previous acceleration is zero
  if (fabs(time) < .001)
    return 0.0f;

  double acceleration = (this->v - this->prev_ego->v) / time;

//  if (acceleration > 20.f) {
//    cout << "error acceleration crash " << acceleration <<endl;
//    exit (-1);
//  }
  return acceleration;
}

string Vehicle::display() {

  ostringstream oss;

  oss << "x:    " << this->x << "\n";
  oss << "y:    " << this->y << "\n";
  oss << "s:    " << this->s << "\n";
  oss << "d:    " << this->d << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";
  oss << "this: " << this << "\n";
  oss << "prev: " << this->prev_ego << "\n";

  return oss.str();
}

// given a state what are the potential successor states
vector<string> Vehicle::SuccessorStates(string current_state) {
//  cout << "'" <<current_state<< "'"<<endl;
  if (current_state == "CS")
    return {"KL","CS"};
  if (current_state == "KL")
    return {"CS", "KL", "PLCL", "PLCR"};
  if (current_state == "PLCL")
    return {"PLCL", "CS",  "KL", "LCL"};
  if (current_state == "PLCR")
    return {"PLCR", "CS", "KL", "LCR"};
  if (current_state == "LCL")
    return {"LCL", "CS", "KL"};
  if (current_state == "LCR")
    return {"LCR", "CS", "KL"};

  cout << "oh oh shouldnt get here!!!" << endl;
  return {"KL"};
}

// possible next states given lane of vehicle
vector<string> Vehicle::PossibleStates(string current_state) {

  vector<string> states = SuccessorStates(current_state);
//  cout << "SuccessorStates: ";
//  for (auto state : states) {
//    cout << state;
//  }
//  cout << endl;

  vector<string> invalid_states;
  vector<string> possible_states;
  // in the left lane so cant change left lane
  if (this->lane == 1) {
    invalid_states.push_back("PLCL");
    invalid_states.push_back("LCL");
    // in the right lane so cant change right lane
  } else if (this->lane == 3) {
    invalid_states.push_back("PLCR");
    invalid_states.push_back("LCR");
  }

  // only append valid states
  for (string state : states) {
    bool valid = true;
    for (string invalid_state : invalid_states)
      if (state == invalid_state)
        valid = false;
    if (valid)
      possible_states.push_back(state);
  }

  return possible_states;
}

void Vehicle::UpdateState(predictionsType predictions) {
  this->state = NextState(predictions);
}

string Vehicle::NextState(predictionsType predictions) {

  vector<string> possible_states = PossibleStates(this->state);
  cout << "PossibleStates lane "<< this->lane << " state " << this->state << ": ";
  for (auto state : possible_states) {
    cout << state <<" ";
  }
  cout << endl;

  int horizon = 5;
//  prediction->GeneratePredictions(horizon+5); // we want to have predictions further then the horizon
//  auto predictions = prediction->predictions;

//  for (auto const &prediction: predictions)
//  {
//    cout << "vehicle.id: "<< prediction.first << " => " << endl;
//
//    for (auto const& location: prediction.second) {
//      cout << "  " << location[0] << " " << location[1] << endl;
//    }
//  }

  // keep track of the total cost of each state
  map<string, double> costs;
  map<string, vector <Vehicle>> trajectories;

  for (auto state : possible_states) {

    costs[state] = 0.0f;
    cout << "state: " << state;
    vector<Vehicle> trajectory = TrajectoryForState(state, predictions,
                                                    horizon);
    trajectories[state] = trajectory;

    for (Vehicle v : trajectory) {
      cout << " l " << v.lane << " s " << v.s << " d " << v.d << " v "
           << v.v << " a " << v.a;
    }
    cout << endl;
    costs[state] = CalculateCost(trajectory, predictions, horizon);
  }

  double min_cost = numeric_limits<double>::max();
  string next_state = this->state;

  for (const auto &cost : costs) {
    // lowest cost wins
    cout << "state: " << cost.first << " cost: " << cost.second << endl;
    if (cost.second < min_cost) {
      min_cost = cost.second;
      next_state = cost.first;
    }
  }

  // save the goal state for the next trajectory
  auto next_trajectory = trajectories[next_state];
  int traj_size = next_trajectory.size();
  auto trajectory_goal = next_trajectory[traj_size-1];

  goal.s = trajectory_goal.s;
  goal.v = trajectory_goal.v;
  goal.a = trajectory_goal.a;
  goal.lane = trajectory_goal.lane;
  goal.d = trajectory_goal.d;

  cout << "state " << this->state << " next_state " << next_state;
  cout << " goal s "<< goal.s << " v " << goal.v << " a " << goal.a << " l " << goal.lane << " d " << goal.d;
  cout << endl;

  return next_state;
}

void Vehicle::increment(int dt = 1) {

  this->s += this->v * dt;
  this->v += this->a * dt;
  if (this->v < 0.0f)
    this->v = 0.0f;
}

vector<double> Vehicle::StateAt(int t) {
  double s = this->s + this->v * t + this->a * t * t / 2;
  double v = this->v + this->a * t;
  if (this->v < 0.0f)
    this->v = 0.0f;
  return {(double)this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {
  /*
   Simple collision detection.
   */
  vector<double> check1 = StateAt(at_time);
  vector<double> check2 = other.StateAt(at_time);

  // TODO check what the value of L should be
  double L = 1.0f;
  return (check1[0] == check2[0]) && (fabs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

vector<vector<double> > Vehicle::GeneratePredictions(int horizon = 6) {
  vector<vector<double> > predictions;
  for (int i = 0; i < horizon; i++) {
    auto check1 = StateAt(i);
    vector<double> lane_s = { check1[0], check1[1] };
    predictions.push_back(lane_s);
  }
  return predictions;
}

// generate a rough trajectory for the state using predictions out for a horizon
vector<Vehicle> Vehicle::TrajectoryForState(string state,
                                            predictionsType predictions,
                                            int horizon = 3) {
  vector<Vehicle> trajectory;

  // work on a copy of ego
  Vehicle * ego_prev = new Vehicle(this);

  // create a rough trajectory out into the horizon
  for (unsigned i=0; i < horizon; i++) {

    Vehicle ego_next(ego_prev);
    ego_next.prev_ego = ego_prev;
    ego_next.state = state;
    ego_next.RealiseState(predictions);
    ego_next.increment();
    ego_prev = &ego_next;

    // remove the head prediction for each vehicle
    for (auto & vehicle_predictions : predictions) {
      vehicle_predictions.second.erase(vehicle_predictions.second.begin());
    }

    trajectory.push_back(ego_next);
  }

  return trajectory;
}

void Vehicle::RealiseState(predictionsType predictions) {
  if (state == "CS")
    RealiseConstantSpeed();
  else if (state == "KL")
    RealiseKeepLane(predictions);
  else if (state == "LCL")
    RealiseLaneChange(predictions, "L");
  else if (state == "LCR")
    RealiseLaneChange(predictions, "R");
  else if (state == "PLCL")
    RealisePrepLaneChange(predictions, "L");
  else if (state == "PLCR")
    RealisePrepLaneChange(predictions, "R");
}

void Vehicle::RealiseConstantSpeed() {
  this->a = 0.0f;
}

double Vehicle::_MaxAccelForLane(predictionsType predictions, int lane, double s) {
  double delta_v_til_target = target_speed - v;

  double max_acc = min(max_acceleration, delta_v_til_target);

  predictionsType::iterator it = predictions.begin();
  vector<vector<vector<double> > > in_front;
  while (it != predictions.end()) {
    int v_id = it->first;

    vector<vector<double> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] > s)) {
      in_front.push_back(v);
    }
    it++;
  }

  if (in_front.size() > 0) {
    int min_s = 1000;

    vector<vector<double>> leading = { };
    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - s) < min_s) {
        min_s = (in_front[i][0][1] - s);
        leading = in_front[i];
      }
    }

    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;

    max_acc = min(max_acc, double(available_room));
  }

  return max_acc;
}

void Vehicle::RealiseKeepLane(predictionsType predictions) {
  this->a = _MaxAccelForLane(predictions, this->lane, this->s);
}

void Vehicle::RealiseLaneChange(predictionsType predictions, string direction) {
  int delta = -1;
  if (direction == "R") {
    delta = 1;
  }

  this->lane += delta;
  this->proposed_lane = lane;
  int lane = this->lane;
  int s = this->s;
  this->a = _MaxAccelForLane(predictions, lane, s);
}

void Vehicle::RealisePrepLaneChange(predictionsType predictions,
                                    string direction) {
  int delta = -1;
  if (direction == "R") {
    delta = 1;
  }
  int lane =  this->lane + delta;
  this->proposed_lane = lane;

  predictionsType::iterator it = predictions.begin();
  vector<vector<vector<double> > > at_behind;
  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<double> > v = it->second;

    // TODO add calc for loop past last s
    if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }
    it++;
  }

  if (at_behind.size() > 0) {
    int max_s = 0;
    vector<vector<double> > nearest_behind = { };
    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if (delta_v != 0) {
      int time = -2 * delta_s / delta_v;
      int a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }

      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }

      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }

      this->a = a;
    } else {
      int max_acceleration = this->max_acceleration;
      int my_min_acc = max(-max_acceleration, -delta_s);
      this->a = my_min_acc;
    }
  }
}

// cost functions

void Vehicle::InitCostLevels() {
  cost_levels["collision"] = pow(10, 6);
  cost_levels["danger"] = pow(10, 5);
  cost_levels["reach_goal"] = pow(10, 5);
  cost_levels["comfort"] = pow(10, 4);
  cost_levels["efficiency"] = pow(10, 2);
  cost_levels["desired_buffer"] = 1.5;
  cost_levels["planning_horizon"] = 2;
}

double Vehicle::ChangeLaneCost(vector<Vehicle> trajectory,
                               predictionsType predictions, int proposed_lane) {
  double cost = 0.0f;

  int cur_lane = trajectory[0].lane;
  if (proposed_lane > cur_lane)
    cost += cost_levels["comfort"];
  if (proposed_lane < cur_lane)
    cost += cost_levels["comfort"];

  cout << " ChangeLane " << cost;
  return cost;
}

double Vehicle::InefficiencyCost(vector<Vehicle> trajectory,
                                 predictionsType predictions,
                                 double target_speed) {
  double cost = 0.0f;

  double speed = trajectory_data.avg_speed;
  double diff = target_speed - speed;

  double multiplier = pow(diff / target_speed, 2);

  cost = multiplier * cost_levels["efficiency"];

  cout << " Efficiency " << cost;
  return cost;
}

double Vehicle::CollisionCost(vector<Vehicle> trajectory,
                              predictionsType predictions) {
  double cost = 0.0f;

  if (trajectory_data.collides) {
    double exponent = pow(trajectory_data.collides_at, 2);
    double multiplier = exp(-exponent);
    cost += multiplier * cost_levels["collision"];
  }

  cout << " Collision " << cost;
  return cost;
}

double Vehicle::BufferCost(vector<Vehicle> trajectory,
                           predictionsType predictions) {
  double cost = 0.0f;

  if (trajectory_data.closest_approach == 0)
    cost += 10 * cost_levels["danger"];

  if (trajectory_data.avg_speed > 0.0f) {
    double timesteps_away = trajectory_data.closest_approach
        / trajectory_data.avg_speed;
    if (timesteps_away <= preferred_buffer) {
      double multiplier = 1.0f - pow(timesteps_away / preferred_buffer, 2);
      cost += multiplier * cost_levels["danger"];
    }

    double timesteps_behind = trajectory_data.closest_behind
          / trajectory_data.avg_speed;
    if (timesteps_behind >= -preferred_buffer) {
      double multiplier = 1.0f - pow(timesteps_behind / preferred_buffer, 2);
      cost += multiplier * cost_levels["danger"];
    }
  }

  cout << " Buffer " << cost;
  return cost;
}

double Vehicle::CalculateCost(vector<Vehicle> trajectory,
                              predictionsType predictions,
                              int horizon = 5) {
  double cost = 0.0f;

  UpdateTrajectoryData(trajectory, predictions, horizon);
  cout << "** trajectory proposed_lane " << trajectory_data.proposed_lane
      << " avg_speed " << trajectory_data.avg_speed
      << " max_accel " << trajectory_data.max_accel
      << " rms_accel " << trajectory_data.rms_acceleration
      << " closest_approach " << trajectory_data.closest_approach
      << " closest_behind " << trajectory_data.closest_behind
      << " collides " << trajectory_data.collides
      << " collides_at " << trajectory_data.collides_at
      ;

  int proposed_lane = trajectory_data.proposed_lane;

  cout << " CalculateCost ";
  cost += ChangeLaneCost(trajectory, predictions, proposed_lane);
  cost += InefficiencyCost(trajectory, predictions, this->target_speed);
  cost += CollisionCost(trajectory, predictions);
  cost += BufferCost(trajectory, predictions);

  cout << " total " << cost << endl;
  return cost;
}

void Vehicle::UpdateTrajectoryData(vector<Vehicle> trajectory,
                                   predictionsType predictions,
                                   int horizon = 5) {
  Vehicle current_shapshot = trajectory[0];
  Vehicle first = trajectory[1];
  Vehicle last = trajectory.back();

  double dt = trajectory.size();
  trajectory_data.proposed_lane = first.proposed_lane;
  trajectory_data.avg_speed = (last.s - current_shapshot.s) / dt;

  vector<double> accels;
  trajectory_data.closest_approach = 9999;
  trajectory_data.closest_behind = -9999;
  trajectory_data.collides = false;
  trajectory_data.collides_at = -1;
  Vehicle last_snap(trajectory[0]);

  predictionsType filtered = FilterPredictionsByLane(
      predictions, trajectory_data.proposed_lane);


  for (int i = 1; i < horizon + 1; i++) {
    int lane = trajectory[i].lane;
    double s = trajectory[i].s;
    double v = trajectory[i].v;
    double a = trajectory[i].a;

    accels.push_back(a);

    for (auto &prediction : filtered) {

      int v_id = prediction.first;
      auto other = filtered[v_id];
      double other_s = other[i][1];
      double other_s_last = other[i - 1][1];
      Vehicle * last_state = trajectory[i].prev_ego;
      trajectory_data.collides_at=-1;
      trajectory_data.collides = CheckCollision(&trajectory[i], other_s_last,
                                                other_s);
      if (trajectory_data.collides)
        if (i < trajectory_data.collides_at)
          trajectory_data.collides_at = i;

      // TODO add code for when s resets


      double dist = other_s - s;

      if (dist > 0 && dist < trajectory_data.closest_approach)
        trajectory_data.closest_approach = dist;

      if (dist < 0 && dist > trajectory_data.closest_behind)
        trajectory_data.closest_behind = dist;
    }
  }

  trajectory_data.max_accel = -1;
  double rms_accels_sum;
  for (double accel : accels) {
    // set max acceleration with the largest absolute value
    if (fabs(accel) > trajectory_data.max_accel)
      trajectory_data.max_accel = fabs(accel);

    rms_accels_sum += accel * accel;
  }

  trajectory_data.rms_acceleration = rms_accels_sum / accels.size();
}

predictionsType Vehicle::FilterPredictionsByLane(predictionsType predictions,
                                                 int lane) {
  predictionsType filtered;
  for (auto &prediction : predictions) {
    // dont want ego
    if (prediction.first == -1)
      continue;

    // making the assumption the other vehicles aren't changing lanes
    int prediction_lane = prediction.second[0][0];
    if (prediction_lane == lane)
      filtered[prediction.first] = prediction.second;
  }
  return filtered;
}

bool Vehicle::CheckCollision(Vehicle * ego_trajected, double s_previous,
                             double s_now) {
  double s = ego_trajected->s;
  double v = ego_trajected->v;

  double v_target = s_now - s_previous;

  if (s_previous < s) {
    if (s_now >= s)
      return true;
    else
      return false;
  }

  if (s_previous > s) {
    if (v_target > v)
      return false;
    else
      return true;
  }

  if (s_previous == s) {
    if (v_target > v)
      return false;
    else
      return true;
  }

  // something went wrong
  return true;
}

