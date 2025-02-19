#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "HighwayMap.h"
#include "SensorFusion.h"
#include "Vehicle.h"
#include "PathPlanner.h"
#include "Prediction.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//double distance(double x1, double y1, double x2, double y2) {
//  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
//}
//int ClosestWaypoint(double x, double y, vector<double> maps_x,
//                    vector<double> maps_y) {
//
//  double closestLen = 100000;  //large number
//  int closestWaypoint = 0;
//
//  for (int i = 0; i < maps_x.size(); i++) {
//    double map_x = maps_x[i];
//    double map_y = maps_y[i];
//    double dist = distance(x, y, map_x, map_y);
//    if (dist < closestLen) {
//      closestLen = dist;
//      closestWaypoint = i;
//    }
//
//  }
//
//  return closestWaypoint;
//
//}
//
//int NextWaypoint(double x, double y, double theta, vector<double> maps_x,
//                 vector<double> maps_y) {
//
//  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
//
//  double map_x = maps_x[closestWaypoint];
//  double map_y = maps_y[closestWaypoint];
//
//  double heading = atan2((map_y - y), (map_x - x));
//
//  double angle = abs(theta - heading);
//
//  if (angle > pi() / 4) {
//    closestWaypoint++;
//  }
//
//  return closestWaypoint;
//
//}
//
//// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//vector<double> getFrenet(double x, double y, double theta,
//                         vector<double> maps_x, vector<double> maps_y) {
//  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
//
//  int prev_wp;
//  prev_wp = next_wp - 1;
//  if (next_wp == 0) {
//    prev_wp = maps_x.size() - 1;
//  }
//
//  double n_x = maps_x[next_wp] - maps_x[prev_wp];
//  double n_y = maps_y[next_wp] - maps_y[prev_wp];
//  double x_x = x - maps_x[prev_wp];
//  double x_y = y - maps_y[prev_wp];
//
//  // find the projection of x onto n
//  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
//  double proj_x = proj_norm * n_x;
//  double proj_y = proj_norm * n_y;
//
//  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
//
//  //see if d value is positive or negative by comparing it to a center point
//
//  double center_x = 1000 - maps_x[prev_wp];
//  double center_y = 2000 - maps_y[prev_wp];
//  double centerToPos = distance(center_x, center_y, x_x, x_y);
//  double centerToRef = distance(center_x, center_y, proj_x, proj_y);
//
//  if (centerToPos <= centerToRef) {
//    frenet_d *= -1;
//  }
//
//  // calculate s value
//  double frenet_s = 0;
//  for (int i = 0; i < prev_wp; i++) {
//    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
//  }
//
//  frenet_s += distance(0, 0, proj_x, proj_y);
//
//  return {frenet_s,frenet_d};
//
//}
//
//// Transform from Frenet s,d coordinates to Cartesian x,y
//vector<double> getXY(double s, double d, vector<double> maps_s,
//                     vector<double> maps_x, vector<double> maps_y) {
//  int prev_wp = -1;
//
//  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
//    prev_wp++;
//  }
//
//  int wp2 = (prev_wp + 1) % maps_x.size();
//
//  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
//                         (maps_x[wp2] - maps_x[prev_wp]));
//  // the x,y,s along the segment
//  double seg_s = (s - maps_s[prev_wp]);
//
//  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
//  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
//
//  double perp_heading = heading - pi() / 2;
//
//  double x = seg_x + d * cos(perp_heading);
//  double y = seg_y + d * sin(perp_heading);
//
//  return {x,y};
//
//}

int main() {
  uWS::Hub h;

//  // Load up map values for waypoint's x,y,s and d normalized normal vectors
//  vector<double> map_waypoints_x;
//  vector<double> map_waypoints_y;
//  vector<double> map_waypoints_s;
//  vector<double> map_waypoints_dx;
//  vector<double> map_waypoints_dy;
//
//  // Waypoint map to read from
//  string map_file_ = "../data/highway_map.csv";
//  // The max s value before wrapping around the track back to 0
//  double max_s = 6945.554;
//
//  ifstream in_map_(map_file_.c_str(), ifstream::in);
//
//  string line;
//  while (getline(in_map_, line)) {
//  	istringstream iss(line);
//  	double x;
//  	double y;
//  	float s;
//  	float d_x;
//  	float d_y;
//  	iss >> x;
//  	iss >> y;
//  	iss >> s;
//  	iss >> d_x;
//  	iss >> d_y;
//  	map_waypoints_x.push_back(x);
//  	map_waypoints_y.push_back(y);
//  	map_waypoints_s.push_back(s);
//  	map_waypoints_dx.push_back(d_x);
//  	map_waypoints_dy.push_back(d_y);
//  }

  // Waypoint map to read from
//  string map_file_ = "../data/highway_map.csv";
  string map_file_ = "/Users/nick/Desktop/Udacity/Term3/CarND-Path-Planning-Project/data/highway_map.csv";

  // create the highway map
  HighwayMap highway_map(map_file_);

  cout << "way points " << highway_map.Waypoints().size()<< endl;
  for (auto wp: highway_map.Waypoints()){
//    cout << wp.x << " " << wp.y << " " << wp.s << " next " << wp.next->s << " dist " << wp.waypoint_distance(*wp.next) << endl;
    cout << wp->x << " " << wp->y << " " << wp->s << " " << wp->dx <<","<< wp->dy << " wp.next " << wp->next->x << " " << wp->next->y << " ";
    cout << " wp * " << wp << " wp->next " << wp->next << " wp-> prev " << wp->prev;
    cout << " delta_s next " << wp->delta_s(wp->next) <<" "<< wp->delta_s(wp->next->next) << " prev " << wp->delta_s(wp->prev) << " " << wp->delta_s(wp->prev->prev);
    cout << " dist " << wp->distance(*wp->next) << endl;
  }

  // create the path planner
  PathPlanner path_planner(&highway_map);



  // The max s value before wrapping around the track back to 0
//  double max_s = highway_map.max_s;
//  vector<WayPoint> map_waypoints = highway_map.way_points();

//  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
//                     uWS::OpCode opCode) {
  h.onMessage(
      [&path_planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
          uWS::OpCode opCode) {


        // clear screen
        cout << string(2,'\n');
//        cout << "\033[2J\033[1;1H";

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

          auto s = hasData(data);

          if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
              // j[1] is the data JSON object

              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
              double car_s = j[1]["s"];
              double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
              double car_speed = j[1]["speed"];

              int lane = path_planner.highway_map->LaneFrenet(car_d);

              cout << "json x " << car_x << " y "<< car_y << " s " << car_s << " d " << car_d << " yaw " << car_yaw << " speed " << car_speed << endl;

              // Previous path's end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];
              path_planner.UpdatePreviousEndPath(end_path_s, end_path_d);
              cout << "prev end s "<< end_path_s << " d " << end_path_d << " ";

              // Previous path data given to the Planner
              auto previous_path_x = j[1]["previous_path_x"];
              auto previous_path_y = j[1]["previous_path_y"];
              path_planner.UpdatePreviousPath(previous_path_x,previous_path_y);

              unsigned prev_path_size = path_planner.previous_path_x.size();
              cout << "prev path size " << prev_path_size << endl;

              if (prev_path_size>0){
                cout << "prev path x (" << path_planner.previous_path_x.size()<<") ";
                for (unsigned i=0; i < prev_path_size; i++) {
                  if (i >0)
                    cout << ",";
                  cout << " " << path_planner.previous_path_x[i];
                }
                cout <<endl;
                cout << "prev path y (" << path_planner.previous_path_y.size()<<") ";
                for (unsigned i=0; i < prev_path_size; i++) {
                  if (i >0)
                    cout << ",";
                  cout << " " << path_planner.previous_path_y[i];
                }
                cout <<endl;
              } else
                cout << " no previous path ";
              cout << endl;

              //  cout << " lane " << lane << endl;
              string prev_state = "CS";
              Vehicle * prev_ego = nullptr;
              if (path_planner.ego) {
                prev_state = path_planner.ego->behaviour_state;
                prev_ego = new Vehicle(path_planner.ego);
              }

//              if (path_planner.ego)
//                cout << "ego state before update " << path_planner.ego->StateDisplay() << endl;

              int points_consumed = path_planner.SimulatorPointsConsumed();
              Vehicle ego(car_x, car_y, car_s, car_d, lane, car_yaw, car_speed, points_consumed, prev_state, prev_ego);

              path_planner.UpdateEgo(&ego);
              cout << "ego state after update " << path_planner.ego->StateDisplay() << endl;

              //              cout << car_x << " " << car_y << " " << car_s << " " << car_d << " " << car_yaw << " " << car_speed
              //                  << endl;

              cout << "ego " << ego.id << " x " << ego.x << " y " << ego.y << " vx " << ego.vx << " vy " << ego.vy;
              cout << " v " << ego.v << " a " << ego.a << " yaw " << ego.yaw << " s " << ego.s << " d " << ego.d;
              cout << " l " << ego.lane << " speed " << ego.speed;
              if (ego.prev_ego)
              cout << " prev x " << ego.prev_ego->x << " y " << ego.prev_ego->y << " v " << ego.prev_ego->v;
              cout << endl;

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              json sensor_fusion = j[1]["sensor_fusion"];

//              cout << "sensor_fusion " << sensor_fusion << endl;
//              SensorFusion sensor_fusion(j[1]["sensor_fusion"]);
//              for (auto v: sensor_fusion.Vehicles()){
//                  cout << v.id << " " << v.x << " " << v.y << " " << endl;
//              }

              // create new predictions from updated sensor fusion data. Then work out the next state for ego
              path_planner.UpdateSensorFusion(sensor_fusion);

              //path_planner.ego->UpdateState(path_planner.predictions);
              path_planner.UpdateBehaviour();

              // create the new path plan based on the current state of ego.
              json msgJson;

              vector<double> next_x_vals;
              vector<double> next_y_vals;

              tie(next_x_vals, next_y_vals) = path_planner.NewPathPlan();

              // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
              msgJson["next_x"] = next_x_vals;
              msgJson["next_y"] = next_y_vals;

              unsigned next_size = next_x_vals.size();
              cout << "next x ("<<next_size<<") " ;
              for (unsigned i=0; i < next_size; i++) {
                if (i >0)
                  cout << ",";
                cout << next_x_vals[i];
              }
              cout << endl;
              cout << "next y ("<<next_size<<") ";
                for (unsigned i=0; i < next_size; i++) {
                  if (i >0)
                    cout << ",";
                  cout << next_y_vals[i];
                }
                cout << endl;
              auto msg = "42[\"control\","+ msgJson.dump()+"]";

              //this_thread::sleep_for(chrono::milliseconds(1000));
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

