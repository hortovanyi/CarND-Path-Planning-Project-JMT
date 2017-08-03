/*
 * TrajectoryGeneration.cpp
 *
 *  Created on: 1 Aug. 2017
 *      Author: nick
 */

#include "TrajectoryGeneration.h"


TrajectoryGeneration::TrajectoryGeneration(HighwayMap * highway_map) {
  this->highway_map = highway_map;
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

tuple<vector<double>,vector<double>> TrajectoryGeneration::TrajectoryFrenetNext(vector< double> s_initial, vector <double> s_final, double d_initial, double d_final, double T) {

  vector<double> next_s_vals;
  vector<double> next_d_vals;

  vector <double>s_coefficients = JMT(s_initial, s_final, T);
  // for d, its reasonable to not need velocity and acceleration (for lane changes)
  vector <double>d_coefficients = JMT({d_initial,0,0}, {d_final,0,0}, T);

  double next_s = s_initial[0];
  double next_d = d_initial;
  next_s_vals.push_back(next_s);
  next_d_vals.push_back(next_d);

  // want to generate the next 49 points
  double t = 0.0f;

  for (int i = 1; i < (T/(point_path_interval_ms/1000)); i++){
    t+=point_path_interval_ms/1000;
    next_s = PolynomialEquate(s_coefficients,t);
    next_d = PolynomialEquate(d_coefficients,t);

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(next_d);
  }

  return make_tuple(next_s_vals, next_d_vals);
}
