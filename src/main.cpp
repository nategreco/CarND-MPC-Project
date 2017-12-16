#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

#define MPH_TO_MPS 0.44704

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px_0 = j[1]["x"];
          double py_0  = j[1]["y"];
          double psi_0 = j[1]["psi"];
          double v_0 = j[1]["speed"];
          double steer_value= j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          /* Calculate steering angle and throttle using MPC. */
          
          // Get desired trajectory waypoints from car's perspective
          Eigen::VectorXd waypts_x(ptsx.size());
          for (unsigned int i = 0; i < ptsx.size(); ++i) {
            waypts_x[i] = (ptsx[i] - px_0) * cos(-psi_0) - (ptsy[i] - py_0) * sin(-psi_0);
          }
          Eigen::VectorXd waypts_y(ptsy.size());
          for (unsigned int i = 0; i < ptsx.size(); ++i) {
            waypts_y[i] = (ptsx[i] - px_0) * sin(-psi_0) + (ptsy[i] - py_0) * cos(-psi_0);
          }
          
          // Get trajectory characterized as 3rd degree polynomial
          Eigen::VectorXd coeffs = polyfit(waypts_x, waypts_y, 3);
          
          // Get cte and epsi_0 at t = 0
          Eigen::VectorXd state(6);
          px_0 = 0;
          py_0  = 0;
          psi_0 = 0;
          v_0 *= MPH_TO_MPS;
          double cte_0 = polyeval(coeffs, 0); // Get the current diff from trajectory
          double epsi_0 = -atan(coeffs[1]); // Get current diff from trajectory
          
          // Add actuator delay
          double px_1 = px_0 + v_0 * cos(psi_0) * delay; 
          double py_1 = py_0  + v_0 * sin(psi_0) * delay;
          double psi_1 = psi_0 - v_0 * steer_value * delay / Lf;
          double v_1 = v_0 + throttle_value * delay;
          double cte_1 = cte_0 + v_0 * sin(epsi_0) * delay;
          double epsi_1 = epsi_0 - v_0 * steer_value * delay / Lf;
          
          // Create state with actuator delay
          state << px_1, py_1, psi_1, v_1, cte_1, epsi_1;
          
          // Solve
          vector<double> values = mpc.Solve(state, coeffs);
          
          // Get actuator values
          steer_value = values[0]; // Scaled -1 to 1
          throttle_value = values[1]; // Scaled -1 to 1

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(turn_lim);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (unsigned int i = 2; i < values.size(); i += 2) {
            mpc_x_vals.push_back(values[i]);
          }
          for (unsigned int i = 3; i < values.size(); i += 2) {
            mpc_y_vals.push_back(values[i]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          unsigned int stp = 2;
          unsigned int cnt = 40;
          for (unsigned int x = 2; x <= stp * cnt; x += stp) {
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
