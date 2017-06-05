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

// for convenience
using json = nlohmann::json;


////////////////
// Parameters //
////////////////

const int waypoints_count = 30;
const double dist_between_waypoints = 3;

// Factors for cost computation
extern double factor_cte    = 2;
extern double factor_epsi   = 5;
extern double factor_v      = 0.1;
extern double factor_steering = 1000;
extern double factor_throttle = 0.1;
extern double factor_seq_steering = 30000.0;
extern double factor_seq_throttle = 1.0;

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

int main(int argc, char **argv) {
  uWS::Hub h;


  if(argc==8)
  {
    factor_cte    = atof(argv[1]);
    factor_epsi   = atof(argv[2]);
    factor_v      = atof(argv[3]);
    factor_steering = atof(argv[4]);
    factor_throttle = atof(argv[5]);
    factor_seq_steering = atof(argv[6]);
    factor_seq_throttle = atof(argv[7]);
  }
  printf("factor_cte    %f\n", factor_cte);
  printf("factor_epsi   %f\n", factor_epsi);
  printf("factor_v      %f\n", factor_v);
  printf("factor_steering %f\n", factor_steering);
  printf("factor_throttle %f\n", factor_throttle);
  printf("factor_seq_steering %f\n", factor_seq_steering);
  printf("factor_seq_throttle %f\n", factor_seq_throttle);

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // cout << data << '\n';
    string sdata = string(data).substr(0, length);
    // cout << sdata <<"  [" << length<<']'<< endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];


          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          //Points in relation to vehicle frame
          Eigen::VectorXd ptsx_rel(ptsx.size());
          Eigen::VectorXd ptsy_rel(ptsx.size());

          // Compute relative values
          for(int i=0;i<ptsx.size();++i)
          {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;

            ptsx_rel[i] = x*cos(psi) + y*sin(psi);
            ptsy_rel[i] = y*cos(psi) - x*sin(psi);
          }

          // Fit a polynomial
          auto coeffs = polyfit(ptsx_rel, ptsy_rel, 3);

          // Generate cte and epsi estimate
          double cte  = polyeval(coeffs, 0.0);
          double epsi = -atan(coeffs[1]);

          // Create a state vector
          Eigen::VectorXd state(ptsx.size());
          state << 0.0, 0.0, 0.0, v, cte, epsi;

          // Run MPC solver
          auto results = mpc.Solve(state, coeffs);

          json msgJson;
          msgJson["steering_angle"] = - results[0];
          msgJson["throttle"] = results[1];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.GetMpcX();
          msgJson["mpc_y"] = mpc.GetMpcY();

          //Display the waypoints/reference line
          vector<double> next_x_vals(waypoints_count);
          vector<double> next_y_vals(waypoints_count);

          for (int k=0; k<waypoints_count; k++)
          {
            next_x_vals[k] = dist_between_waypoints * (double)k;
            next_y_vals[k] = polyeval(coeffs, next_x_vals[k]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
    else
    {
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
