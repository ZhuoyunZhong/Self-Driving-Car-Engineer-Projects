#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::VectorXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// MPC parameteres
const size_t N = 8;
const size_t dt = 0.1;
const double Lf = 2.67;

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
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
		  // state
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  // control
		  double delta = j[1]["steering_angle"];
		  double a = j[1]["throttle"];

		  // Final Control Output
          double steer_value;
          double throttle_value;


		  // Convert pts to vehicle (local) coordinate
		  for (size_t i = 0; i < ptsx.size(); ++i){
			// shift
			double local_x = ptsx[i] - px;
			double local_y = ptsy[i] - py;
			// rotate
            ptsx[i] = cos(-psi) * local_x - sin(-psi) * local_y;
            ptsy[i] = sin(-psi) * local_x + cos(-psi) * local_y;
          }


		  // Fit reference path
		  // convert from std::vector to Eigen:VectorXd
		  double* ptrx = &ptsx[0];
		  double* ptry = &ptsy[0];
		  Eigen::Map<Eigen::VectorXd> ptsx_fit(ptrx, ptsx.size());
		  Eigen::Map<Eigen::VectorXd> ptsy_fit(ptry, ptsy.size());
		  // fit
		  auto coeffs = polyfit(ptsx_fit, ptsy_fit, 3);


		  // Compute cross track error and orientation error
		  // for local cordinate, px = py = psi = 0
		  double cte = polyeval(coeffs, 0);
		  double epsi = - atan(coeffs[1]);


		  // Due to 100ms latency problem
		  const double latency = 0.1;
		  // Predict future state
		  double pred_px = 0.0 + v * latency;
          double pred_py = 0.0;
          double pred_psi = 0.0 + v * -delta / Lf * latency;
          double pred_v = v + a * latency;
          double pred_cte = cte + v * sin(epsi) * latency;
          double pred_epsi = epsi + v * (-delta) / Lf * latency;
		  // State Vector
		  VectorXd state(6);
		  state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;


		  // Control result
		  auto vars = mpc.Solve(state, coeffs);
		  // Normalize
		  steer_value = vars[0] / (deg2rad(25)*Lf);
		  throttle_value = vars[1];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

		  for (size_t i = 2; i < vars.size(); i+= 2){
			mpc_x_vals.push_back(vars[i]);
			mpc_y_vals.push_back(vars[i+1]);
		  }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int num_p = 40;
		  for(int i = 2; i < num_p; i+=4){
			next_x_vals.push_back(i);
			next_y_vals.push_back(polyeval(coeffs, i));
		  }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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