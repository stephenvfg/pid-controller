#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  Twiddle twiddle;

  // set the initial PID coefficients
  double Kp_ = 0.188238;
  double Ki_ = 0.0000428868;
  double Kd_ = 2.8665;

  // initialize the PID controller
  pid.Init(Kp_, Ki_, Kd_);

  // initialize the Twiddle
  double tol = 0.005;
  twiddle.Init(tol, Kp_, Ki_, Kd_);

  // use this bool to control whether or not we want to iterate through Twiddle prior to racing the track
  bool do_twiddle = false; // set to TRUE to do twiddle

  // if this value is true then display output showing PID values to help debug
  bool debug_mode = false; // CHANGE TO TRUE TO DEBUG

  h.onMessage([&pid, &twiddle, &do_twiddle, &debug_mode]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          // Not using speed or angle for this implementation. They are available here if needed.
          //double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());

          // update the PID error values and then use those to determine the steering value
          pid.UpdateError(cte);
          double error = pid.TotalError();

          // as long as twiddle isn't optimized, run through twiddle
          if (do_twiddle && !twiddle.Finished()) {
            // use the squared error for twiddle optimization to work around negatives
            double sq_error = error*error;
            
            // move the car
            // if twiddle reaches the end of a cycle, it will run through an optimization step
            twiddle.Run(sq_error);

            // if twiddle has gone through an optimization step and it's time to reset, do so now
            if (twiddle.TimeToReset()) {
              // update the PID controller's parameters
              vector<double> p = twiddle.Parameters();
              pid.UpdateCoefficients(p[0], p[1], p[2]);

              // reset the vehicle
              string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
          }

          // use the PID error to determine the steering value
          double steer_value = error;

          // confine the steer_value between the accpetable range of [-1, 1]
          if (steer_value < -1.0) { steer_value = -1.0; }
          else if (steer_value > 1.0) { steer_value = 1.0; }
          
          if (debug_mode) {
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (debug_mode) {
            std::cout << msg << std::endl;
          }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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