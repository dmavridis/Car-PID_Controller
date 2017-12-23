#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}



// Global Variable Definition
std::vector<double> p({0, 0, 0});
std::vector<double> dp({0.05,0.001,0.5});
double dpsum = dp[0] + dp[1] + dp[2];
bool twiddle = true;
int ctrl = 0;
bool meas1 = false;
bool meas2 = false;
double best_err = 10; // large initial value

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init( p[0], p[1], p[2]);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    // Variables for twiddle


    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));

      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          double tol = 0.001;
          int it = 0;

          // Initial run
          pid.UpdateError(cte);

//          throttle = 0.3;
          throttle = std::max(2 -1*deg2rad(std::abs(angle)) -0.3*std::sqrt(pid.TotalError()),0.3);

          pid.Kp = p[0];
          pid.Ki = p[1];
          pid.Kd = p[2];

          steer_value = std::max(std::min(-pid.Kp*pid.p_error \
                                          -pid.Ki*pid.i_error \
                                          -pid.Kd*pid.d_error, 1.0),-1.0);

          // Send data to the socket/
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

//          best_err = cte;
//          std::cout<<"cte, best:  "<<cte<<"    "<<best_err<< std::endl;
          // Adding twiddle

          if(dpsum > tol && speed > 10){
              if (!meas1){
                  p[ctrl] = p[ctrl] + dp[ctrl];

                  meas1 = true;
//                  std::cout<< "p1"<<std::endl;
               }
               else{
                  // make a measurement and recalculate
                  if (cte < best_err && meas1){
                      best_err = cte;
                      dp[ctrl] *= 1.1;
                      meas1 = false;
                      ctrl = (ctrl + 1) % 3;
 //                     std::cout<< "p2"<<std::endl;

                  }
                  else{
                      if (!meas2){
                      p[ctrl] = p[ctrl] - 2*dp[ctrl];
                      // make a measurement and recalculate
                      meas2 = true;
//                      std::cout<< "p3"<<std::endl;
                      }
                      else{
                          if (cte < best_err && meas2){
                                best_err = cte;
                                dp[ctrl] *= 1.1;
                                meas1 = false;
                                meas2 = false;
 //                               std::cout<< "p4"<<std::endl;
                                ctrl = (ctrl + 1) % 3;
                            }
                            else{
                                p[ctrl] = p[ctrl] + dp[ctrl];
                                dp[ctrl] *= 0.9;
 //                               std::cout<< "p5"<<std::endl;
                                meas1 = false;
                                meas2 = false;
                                ctrl = (ctrl + 1) % 3;
                            }
                      }
                    }
                  }
                  it += 1;
                  dpsum = dp[0]+dp[1]+dp[2];
 //                 std::cout<<ctrl<<std::endl;
                  std::cout<<"p:"<<p[0]<<"   "<<p[1]<<"   "<<p[2]<<"   "<<dpsum<<std::endl;

              }
           }
        }
        else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

