#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

//Global variable
bool run_twiddle = false;

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

int main(int argC, char** argV)
{
  uWS::Hub h;

  PID pid;

  double Kp_initial, Ki_initial, Kd_initial;

  if (argC > 1) {
    Kp_initial = atof(argV[1]);
    Ki_initial = atof(argV[2]);
    Kd_initial = atof(argV[3]);
    if (argC > 4) {
      std::string is_run_twiddle = argV[4];
      if (is_run_twiddle.compare("twiddle") == 0) {
        run_twiddle = true;
      }
    }
  } else {
    Kp_initial = -0.091;
    Ki_initial = -0.0005;
    Kd_initial = -1.693;
  }
  //Init PID with P, I and D constants
  std::cout<<"Kp is: "<<Kp_initial<<" Ki is: "<<Ki_initial<<" Kd is: "<<Kd_initial<<std::endl;
  pid.Init(Kp_initial, Ki_initial, Kd_initial, run_twiddle);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static unsigned int timesteps = 0;
    static double total_error = 0.0;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          
          //Update the error in system using cte.
          pid.UpdateError(cte);
          //Derive the steering angle in order to minimize the error (or cte).
          steer_value = pid.TotalError();
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "Timestep: " << timesteps << std::endl;
          if (run_twiddle) {
            if (timesteps > 500) {
              pid.Twiddle(total_error, pid.Kp);
              pid.Restart(ws);
              timesteps = 0;
              total_error = 0.0;
              return;
            } else {
              total_error += pow(cte, 2);
            }
            timesteps++;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.25;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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