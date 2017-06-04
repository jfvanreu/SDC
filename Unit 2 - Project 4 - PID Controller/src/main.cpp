#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#define TWIDDLE FALSE
#define RACEDRIVING FALSE

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

int main()
{
  uWS::Hub h;

  PID steer_pid;
  PID throttle_pid;   

  // TODO: Initialize the steer_pid variable.
    steer_pid.Init(0.0055, 0.0001, 0.2);
    
  // Initialize Speed PID IN RACEDRIVING mode
   if (RACEDRIVING) {
    throttle_pid.Init(1, 0, 0);
    }

  h.onMessage([&steer_pid, &throttle_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double steer_value = 0;
          double throttle_value = 0.3;
          double run_error = 0;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          //Provides the steering angle based on the cte
          steer_pid.UpdateError(cte);
          steer_value = steer_pid.TotalError();
          
          //Provides the throttle based on the cte
          if (RACEDRIVING) {
              throttle_pid.UpdateError(cte);
              throttle_value = throttle_pid.TotalError();
           } else if (steer_pid.steps < 100) {
            //start with higher throttle value to reduce initial sinusoidal moves
            throttle_value = 1;
           }
           
                     
          //collect all errors on this run
          steer_pid.error_sum += fabs(cte);
          

          //ensure steer_value remains within [-1,1]
          if (steer_value < -1) {
            steer_value = -1;
          } else if (steer_value > 1) {
            steer_value = 1;
          }
          
          //inverse the throttle value so we accelerate on the straight aways
          
          if (RACEDRIVING) {
            std::cout << "Throttle Value: " << throttle_value << std::endl;
            // go fast when CTE is small and slow(er) when CTE is large
            throttle_value = 1/(fabs(throttle_value));
            
            //try to control the car a bit more at high-speed
            if (speed > 50.0) {
                //reduce speed in a high curve
                if (fabs(cte) > 0.5) {
                    if (fabs(angle)> 7.5) { //fast in a curve
                        throttle_value = -2.00;
                    }                         
                } else { // don't turn as much at high speed on straight aways
                steer_value = steer_value*0.90;
                }
            }
          }
          // convert steer_value to angle
          steer_value = rad2deg(steer_value);

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; //was 0.3
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          steer_pid.steps++; //the car took one step
          //std::cout << "Steps: " << steer_pid.steps << std::endl;
          if ((steer_pid.steps > steer_pid.max_steps) && TWIDDLE) { // only drive so many steps then restart with new parameters     
            //update parameters
            run_error = (steer_pid.error_sum/steer_pid.max_steps);
            std::cout << "Average error:" << run_error \
                << " with Kp (" << steer_pid.Kp << "), Ki (" << steer_pid.Ki << "), Kd (" << steer_pid.Kd << ")" << std::endl;
            steer_pid.TwiddleUpdate(0.2, ws);
          }
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
