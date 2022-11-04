#include <fstream>
#include <math.h>
#include "uWS/uWS.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "spline/src/spline.h"
#include "path.h"
#include <map>
#include <iterator>
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

map<int, vector<double> > predict(vector<vector <double>> sensor_fusion, double time_delay) {
 
 map<int, vector<double> > predictions;
 
 //analyze the sensor_fusion and predict future states of other cars
 //sensor fusion data:     [ id, x, y, vx, vy, s, d].

    vector<double> pred_state(6);
    double speed;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        speed = sqrt(sensor_fusion[i][3]*sensor_fusion[i][3] + sensor_fusion[i][4]*sensor_fusion[i][4]);
        pred_state[0] = sensor_fusion[i][1] + speed * time_delay;
        pred_state[1] = sensor_fusion[i][2] + speed * time_delay;
        pred_state[2] = sensor_fusion[i][3];
        pred_state[3] = sensor_fusion[i][4];
        pred_state[4] = sensor_fusion[i][5] + speed * time_delay; //may need to convert speed to s,d space?
        pred_state[5] = sensor_fusion[i][6];  //assuming cars stay on their lane. A risky move? Could use Naive-Bayes prediction

        predictions.insert(std::make_pair(sensor_fusion[i][0], pred_state)); 
    }

return predictions;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  Path path;
  Vehicle ego;
    
  path.define_spline(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

  // upsample waypoints once for all
  
  vector<double>map_extendedwp_s;
  vector<double>map_extendedwp_x;
  vector<double>map_extendedwp_y;
  vector<double>map_extendedwp_dx;
  vector<double>map_extendedwp_dy;  
  
  for (double s = 0; s < max_s; s = s + 0.5) {
    map_extendedwp_s.push_back(s);
    map_extendedwp_x.push_back(path.s2x(s));   
    map_extendedwp_y.push_back(path.s2y(s));   
    map_extendedwp_dx.push_back(path.s2dx(s));   
    map_extendedwp_dy.push_back(path.s2dy(s));   
}

  

  h.onMessage([&map_extendedwp_x,&map_extendedwp_y,&map_extendedwp_s,&map_extendedwp_dx,&map_extendedwp_dy, &path, &ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
          	
          	//initialize ego vehicle
          	int lane = trunc(car_d/4.0);  //4 is the lane width (could be set as a constant)
          	ego.init(lane, car_s, car_speed, 0, end_path_s, end_path_d, previous_path_x, previous_path_y);    //this is where my car is at the present
          	//cout << "Ego car data:" << endl<< ego.display() << endl;
          	
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	// In this case, we consider the points that haven't been processed yet.
            
            map<int, vector <double> > predictions;
            double time_delay=0;                        
            //we predict where cars will be in the future to avoid collisions and optimize path
            if ((previous_path_x.size() > 0) && (sensor_fusion.size() > 0)) {
                time_delay = path.time_interval * path.path_size;
                predictions = predict(sensor_fusion, time_delay);

                //print predictions at time_delay
                //std::map<int, vector<double>>::iterator it = predictions.begin();
                //while(it != predictions.end())
                //{
                //    std::cout << it->first << " :: " << it->second[5] << std::endl; //[5] is the lane
                //    it++;
                //}
            }    
            
            //using a state machine, we determine the best next state
            cout << "==========================================" << endl;
            cout << "Data received from the simulator (car NOW)" << endl;
            cout << "Ego state:" << ego.state << ", Ego lane:" << ego.lane << endl;
            cout << "Ego parameters:" << endl << ego.display() << endl;
            ego.update_state(predictions);
            cout << "Data suggested to the simulator (car NEXT)" << endl;
            cout << "Ego state:" << ego.state << ", Ego lane:" << ego.lane << endl;
            cout << "Ego parameters:" << endl << ego.display() << endl;
            double ref_v, ref_d;
            if (car_speed == 0) {
                ref_v = 5   ;   // was ego.target_speed;
            } else {
                ref_v = ego.v;
            }
            
            if (ego.target_lane != (ego.lanes_available-1)) {
                ref_d = 2 + 4 * ego.target_lane;
            } else {
                ref_d = (2 + 4 * ego.target_lane)*0.98; //reduce d slightly to avoid rare lane violations on the right most lane.
            }    
            
            cout << "Reference d (just before sending to path):" << ref_d << endl;
            path.Init(ref_v, ref_d, 5, car_x, car_y, car_s, car_d, car_yaw, car_speed, previous_path_x, previous_path_y);
            path.create(next_x_vals, next_y_vals);
            
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

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
















































































