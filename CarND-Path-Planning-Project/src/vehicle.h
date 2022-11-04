#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int lane;

  double s;

  double v;

  double a;
  
  double end_path_s;
  
  double end_path_d;
  
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  
  int target_lane = 1;  //we start in the middle lane
  
  int preferred_buffer = 30;
  
  int preferred_back_buffer = 12;

  double target_speed = 49;

  int lanes_available=3;

  double max_acceleration = 0.4;    //used to slow down was 0.5
  double min_acceleration = 0.05;    //used to accelerate forward

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void init(int lane, double s, double v, double a, double end_path_s, double end_path_d, vector<double> previous_path_x, vector<double> previous_path_y);

  void update_state(map<int,vector<double> >& predictions);
  
  void state_machine(vector<string>& possible_states);

  //void configure(vector<int> road_data);

  string display();

  //void increment(int dt);

  //vector<int> state_at(int t);

  //bool collides_with(Vehicle other, int at_time);

  //collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int,vector<double> >& predictions);

  //void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<double> >& predictions, int lane, int s);

  void realize_keep_lane(map<int,vector<double> >& predictions);
  
  void realize_keep_lane_slow_down(map<int,vector<double> >& predictions);

  void realize_lane_change(map<int,vector<double> >& predictions, string direction);

  //void realize_prep_lane_change(map<int,vector<double> > predictions, string direction);

  //vector<vector<int> > generate_predictions(int horizon);
  
  vector<double> distance_with_others(map<int,vector<double>>& predictions);
  int keep_lane_cost_function(map<int,vector<double>>& predictions);
  int slow_down_cost_function();
  int change_lane_cost_function(map<int,vector<double>>& predictions);
  double speed_in_front(map<int,vector<double>>& predictions);

};

#endif