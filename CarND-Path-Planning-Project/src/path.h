#ifndef PATH_H
#define PATH_H
#include <vector>
#include "spline/src/spline.h"

using namespace std;

class Path {

public:
    tk::spline s2x;
    tk::spline s2y;
    tk::spline s2dx;
    tk::spline s2dy;
    
    int path_size;
    vector<double> path_x;
    vector<double> path_y;
    vector<double> path_s;
    vector<double> path_d;
    
    //current lane
    double current_d;
    
    //record previous path so we can merge it with recent one
    vector<double> prev_path_x;
    vector<double> prev_path_y;
    vector<double> prev_path_s;
    vector<double> prev_path_d;
    
    
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    
    //ref positions in the path
    double ref_x;
    double ref_y;
    double ref_s;
    double ref_d;
    double ref_yaw;
            
    //path variables
    const double time_interval = 0.02;
    const int T = 1;
    const double dist_inc = 0.25;   //was 0.5
    
    //path constraints
    double target_speed;
    double max_speed;
    double max_acc;
    
  /**
  * Constructor
  */
  
  Path();
  
  /**
  * Destructor
  */
  virtual ~Path();

  void Init(double target_speed, double target_d, double max_acc, double car_x, double car_y, \
                    double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, \
                    vector<double> previous_path_y);
  void create(vector<double>& next_x_vals, vector<double>& next_y_vals);
  int ClosestWaypoint(double x, double y, vector<double>& maps_x, vector<double>& maps_y);
  int NextWaypoint(double x, double y, double theta, vector<double>& maps_x, vector<double>& maps_y);
  void define_spline(vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_dx, const vector<double>& maps_dy); 
  vector<double> getFrenet(double x, double y, double theta, vector<double>& maps_x, vector<double>& maps_y);
  vector<double> convert_sd_to_xy(const double s, const double d);
  vector<double> JMT(vector< double> start, vector <double> end, double T);

};

#endif