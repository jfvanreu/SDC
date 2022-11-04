#ifndef PATHCONVERTER_H
#define PATHCONVERTER_H
#include <vector>
#include "spline/src/spline.h"

using namespace std;

class PathConverter {

public:
    tk::spline s2x;
    tk::spline s2y;
    tk::spline s2dx;
    tk::spline s2dy;
    
    vector<double> path_x;
    vector<double> path_y;
    vector<double> path_s;
    vector<double> path_d;
    
    //record previous path so we can merge it with recent one
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    vector<double> previous_path_s;
    vector<double> previous_path_d;
    
  /**
  * Constructor
  */
  
  PathConverter(vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_dx, const vector<double>& maps_dy);

  /**
  * Destructor
  */
  virtual ~PathConverter();

  vector<double> convert_sd_to_xy(const double s, const double d);

};

#endif