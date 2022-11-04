#include "pathconverter.h"
#include <vector>

/**
 * Initializes PathConverter
 */

PathConverter::PathConverter(vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_dx, const vector<double>& maps_dy) {

    //create the spline functions with all 181 entries; input data is sorted by s.
    
    this->s2x.set_points(maps_s, maps_x);
    this->s2y.set_points(maps_s, maps_y);
    this->s2dx.set_points(maps_s, maps_dx);
    this->s2dy.set_points(maps_s, maps_dy);
}

PathConverter::~PathConverter() {}

vector<double> PathConverter::convert_sd_to_xy(const double s, const double d) {

  const double x_edge = this->s2x(s);
  const double y_edge = this->s2y(s);
  const double dx = this->s2dx(s);
  const double dy = this->s2dy(s);

  const double x = x_edge + dx * d;
  const double y = y_edge + dy * d;

  return {x, y};
}
