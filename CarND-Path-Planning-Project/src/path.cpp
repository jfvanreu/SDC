#include "path.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include <iostream>
#include <fstream>
    
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

//sort function used to sort coordinates
void sort_coords(vector<double>& v1, vector<double>& v2)
{
vector<vector<double> > vv;
int vsize = v1.size();
// create the combined vector with both coordinates
for(int i = 0; i < vsize; ++i) {
    vector<double> vt = {v1[i], v2[i]};
    vv.push_back(vt);
    }

sort(vv.begin(), vv.end());

v1.clear();
v2.clear();
for(int i = 0; i < vsize; ++i) {
	v1.push_back(vv[i][0]);
	v2.push_back(vv[i][1]);
    }
}


/**
 * Initializes PathConverter
 */

Path::Path() {}
Path::~Path() {}

void Path::Init(double target_speed, double target_d, double max_acc, double car_x, double car_y, \
                    double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, \
                    vector<double> previous_path_y) {

    //cout << "Initialize speed factors..." << endl;
    this -> target_speed = target_speed;
    this -> max_acc = max_acc;
    
    path_x={};
    path_y={};
    path_s={};
    path_d={};

    path_size = previous_path_x.size();
    cout << "Path size is ..." << path_size << endl;

    ref_x = car_x;
    ref_y = car_y;
    ref_s = car_s;
    ref_d = target_d;
    ref_yaw = deg2rad(car_yaw);
    current_d = car_d;
    
    this -> previous_path_x = previous_path_x;
    this -> previous_path_y = previous_path_y;
        
    //cout << "complete init function" << endl;
}

void Path::create(vector<double>& next_x_vals, vector<double>& next_y_vals){

    tk::spline x2y;  //lane tracker
    
    //cout << "enters create function" << endl;
    
    //we first create the path we would like to take using past and future coordinates
    
    if (path_size == 0 ) { //we don't have a path yet
        
        //double prev_car_x = ref_x - cos(ref_yaw);   
        //double prev_car_y = ref_y - sin(ref_yaw);
        
        //path_x.push_back(prev_car_x);
        path_x.push_back(ref_x);
        //path_y.push_back(prev_car_y);
        path_y.push_back(ref_y);
    
    } else { //we have a longer path. We can use the last two points to build our new path
    
        ref_x = previous_path_x[path_size-1];
        ref_y = previous_path_y[path_size-1];
        
        double ref_x_prev = previous_path_x[path_size-2];
        double ref_y_prev = previous_path_y[path_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        
        path_x.push_back(ref_x_prev);
        path_y.push_back(ref_y_prev);
        
        path_x.push_back(ref_x);
        path_y.push_back(ref_y);
    }
    
    // add future points to our path
    
    vector<double> xy_coord;
    double next_s, next_d;
    vector<double> trajectory_points;
    
    // if lane change use {45,60,75} as it is softer trajectory
    // otherwise use {45, 60, 75} which is a bit tighter and avoids to leave lane.
    
    if (trunc(ref_d/4) != trunc(current_d/4)) { // if lane change
        trajectory_points = {60,75,90};
    } else {
        trajectory_points = {30,60,90};
    }   
               
    for(int i = 0; i < 3; i++) {
      next_s = ref_s + trajectory_points[i]; // we pick a point 30 meters away
      next_d = ref_d;
      xy_coord = convert_sd_to_xy(next_s, next_d);
      path_x.push_back(xy_coord[0]);
      path_y.push_back(xy_coord[1]);
    }
    
    // convert path points to car coordinate system
    for (int i=0; i < path_x.size(); i++) {
      double shift_x = path_x[i] - ref_x;
      double shift_y = path_y[i] - ref_y;
      
      path_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0-ref_yaw));
      path_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0-ref_yaw));
      
    }
    
    //create a spline that goes through the path_x and path_y points
    x2y.set_points(path_x, path_y);
    
    //let's now create a trajectory using the previous path and spline
    //we first recycle all the points that were not processed from previous_path
    for (int i = 0; i < path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    
    double target_x = 30;   //we create a path from our current position x to 30 meters further.
    double target_y = x2y(target_x);
    double target_distance = sqrt((target_x)*(target_x) - (target_y)*(target_y));
    
    double x_add_on = 0;
    
    //we generate points for our trajectory such that it includes 50 points
    for (int i=0; i < 50 - path_size; i++) {
        //compute the number of N's needed to 
        double N = (target_distance/(time_interval*target_speed/2.24)); //divided by 2.24 because of mph to m/s speed conversion.
        double x_point = x_add_on + (target_x)/N;
        double y_point = x2y(x_point);
        
        x_add_on = x_point;
        double x_ref = x_point;
        double y_ref = y_point;
        
        //convert point from car coordinate system back to global coord system
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
        
        x_point += ref_x;
        y_point += ref_y;
        
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
            
    //next_x_vals = path_x;
    //next_y_vals = path_y;
    //prev_path_x = path_x;
    //prev_path_y = path_y;
    //prev_path_s = path_s;
    //prev_path_d = path_d;
    cout << "Goes to the end of Create" << endl;   
}


vector<double> Path::convert_sd_to_xy(const double s, const double d) {

  const double x_edge = this->s2x(s);
  const double y_edge = this->s2y(s);
  const double dx = this->s2dx(s);
  const double dy = this->s2dy(s);

  const double x = x_edge + dx * d;
  const double y = y_edge + dy * d;

  return {x, y};
}


int Path::ClosestWaypoint(double x, double y, vector<double>& maps_x, vector<double>& maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


int Path::NextWaypoint(double x, double y, double theta, vector<double>& maps_x, vector<double>& maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

void Path::define_spline(vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_dx, const vector<double>& maps_dy) {

    //create the spline functions with all 181 entries; input data is sorted by s.
    
    this->s2x.set_points(maps_s, maps_x);
    this->s2y.set_points(maps_s, maps_y);
    this->s2dx.set_points(maps_s, maps_dx);
    this->s2dy.set_points(maps_s, maps_dy);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Path::getFrenet(double x, double y, double theta, vector<double>& maps_x, vector<double>& maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

vector<double> Path::JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    Eigen::MatrixXd T_mat (3,3), T_mat_inv (3,3);
    Eigen::VectorXd vect(3);
    Eigen::VectorXd alpha3_6_vect(3);
    
    vector<double> ret_vect(6);
    
    T_mat <<    pow(T,3), pow(T,4), pow(T,5),
                3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
                6*T, 12*pow(T,2), 20*pow(T,3);
    
    vect <<     end[0] - (start[0]+start[1]*T+0.5*start[2]*pow(T,2)),
                end[1] - (start[1] + start[2]*T),
                end[2] - start[2];
    
    T_mat_inv = T_mat.inverse();
        
    alpha3_6_vect = T_mat_inv * vect;
        
    ret_vect[0] = start[0];
    ret_vect[1] = start[1];
    ret_vect[2] = start[2]*0.5;
    ret_vect[3] = alpha3_6_vect[0];
    ret_vect[4] = alpha3_6_vect[1];
    ret_vect[5] = alpha3_6_vect[2];
    
    return ret_vect;
}