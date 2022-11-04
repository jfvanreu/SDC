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

void Path::Init(double target_speed, double max_speed, double max_acc, double car_x, double car_y, \
                    double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, \
                    vector<double> previous_path_y ) {

    cout << "Initialize speed factors..." << endl;
    this -> target_speed = target_speed;
    this -> max_speed = max_speed;
    this -> max_acc = max_acc;
    
    path_x={};
    path_y={};
    path_s={};
    path_d={};

    path_size = previous_path_x.size();
    cout << "Path size is ..." << path_size << endl;

    pos_x = car_x;
    pos_y = car_y;
    pos_s = car_s;
    pos_d = car_d;
    
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;

    cout << "complete init function" << endl;
}

void Path::create(vector<double>& next_x_vals, vector<double>& next_y_vals){

    tk::spline x2y;  //lane tracker
    tk::spline t2x; //velocity tracker

    cout << "enters create function" << endl;
    
    vector<double> xy_coord;
    
    if (path_size == 0 && prev_path_x.size() == 0) { //means we're starting a brand new path        
            
            for(int i = 0; i < 50; i++)
                {
                  pos_s = pos_s + dist_inc; // we move 0.5 m per 0.02 sec or 25 m/s or 56 mph
                  pos_d = 6;
                  xy_coord = convert_sd_to_xy(pos_s, pos_d);
                  path_x.push_back(xy_coord[0]);
                  path_y.push_back(xy_coord[1]);
                }
        cout << "Good start!" << endl;
    }
            
  else {    //we can leverage previous data to extend our path
        
       //we create a x(t) spline using the 10 previous path inputs + a point in the future
       vector<double> x_4_spline = {};
       vector<double> y_4_spline = {};
       vector<double> t_4_spline = {};
       
       for (int i=0; i < path_size; i++) { // pick up points that were not processed yet
         x_4_spline.push_back(previous_path_x[i]);   
         y_4_spline.push_back(previous_path_y[i]);
         t_4_spline.push_back(i* time_interval);
       }
       
       //car is currently at pos_s;
       double goal_s = pos_s + 25; //add 1 point  (i.e. 1 sec)
       double goal_d = 6;
       double target_speed = 25;
    
       // we use previous information to create a spline for position and velocity
       xy_coord = convert_sd_to_xy(goal_s, goal_d);
    
       x_4_spline.push_back(xy_coord[0]);    // add goal point in the future
       y_4_spline.push_back(xy_coord[1]);
       cout << "added goal point!" << endl;
       cout << "x2y setpoints after pushback: " << x_4_spline.size() << "," << y_4_spline.size() << endl;
                    
       //we create a lane tracker based on the previous path
       cout << "Before x & y sorting...." << x_4_spline.size() << " & " << y_4_spline.size();
       sort_coords(x_4_spline, y_4_spline);   //returns smaller vectors if equals!!
       cout << "After x & y sorting....size is: " << x_4_spline.size() << " & " << y_4_spline.size();
       x2y.set_points(x_4_spline, y_4_spline);
          
       //create a speed tracker based on the previous path
       // can we assume that all points
       //still need to add time expected for reaching goal.
       t_4_spline.push_back(T - time_interval); //we want to reach goal at time T.
        
       //sort_coords(t_4_spline, x_4_spline); //not really needed since time is incremental
       t2x.set_points(t_4_spline, x_4_spline);
        
       //print the splines
       for (int i=0; i < x_4_spline.size(); i++) {
           cout << "x4spline:" << x_4_spline[i] << ", y4spline:" << y_4_spline[i] << ", t4spline:" << t_4_spline[i] << endl;
       }

       // splines are now created, we can use them to generate data

       //we can now generate our path. We start by using the points that were not processed.
       //starts to feed the path with the previous values that haven't been processed yet
       //double t = 0;
       /*for (int i = 0; i < path_size; i++) {
            path_x.push_back(previous_path_x[i]);
            path_y.push_back(previous_path_y[i]);
            t = t + time_interval;
        }*/
        
        //we can now regenerate the path for the future points
        
        for (int t = 0; t < T; t=t + time_interval) {
            pos_x = t2x(t);
            cout << "pos_x is:" << pos_x << "\t";
            pos_y = x2y(pos_x);
            cout << "pos_y is:" << pos_y << endl;
            path_x.push_back(pos_x);
            path_y.push_back(pos_y);
        } 
    }
    cout << "Path X size: " << path_x.size();
    cout << "Path Y size: " << path_y.size();
    next_x_vals = path_x;
    next_y_vals = path_y;
    prev_path_x = path_x;
    prev_path_y = path_y;
    prev_path_s = path_s;
    prev_path_d = path_d;
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