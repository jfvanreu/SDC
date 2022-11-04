#include <iostream>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>


/** 
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::init(int lane, double s, double v, double a, double end_path_s, double end_path_d, vector<double> previous_path_x, vector<double> previous_path_y) {
    
    // we initialize the car with the parameters at the end of the path
    this->lane = trunc(end_path_d/4);
    this->s = end_path_s;
    
    //initialize the speed; it's either v (if we don't have two elements in previous_path_x and previous_path_y)
    if (previous_path_x.size() > 2) {
        int last = previous_path_x.size() - 1;
        double vx = (previous_path_x[last] - previous_path_x[last-1])/0.02;     //distance meters per sec
        double vy = (previous_path_y[last] - previous_path_y[last-1])/0.02;     //distance meters per sec
        double speed = sqrt(vx*vx + vy*vy); //speed in meters per sec;
        this -> v = speed*2.24;    
    } else {
        this->v = v;
    } 
    
    cout << "velocity at the end of the path:" << this-> v << endl;
    //this->a = a;
    //this->max_acceleration = -1;
    
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
}

void Vehicle::state_machine(vector<string>& possible_states) {
    
    //returns the possible states based on the current state
    
    //KL: Keep Lane at max speed (50 mph for this project)
    //KLSD: Keep Lane but Slow Down (to stay away from car in front)
    //LCR: Lane Change Right
    //LCL: Lane Change Left
    
    if (this->state.compare("KL") == 0) {
        possible_states = {"KL", "KLSD"};
        if (lane == target_lane) {
            if (target_lane !=0) {possible_states.push_back("LCL");}
            if (target_lane != lanes_available - 1) {possible_states.push_back("LCR");}
        }
    } else if (this->state.compare("KLSD") == 0) {  //can't change lane if we slow down
       possible_states = {"KL", "KLSD"};
    } else if (this->state.compare("LCL") == 0) {
        possible_states = {"KL"};    //we could possibly add LCR and KLSD
    } else if (this->state.compare("LCR") == 0) {
        possible_states = {"KL"};
    } else possible_states = {"KL"};
}

void Vehicle::update_state(map<int, vector<double>>& predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    
    //determine the possible states using a state machine
    vector<string> possible_states={};
    
    //generate our possible states    
    cout << "Our last state is:" << state << ". The car is aiming lane:" << target_lane << endl;
    state_machine(possible_states);

    cout << "The possible next states are:" << endl;
    for (int i=0; i< possible_states.size(); i++) {
        cout << possible_states[i] << endl;
    }
    
	// create test vehicle to measure potential cost if state change is realized
	vector<int> possible_costs(possible_states.size());

    for (int i=0; i < possible_states.size(); i++) {
   	  	Vehicle check_car = * this;  //I create a copy of the car to see how it would react to state changes
       	//check_car.init(this->lane, this->s, this->v, 0, this->end_path_s, this->end_path_d, this->previous_path_x, this->previous_path_y);
       	//check_car.target_lane = this -> target_lane; // I create a copy of the ego car
        check_car.state = possible_states[i];
        check_car.realize_state(predictions);   //applies a state transition to the check_car
        if ((check_car.state == "KL") && (predictions.size() > 0)) {
            cout << "Checking keep lane costs...entering" << endl;
            possible_costs[i] = check_car.keep_lane_cost_function(predictions);
            cout << "Checking keep lane costs...exiting" << endl;
        } else if (check_car.state == "KLSD") {
            possible_costs[i] = check_car.slow_down_cost_function();
        } else if (check_car.state == "LCL") {
            possible_costs[i] = check_car.change_lane_cost_function(predictions);
        } else if (check_car.state == "LCR") {
            possible_costs[i] = check_car.change_lane_cost_function(predictions);   
        } else {
            cout << "Error with cost functions" << endl;         	
        }
    }    
    
    // identify the lowest cost
    double min_cost = 99999;
    string min_state;
    for (int i=0; i< possible_costs.size(); i++) {
        cout << possible_states[i] << ":";
        cout << possible_costs[i] << endl;
        if (possible_costs[i] < min_cost) {
            min_cost = possible_costs[i];
            min_state = possible_states[i];
        }
    }
    
    cout << "Selected the following state:" << min_state << endl;
    state = min_state; // this is an example of how you change state.
    cout << "Before applying the change: " << display() << endl;
    //realize best state
    realize_state(predictions);
    cout << "After applying the change: " << display() << endl;

}

void Vehicle::realize_state(map<int,vector<double>>& predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if (state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("KLSD") == 0)
    {
    	realize_keep_lane_slow_down(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
}

void Vehicle::realize_keep_lane(map<int,vector<double>>& predictions) {

//we continue to increase the speed until we reach the target speed
double velocity_diff = target_speed - this-> v;
double next_vel_incr = min (velocity_diff, min_acceleration);   //limiting acceleration based on max acceleration requirement

this -> v += next_vel_incr;

if (this -> v > target_speed) {v = target_speed;}

//we compute the new value of s at the end of the path

this -> s = this-> end_path_s + 0.02 * this-> v;      //we set s to the last s on the path

}	

void Vehicle::realize_keep_lane_slow_down(map<int,vector<double>>& predictions) {	
    
    double speed_target = speed_in_front(predictions);
    
    //we need to decrease the speed to align with the car in front of us but without decelerating too much.
    double velocity_diff = abs(speed_target - this-> v);
    double vel_decr = min(velocity_diff, max_acceleration);   //limiting acceleration based on max acceleration requirement
    
    this -> v -= vel_decr; 
    cout << "Reduce speed!! Reduce speed!!" << endl;    
    this -> s = this -> end_path_s + 0.02 * this -> v;    //using the last s on the path and time interval of 0.02
}

void Vehicle::realize_lane_change(map<int,vector<double>>& predictions, string direction) {
	int delta = 1;
	
	//if speed is close to max speed, keep speed
	if ((this-> v + min_acceleration) > target_speed) {
	    this -> v -= min_acceleration;   // reduce speed slightly
	} else {    //accelerate slightly
	    this -> v += min_acceleration;
	}
	
	int lane = this -> lane;    //may need to be target_lane.
    cout << "Before realizing lane change:" << lane << endl;
    if (direction.compare("L") == 0)
    {
    	delta = -1;
    }
    lane += delta;
    cout << "After realizing " << direction << " lane change:" << lane << endl;
    int s = this->s;
    this -> s = this-> end_path_s + 0.02 * this-> v;
    this -> lane = lane;
    target_lane = lane;
    //this->a = _max_accel_for_lane(predictions, lane, s);
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    oss << "state:" << this->state << "\n";
    oss << "last s:" << this->end_path_s << "\n";
    oss << "last d:" << this->end_path_d << "\n";
    oss << "target lane:" << this->target_lane << "\n";
    
    return oss.str();
}

vector<double> Vehicle::distance_with_others(map<int,vector<double>>& predictions) {
/*This method provides the distance to the closest car in front and behind of us*/
    vector<double> distance_front_back={500,500};
    vector<double> distance_front={};
    vector<double> distance_back={};
    std::map<int, vector<double>>::iterator it = predictions.begin();
    
    cout << "Predictions is size:" << predictions.size() << endl;
    while(it != predictions.end())
    {   
        double d_it = it->second[5];
        double s_it = it->second[4];

        int lane_it = trunc(it->second[5]/4.0);

        if (lane_it == this->lane) {    //check other cars in the same lane as this car
            if (s_it >= this->s) {  
                // in this case, we're looking at the future (end of the green path) cars in front of us
                cout << "there is car " << it->first << " close AHEAD of us...." << (s_it - this->s) << endl;
                distance_front.push_back(s_it - this -> s);              
            } else if (s_it <= this->s) {
                //cout << "there is car " << it->first << " close BEHIND us...." << (this->s - s_it) << endl;
                distance_back.push_back(this -> s - s_it);
            }
        }        
        it++;
    }

    if (distance_front.size() > 0) {
        auto dist_front = min_element(begin(distance_front), end(distance_front));
        distance_front_back[0] = double(*dist_front);
    
    } else {
        distance_front_back[0] = 500.0;   //put a high distance if there is no vehicle ahead of us
    }
    
    if (distance_back.size() > 0) {
        auto dist_back = min_element(begin(distance_back), end(distance_back));    
        distance_front_back[1] = double(*dist_back);
    } else {
        distance_front_back[1] = 500.0;   //put a high distance if there is no vehicle close to us
    }
        
    return distance_front_back;
    
    //we only look at the closest cars, we could check all the cars within the buffer distance in the lane
}

int Vehicle::keep_lane_cost_function(map<int,vector<double>>& predictions) {

    int KL_cost=0;
    vector<double> distance={};
    
    //check if the closest car is within safe distance
    distance = distance_with_others(predictions);
    if (distance[0] < this -> preferred_buffer) {KL_cost = 500;} 
    
    return KL_cost;
}

int Vehicle::slow_down_cost_function() {
//simply return 200 so it's the least preferred option after Keep lane and Change Lane
    
    int SD_cost = 200;
    return SD_cost;
}

int Vehicle::change_lane_cost_function(map<int,vector<double>>& predictions) {

    int CL_cost = 0;
    vector<double> distance={};

    distance = distance_with_others(predictions);
    if ((distance[0] > this->preferred_buffer) && (distance[1] > this->preferred_back_buffer)) {
        if (distance[0] > 200) {
            CL_cost += 120; //favors lane where cars are far away in case we need to chose
        } else {
            CL_cost += 150;
        }          
    } else {  //we still prefer to stay on the lane if it's not safe
        CL_cost += 600;
        }
    return CL_cost;
}

double Vehicle::speed_in_front(map<int,vector<double>>& predictions) {
/*This method provides the speed of the closest car in front of us*/
    std::map<int, vector<double>>::iterator it = predictions.begin();
    vector<double> dist = distance_with_others(predictions);
    double speed;
       
    while(it != predictions.end())
    {   
        double d_it = it->second[5];
        double s_it = it->second[4];

        int lane_it = trunc(it->second[5]/4.0);

        if ((lane_it == this->lane) && (s_it >= this->s)) {    //check other cars in the same lane as this car 
        //we look for the speed of the car in front of us
            if (dist[0] == (s_it - this->s)) {   //means we have found the closest car in front of us
                cout << "Car ahead of us is at distance:" << (s_it - this->s) << endl;
                speed = sqrt(it->second[2]*it->second[2] + it->second[3]*it->second[3])*2.24;   //vx,vy are in meters per sec
            }
        }        
        it++;
    }
   
   cout << "the car ahead of us drives at the speed: " << speed << endl;
   return speed; 
}