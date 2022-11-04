#include <iostream>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>


bool removeState(vector<string> current_states, string label) {
   bool ret_val = false;
   
   string activeState;
   for (int i=0; i < current_states.size(); i++) {
       activeState = current_states[i];
       if (activeState == label) {
           current_states.erase(current_states.begin()+i);
           ret_val = true;
       }
   }
   cout << "current_states is now of size:" << current_states.size() << endl;
   return ret_val;
}

/** 
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::init(int lane, double s, double v, double a, double end_path_s, double end_path_d) {
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->max_acceleration = -1;
    
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
}

void Vehicle::state_machine(vector<string>& possible_states) {
    
    //returns the possible states based on the current state
    
    //KL: Keep Lane at max speed (50 mph for this project)
    //KLSD: Keep Lane but Slow Down (to stay away from car in front)
    //LCR: Lane Change Right
    //LCL: Lane Change Left
    
    if (this->state.compare("KL") == 0) {
        possible_states = {"KL", "KLSD", "LCL", "LCR"};
        //possible_states = {"KL", "KLSD","LCR", "LCL"};
    } else if (this->state.compare("KLSD") == 0) {
        possible_states = {"KL", "KLSD", "LCL", "LCR"};
        //possible_states = {"KL", "KLSD", "LCL", "LCR"};    
    } else if (this->state.compare("LCL") == 0) {
        possible_states = {"KL"};    //we could possibly add LCR and KLSD
    } else if (this->state.compare("LCR") == 0) {
        possible_states = {"KL"};
    } else possible_states = {"KL"}; 
    
    if (this->lane == 0) { //if we're on the left lane, we can't go left anymore
        removeState(possible_states, "LCL");
    } else if (this->lane == this->lanes_available - 1) { //can't go right of the right lane
        removeState(possible_states,"LCR");
    }
}

void Vehicle::update_state(map<int, vector<double>>& predictions, double time_delay) {
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
    vector<string> possible_states;
    state_machine(possible_states);
    
	// create test vehicle to measure potential cost if state change is realized
	//cout << "ego car info:" << this -> display() << endl;
	vector<double> possible_costs(possible_states.size());
    
    cout << "possible states size:" << possible_states.size() << endl;

    for (int i=0; i < possible_states.size(); i++) {
   	  	Vehicle check_car;
       	check_car.init(this->lane, this->s, this->v, 0, this->end_path_s, this->end_path_d);
        check_car.state = possible_states[i];
        check_car.realize_state(predictions, time_delay);
        possible_costs[i] = 0; //initialize cost associated to possible state
        
       	//cout << "check car info for state:" << possible_states[i] << endl;
    	//cout << check_car.display() << endl;
        
        //compute the various costs associated to this state change
        
        // too close to car(s) in front, so it 
        std::map<int, vector<double>>::iterator it = predictions.begin();
                while(it != predictions.end())
                {   
                    double d_it = it->second[5];
                    double s_it = it->second[4];
                
                    int lane_it = trunc(it->second[5]/4.0);
                
                    if (lane_it == check_car.lane) {    //check other cars in the same lane as check_car
                        if (s_it > check_car.s) {  
                            // in this case, we're looking at the future (end of the green path)
                            cout << "there is car " << it->first << " close ahead of us...." << (s_it - check_car.s) << endl;
                            if ((s_it - check_car.s) < preferred_buffer) {//   (buffer zone)
                                possible_costs[i] += 1/(s_it - check_car.s); //penalize when we are too close                            
                                //cout << "Possible cost with s_it - check_car.s: " << possible_costs[i] << endl;
                            }    
                        } else {
                            possible_costs[i] += 0; // no issues (assuming nobody will rear end us)
                        }                 
                    }
                    it++;
                }
    
        // current speed compared to target speed
        possible_costs[i] += 5*(target_speed - check_car.v);
    
        // change lane cost
        // collision?
    
        //possible_costs[i] += 1*(10 - check_car.v);
        //possible_costs[i] += 10* pow(check_car.lane - this->goal_lane,2);
        //possible_costs[i] += 10*(1 - exp(-abs(check_car.lane - 3)/(300 - (double)check_car.s)));
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
    //realize state again
    realize_state(predictions, time_delay);
}

void Vehicle::realize_state(map<int,vector<double>> predictions, double time_delay) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if (state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions, time_delay);
    }
    else if(state.compare("KLSD") == 0)
    {
    	realize_keep_lane_slow_down(predictions, time_delay);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L", time_delay);
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R", time_delay);
    }
}

void Vehicle::realize_keep_lane(map<int,vector<double>> predictions, double time_delay) {

//we only compute what s will be in the future.
double prev_v = this -> v;

// trying to manage speed to avoid rough acceleration start
if (this-> v ==0) {v += 1;}
else {this -> v *= 1.05};
if (this -> v > target_speed) {v = target_speed;}
this -> s += time_delay * prev_v + 0.02 * this -> v;    //using time interval of 0.02
this->a = _max_accel_for_lane(predictions, this->lane, this->s);

}	

void Vehicle::realize_keep_lane_slow_down(map<int,vector<double>> predictions, double time_delay) {	
	//cout << "target speed is...." << target_speed << endl;
    double prev_v = this -> v;
    this -> v *= 0.96;  //for a target speed of 49.5 was 2.24;
    cout << "Reduce speed!! Reduce speed!!" << endl;    
    this -> s += time_delay * prev_v + 0.02 * this -> v;    //using time interval of 0.02
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector<double>> predictions, string direction, double time_delay) {
	int delta = 1;
    cout << "Before realizing lane change:" << this->lane << endl;
    if (direction.compare("L") == 0)
    {
    	delta = -1;
    }
    this->lane += delta;
    cout << "After realizing " << direction << " lane change:" << this->lane << endl;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
    
    //still need to consider time_delay

}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    oss << "state:" << this->state << "\n";
    
    return oss.str();
}

int Vehicle::_max_accel_for_lane(map<int,vector<double> > predictions, int lane, int s) {

	double delta_v_til_target = target_speed - v;
    double max_acc = std::min(max_acceleration, delta_v_til_target);

    map<int, vector<double> >::iterator it = predictions.begin();
    vector<vector<double>> in_front={};
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
        int lane_it = trunc(it->second[5]/4.0);
        double s_it = it->second[4];
    	
        vector<double> v = it->second;
        
        if((lane_it == lane) && (s_it > s))
        {
        	in_front.push_back(v);
        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<double> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if ((in_front[i][4]-s) < min_s)
    		{
    			min_s = (in_front[i][4]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[4];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	double available_room = separation_next - preferred_buffer;
    	max_acc = std::min(max_acc, available_room);
    }
    
    return max_acc;
}
