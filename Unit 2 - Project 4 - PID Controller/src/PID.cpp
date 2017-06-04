#include "PID.h"

//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

//Initialize PID coefficients
Kp = Kp_;
Ki = Ki_;
Kd = Kd_;

i_error = 0;
p_error = 0;
d_error = 0;

error_sum = 0;
max_steps = 1000;
steps = 0;

best_error = 10000;

best_Kp = Kp;
best_Ki = Ki;
best_Kd = Kd;

//Initialize DP coefficients
dKp = 0.0001;
dKi = 0.00001;
dKd = 0.001;

}

void PID::UpdateError(double cte) {

d_error = cte - p_error;
p_error = cte;
i_error = i_error + cte;
}

double PID::TotalError() {

double total_error = -Kp*p_error - Kd*d_error - Ki*i_error;

return total_error; //return steering value for example
}

void PID::TwiddleUpdate(double tolerance, uWS::WebSocket<uWS::SERVER> ws) {

std::cout << "Twiddle update" << std::endl;

//Kp -= dKp; //increase first parameter
//Ki += dKi; //increase first parameter
//Kd -= dKd; //increase first parameter

Restart(ws);
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  Init(Kp, Ki, Kd);    
}
