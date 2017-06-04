#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;


  // compute the sum of all errors to see which paramaters combo offers best results
  double error_sum;
  
  int max_steps;
  int steps;
  int best_error;
  double best_Kp;
  double best_Ki;
  double best_Kd;

  double dKp;
  double dKi;
  double dKd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  /*
  * Applied the Twiddle algo to optimize
  */

  void TwiddleUpdate(double tolerance, uWS::WebSocket<uWS::SERVER> ws);

  /*
  * Restart the simulator with new parameters as part to Twiddle
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);  
};

#endif /* PID_H */
