#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
        
  VectorXd rmse(4);
  rmse << 0,0,0,0;
        
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() != 0) {
     //  * the estimation vector size should equal ground truth vector size
     // ... your code here
    if (estimations.size() == ground_truth.size()) {
      //accumulate squared residuals
      for(unsigned int i=0; i < estimations.size(); ++i){
        
         VectorXd residual = estimations[i] - ground_truth[i];
                    
         //coefficient-wise multiplication
         residual = residual.array()*residual.array();
         rmse += residual;
         }
                
      //calculate the mean
      rmse = rmse/estimations.size();
                
      //calculate the squared root
      rmse = rmse.array().sqrt();
      
      //return the result
      return rmse;
      }
  }
  else {
            std::cout << "Both estimation and ground truth vectors have different sizes" << std::endl;
        }
}

bool Tools::CalculateJacobian(const VectorXd& x_state, MatrixXd& Hj) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  
  //MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  bool div_issue=false;
    
  float sum_sq = pow(px,2)+pow(py,2);
  float sqrt_sum = sqrt(sum_sq);
    
  //check division by zero
  if (sum_sq != 0) {
    Hj <<   px/sqrt_sum, py/sqrt_sum, 0, 0,
    -py/sum_sq, px/sum_sq, 0, 0,
    py*(vx*py-vy*px)/pow(sum_sq,1.5), px*(vy*px-vx*py)/pow(sum_sq,1.5), px/sqrt_sum, py/sqrt_sum;
  } else {

    std::cout << "\nDivision by zero error with the Jacobian matrix! Skipping the data update..." << std::endl;
    div_issue=true;
  }
  
  return div_issue;
  
}
