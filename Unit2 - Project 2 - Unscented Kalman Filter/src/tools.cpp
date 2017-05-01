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
            
        }
    }
    else {
        std::cout << "Both estimation and ground truth vectors have different sizes" << std::endl;
    }
    //return the result -- all zeros if problem.
    return rmse;
}
