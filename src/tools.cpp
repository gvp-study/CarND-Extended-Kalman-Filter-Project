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
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

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

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
    
  //TODO: YOUR CODE HERE 
  float pxy = hypot(px, py);

  //check division by zero
  if(pxy < 0.0001)
  {
    cout << "px and py are zero" << endl;
    return Hj;
  }
  float pxy2 = pxy*pxy;
  float pxy3 = pxy*pxy*pxy;
  float vpxy = (vx*py - vy*px);
  float txy = py * vpxy;
  float tyx =-px * vpxy;
  //compute the Jacobian matrix
  Hj(0, 0) = px / pxy;   Hj(0, 1) = py / pxy;   Hj(0, 2) = 0.0;      Hj(0, 3) = 0.0;
  Hj(1, 0) =-py / pxy2;  Hj(1, 1) = px / pxy2;  Hj(1, 2) = 0.0;      Hj(1, 3) = 0.0;
  Hj(2, 0) = txy / pxy3; Hj(2, 1) = tyx / pxy3; Hj(2, 2) = px / pxy; Hj(2, 3) = py / pxy;

  return Hj;
}
