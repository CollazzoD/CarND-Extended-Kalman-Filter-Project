#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

/* CalculateRMSE is a function seen during the course*/
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // compute the Jacobian matrix
  float drange_dpx, drange_dpy, drange_dvx, drange_dvy;
  float dbearing_dpx, dbearing_dpy, dbearing_dvx, dbearing_dvy;
  float drangerate_dpx, drangerate_dpy, drangerate_dvx, drangerate_dvy;
  
  float px2_py2 = (px * px) + (py * py);
  float sqrt_px2_py2 = sqrt(px2_py2);
  float px2_py2_3_2 = px2_py2 * sqrt_px2_py2;
  
  // check division by zero
  if (fabs(px2_py2) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  drange_dpx = px / sqrt_px2_py2;
  drange_dpy = py / sqrt_px2_py2;
  drange_dvx = 0;
  drange_dvy = 0;

  dbearing_dpx = - py / px2_py2;
  dbearing_dpy = px / px2_py2;
  dbearing_dvx = 0;
  dbearing_dvy = 0;

  drangerate_dpx = py * (vx * py - vy * px) / px2_py2_3_2;
  drangerate_dpy = px * (vy * px - vx * py) / px2_py2_3_2;
  drangerate_dvx = px / sqrt_px2_py2;
  drangerate_dvy = py / sqrt_px2_py2;

  Hj << drange_dpx, drange_dpy, drange_dvx, drange_dvy,
        dbearing_dpx, dbearing_dpy, dbearing_dvx, dbearing_dvy,
        drangerate_dpx, drangerate_dpy, drangerate_dvx, drangerate_dvy;

  return Hj;
}
