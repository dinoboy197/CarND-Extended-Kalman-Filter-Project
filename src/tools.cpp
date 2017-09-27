#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // calculate root mean squared error
  VectorXd rsmes(4);
  rsmes << 0, 0, 0, 0;
  
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rsmes;
  }

  for(int i = 0; i < estimations.size(); i++) {
    VectorXd item = (estimations[i] - ground_truth[i]);
    VectorXd me = item.array()*item.array();
    rsmes << rsmes + me;
  }
  
  // mean
  rsmes = rsmes/estimations.size();

  // square root
  rsmes = rsmes.array().sqrt();
  
  return rsmes;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  MatrixXd jacobian(3,4);

  float px2py2 = pow(px, 2) + pow(py, 2);

  // prevent division by zero
  if(fabs(px2py2) < 0.0001){
    return jacobian;
  }
  
  float sqrtpx2py2 = sqrt(px2py2);
  float sqrt3px2py2= pow(sqrtpx2py2, 3);
  float px_by_sqrtpx2py2 = px / sqrtpx2py2;
  float py_by_sqrtpx2py2 = py / sqrtpx2py2;

  float a = px_by_sqrtpx2py2;
  float b = py_by_sqrtpx2py2;
  float c = 0;
  float d = 0;
  float e = -py / px2py2;
  float f = -px / px2py2;
  float g = 0;
  float h = 0;
  float i = py * (vx*py - vy*px) / sqrt3px2py2;
  float j = px * (vy*px - vx*py) / sqrt3px2py2;
  float k = px_by_sqrtpx2py2;
  float l = py_by_sqrtpx2py2;
  
  jacobian << a, b, c, d,
              e, f, g, h,
              i, j, k, l;
  
  return jacobian;
}
