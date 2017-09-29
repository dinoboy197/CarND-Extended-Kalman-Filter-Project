#include <iostream>
#include "kalman_filter.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;

  UpdateInternal(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float rho_dot = 0;
  if (fabs(rho) >= 0.0001) {
    // only divide when not by zero
    rho_dot = (px * vx + py * vy) / rho;
  }
  float phi_raw = atan2(py, px);

  VectorXd h_prime_x(3);
  h_prime_x << rho, atan2(py, px), rho_dot;

  VectorXd y = z - h_prime_x;

  // ensure that phi is between -pi and pi
  while (y[1] > M_PI) {
    y[1] = y[1] - 2 * M_PI;
  }
  while (y[1] < -M_PI) {
    y[1] = y[1] + 2 * M_PI;
  }
  UpdateInternal(y);
}

void KalmanFilter::UpdateInternal(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht + R_;
  MatrixXd K_ = P_ * Ht * S_.inverse();
  x_ = x_ + (K_ * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K_ * H_) * P_;
}
