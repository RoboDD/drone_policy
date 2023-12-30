// Implements Kalman-Filter with constant acceleration model
KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd &x0) {
  x_ = x0;
  P_ = Matrix<>::Identity(state_size_, state_size_);
  t_ = t0;

  // constant velocity model for drift. time dependency is updated in every
  // update/predict call
  F_ = Matrix<state_size_, state_size_>::Identity();
  Q_ = Matrix<state_size_, state_size_>::Zero();
  I_ = Matrix<state_size_, state_size_>::Identity();

  q_acc_ = 50;

  initialized_ = true;
}

void KalmanFilter::predict(const Scalar &t) {
  const Scalar t_predict = std::min(0.5, std::max(0.0, t - t_));
  const Scalar t_predict2 = t_predict * t_predict;
  const Scalar t_predict3 = t_predict * t_predict2;
  const Scalar t_predict4 = t_predict2 * t_predict2;

  F_.block<3, 3>(0, 3) = Matrix<3, 3>::Identity() * t_predict;

  Q_.block<3, 3>(0, 0) = Matrix<3, 3>::Identity() * t_predict4 / 4 * q_acc_;
  Q_.block<3, 3>(3, 3) = Matrix<3, 3>::Identity() * t_predict2 * q_acc_;

  Q_.block<3, 3>(0, 3) = Matrix<3, 3>::Identity() * t_predict3 / 2 * q_acc_;
  Q_.block<3, 3>(3, 0) = Matrix<3, 3>::Identity() * t_predict3 / 2 * q_acc_;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  t_ = t;
}


void KalmanFilter::update(const std::vector<Vector<3>> &z_vec,
                          const std::vector<Matrix<3, 3>> &R_vec,
                          const Scalar &t) {
  if (!initialized_) throw std::runtime_error("Filter is not initialized!");

  if (z_vec.empty()) {
    return;
  }

  // assemble measurement
  Vector<> z(z_vec.size() * z_vec.front().size());
  for (int i = 0; i < (int)z_vec.size(); i++) {
    z.segment<3>(3 * i) = z_vec.at(i);
  }
  // assemble measurement matrix
  Matrix<3, state_size_> H_single = Matrix<3, state_size_>::Zero();
  H_single.block<3, 3>(0, 0) = Matrix<3, 3>::Identity();
  Matrix<> H(z_vec.size() * z_vec.front().size(), state_size_);
  Matrix<> R(z_vec.size() * z_vec.front().size(),
             z_vec.size() * z_vec.front().size());
  R.setZero();
  for (int i = 0; i < (int)z_vec.size(); i++) {
    H.block<3, state_size_>(3 * i, 0) = H_single;
    R.block<3, 3>(3 * i, 3 * i) = R_vec.at(i);
  }

  predict(t);

  // y = z - Hx
  // error (residual) between measurement and prediction
  Vector<> y = z - H * x_;

  Matrix<> PHT = P_ * H.transpose();

  // S = HPH' + R
  // project system uncertainty into measurement space
  Matrix<> S = H * PHT + R;
  Matrix<> S_inv = S.inverse();

  // K = PH'inv(S)
  // map system uncertainty into kalman gain
  Matrix<> K = PHT * S_inv;

  // x = x + Ky
  // predict new x with residual scaled by the kalman gain
  x_ = x_ + K * y;

  // P = (I-KH)P(I-KH)' + KRK'
  Matrix<> I_KH = I_ - K * H;
  P_ = (I_KH * P_) * I_KH.transpose() + (K * R) * K.transpose();
}

Vector<> KalmanFilter::getState(const Scalar &t) {
  const Scalar t_predict = std::max(0.0, t - t_);
  const Scalar t_predict2 = t_predict * t_predict;

  F_.block<3, 3>(0, 3) = Matrix<3, 3>::Identity() * t_predict;
  const Vector<> x = F_ * x_;
  return x;
}
