// Drift estimator module that handles gate detections and VIO readings and
// estimates VIO drift using a Kalman filter.
DriftEstimator::DriftEstimator(const std::string &camera_file,
                               const std::string &track_file) {
  // load camera parameters based on camera calibration file (every camera is
  // calibrated individually for better results). We use an equidistant
  // distortion model identified using Kalibr.
  camera_ = std::make_shared<Camera>(camera_file);

  // Load file specifying the race track layout.
  loadTrack(track_file);
}

void DriftEstimator::detectionCallback(
    const racesim_msgs::GateDetection &det_msg,
    std::vector<geometry_msgs::PoseWithCovarianceStamped> *const gate_meas) {
  if (!vio_init_ || vio_queue_.empty()) {
    return;
  }
  // Predict drift of the VIO pipeline at the time of the gate detection
  kalman_filter_.predict(det_msg.header.stamp.toSec());

  // perform PnP to transform image plane detections into relative pose
  // measurements
  for (auto detection : det_msg.gates) {
    if (processSingleGate(detection, closest_vio_state, &R, &R_cam,
                          &p_gate_world, &R_quat, best_gate_match)) {
      // accumulate pose_measurements
    }
  }

  kalman_filter_.update(pose_measurements, R_vec, det_msg.header.stamp.toSec());
}

void DriftEstimator::vioOdomCallback(const agi::QuadState &vio_state) {
  if (!vio_state.valid()) {
    return;
  }

  if (!vio_init_) {
    // We use the first few seconds of data to compute the relative yaw between
    // the VIO frame and the world frame.

    // accumulate data, average & compute relative yaw
  } else {
    // we transform the VIO state to the world frame (asssumed to be rotated by
    // a constant yaw angle) before adding it to the queue
    agi::QuadState vio_state_trans = vio_state;
    Quaternion q_yaw = Quaternion(std::cos(yaw_angle_rad_ / 2.0), 0.0, 0.0,
                                  std::sin(yaw_angle_rad_ / 2.0));
    vio_state_trans.p = q_yaw.inverse() * vio_state.p;
    Vector<3> camera_x = Vector<3>::UnitX();
    vio_state_trans.q(q_yaw.inverse() * vio_state.q());

    vio_queue_.push(vio_state_trans);
  }
}

bool DriftEstimator::getVioEstimate(
    const Scalar &t, agi::QuadState *vio_state,
    geometry_msgs::PoseWithCovarianceStamped *const pose_with_cov) {
  // VIO state in camera frame
  *vio_state = vio_queue_.back();

  // Rotation from camera frame to drone body frame
  Quaternion q_B_C = Quaternion(
      Eigen::AngleAxis<Scalar>(pitch_angle_rad_, Vector<3>::UnitY()));

  // Rotation from drone body frame to world frame
  Quaternion q_I_B = vio_state->q() * q_B_C.inverse();

  // correct raw VIO estimate with Kalman filter state.
  vio_state->p += kalman_filter_.getState(t).segment<3>(0);
  vio_state->v += q_I_B.inverse() * kalman_filter_.getState(t).segment<3>(3);
  return true;
}
}

bool DriftEstimator::processSingleGate(const racesim_msgs::Gate &gate,
                                       const agi::QuadState &vio_state,
                                       Matrix<3, 3> *const R,
                                       Matrix<3, 3> *const R_cam,
                                       Vector<3> *const p_gate_world,
                                       Vector<4> *const R_quat,
                                       int &best_gate_match) {
  std::vector<cv::Point3_<Scalar>> obj_pts{
      cv::Point3_<Scalar>(-gate_size_ / 2.0, gate_size_ / 2.0, 0.0),
      cv::Point3_<Scalar>(gate_size_ / 2.0, gate_size_ / 2.0, 0.0),
      cv::Point3_<Scalar>(gate_size_ / 2.0, -gate_size_ / 2.0, 0.0),
      cv::Point3_<Scalar>(-gate_size_ / 2.0, -gate_size_ / 2.0, 0.0)};

  bool all_corners_detected =
      std::isfinite(gate.bl.x) && std::isfinite(gate.br.x) &&
      std::isfinite(gate.tr.x) && std::isfinite(gate.tl.x);
  if (!all_corners_detected) {
    return false;
  }

  std::vector<cv::Point_<Scalar>> img_pts{
      cv::Point_<Scalar>(gate.bl.x, gate.bl.y),
      cv::Point_<Scalar>(gate.br.x, gate.br.y),
      cv::Point_<Scalar>(gate.tr.x, gate.tr.y),
      cv::Point_<Scalar>(gate.tl.x, gate.tl.y)};

  std::vector<cv::Point_<Scalar>> img_pts_rect;
  // undistort detected gate corners
  camera_->undistortEquidist(img_pts, &img_pts_rect);

  // compute relative pose from gate detection
  cv::Mat distCoeffs(4, 1, cv::DataType<Scalar>::type);
  distCoeffs.at<Scalar>(0) = 0;
  distCoeffs.at<Scalar>(1) = 0;
  distCoeffs.at<Scalar>(2) = 0;
  distCoeffs.at<Scalar>(3) = 0;

  cv::Mat rvec(3, 1, cv::DataType<Scalar>::type);
  cv::Mat tvec(3, 1, cv::DataType<Scalar>::type);

  cv::solvePnP(obj_pts, img_pts_rect, camera_->getKMatrix(), distCoeffs, rvec,
               tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

  // compute reprojection error and check if below threshold
  std::vector<cv::Point_<Scalar>> proj_pts_rect;
  std::vector<cv::Point_<Scalar>> proj_pts;
  cv::projectPoints(obj_pts, rvec, tvec, camera_->getKMatrix(), distCoeffs,
                    proj_pts_rect);
  camera_->distortEquidist(proj_pts_rect, &proj_pts);

  const Vector<2> gate_dim1 = Vector<2>(img_pts.at(0).x - img_pts.at(2).x,
                                        img_pts.at(0).y - img_pts.at(2).y);
  const Vector<2> gate_dim2 = Vector<2>(img_pts.at(1).x - img_pts.at(3).x,
                                        img_pts.at(1).y - img_pts.at(3).y);
  const Scalar max_dim = std::max(gate_dim1.norm(), gate_dim2.norm());

  Scalar mean_repr_err = computeReprError(img_pts, proj_pts) / max_dim;
  if (mean_repr_err > max_mean_repr_error_) {
    if (debug_) {
      std::cout << "Mean repr error too large: " << mean_repr_err << " vs "
                << max_mean_repr_error_ << std::endl;
    }
    return false;
  }

  // recompute relative pose estimate on a set of sampled gate detection
  // perturbations to compute measurement covariance
  for (int i_sample = 0; i_sample < kf_samples_; i_sample++) {
    // perturb points
    // undistort
    // cv::solvePnP(...);
    // append relative pose estimate to vector
  }
  // get covariance of the position based on the randomly perturbed
  // gate detections. This monte-carlo sampling allows us to estimate
  // the measurement uncertainty.
  return true;
}

Scalar DriftEstimator::computeReprError(
  const std::vector<cv::Point_<Scalar>> &det_points,
  const std::vector<cv::Point_<Scalar>> &repr_points) {
  if (det_points.size() != repr_points.size()) {
    return 100.0;
  }

  Scalar repr_error = 0.0;
  for (int i = 0; i < (int)det_points.size(); i++) {
    repr_error +=
      (cvToEigen(det_points.at(i)) - cvToEigen(repr_points.at(i))).norm();
  }
  repr_error /= (Scalar)det_points.size();

  return repr_error;
}


bool DriftEstimator::loadTrack(const std::string &track_filename) {
  std::cout << "Loading track from [" << track_filename << "]." << std::endl;
  // loading track from file

  return true;
}
