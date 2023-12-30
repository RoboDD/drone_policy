// ROS-wrapper for the drift estimator
DriftEstimatorRos::DriftEstimatorRos(const std::string &camera_file,
                                     const std::string &track_file,
                                     const ros::NodeHandle &nh,
                                     const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    drift_estimator_module_(std::make_shared<drift_estimator::DriftEstimator>(
      camera_file, track_file)) {
      // subscriber for detections
      // subscriber for realsense odometry
      // subscriber for vicon odometry (records ground truth data)
      // publisher for drift-corrected odometry (whenever there is an input 
      //  odometry, this one publishes the corrected odometry)
      // logger to write all of this to disk
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "drift_estimator_node");
  drift_estimator::DriftEstimatorRos drift_estimator_ros{camera_file,
                                                         track_file};
  ros::spin();

  return 0;
}
