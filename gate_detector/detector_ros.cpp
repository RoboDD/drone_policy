// ROS-wrapper for gate detector module.

DetectorROS::DetectorROS(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh) {
  // gate_pub = gate detection publisher
  // img_sub = realsense image subscriber
  // odom_sub = realsense ododmetry subscripter
}

void DetectorROS::imgCallback(const sensor_msgs::ImageConstPtr &msg) {
  // Get input images and orientate them according to realsense mounting
  // to make gate detection easier, rotate images by 90 degrees at high roll
  // angles.
  if (roll_angle < (-M_PI / 4.0)) {
    rotate_code = 1;
  } else if (roll_angle > (M_PI / 4.0)) {
    rotate_code = 2;
  }
  // rotate images
  // push images back to queue
  input_images.push_back(resized_img);
  detector_module_->detectGate(input_images, rotate_code, &detected_gates_v,
                               &heatmap_viz, &paf_viz, &detection_viz);

  racesim_msgs::GateDetection gate_detection = racesim_msgs::GateDetection();
  detection_pub_.publish(gate_detection);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "detector_node");

  DetectorROS detector_ros;
  ros::spin();

  return 0;
}
