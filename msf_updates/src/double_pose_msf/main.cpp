#include "double_pose_sensormanager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "double_pose_sensor");

  msf_double_pose_sensor::DoublePoseSensorManager manager;

  ros::spin();

  return 0;
}
