#include "double_pose_sensormanager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "msf_pose_pressure_sensor");

  //msf_pose_pressure_sensor::PosePressureSensorManager manager;

  ros::spin();

  return 0;
}
