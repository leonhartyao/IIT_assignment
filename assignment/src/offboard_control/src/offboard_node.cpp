#include "offboard_control/offboard_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_control_node");
  auto offboard_control = std::make_unique<OffboardControl>();

  offboard_control->offboardControl();

  return 0;
}