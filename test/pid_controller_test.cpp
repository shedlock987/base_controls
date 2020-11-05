#include <ros/ros.h>
#include "pid_controller/pid_controller.h"
using namespace controls;

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}