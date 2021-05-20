#include "task_server.h"
#include "arm_control.h"

// using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Task Server");
  IQR::ArmControl ac;
  TaskServer ts(&ac);
  // ros::AsyncSpinner spinner(3);
  // spinner.start();
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
