/**********************************************************************************
** MIT License
** 
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** 
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/
#ifndef ARMCONTROL_H
#define ARMCONTROL_H

#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/JointAngles.h>

#include <kinova_msgs/Stop.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/PoseVelocityWithFingers.h>
#include <kinova_msgs/JointTorque.h>
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/KinovaPose.h>
#include <kinova_msgs/SetForceControlParams.h>
#include <kinova_msgs/SetEndEffectorOffset.h>
#include <kinova_msgs/SetNullSpaceModeState.h>
#include <kinova_msgs/SetTorqueControlMode.h>
#include <kinova_msgs/SetTorqueControlParameters.h>
#include <kinova_msgs/ClearTrajectories.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/ZeroTorques.h>
#include <kinova_msgs/RunCOMParametersEstimation.h>
#include <kinova_msgs/CartesianForce.h>


// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> FingerClient;
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> PoseClient;
typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> JointClient;

namespace IQR
{
class ArmControl
{

public:
  ArmControl();
  ~ArmControl();
  ros::NodeHandle nh_;
  FingerClient finger_client_;
  PoseClient pose_client_;
  JointClient joint_client_;
  ros::Publisher cartesian_vel_pub_, cartesian_vel_with_finger_pub_, cartesian_force_pub_, joint_vel_pub_, joint_torques_pub_;
  ros::Subscriber finger_position_sub_, joint_angles_sub_, joint_commond_sub_, joint_status_sub_, joint_torques_sub_, tool_pose_sub_, tool_wrench_sub_;
  ros::ServiceClient home_arm_client_, stop_client_, start_client_, stop_force_client_, start_force_client_;
  bool finger_stop_ = false, vel_pub_ = false;
  kinova_msgs::JointAngles joint_angles_;
  void JointAnglesCB(kinova_msgs::JointAngles msg);

  void FingerPositionCmd(float finger1, float finger2, float finger3);
  void CartesianPoseCmd(std::string frame_id, float position_x, float position_y, float position_z, float orientation_x, float orientation_y, float orientation_z, float orientation_w);
  void JointPoseCmd(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float joint7);
  void FingerPercentCmd(float finger);
  void CartesianVelocityCmd(float twist_linear_x, float twist_linear_y, float twist_linear_z, float twist_angular_x, float twist_angular_y, float twist_angular_z);
  void JointVelocityCmd(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float joint7);
  void CartesianForceCmd(float force_x, float force_y, float force_z, float torque_x, float torque_y, float torque_z);
  void JointTorqueCmd(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float joint7);
  void HomeArm();
  bool ArmInit();
protected:

private:

};
} // namespace IQR

#endif //ARMCONTROL_H