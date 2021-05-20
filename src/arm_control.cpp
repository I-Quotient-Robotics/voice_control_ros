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
#include "arm_control.h"

IQR::ArmControl::ArmControl() :
  finger_client_("j2n6s300_driver/fingers_action/finger_positions", true),
  pose_client_("j2n6s300_driver/pose_action/tool_pose", true),
  joint_client_("j2n6s300_driver/joints_action/joint_angles", true)

{
  //init params

  joint_angles_sub_ = nh_.subscribe<kinova_msgs::JointAngles>("j2n6s300_driver/out/joint_angles", 1000, &ArmControl::JointAnglesCB, this);

  cartesian_vel_pub_ = nh_.advertise<kinova_msgs::PoseVelocity>("j2n6s300_driver/in/cartesian_velocity",1000);
  cartesian_vel_with_finger_pub_ = nh_.advertise<kinova_msgs::PoseVelocityWithFingers>("j2n6s300_driver/in/cartesian_velocity_with_fingers",1000);
  cartesian_force_pub_ = nh_.advertise<kinova_msgs::CartesianForce>("/j2n6s300_driver/in/cartesian_force",1000);
  joint_vel_pub_ = nh_.advertise<kinova_msgs::JointVelocity>("/j2n6s300_driver/in/joint_velocity",1000);
  joint_torques_pub_ = nh_.advertise<kinova_msgs::JointTorque>("/j2n6s300_driver/in/joint_torque",1000);

  stop_client_ = nh_.serviceClient<kinova_msgs::Stop>("/j2n6s300_driver/in/stop");
  start_client_ = nh_.serviceClient<kinova_msgs::Start>("/j2n6s300_driver/in/start");
  stop_force_client_ = nh_.serviceClient<kinova_msgs::Stop>("/j2n6s300_driver/in/stop_force_control");
  start_force_client_ = nh_.serviceClient<kinova_msgs::Start>("/j2n6s300_driver/in/start_force_control");
  home_arm_client_ = nh_.serviceClient<kinova_msgs::HomeArm>("/j2n6s300_driver/in/home_arm");
}

IQR::ArmControl::~ArmControl()
{

}

void IQR::ArmControl::JointAnglesCB(kinova_msgs::JointAngles msg) {
  joint_angles_ = msg; 
}

void IQR::ArmControl::FingerPositionCmd(float finger1, float finger2, float finger3) {
  kinova_msgs::SetFingersPositionGoal finger_position;
  finger_position.fingers.finger1 = finger1; 
  finger_position.fingers.finger2 = finger2; 
  finger_position.fingers.finger3 = finger3;
  ROS_INFO("Using fingers"); 
  finger_client_.sendGoal(finger_position);
  finger_client_.waitForResult(ros::Duration(0.0));
  return;
}

void IQR::ArmControl::CartesianPoseCmd(std::string frame_id, float position_x, float position_y, float position_z, float orientation_x, float orientation_y, float orientation_z, float orientation_w) {
  kinova_msgs::ArmPoseGoal eff_pose;
  eff_pose.pose.header.frame_id = frame_id;
  eff_pose.pose.pose.position.x = position_x;
  eff_pose.pose.pose.position.y = position_y;
  eff_pose.pose.pose.position.z = position_z;
  eff_pose.pose.pose.orientation.x = orientation_x;
  eff_pose.pose.pose.orientation.y = orientation_y;
  eff_pose.pose.pose.orientation.z = orientation_z;
  eff_pose.pose.pose.orientation.w = orientation_w;
  pose_client_.sendGoal(eff_pose);
  pose_client_.waitForResult(ros::Duration(0.0));
}

void IQR::ArmControl::JointPoseCmd(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float joint7){
  kinova_msgs::ArmJointAnglesGoal joint_angles;
  joint_angles.angles.joint1 = joint1;
  joint_angles.angles.joint2 = joint2;
  joint_angles.angles.joint3 = joint3;
  joint_angles.angles.joint4 = joint4;
  joint_angles.angles.joint5 = joint5;
  joint_angles.angles.joint6 = joint6;
  joint_angles.angles.joint7 = joint7;
  joint_client_.sendGoal(joint_angles);
  joint_client_.waitForResult(ros::Duration(0.0));
}

void IQR::ArmControl::FingerPercentCmd(float finger) {
  kinova_msgs::PoseVelocityWithFingers PoseVelocity;
  PoseVelocity.twist_linear_x = 0.0;
  PoseVelocity.twist_linear_y = 0.0;
  PoseVelocity.twist_linear_z = 0.0;
  PoseVelocity.twist_angular_x = 0.0;
  PoseVelocity.twist_angular_y = 0.0;
  PoseVelocity.twist_angular_z = 0.0;
  PoseVelocity.fingers_closure_percentage = finger;
  ros::Rate loop_rate(100);
  finger_stop_ = false;
  while(ros::ok() && !finger_stop_){
    cartesian_vel_with_finger_pub_.publish(PoseVelocity);
    loop_rate.sleep();
  }
}

void IQR::ArmControl::CartesianVelocityCmd(float twist_linear_x, float twist_linear_y, float twist_linear_z, float twist_angular_x, float twist_angular_y, float twist_angular_z) {
  ros::Rate loop_rate(100);
  ROS_INFO("Ready to move");
  kinova_msgs::PoseVelocity msg;
  msg.twist_linear_x = twist_linear_x;
  msg.twist_linear_y = twist_linear_y;
  msg.twist_linear_z = twist_linear_z;
  msg.twist_angular_x = twist_angular_x;
  msg.twist_angular_y = twist_angular_y;
  msg.twist_angular_z = twist_angular_z;
  vel_pub_ = false;
  while(ros::ok() && !vel_pub_){
    cartesian_vel_pub_.publish(msg);
    loop_rate.sleep();
  }
}

void IQR::ArmControl::JointVelocityCmd(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float joint7) {
  ros::Rate loop_rate(100);
  kinova_msgs::JointVelocity msg;
  msg.joint1 = joint1;
  msg.joint2 = joint2;
  msg.joint3 = joint3;
  msg.joint4 = joint4;
  msg.joint5 = joint5;
  msg.joint6 = joint6;
  msg.joint7 = joint7;
  vel_pub_ = false;
  while(ros::ok() && !vel_pub_){
    joint_vel_pub_.publish(msg);
    loop_rate.sleep();
  }
}

// Temporarily unavailable
void IQR::ArmControl::CartesianForceCmd(float force_x, float force_y, float force_z, float torque_x, float torque_y, float torque_z) {
// cartesian_force_pub_.publish();
}
// Temporarily unavailable
void IQR::ArmControl::JointTorqueCmd(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float joint7) {
  ros::Rate loop_rate(100);
  kinova_msgs::JointTorque msg;
  msg.joint1 = joint1;
  msg.joint2 = joint2;
  msg.joint3 = joint3;
  msg.joint4 = joint4;
  msg.joint5 = joint5;
  msg.joint6 = joint6;
  msg.joint7 = joint7;
  vel_pub_ = false;
  while(ros::ok() && !vel_pub_){
    joint_torques_pub_.publish(msg);
    loop_rate.sleep();
  }
}

void IQR::ArmControl::HomeArm() {
  // std_srvs::Empty empty_msg;
  kinova_msgs::HomeArm home_arm_msg;
  home_arm_client_.call(home_arm_msg);
}

bool IQR::ArmControl::ArmInit() {
  JointPoseCmd(joint_angles_.joint1, joint_angles_.joint2, joint_angles_.joint3, joint_angles_.joint4, joint_angles_.joint5, joint_angles_.joint6, joint_angles_.joint7);
}