//sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
#ifndef TASKSERVER_H
#define TASKSERVER_H

#include <arm_control.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <thread>
#include <std_srvs/Empty.h>
#include <voice_control/Motion.h>
#include <mutex>

#include <deque>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <voice_control/TaskServerAction.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/Stop.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pan_tilt_msgs/PanTiltCmd.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef std::map<std::string, std::vector<double>> Map;
typedef std::map<std::string, std::vector<std::string>> MapTask;
typedef std::map<std::string, std::vector<std::string>>::iterator ItTask;
typedef std::map<std::string, std::vector<double>>::iterator ItValue;
typedef actionlib::ActionServer<voice_control::TaskServerAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::ServerGoalHandle<voice_control::TaskServerAction> GoalHandle;

class TaskServer
{
public:

  ros::NodeHandle nh_;
  Server as_;
  IQR::ArmControl *arm_control_;
  GoalHandle goal_handle_;
  MoveBaseClient move_base_client_;
  
  std::mutex mu_;

  std::string action_name_;

  voice_control::TaskServerFeedback feedback_;
  voice_control::TaskServerResult result_;

  double body_height, height_goal_, gripper_position_, object_distance_, pan_tilt_yaw_, pan_tilt_pitch_;
  int get_new_target, obstacle_id_, object_id_, step, box_count_;
  bool find_table_ = true, find_object_, task_active_ = false, get_goal, away_cabine_, marker_nav, get_object_distanse_ = false,
  base_stop_ = false, get_table_ = true;
  std::map<std::string, std::vector<double>> mm;

  ros::ServiceServer motion_server_;
  ros::ServiceClient stop_client_, start_client_, clear_costmaps_client_, start_force_client_, stop_force_client_;
  ros::Publisher body_pub, head_pub, planning_scene_diff_publisher, base_cmd_pub_;
  ros::Subscriber joint_states_sub, marker_pose_sub, table_pose_sub;
  
  // pan_tilt_msgs::PanTiltCmd pan_tilt_msgs;
  aruco_msgs::MarkerArray marker_array_msg_;

  std::deque<std::string> task_queue_;
  std::vector<std::string> motion_info_;
  std::vector<double> arm_standby_pose_, arm_box_position_, arm_box_size_, motion_value_, arm_out_pose_;
  std::vector<std::vector<double>> arm_box_pose_, pose_vector_, orientation_vector_;

  geometry_msgs::Pose standby_position_, pre_pick_position_, pick_position_, place_position_, table_pose_, object_pose_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;

  TaskServer(IQR::ArmControl *ac);
  ~TaskServer();

	void MarkerPoseCallback(const aruco_msgs::MarkerArrayConstPtr &msg);
	void TablePoseCallback(const visualization_msgs::MarkerConstPtr &msg);
	void ExcuteThread(GoalHandle gh);
	void ExecuteCb(GoalHandle gh);
	void preemptCB(GoalHandle gh);
	void GetParam();
	geometry_msgs::Pose GetGoalPose(std::string goal_name);
	void PickAndPlace(GoalHandle gh, std::vector<std::string> task, actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action);
	void SetArmJointPose(std::vector<double>& joint_value);
	void SetArmTargetPose(geometry_msgs::PoseStamped target);
	bool SetHeight(double height);
	void SetGripper(double box_size);
	bool ArmMoveAndCheck(moveit::planning_interface::MoveGroupInterface& move_group, actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action, GoalHandle gh);
	bool MoveToGoal(geometry_msgs::Pose pose, GoalHandle gh);
	void UpdateTable(); 
	void UpdateObject(double box_size, int id);
	void DeleltCollsion(std::vector<std::string> object_ids);
	void JointStatesCB(const sensor_msgs::JointState::ConstPtr& msg);
  void BaseControlCmd(float x, float z);
  bool BodyStop();
  bool PanTiltStop();
  bool SetHead(float yaw, float pitch, int speed);

private:
  std::string task_now_;
  MapTask *map_task_;
  Map *map_;
  ItTask it_map_task_, it_map_task_cancel_;
  ItValue it_map_;
  kinova_msgs::Stop stop_msg_;
  kinova_msgs::Start start_msg_;
  std::deque<GoalHandle> gh_queue_;
  void TaskManager(GoalHandle gh);
};
#endif //ARMCONTROL_H