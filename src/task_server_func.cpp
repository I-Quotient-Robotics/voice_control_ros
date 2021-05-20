#include "task_server.h"
#include <arm_control.h>

bool move_goal_flag = true;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  ROS_INFO_STREAM("Move base done callback");
}

TaskServer::TaskServer(IQR::ArmControl *ac) :
  as_(nh_, "task_server", boost::bind(&TaskServer::ExecuteCb, this, _1), boost::bind(&TaskServer::preemptCB, this, _1), false),
  move_base_client_("move_base", true),
  arm_group_("arm"),
  gripper_group_("gripper")
{
  map_task_ = new MapTask();
  map_ = new Map();
  arm_control_ = ac;
  marker_pose_sub = nh_.subscribe<aruco_msgs::MarkerArray>("object_marker_publisher/markers", 1000, &TaskServer::MarkerPoseCallback, this);
  table_pose_sub = nh_.subscribe<visualization_msgs::Marker>("plane_marker_publisher/marker", 1000, &TaskServer::TablePoseCallback, this);
  // ros::Subscriber marker_pose_sub = nh.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, boost::bind(MarkerPoseCallback, _1, boost::ref(planning_scene_interface)));
  planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

  joint_states_sub = nh_.subscribe("joint_states", 1000, &TaskServer::JointStatesCB, this);
  body_pub = nh_.advertise<std_msgs::Float64>("caster/body_controller/command", 1000);
  head_pub = nh_.advertise<pan_tilt_msgs::PanTiltCmd>("pan_tilt_driver_node/pan_tilt_cmd", 1000);
  stop_force_client_ = nh_.serviceClient<kinova_msgs::Stop>("/j2n6s300_driver/in/stop_force_control");
  start_force_client_ = nh_.serviceClient<kinova_msgs::Start>("/j2n6s300_driver/in/start_force_control");

  arm_group_.setPlanningTime(45.0);

  GetParam();
  as_.start();
}

TaskServer::~TaskServer()
{
}

void TaskServer::JointStatesCB(const sensor_msgs::JointState::ConstPtr& msg) {
  for(uint8_t i=0; i<msg->name.size(); i++) {
    if(msg->name[i] == "caster_body_connected_joint") {
      body_height = msg->position[i];
      // ROS_INFO("Get height %lf", body_height);
      break;
    }
    if(msg->name[i] == "pan_tilt_yaw_joint") {
      pan_tilt_yaw_ = msg->position[i];
      pan_tilt_pitch_ = msg->position[i+1];
      // ROS_INFO("Get pan tilt yaw %lf pitch %lf", pan_tilt_yaw_, pan_tilt_pitch_);
      break;
    }
  }
}

void TaskServer::MarkerPoseCallback(const aruco_msgs::MarkerArrayConstPtr &msg) {

  if (!get_object_distanse_)
  {
    object_distance_ = msg->markers[0].pose.pose.position.z;
    ROS_INFO("%f", object_distance_);
    get_object_distanse_ = true;
  }
}

void TaskServer::TablePoseCallback(const visualization_msgs::MarkerConstPtr &msg) {
  // marker_array_msg_ = *msg;

  // geometry_msgs::TransformStamped marker_transform, object_transform;
  if (!find_table_)
  {
    table_pose_ = msg->pose;
    find_table_ = true;
  }
  if ((msg->pose.position.z) <= 1.05 and !get_goal)
  {
    get_goal = true;
  }
  if ((msg->pose.position.z) <= 1.0 and !get_table_)
  {
    get_table_ = true;
  }

}

void TaskServer::preemptCB(GoalHandle gh)
{  
  if (task_now_ == "PICKPLACE")
  {
    if (move_goal_flag) 
    {
      move_base_client_.cancelAllGoals();
    }
    stop_client_.call(stop_msg_);
    start_client_.call(start_msg_);
  } 
  else if (task_now_ == "BASEMOVE") {
    base_stop_ = true;
  }
  else if (task_now_ == "ARMPOSE") {
    stop_client_.call(stop_msg_);
    start_client_.call(start_msg_);    
  }
  else if (task_now_ == "ARMEFFPOSE") {
    arm_control_->pose_client_.cancelAllGoals();
  }
  else if (task_now_ == "ARMCARVEL") {
    arm_control_->vel_pub_ = true;
  }
  else if (task_now_ == "ARMJOINTVEL") {
    arm_control_->vel_pub_ = true;
  }
  else if (task_now_ == "ARMJOINTPOSITION") {
    arm_control_->joint_client_.cancelAllGoals();
  }
  else if (task_now_ == "FINGERPOSITION") {
    arm_control_->finger_client_.cancelAllGoals();
  }
  else if (task_now_ == "FINGERPERCENT") {
    arm_control_->finger_stop_ = true;
  }
  else if (task_now_ == "BASENAVGOAL") {
    move_base_client_.cancelAllGoals();
  }
  gh.setCanceled();
  // task_active_ = false;
}

void TaskServer::ExecuteCb(GoalHandle gh)
{
  ROS_INFO("``````````````````` new task in `````````````````````");

  if (task_active_)
  {
    gh.setRejected();
    return;
  }
  gh.setAccepted();
  task_active_ = true;
  std::thread thread(&TaskServer::TaskManager, this, gh);
  thread.detach();
}



void TaskServer::TaskManager(GoalHandle gh) {

  actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action = arm_group_.getMoveGroupClient(); 
  actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &gripper_group_action = gripper_group_.getMoveGroupClient();

  std::vector<std::string> task = gh.getGoal()->task;
  task_now_ = task[0];
  std::vector<double> value = gh.getGoal()->value;
  ros::Rate loop_rate(1);

  if (task[0] == "PICKPLACE") {
    ROS_INFO("PICKPLACE");
    PickAndPlace(gh, task, arm_group_action);
    task_active_ = false;
  }
  else if(task[0] == "NAVGOAL") {
  //navigation to goal
  }
  else if(task[0] == "BASEMOVE") {
    BaseControlCmd(value[0], value[1]);
    task_active_ = false;
  }
  else if(task[0] == "ARMPOSE") {
    ROS_INFO("home arm");
    if(task[1] == "home") {
      SetArmJointPose(arm_standby_pose_);
    } else if (task[1] == "out")
    {
      SetArmJointPose(arm_out_pose_);
    }
    if(ArmMoveAndCheck(arm_group_, arm_group_action, gh)) {
      gh.setSucceeded();
    }
    // arm_control_->HomeArm();
    task_active_ = false;
  }
  else if(task[0] == "ARMAPI") {
    if (task[1] == "stop")
    {
      stop_client_.call(stop_msg_);
    } else {
      start_client_.call(start_msg_);
    }    
    task_active_ = false;
  }
  else if(task[0] == "ARMFORCECONTROL") {
    if (task[1] == "stop") {
      stop_force_client_.call(stop_msg_);
    } else {
      start_force_client_.call(start_msg_);
    }  
    task_active_ = false;
  }
  else if(task[0] == "ARMEFFPOSE") {
    arm_control_->CartesianPoseCmd(task[1], value[0], value[1], value[2], value[3], value[4], value[5], value[6]);
    task_active_ = false;
  }
  else if(task[0] == "ARMCARVEL") {
    arm_control_->CartesianVelocityCmd(value[0], value[1], value[2], value[3], value[4], value[5]);
    task_active_ = false;
  }
  else if(task[0] == "ARMJOINTVEL") {
    arm_control_->JointVelocityCmd(value[0], value[1], value[2], value[3], value[4], value[5], value[6]);
    task_active_ = false;
  }
  else if(task[0] == "ARMJOINTPOSITION") {
    arm_control_->JointPoseCmd(value[0], value[1], value[2], value[3], value[4], value[5], value[6]);
    task_active_ = false;
  }
  else if(task[0] == "ARMREADYPOSE") {
    
  }
  else if(task[0] == "ARMCARFORCE") {
  }
  else if(task[0] == "ARMJOINTFORCE") {
  }
  else if(task[0] == "ARMJOINTTORQUE") {
  } 
  else if(task[0] == "FINGERPOSITION") {
    arm_control_->FingerPositionCmd(value[0], value[1], value[2]);
    task_active_ = false;
  } 
  else if(task[0] == "FINGERPERCENT") {
    arm_control_->FingerPercentCmd(value[0]);
    task_active_ = false;
  } 
  else if(task[0] == "BASENAVGOAL") {
    // if (nh_.hasParam(task[1]))
    // {
    geometry_msgs::Pose pose = GetGoalPose(task[1]);
    MoveToGoal(pose, gh);
    // }
    // else {
      // ROS_INFO("Parameter not found");
    // }
    task_active_ = false;
  }
  else {
    ROS_INFO("No such command");
    task_active_ = false;
  }
}

void TaskServer::GetParam() {
  // get arm standby pose
  arm_standby_pose_.resize(6);
  nh_.getParam("arm_pose/standby", arm_standby_pose_);
  arm_out_pose_.resize(6);
  nh_.getParam("arm_pose/out", arm_out_pose_);
}

geometry_msgs::Pose TaskServer::GetGoalPose(std::string goal_name) {

  std::vector<float> pose_vector(3), orientation_vector(4);
  nh_.getParam("target_goal/"+goal_name+"/pose", pose_vector);
  nh_.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

  geometry_msgs::Pose pose;
  pose.position.x = pose_vector[0];
  pose.position.y = pose_vector[1];
  pose.position.z = pose_vector[2];
  pose.orientation.x = orientation_vector[0];
  pose.orientation.y = orientation_vector[1];
  pose.orientation.z = orientation_vector[2];
  pose.orientation.w = orientation_vector[3];

  return pose;
}

void TaskServer::BaseControlCmd(float x, float z) {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = x;
  cmd_vel.angular.z = z;
  base_stop_ = false;
  while(ros::ok() && !base_stop_) {
    base_cmd_pub_.publish(cmd_vel);
  }
  // return;
}

//Set joint angle target,topic of "/joint_states"
void TaskServer::SetArmJointPose(std::vector<double>& joint_value) {
  arm_group_.setJointValueTarget(joint_value);
}

//Set pose target,send pose msg to Moveit
void TaskServer::SetArmTargetPose(geometry_msgs::PoseStamped target) {
  arm_group_.setPoseTarget(target);

}

//Set body height
bool TaskServer::SetHeight(double height) {
  if(height < 0.0) {
    height = 0;
  } else if (height > 0.4) {
    height = 0.4;
  }

  ROS_INFO("%lf, %lf", height, body_height);

  std_msgs::Float64 msg;
  msg.data = height;
  body_pub.publish(msg);

  while(abs(body_height-height) > 0.01) {
    ros::WallDuration(0.2).sleep();
  }
}

bool TaskServer::SetHead(float yaw, float pitch, int speed) {

  pan_tilt_msgs::PanTiltCmd pan_tilt_msgs;
  // float pan_tilt_yaw_goal = pan_tilt_yaw_;
  // float pan_tilt_pitch_goal = pan_tilt_pitch_;
  pan_tilt_msgs.pitch = pitch;
  pan_tilt_msgs.yaw = yaw;
  pan_tilt_msgs.speed = speed;  
  head_pub.publish(pan_tilt_msgs);

  while(abs(pan_tilt_yaw_-yaw) > 0.01 and abs(pan_tilt_pitch_-pitch) > 0.01) {
    ros::WallDuration(0.2).sleep();
  }
}

bool TaskServer::BodyStop() {
  SetHeight(body_height);
}



bool TaskServer::PanTiltStop() {
  SetHead(pan_tilt_yaw_, pan_tilt_pitch_, 30);
}

// Using Moveit control grippers 
void TaskServer::SetGripper(double box_size) {
  std::vector<double> gripper_pose(2);

  gripper_pose[0] = box_size*0.2;
  gripper_pose[1] = box_size*0.2;
  // gripper_pose[2] = value;

  gripper_group_.setJointValueTarget(gripper_pose);
  // move_group.move();
}

// Using Moveit move arm and check execution
bool TaskServer::ArmMoveAndCheck(moveit::planning_interface::MoveGroupInterface& move_group, actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action, GoalHandle gh) {
  arm_group_.move();
  if (arm_group_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("SUCCEEDED");
      return true;
  }
  else {
      if (gh.getGoalStatus().status == 1)
      {
        gh.setAborted();
      }
      // task_active_ = false;
      return false;
      // ROS_INFO("Cancel Goal!");
      // arm_group_action.cancelAllGoals();
  }
}

// void SetArmNamePose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double>& joint_value) {
//   move_group.setJointValueTarget(joint_value);
//   move_group.move();
// }

// Using move_base control caster for navigation
bool TaskServer::MoveToGoal(geometry_msgs::Pose pose, GoalHandle gh) {
  ROS_INFO("%lf", pose.position.x);
  // std_srvs::Empty clear_task;
  // clear_costmaps_client_.call(clear_task);
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = "map";
  mb_goal.target_pose.pose = pose;

  move_goal_flag = false;
  move_base_client_.sendGoal(mb_goal,
            boost::bind(&MovebaseDoneCallback, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&MovebaseFeedbackCallback, _1));

  while(move_goal_flag==false) {
    ros::WallDuration(1.0).sleep();
    ROS_INFO_STREAM("moving...");
  }

  if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO_STREAM("get goal");
      // ROS_INFO("SUCCEEDED");
      return true;
  }
  else {
      ROS_INFO("Cancel Goal!");
      // move_base_client_.cancelAllGoals();
      if (gh.getGoalStatus().status == 1)
      {
        gh.setAborted();
      }
      task_active_ = false;
      return false;
  }
}

// Add table collision
void TaskServer::UpdateTable() {
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0].header.frame_id = "table2";
  collision_objects[0].id = "table_collision";

  // define the primitive and its dimensions. 
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.1;
  collision_objects[0].primitives[0].dimensions[1] = 2.0;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  // define the pose of the object
  collision_objects[0].primitive_poses.resize(1);
  tf2::Quaternion q;
  q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);

  collision_objects[0].primitive_poses[0].position.x = -0.06;
  collision_objects[0].primitive_poses[0].position.y = -0.05;
  collision_objects[0].primitive_poses[0].position.z = 0.4;
  collision_objects[0].primitive_poses[0].orientation.x = q.x();
  collision_objects[0].primitive_poses[0].orientation.y = q.y();
  collision_objects[0].primitive_poses[0].orientation.z = q.z();
  collision_objects[0].primitive_poses[0].orientation.w = q.w();
  // add object to planning scene
  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

//Add object collision
void TaskServer::UpdateObject(double box_size, int id) {
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0].header.frame_id = "object" + std::to_string(id);
  collision_objects[0].id = "object_collision_" + std::to_string(id);

  // define the primitive and its dimensions. 
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.05;
  collision_objects[0].primitives[0].dimensions[1] = 0.06;
  collision_objects[0].primitives[0].dimensions[2] = 0.05;

  // define the pose of the object
  collision_objects[0].primitive_poses.resize(1);

  // add object to planning scene
  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Delelt collision
void TaskServer::DeleltCollsion(std::vector<std::string> object_ids) {
  planning_scene_interface.removeCollisionObjects(object_ids);
}

void TaskServer::PickAndPlace(GoalHandle gh, std::vector<std::string> task, actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action){
  ROS_INFO("pick and place");
  // get Move Group client
  
  geometry_msgs::PoseStamped target;
  kinova_msgs::SetFingersPositionGoal finger_position;
  pan_tilt_msgs::PanTiltCmd pan_tilt_msgs;
  std::vector<std::string> object_ids;

  // ros::WallDuration(1.0).sleep();
  // std::vector<double> arm_box_pose(box_count_);
  std::vector<double> arm_box_pose(6);
  double box_position, box_size; 
  int id;
  // std::string box_name = "item_information/" + task[2];
  std::string box_name = "item_information/box";
  nh_.getParam(box_name + "/arm_pose", arm_box_pose);
  nh_.getParam(box_name + "/height", box_position);
  nh_.getParam(box_name + "/size", box_size);
  nh_.getParam(box_name+ "/id", id);

  // std::string place_test = "/target_goal/" + task[1];
  // std::string pick_test = "/target_goal/" + task[3];
  std::string place_test = "place";
  std::string pick_test = "pick";

  geometry_msgs::Pose pick_pose, place_pose;
  pick_pose = GetGoalPose(pick_test);
  place_pose = GetGoalPose(place_test);

  step = 1;
  while(true) {
    ros::spinOnce();

    if (step == 1) {
      /* code */
      // ros::WallDuration(1.0).sleep();
      ROS_INFO("step0: Set pan tilt");
      SetHead(0.0, 0.0, 30);
      ROS_INFO("setp1: Set arm to ready pose");
      SetArmJointPose(arm_standby_pose_);
      if (!ArmMoveAndCheck(arm_group_, arm_group_action, gh))
      {
        task_active_ = false;
        return;
      }
    }
    else if(step == 2) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step2: Set body height to zero");
      SetHeight(0.0); 
      arm_control_->FingerPositionCmd(6500.0, 6500.0, 6500.0); 
    }
    else if(step == 3) {
      if (!MoveToGoal(pick_pose, gh)) {
        return;
      }
      ros::WallDuration(1.0).sleep();
    }
    else if(step == 4) {
      SetHead(0.0, 40.0, 30);
      ros::WallDuration(1.0).sleep();
      get_goal = false;
      ros::Rate loop_rate(10);
      while(!get_goal) {
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = 0.1;
        base_cmd_pub_.publish(cmd_msg);
        loop_rate.sleep();
      }
      ros::WallDuration(1.0).sleep();
      if (find_table_) {
        UpdateTable();
      }
      else {
        task_active_ = false;
        return;
      }
    }
    else if(step == 5) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step6: Update cabinet collision");
      SetArmJointPose(arm_box_pose);
      if (!ArmMoveAndCheck(arm_group_, arm_group_action, gh))
      {
        task_active_ = false;
        return;
      }
      ros::WallDuration(1.0).sleep();
    }
    else if(step == 6) {
      ROS_INFO("step5: Set height for pick");
      // ros::WallDuration(2.0).sleep();
      SetHeight(box_position);       
      ros::WallDuration(1.0).sleep();
    }
    else if(step == 7) {

    }
    else if(step == 8) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step8: Add object collision");
      // delele collsion
      object_ids.push_back("cabinet_collision");
      DeleltCollsion(object_ids);
      object_ids.clear();
      // add collsion on object link
      // UpdateBox(id);
      // UpdateObject(box_size, id);
    }         
    else if(step == 9) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step9: Open gripper");
      arm_control_->FingerPositionCmd(0.0, 0.0, 0.0); 
    }         
    else if(step == 10) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step10: Move arm to pre pick pose");
      target.header.stamp = ros::Time::now();
      target.header.frame_id = "object" + std::to_string(id);
      target.pose.position.x = 0.0;
      target.pose.position.y = 0.02;
      target.pose.position.z = -0.18;
      target.pose.orientation.w = 1.0;
      SetArmTargetPose(target);
      if (!ArmMoveAndCheck(arm_group_, arm_group_action, gh))
      {
        task_active_ = false;
        return;
      }
      ros::WallDuration(1.0).sleep();
    }
    else if(step == 11) {
      ROS_INFO("step11: Move arm to pick pose");
      // ros::WallDuration(1.0).sleep();
      target.header.stamp = ros::Time::now();
      target.pose.position.y = -0.01;
      target.pose.position.z = -0.02;
      SetArmTargetPose(target);
      if (!ArmMoveAndCheck(arm_group_, arm_group_action, gh))
      {
        task_active_ = false;
        return;
      }
    }
    else if(step == 12) {
      ROS_INFO("step10: Close gripper");
      ros::WallDuration(1.0).sleep();
      arm_control_->FingerPositionCmd(5000.0, 5000.0, 5000.0); 
      // arm_group_.attachObject("object_collision_" + std::to_string(id));
    }
    else if(step == 13) {
      ROS_INFO("step13: Move arm up to pick pose");
      ros::WallDuration(1.0).sleep();
      target.header.stamp = ros::Time::now();
      target.pose.position.y = 0.06;
      SetArmTargetPose(target);
      if (!ArmMoveAndCheck(arm_group_, arm_group_action, gh))
      {
        task_active_ = false;
        return;
      }
    }
    else if(step == 14) {
      ROS_INFO("step15: Move arm up to pick positio");
      ros::WallDuration(1.0).sleep();
      target.header.stamp = ros::Time::now();
      // target.pose.position.y = 0.06;
      target.pose.position.z = -0.15;
      SetArmTargetPose(target);
      ArmMoveAndCheck(arm_group_, arm_group_action, gh);

    }         
    else if(step == 16) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step16: Move arm to standby pose");
      ros::Rate loop_rate(10);
      for (int i = 0; i < 35; ++i)
      {
        geometry_msgs::Twist cmd_msg_back;
        cmd_msg_back.linear.x = -0.1;
        base_cmd_pub_.publish(cmd_msg_back);
        loop_rate.sleep();
      }

      SetArmJointPose(arm_standby_pose_);
      if (!ArmMoveAndCheck(arm_group_, arm_group_action, gh))
      {
        task_active_ = false;
        return;
      }
    }         
    else if(step == 17) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step17: Set body to zero");
      SetHeight(0.0);       
    }
    else if(step == 18) {
      ros::WallDuration(1.0).sleep();
      ROS_INFO("step18: Move to place position");
      SetHead(0.0, 0.0, 30);
      if (!MoveToGoal(place_pose, gh))
      {
        return;
      }
    }         
    // if cancel
    if (gh.getGoalStatus().status == 2) {
      step = 1;
      task_active_ = false;
      return;
    }
    // if finished
    else if(step == 18) {
      ROS_INFO("%i", step);
      // result_.finished_step = 7;
      // gh.setSucceeded(result_, "all");
      gh.setSucceeded();
      ROS_INFO("set succeeded");
      return;
    }
    step += 1;
  }
}