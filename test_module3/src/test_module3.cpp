#include "test_module3/test_module3.h"

using namespace adol;

TestModule3::TestModule3()
{
  enable_ = false;
  module_name_ = "test_module";
  control_mode_ = robotis_framework::PositionControl;

  // result
  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["r_sho_roll"] = new robotis_framework::DynamixelState();
  result_["r_el"] = new robotis_framework::DynamixelState();

  result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["l_sho_roll"] = new robotis_framework::DynamixelState();
  result_["l_el"] = new robotis_framework::DynamixelState();

  result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"] = new robotis_framework::DynamixelState();

  result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll"] = new robotis_framework::DynamixelState();

  result_["head_pan"] = new robotis_framework::DynamixelState();
  result_["head_tilt"] = new robotis_framework::DynamixelState();

  op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);

  total_mass_kg_ = op3_kd_->calcTotalMass(0);

  default_hip_height_ = 0.21 + 0.035;

  previous_orientation_ = Eigen::Matrix3d::Identity();
  current_orientation_ = Eigen::Matrix3d::Identity();
}

TestModule3::~TestModule3()
{
  queue_thread_.join();
}

void TestModule3::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  control_cycle_sec_ = 0.001 * control_cycle_msec_;

  // init result, joint_id_table
  for (std::map<std::string, robotis_framework::Dynamixel *>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel *dxl_info = it->second;

    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_ = 0;
    result_[joint_name]->present_position_ = dxl_info->dxl_state_->present_position_;
  }

  amplitude_ = 1.0;
  period_ = 2.0;
  current_time_sec_ = 0;


  ros::NodeHandle ros_node;
  ros_node.param<bool>("gazebo", simulation_flag_, false);

  // making subscribers and ros spin
  queue_thread_ = boost::thread(boost::bind(&TestModule3::queueThread, this));
}

void TestModule3::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  ros::Subscriber imu_sub;
  if (simulation_flag_ == true)
  {
    /* subscribe topics */
    imu_sub = ros_node.subscribe("/robotis_op3/torso_orientation", 5, &TestModule3::torsoIMUMsgCallback, this);
  }

  ros::Subscriber amplitude_sub = ros_node.subscribe("/adol/op3/test/amplitude", 5, &TestModule3::amplitudeCallback, this);
  ros::Subscriber period_sub = ros_node.subscribe("/adol/op3/test/period", 5, &TestModule3::periodCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TestModule3::torsoIMUMsgCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  torso_pose_.position = msg->position;
  torso_pose_.orientation.x = msg->orientation.x;
  torso_pose_.orientation.y = msg->orientation.y;
  torso_pose_.orientation.z = msg->orientation.z;
  torso_pose_.orientation.w = msg->orientation.w;
}

void TestModule3::amplitudeCallback(const std_msgs::Float64::ConstPtr &msg)
{
  amplitude_ = msg->data;
}

void TestModule3::periodCallback(const std_msgs::Float64::ConstPtr &msg)
{
  period_ = msg->data;
}

void TestModule3::stop()
{

}

bool TestModule3::isRunning()
{
  return false;
}

void TestModule3::onModuleEnable()
{
  current_time_sec_ = 0;
}

void TestModule3::onModuleDisable()
{

}

void TestModule3::updateJointAngles(std::map<std::string, robotis_framework::Dynamixel *> dxls)
{
  // getting present position of dxls
  op3_kd_->op3_link_data_[1]->joint_angle_ = dxls["r_sho_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[2]->joint_angle_ = dxls["l_sho_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[3]->joint_angle_ = dxls["r_sho_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[4]->joint_angle_ = dxls["l_sho_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[5]->joint_angle_ = dxls["r_el"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[6]->joint_angle_ = dxls["l_el"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[7]->joint_angle_ = dxls["r_hip_yaw"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[8]->joint_angle_ = dxls["l_hip_yaw"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[9]->joint_angle_ = dxls["r_hip_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[10]->joint_angle_ = dxls["l_hip_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[11]->joint_angle_ = dxls["r_hip_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[12]->joint_angle_ = dxls["l_hip_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[13]->joint_angle_ = dxls["r_knee"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[14]->joint_angle_ = dxls["l_knee"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[15]->joint_angle_ = dxls["r_ank_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[16]->joint_angle_ = dxls["l_ank_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[17]->joint_angle_ = dxls["r_ank_roll"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[18]->joint_angle_ = dxls["l_ank_roll"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[19]->joint_angle_ = dxls["head_pan"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[20]->joint_angle_ = dxls["head_tilt"]->dxl_state_->present_position_;
}

void TestModule3::computeForwardKinematics(void)
{
  op3_kd_->calcForwardKinematics(0);
  Eigen::MatrixXd com = op3_kd_->calcMC(0)/total_mass_kg_;
  ROS_INFO_STREAM("current COM: " << com(0,0) << " " << com(1,0) << " " << com(2,0));
}

double TestModule3::getHipHeight(void)
{
  Eigen::MatrixXd hip_pos = op3_kd_->op3_link_data_[11]->position_ - 0.5*(op3_kd_->op3_link_data_[30]->position_ + op3_kd_->op3_link_data_[31]->position_);
  return hip_pos(2,0);
}

void TestModule3::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;
  
  if (simulation_flag_ == false)
  {
    rl_gyro_err = 0.0 - sensors["gyro_x"];
    fb_gyro_err = 0.0 - sensors["gyro_y"];

    previous_orientation_ = current_orientation_;
    current_orientation_ = robotis_framework::convertRPYToRotation(sensors["roll"], sensors["pitch"], sensors["yaw"]);
  }
  else 
  {
    previous_orientation_ = current_orientation_;
    current_orientation_ =
     Eigen::Quaterniond(torso_pose_.orientation.w, torso_pose_.orientation.x, torso_pose_.orientation.y, torso_pose_.orientation.z).toRotationMatrix();

    Eigen::Vector3d angular_velocity_rps = robotis_framework::convertRotToOmega(current_orientation_*previous_orientation_.transpose());

    rl_gyro_err = 0.0 - angular_velocity_rps(0)/control_cycle_sec_;
    fb_gyro_err = 0.0 - angular_velocity_rps(1)/control_cycle_sec_;
  }

  updateJointAngles(dxls);
  computeForwardKinematics();
  //ROS_INFO_STREAM("hip_height: " << getHipHeight());
  
  Eigen::Matrix4d mat_RH_to_Pelvis = robotis_framework::getTransformationXYZRPY(0,
  -op3_kd_->op3_link_data_[7]->relative_position_(1,0),
  -op3_kd_->op3_link_data_[9]->relative_position_(2,0), 
  0, 0, 0);

  Eigen::Matrix4d mat_LH_to_Pelvis = robotis_framework::getTransformationXYZRPY(0,
  -op3_kd_->op3_link_data_[8]->relative_position_(1,0),
  -op3_kd_->op3_link_data_[10]->relative_position_(2,0), 
  0, 0, 0);

  Eigen::Matrix4d mat_G_to_Pelvis = robotis_framework::getTransformationXYZRPY(0, 0, default_hip_height_, 0, 0, 0);
  Eigen::Matrix4d mat_G_to_RF = robotis_framework::getTransformationXYZRPY(0, -0.5*(op3_kd_->leg_side_offset_m_), 0, 0, -0.05*fb_gyro_err, 0);
  Eigen::Matrix4d mat_G_to_LF = robotis_framework::getTransformationXYZRPY(0,  0.5*(op3_kd_->leg_side_offset_m_), 0, 0, -0.05*fb_gyro_err, 0);

  robotis_framework::Pose3D pose_RH_to_RF = robotis_framework::getPose3DfromTransformMatrix( (mat_RH_to_Pelvis * robotis_framework::getInverseTransformation(mat_G_to_Pelvis)) * mat_G_to_RF);
  robotis_framework::Pose3D pose_LH_to_LF = robotis_framework::getPose3DfromTransformMatrix( (mat_LH_to_Pelvis * robotis_framework::getInverseTransformation(mat_G_to_Pelvis)) * mat_G_to_LF);

  // op3_kd_->calcInverseKinematicsForRightLeg(r_leg_angle_rad_, 0, 0, -default_hip_height_ - 0.02*sin(2.0 * M_PI * current_time_sec_/ period_), 0, 0, 0);
  // op3_kd_->calcInverseKinematicsForLeftLeg(l_leg_angle_rad_, 0, 0, -default_hip_height_ - 0.02*sin(2.0 * M_PI * current_time_sec_/ period_), 0, 0, 0);

  op3_kd_->calcInverseKinematicsForRightLeg(r_leg_angle_rad_, pose_RH_to_RF.x, pose_RH_to_RF.y, pose_RH_to_RF.z, pose_RH_to_RF.roll, pose_RH_to_RF.pitch, pose_RH_to_RF.yaw);
  op3_kd_->calcInverseKinematicsForLeftLeg(l_leg_angle_rad_, pose_LH_to_LF.x, pose_LH_to_LF.y, pose_LH_to_LF.z, pose_LH_to_LF.roll, pose_LH_to_LF.pitch, pose_LH_to_LF.yaw);

  result_["r_hip_yaw"]->goal_position_   = r_leg_angle_rad_[0];
  result_["r_hip_roll"]->goal_position_  = r_leg_angle_rad_[1];
  result_["r_hip_pitch"]->goal_position_ = r_leg_angle_rad_[2];
  result_["r_knee"]->goal_position_      = r_leg_angle_rad_[3];
  result_["r_ank_pitch"]->goal_position_ = r_leg_angle_rad_[4];
  result_["r_ank_roll"]->goal_position_  = r_leg_angle_rad_[5];

  result_["l_hip_yaw"]->goal_position_   = l_leg_angle_rad_[0];
  result_["l_hip_roll"]->goal_position_  = l_leg_angle_rad_[1];
  result_["l_hip_pitch"]->goal_position_ = l_leg_angle_rad_[2];
  result_["l_knee"]->goal_position_      = l_leg_angle_rad_[3];
  result_["l_ank_pitch"]->goal_position_ = l_leg_angle_rad_[4];
  result_["l_ank_roll"]->goal_position_  = l_leg_angle_rad_[5];

  //result_["r_sho_roll"]->goal_position_ = amplitude_ * sin(2.0 * M_PI * current_time_sec_/ period_);
  //result_["l_sho_roll"]->goal_position_ = amplitude_ * sin(2.0 * M_PI * current_time_sec_/ period_);
  
  current_time_sec_ += control_cycle_sec_;
}


