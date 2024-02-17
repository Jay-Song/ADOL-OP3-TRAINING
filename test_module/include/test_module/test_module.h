#ifndef TESTODULE_H_
#define TESTODULE_H_

#include <map>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

#include "robotis_framework_common/motion_module.h"

#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace adol
{

class TestModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<TestModule>
{
public:
  TestModule();
  virtual ~TestModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

private:
  void queueThread();
  void torsoIMUMsgCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void amplitudeCallback(const std_msgs::Float64::ConstPtr& msg);
  void periodCallback(const std_msgs::Float64::ConstPtr& msg);
  
  double amplitude_;
  double period_;
  geometry_msgs::Pose torso_pose_;
  
  bool simulation_flag_;
  
  int control_cycle_msec_;
  double control_cycle_sec_;
  double current_time_sec_;
  boost::thread queue_thread_;
  robotis_op::OP3KinematicsDynamics *op3_kd_;

  void updateJointAngles(std::map<std::string, robotis_framework::Dynamixel *> dxls);
  void computeForwardKinematics(void);
  double getHipHeight(void);

  double default_hip_height_;

  double total_mass_kg_;

  double r_leg_angle_rad_[6];
  double l_leg_angle_rad_[6];
};
}


#endif

