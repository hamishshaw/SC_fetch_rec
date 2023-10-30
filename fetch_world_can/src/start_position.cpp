#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"

#include "geometry_msgs/Twist.h"

double integrate(double desired, double present, double max_rate, double dt)
{
  if (desired > present)
    return std::min(desired, present + max_rate * dt);
  else
    return std::max(desired, present - max_rate * dt);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "start_position");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // first define planning group
  static const std::string PLANNING_GROUP = "arm_with_torso";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  
  // get the robots current joint state
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  std::vector<std::string> joint_names =  move_group.getJointModelGroupNames();

  for(auto joint : joint_names)
  { 
    ROS_INFO_STREAM("model with name "<<joint);
  }
  std::vector<std::string> link_names = joint_model_group->getLinkModelNames();
  for(auto link : link_names)
  { 
    ROS_INFO_STREAM("link with name "<<link);
  }
  //joint_model_group->;
  // An idea on moving a single joint
  std::vector<double> x;
  move_group.getCurrentState() ->copyJointGroupPositions(move_group.getCurrentState() ->getRobotModel() ->getJointModelGroup(move_group.getName()), x);
  x[0]=0.365;
  move_group.setJointValueTarget(x);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if(!success)
  {
    ROS_ERROR_STREAM("that plan dont work");
  }
  else
  {
    move_group.execute(my_plan);
  }
  spinner.stop();
  // execute is blocking and we wait until this is completed to tilt head
  // now that the torso is in position lets move the head
  ros::Publisher control_pub;
  ros::Publisher control_vel_pub;
  control_pub = node_handle.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/head_controller/follow_joint_trajectory/goal", 3, true);
  control_vel_pub = node_handle.advertise<geometry_msgs::Twist>("/base_controller/command",3,true);
  // set up some values to publisher head tilt
  double step = 0.125;
  double pan_vel = 0;
  double pan_travel = step * (pan_vel + 0) / 2.0;
  double pan =  0; //std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
  double tilt_vel = integrate(1, 0, 3.0, step);
  double tilt_travel = step * (tilt_vel + 0) / 2.0;
  double tilt = 0.7;
  // Publish message
  control_msgs::FollowJointTrajectoryActionGoal goal;
  goal.goal.trajectory.joint_names.push_back("head_pan_joint");
  goal.goal.trajectory.joint_names.push_back("head_tilt_joint");
  trajectory_msgs::JointTrajectoryPoint p;
  p.positions.push_back(pan);
  p.positions.push_back(tilt);
  p.velocities.push_back(pan_vel);
  p.velocities.push_back(tilt_vel);
  p.time_from_start = ros::Duration(step);
  goal.goal.trajectory.points.push_back(p);
  goal.goal.goal_time_tolerance = ros::Duration(0.0);
  
  control_pub.publish(goal);
  // create twist message to move base forward
  geometry_msgs::Twist vel;
  vel.linear.x=0.5; vel.linear.y=0; vel.linear.z=0;
  vel.angular.x=0;vel.angular.y=0;vel.angular.z=0;
  

  // spin until finished moving head to do this subscribe to status topic
  // instead of doing something fancy with waiting until status is complete this works fine for such a simple movement
  int i = 0;
  ros::Rate limit(5);
  while(ros::ok && i < 12)
  {
    ros::spinOnce();
    limit.sleep();
    i++;
    control_vel_pub.publish(vel);
  }

}