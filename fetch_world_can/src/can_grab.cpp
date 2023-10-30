#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//some global variables
geometry_msgs::PoseStamped pose_can_;
bool pose_updated = false;
void poseCB(const geometry_msgs::PoseStamped &pose)
{
    pose_can_ = pose;
    pose_updated = true;
    ROS_INFO_STREAM("pose received");
} 
void move_to_pose(moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
    {
        ROS_INFO_STREAM("path failed to calcuate");
    }
    else
    {
        ROS_INFO_STREAM("moving arm");
        move_group.move();
        move_group.clearPoseTarget();

    }
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "can_grab");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub;
    pose_sub = nh.subscribe("/can_location", 1, poseCB);

    ros::Rate limiter(5);
    while( ros::ok() && !pose_updated)
    {
        ros::spinOnce();
        limiter.sleep();
    }
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // first define planning group
    static const std::string PLANNING_GROUP = "arm_with_torso";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
 
	//use PlanningSceneInterface class to deal directly with world
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // add collision of table into moveit
    // centre of table is at x = 1 y = 0, size of table is 1.5 * 0.8 *1.03
    
	//adding/removing objects and attaching /detaching objects
    //first, define the collision object message
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "/odom";

	collision_object.id = "table";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.8;
	primitive.dimensions[1] = 1.9;
	primitive.dimensions[2] = 1.03;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1;
	box_pose.position.x = 1;
	box_pose.position.y = 0;
	box_pose.position.z = 0.5015;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
    
    moveit_msgs::CollisionObject collision_object_2;
	collision_object_2.header.frame_id = "/odom";

	collision_object_2.id = "table_ABOVE";

	geometry_msgs::Pose box_pose_2;
	box_pose_2.orientation.w = 1;
	box_pose_2.position.x = 1;
	box_pose_2.position.y = 0;
	box_pose_2.position.z = 1.5015;

	collision_object_2.primitives.push_back(primitive);
	collision_object_2.primitive_poses.push_back(box_pose_2);
	collision_object_2.operation = collision_object.ADD;

    //collision_objects.push_back(collision_object_2);
    
    //add the collision object into the world
    // this has been commented out as this caused path planning to become extremely unstable and imposible to pickup can
	//planning_scene_interface.addCollisionObjects(collision_objects);
    //planning_scene_interface.applyCollisionObjects(collision_objects);
    

    // transform pose_can_ into move_group planning frame
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped camera_to_base; // My frames are named "base_link" and "leap_motion"

    camera_to_base = tf_buffer.lookupTransform(move_group.getPlanningFrame(), pose_can_.header.frame_id, ros::Time(0), ros::Duration(1.0) );

    tf2::doTransform(pose_can_.pose, pose_can_.pose, camera_to_base); // robot_pose is the PoseStamped I want to transform
    pose_can_.pose.orientation.w = 1;
    
    // setting some values to allow for easier path planning
    move_group.setGoalTolerance(0.01);
    move_group.setPlanningTime(10);
    // adding constraint on torso joint
    moveit_msgs::Constraints constraint;
    moveit_msgs::JointConstraint joint;
    joint.joint_name = "torso_lift_joint";
    joint.position = 0.365;
    joint.tolerance_above = 0.01;
    joint.tolerance_below = 0.01;
    joint.weight = 1;
    constraint.joint_constraints.push_back(joint);
    //move_group.setPathConstraints(constraint);

    ROS_INFO_STREAM("pose transformed  x="<<pose_can_.pose.position.x<<" y="<<pose_can_.pose.position.y<<" z="<<pose_can_.pose.position.z);
    // first move to a set pose that can be found easily
    geometry_msgs::Pose target_pose0;
    target_pose0.orientation.w = 1.0;
	target_pose0.position.x = pose_can_.pose.position.x - 0.4;
	target_pose0.position.y = pose_can_.pose.position.y;
	target_pose0.position.z = pose_can_.pose.position.z;
    
    // move to this first pose close to can
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(target_pose0);
    move_to_pose(move_group, my_plan);
    move_group.setMaxVelocityScalingFactor(0.1);
    // create a cartesian path to approach can and pickup
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose1 = target_pose0;

    target_pose1.position.x += 0.05;
    waypoints.push_back(target_pose1);  // out 

    geometry_msgs::Pose target_pose2 = target_pose1;

    target_pose2.position.x += 0.05;
    waypoints.push_back(target_pose2);

    geometry_msgs::Pose target_pose3 = target_pose2;

    target_pose3.position.x += 0.05;
    waypoints.push_back(target_pose3);

    geometry_msgs::Pose target_pose4 = target_pose3;

    target_pose4.position.x += 0.065;
    waypoints.push_back(target_pose4);    

    for( auto pose: waypoints)
    {   
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(pose);
        move_to_pose(move_group,my_plan);
        move_group.clearPoseTarget();
    }


    // now we close the gripper
    // first define planning group
    static const std::string PLANNING_GROUP_gripper = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group_gripper(PLANNING_GROUP_gripper);
    
    // set joint positions with the following vector
    std::vector<double> x;
    x.push_back(0.03);
    x.push_back(0.03);
    move_group_gripper.setJointValueTarget(x);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_grip;
    bool success_grip = (move_group_gripper.plan(my_plan_grip) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success_grip)
    {
        move_group_gripper.execute(my_plan_grip);
    }
    geometry_msgs::Pose target_pose5 = target_pose4;

    target_pose5.position.z += 0.25;
    move_group.setPoseTarget(target_pose5);
    move_to_pose(move_group,my_plan);  

    
}
    