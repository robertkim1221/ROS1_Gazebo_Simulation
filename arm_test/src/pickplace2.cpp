
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_operations.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "arm_test/go.h"
#include "arm_test/goRequest.h"
#include "arm_test/goResponse.h"
#include <iostream>


class pick_and_place
{
  public:
    ros::ServiceClient object_position;
    
    void subpubclient()
    {
      object_position = node_handle.serviceClient<arm_test::go>("/go");
      object_position.waitForExistence();
      display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/arm2/move_group/display_planned_path", 1, true);
      planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("/arm2/apply_planning_scene");
      planning_scene_diff_client.waitForExistence();
    }

    
    void graspobject(geometry_msgs::Point position)
    {
      attached_object.link_name = "gripper_base";
      attached_object.touch_links.push_back("right_finger");
      attached_object.touch_links.push_back("left_finger");

      attached_object.object.header.frame_id = "world";
      attached_object.object.id = "object";

      geometry_msgs::Pose pose1;
      pose1.orientation.w = 1.0;
      pose1.position = position;
      pose1.position.x = position.x + 0.95;
      pose1.position.y = position.y - 1.0;
      shape_msgs::SolidPrimitive primitive1;
      primitive1.type = primitive1.CYLINDER;
      primitive1.dimensions.resize(2);
      primitive1.dimensions[0] = 0.099;
      primitive1.dimensions[1] = 0.05;

      attached_object.object.primitives.clear();
      attached_object.object.primitive_poses.clear();
      attached_object.object.primitives.push_back(primitive1);
      attached_object.object.primitive_poses.push_back(pose1);

      attached_object.object.operation = attached_object.object.ADD;

      ROS_INFO("Adding the object into the world.");
      planning_scene.world.collision_objects.push_back(attached_object.object);
      planning_scene.is_diff = true;

// and send the diffs to the planning scene via a service call:
      moveit_msgs::ApplyPlanningScene srv1;
      srv1.request.scene = planning_scene;
      planning_scene_diff_client.call(srv1);
    }

    void attach_object()
    {
      moveit_msgs::CollisionObject remove_object;
      remove_object.id = "object";
      remove_object.header.frame_id = "odom_combined";
      remove_object.operation = remove_object.REMOVE;

      ROS_INFO("Attaching the object to the wrist and removing it from the world.");
      planning_scene.world.collision_objects.clear();
      planning_scene.world.collision_objects.push_back(remove_object);
      planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
      moveit_msgs::ApplyPlanningScene srv2;
      srv2.request.scene = planning_scene;
      planning_scene_diff_client.call(srv2);
    }

    void detach_object()
    {
      moveit_msgs::AttachedCollisionObject detach_object;
      detach_object.object.id = "object";
      detach_object.link_name = "gripper_base";
      detach_object.object.operation = attached_object.object.REMOVE;

      ROS_INFO("DEtaching the object to the wrist and re-adding it to the world.");
      planning_scene.robot_state.is_diff = true;
      planning_scene.robot_state.attached_collision_objects.clear();
      planning_scene.robot_state.attached_collision_objects.push_back(detach_object);

      moveit_msgs::CollisionObject remove_object;
      remove_object.id = "object";
      remove_object.header.frame_id = "odom_combined";
      remove_object.operation = remove_object.REMOVE;

      planning_scene.world.collision_objects.clear();
      planning_scene.world.collision_objects.push_back(remove_object);

      moveit_msgs::ApplyPlanningScene srv3;
      srv3.request.scene = planning_scene;
      planning_scene_diff_client.call(srv3);
    }

    void pre_grasp(moveit::planning_interface::MoveGroup &group, geometry_msgs::Point position)
    {
    moveit_msgs::DisplayTrajectory display_trajectory;  
    geometry_msgs::Pose target_pose1;

    target_pose1.position.x = position.x + 0.95; //subtract x position of robot (0.80) and add half the width of object (0.0125)
    target_pose1.position.y = position.y - 1.0;
    target_pose1.position.z = position.z + 0.1; //pose at 5cm above object(0.89)

    if (target_pose1.position.x > 0)
    {
      target_pose1.orientation.w = 0.5;
      target_pose1.orientation.x = 0.5;
      target_pose1.orientation.y = 0.5;
      target_pose1.orientation.z = 0.5;
    }
    else
    {
      target_pose1.orientation.w = 0.5;
      target_pose1.orientation.x = 0.5;
      target_pose1.orientation.y = -0.5;
      target_pose1.orientation.z = -0.5;
    }
  
    group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    group.move();

    ROS_INFO("Visualizing pre_grasp motion %s",success?"":"FAILED");    

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.z -= 0.08;
    waypoints.push_back(target_pose2);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    group.setPoseTarget(target_pose2);
    group.move();
    }
    void deproach(moveit::planning_interface::MoveGroup &group)
    {
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose2 = group.getCurrentPose().pose;
    waypoints.push_back(target_pose2);
    target_pose2.position.z += 0.1;
    waypoints.push_back(target_pose2);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    group.setPoseTarget(target_pose2);
    group.move();
    }
    void place(moveit::planning_interface::MoveGroup &group)
    {
    moveit_msgs::DisplayTrajectory display_trajectory;

    geometry_msgs::Pose target_pose2;

    target_pose2.position.x = -0.725;
    target_pose2.position.y = 0.055;
    target_pose2.position.z = 0.74;

    if (target_pose2.position.x > 0)
    {
      target_pose2.orientation.w = 0.5;
      target_pose2.orientation.x = 0.5;
      target_pose2.orientation.y = 0.5;
      target_pose2.orientation.z = 0.5;
    }
    else
    {
      target_pose2.orientation.w = 0.5;
      target_pose2.orientation.x = 0.5;
      target_pose2.orientation.y = -0.5;
      target_pose2.orientation.z = -0.5;
    }
    group.setPoseTarget(target_pose2);

    // moveit_msgs::OrientationConstraint ocm;
    // ocm.link_name = "link6";
    // ocm.header.frame_id = "stand";
    // ocm.orientation = target_pose2.orientation;
    // ocm.absolute_x_axis_tolerance = 0.1;
    // ocm.absolute_y_axis_tolerance = 0.1;
    // ocm.absolute_z_axis_tolerance = 0.1;
    // ocm.weight = 0.8;

    // moveit_msgs::Constraints orientation_const;
    // orientation_const.orientation_constraints.push_back(ocm);
    // group.setPathConstraints(orientation_const);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    group.move();

    ROS_INFO("Visualizing placing motion %s",success?"":"FAILED");
    }

  void start_state(moveit::planning_interface::MoveGroup &group)
  {
    moveit_msgs::DisplayTrajectory display_trajectory;

    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    group_variable_values[0] = 0;
    group_variable_values[1] = 0;
    group_variable_values[2] = 0;
    group_variable_values[3] = 0;
    group_variable_values[4] = 0;
    group_variable_values[5] = 0;

    group.setJointValueTarget(group_variable_values);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    group.move();

    ROS_INFO("Visualizing gripper closing %s",success?"":"FAILED");    
  }

//gripper control function that closes the gripper to pick
  void closegripper(moveit::planning_interface::MoveGroup &group)
  {
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  

    group_variable_values[0] = -0.0131;
    group_variable_values[1] = 0.0131;
    group.setJointValueTarget(group_variable_values);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    group.move();

    ROS_INFO("Visualizing gripper closing %s",success?"":"FAILED");    
  }

//gripper control function that closes the gripper to pick
  void opengripper(moveit::planning_interface::MoveGroup &group)
  {
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  

    group_variable_values[0] = 0.01;
    group_variable_values[1] = -0.01;
    group.setJointValueTarget(group_variable_values);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    group.move();

    ROS_INFO("Visualizing gripper opening %s",success?"":"FAILED");    
  }


  private:
  ros::NodeHandle node_handle;
  ros::Publisher display_publisher;
  ros::ServiceClient planning_scene_diff_client;
  moveit_msgs::PlanningScene planning_scene;
  moveit_msgs::AttachedCollisionObject attached_object;
};

//path planning to its initial state, where all the joint values are 0


int main(int argc, char **argv)
{

  //initializing ROS, nodes and publishing
  ros::init (argc, argv, "pickandplace");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Starting pick and place node...");


//intializing subscribers/publishers and service clients
  pick_and_place pp;
  pp.subpubclient();
  ROS_INFO("Initializing subscriber, publisher, and service clients");

//request of grasping object position and calling the service
  arm_test::go positionsrv1;
  positionsrv1.request.arm_type = "arm2";

  pp.object_position.call(positionsrv1);
  ROS_INFO("calling grasp position vectors");

  while (positionsrv1.response.go_vector.empty())
  {
    sleep(0.5);
    pp.object_position.call(positionsrv1);
  }

  std::vector<arm_test::graspposition> target_vec;
  target_vec = positionsrv1.response.go_vector;
  int i=0; //for #1 position
  
  while (pp.object_position.call(positionsrv1)) //another condition that terminates the process when the go_vector is at the end or waits until the next index is filled
  {
    moveit::planning_interface::MoveGroup group1("gripper");
    target_vec = positionsrv1.response.go_vector;
    std::cout<<target_vec[i].position.x<<std::endl;
    std::cout<<target_vec[i].position.y<<std::endl;
    std::cout<<target_vec[i].position.z<<std::endl;

  //placing object of interest (grasp object) in the planning scene
    pp.graspobject(target_vec[i].position);
    sleep(1.0);
    pp.opengripper(group1);

  //motion planning to grasp object
    moveit::planning_interface::MoveGroup group("arm");
    pp.pre_grasp(group, target_vec[i].position);

//attaching object to the gripper to avoid collision
    pp.attach_object();

//closing gripper
    pp.closegripper(group1);
    sleep(1.5);
//placing object at designated position (bin)
    pp.place(group);
//opening gripper
    pp.opengripper(group1);

//adding object back into collision world
    pp.detach_object();

//returning to start state
    pp.start_state(group);

    pp.object_position.call(positionsrv1);
    while ((positionsrv1.response.go_vector[positionsrv1.response.go_vector.size()-1].object_names==target_vec[i].object_names))
    {
      sleep(0.5);
      pp.object_position.call(positionsrv1);
    }

    i++; //for #1 position
  }

  ros::shutdown();
  return 0;
}
