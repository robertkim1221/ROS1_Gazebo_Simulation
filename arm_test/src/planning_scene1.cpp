
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_operations.h>

#include <iostream>

using namespace Eigen;
int main(int argc, char **argv)
{

  //initializing ROS, nodes and publishing
  ros::init (argc, argv, "planning_scene");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  //for attaching/detaching objects, use diff service
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  ros::Duration sleep_time(1);
  sleep_time.sleep();

/**********ADDING OBJECT and table to grasp INTO ENVIRONMENT START***********/

  moveit_msgs::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "table";
  
  Vector3d vectorScale(0.01, 0.01, 0.01);
  shapes::Mesh* m = shapes::createMeshFromResource("package://mrm_description/meshes/testbed_wo_sensor.STL", vectorScale);
  ROS_INFO("Testbed mesh file loaded");

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  geometry_msgs::Pose pose;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y= 0.7071068;
  pose.orientation.z = 0.7071068;
  pose.position.x = -0.6429;
  pose.position.y = -0.632;
  pose.position.z = -0.05;
  

  table.meshes.push_back(mesh);
  table.mesh_poses.push_back(pose);
  table.operation = table.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(table);
  sleep_time.sleep();

  moveit_msgs::CollisionObject bin;
  bin.header.frame_id = "world";
  bin.id = "bin";
  
  Vector3d vectorScale1(0.01, 0.01, 0.01);
  shapes::Mesh* m1 = shapes::createMeshFromResource("package://mrm_description/meshes/bin.STL", vectorScale1);
  ROS_INFO("Testbed mesh file loaded");

  shape_msgs::Mesh mesh1;
  shapes::ShapeMsg mesh_msg1;
  shapes::constructMsgFromShape(m1, mesh_msg1);
  mesh1 = boost::get<shape_msgs::Mesh>(mesh_msg1);

  geometry_msgs::Pose pose1;
  pose1.orientation.w = 0.7071068;
  pose1.orientation.x = 0.7071068;
  pose1.orientation.y= 0;
  pose1.orientation.z = 0;
  pose1.position.x = 0.55; //(bin x position - robot x position)
  pose1.position.y = 0.3;
  pose1.position.z = -0.18;
  

  bin.meshes.push_back(mesh1);
  bin.mesh_poses.push_back(pose1);
  bin.operation = bin.ADD;

  collision_objects.push_back(bin);
  planning_scene_interface.addCollisionObjects(collision_objects);

  
  sleep_time.sleep();

  ros::shutdown();
  return 0;
}
