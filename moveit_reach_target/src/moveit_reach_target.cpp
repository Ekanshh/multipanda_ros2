#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_reach_target",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto logger = rclcpp::get_logger("moveit_reach_target");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_move_group_interface(node, "panda_arm");
  MoveGroupInterface gripper_move_group_interface(node, "hand");

  // Open the gripper
  RCLCPP_INFO(logger, "Opening the gripper...");
  gripper_move_group_interface.setNamedTarget("open");
  if (gripper_move_group_interface.move()) {
    RCLCPP_INFO(logger, "Gripper opened.");
  } else {
    RCLCPP_ERROR(logger, "Failed to open gripper.");
    return 1;
  }
  
  rclcpp::sleep_for(std::chrono::seconds(2)); // Wait for the gripper to open

  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.347324;
  target_pose.position.y = -0.005022;
  target_pose.position.z = 0.578285;
  target_pose.orientation.x = 0.922637;
  target_pose.orientation.y = 0.385670;
  target_pose.orientation.z = 0.000355;
  target_pose.orientation.w = 0.000594;
  
  // Print the target pose of the end effector
  RCLCPP_INFO(logger, "Target pose: %f %f %f %f %f %f %f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w);
  
  arm_move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm_move_group_interface.plan(plan));

  // Execute the plan
  if(success) {
    RCLCPP_INFO(logger, "Executing plan to target pose...");
    if (arm_move_group_interface.execute(plan)) {
      RCLCPP_INFO(logger, "Reached target pose!");
    } else {
      RCLCPP_ERROR(logger, "Failed to execute plan to target pose.");
      return 1;
    }
  } else {
    RCLCPP_ERROR(logger, "Planning to target pose failed!");
    return 1;
  }

  // Close the gripper
  RCLCPP_INFO(logger, "Closing the gripper...");
  gripper_move_group_interface.setNamedTarget("close");
  if (gripper_move_group_interface.move()) {
    RCLCPP_INFO(logger, "Gripper closed.");
  } else {
    RCLCPP_ERROR(logger, "Failed to close gripper.");
    return 1;
  }
  
  rclcpp::sleep_for(std::chrono::seconds(2)); // Wait for the gripper to close

  // Send next pose
  geometry_msgs::msg::Pose target_pose_2;
  target_pose_2.position.x = 0.347324;
  target_pose_2.position.y = -0.005022;
  target_pose_2.position.z = 0.508285;
  target_pose_2.orientation.x = 0.922637;
  target_pose_2.orientation.y = 0.385670;
  target_pose_2.orientation.z = 0.000355;
  target_pose_2.orientation.w = 0.000594;
  
  arm_move_group_interface.setPoseTarget(target_pose_2);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  bool success2 = static_cast<bool>(arm_move_group_interface.plan(plan2));

  // Execute the plan
  if(success2) {
    RCLCPP_INFO(logger, "Executing plan to target pose 2...");
    if (arm_move_group_interface.execute(plan2)) {
      RCLCPP_INFO(logger, "Reached target pose 2!");
    } else {
      RCLCPP_ERROR(logger, "Failed to execute plan to target pose 2.");
      return 1;
    }
  } else {
    RCLCPP_ERROR(logger, "Planning to target pose 2 failed!");
    return 1;
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
