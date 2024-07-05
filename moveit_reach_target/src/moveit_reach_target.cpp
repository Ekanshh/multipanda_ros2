#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_reach_target",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_reach_target");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_move_group_interface = MoveGroupInterface(node, "panda_arm");
  auto gripper_move_group_interface = MoveGroupInterface(node, "hand");

  // Open the gripper
  gripper_move_group_interface.setNamedTarget("open");
  gripper_move_group_interface.move();

  // Set a target Pose
  // 0.299 -0.019447 0.863787 0.675284 0.197643 0.630325 0.328054
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.299;
    msg.position.y = -0.019447;
    msg.position.z = 0.873787;
    msg.orientation.x = 0.675284;
    msg.orientation.y = 0.197643;
    msg.orientation.z = 0.630325;
    msg.orientation.w = 0.328054;
    return msg;
  }();
  
  // Print the target pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Target pose: %f %f %f %f %f %f %f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w);
  
  arm_move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&arm_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    arm_move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Gripper close
  gripper_move_group_interface.setNamedTarget("close");
  gripper_move_group_interface.move();

  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}