#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>
#include <vector>
#include <iomanip>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the ROS 2 node
  auto node = std::make_shared<rclcpp::Node>("moveit_plan_and_execute");

  // Create a ROS 2 executor and add the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Start spinning the executor in a separate thread
  std::thread spinner([&executor]() { executor.spin(); });

  // Initialize MoveGroupInterface for the arm
  auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");

  // Set the predefined pose "pregrasp_handle" from the SRDF
  move_group_interface->setNamedTarget("pregrasp_handle");

  // Create a plan to the predefined pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveItErrorCode success = move_group_interface->plan(plan);

  if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Planning to 'pregrasp_handle' pose succeeded.");

    // Extract end-effector trajectory
    std::vector<geometry_msgs::msg::Pose> end_effector_trajectory;

    for (const auto& point : plan.trajectory_.joint_trajectory.points) {
      // Set joint values and compute end-effector pose
      move_group_interface->setJointValueTarget(point.positions);
      move_group_interface->move(); // Ensure the move completes before getting the pose
      auto end_effector_pose = move_group_interface->getCurrentPose().pose;
      end_effector_trajectory.push_back(end_effector_pose);
    }

    // Save end-effector trajectory to a file
    std::ofstream file("end_effector_trajectory.csv");
    if (file.is_open()) {
      file << "Position X,Position Y,Position Z,Orientation X,Orientation Y,Orientation Z,Orientation W\n";
      for (const auto& pose : end_effector_trajectory) {
        file << std::fixed << std::setprecision(6)
             << pose.position.x << ","
             << pose.position.y << ","
             << pose.position.z << ","
             << pose.orientation.x << ","
             << pose.orientation.y << ","
             << pose.orientation.z << ","
             << pose.orientation.w << "\n";
      }
      file.close();
      RCLCPP_INFO(node->get_logger(), "End-effector trajectory saved to 'end_effector_trajectory.csv'.");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to open file for saving end-effector trajectory.");
    }

    // Execute the plan
    RCLCPP_INFO(node->get_logger(), "Executing plan to 'pregrasp_handle' pose...");
    moveit::planning_interface::MoveItErrorCode execute_result = move_group_interface->execute(plan);

    if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Reached 'pregrasp_handle' pose!");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to execute plan to 'pregrasp_handle' pose.");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning to 'pregrasp_handle' pose failed!");
  }

  // Shut down and clean up
  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }

  return 0;
}
