#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>  // Include the queue header

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher()
    : Node("pose_publisher")
  {
    // Create a publisher for end-effector poses
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/franka/pose", 10);

    // Load poses from the CSV file
    load_poses_from_file("end_effector_trajectory.csv");

    // Publish the poses with a delay
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (!pose_queue_.empty()) {
          auto pose = pose_queue_.front();
          pose_queue_.pop();
          pose_publisher_->publish(pose);
          RCLCPP_INFO(this->get_logger(), "Published pose: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                      pose.position.x, pose.position.y, pose.position.z,
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
      });
  }

private:
  void load_poses_from_file(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return;
    }

    std::string line;
    // Skip header line
    std::getline(file, line);

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      geometry_msgs::msg::Pose pose;

      // Read position
      std::getline(ss, value, ',');
      pose.position.x = std::stod(value);
      std::getline(ss, value, ',');
      pose.position.y = std::stod(value);
      std::getline(ss, value, ',');
      pose.position.z = std::stod(value);

      // Read orientation
      std::getline(ss, value, ',');
      pose.orientation.x = std::stod(value);
      std::getline(ss, value, ',');
      pose.orientation.y = std::stod(value);
      std::getline(ss, value, ',');
      pose.orientation.z = std::stod(value);
      std::getline(ss, value, ',');
      pose.orientation.w = std::stod(value);

      pose_queue_.push(pose);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::queue<geometry_msgs::msg::Pose> pose_queue_; // Declare the queue here
};

int main(int argc, char* argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create and spin the PosePublisher node
  auto node = std::make_shared<PosePublisher>();
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
