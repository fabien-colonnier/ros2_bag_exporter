/*
 * Author: Fabien Colonnier
 * Date: 06.03.2025
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__ODOM_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__ODOM_HANDLER_HPP_

#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace rosbag2_exporter
{

class OdomHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  OdomHandler(const std::string & output_dir, rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {
    // Create the full file path with '.csv' as the extension
    std::string filepath = output_dir_ + "/data.csv";

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = output_dir_;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Open file and write Odom data as CSV
    outfile_ = std::ofstream(filepath);
    if (!outfile_.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write Odom data: %s", filepath.c_str());
      return;
    }

    // Write first line
    outfile_ << "#timestamp [ns],"
            << "position x [m],position y [m],position z [m],"
            << "orientation qx [-],orientation qy [-],orientation qz [-],orientation qw [-],"
            << "linear_acceleration_x [m s^-2],linear_acceleration_y [m s^-2],linear_acceleration_z [m s^-2],"
            << "angular_velocity_x [rad s^-1],angular_velocity_y [rad s^-1],angular_velocity_z [rad s^-1],"  
            << std::endl;
  }

  // Destructor to close the logger
  ~OdomHandler()
  {
    outfile_.close();
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    nav_msgs::msg::Odometry odom_data;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
    serializer.deserialize_message(&serialized_msg, &odom_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << odom_data.header.stamp.sec
                << std::setw(9) << std::setfill('0') << odom_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Write Odom data (position, orientation, linear acceleration, angular velocity)
    outfile_ << timestamp << ","
            << odom_data.pose.pose.position.x << "," << odom_data.pose.pose.position.y << "," << odom_data.pose.pose.position.z << ","
            << odom_data.pose.pose.orientation.x << "," << odom_data.pose.pose.orientation.y << "," << odom_data.pose.pose.orientation.z << "," << odom_data.pose.pose.orientation.w << ","
            << odom_data.twist.twist.linear.x << "," << odom_data.twist.twist.linear.y << "," << odom_data.twist.twist.linear.z << ","
            << odom_data.twist.twist.angular.x << "," << odom_data.twist.twist.angular.y << "," << odom_data.twist.twist.angular.z      
            << std::endl;

    RCLCPP_INFO(logger_, "Successfully wrote Odom data, timestamp = %s ns", timestamp.c_str());
  }

private:
  std::string output_dir_;
  std::ofstream outfile_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__ODOM_HANDLER_HPP_
