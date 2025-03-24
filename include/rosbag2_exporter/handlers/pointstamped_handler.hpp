/*
 * Author: Fabien Colonnier
 * Date: 06.03.2025
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__POINTSTAMPED_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__POINTSTAMPED_HANDLER_HPP_

#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

namespace rosbag2_exporter
{

class PointStampedHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  PointStampedHandler(const std::string & output_dir, rclcpp::Logger logger)
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

    // Open file and write POINTSTAMPED data as CSV
    outfile_ = std::ofstream(filepath);
    if (!outfile_.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write POINTSTAMPED data: %s", filepath.c_str());
      return;
    }

    // Write first line
    outfile_ << "#timestamp [ns],"
            << "position x [m],position y [m],position z [m]"
            << std::endl;
  }

  // Destructor to close the logger
  ~PointStampedHandler()
  {
    outfile_.close();
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    geometry_msgs::msg::PointStamped point_data;
    rclcpp::Serialization<geometry_msgs::msg::PointStamped> serializer;
    serializer.deserialize_message(&serialized_msg, &point_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << point_data.header.stamp.sec
                << std::setw(9) << std::setfill('0') << point_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Write Odom data (position, orientation, linear acceleration, angular velocity)
    outfile_ << timestamp << ","
            << point_data.point.x << "," << point_data.point.y << "," << point_data.point.z
            << std::endl;

    RCLCPP_INFO(logger_, "Successfully wrote PointStamped data, timestamp = %s ns", timestamp.c_str());
  }

private:
  std::string output_dir_;
  std::ofstream outfile_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__POINTSTAMPED_HANDLER_HPP_
