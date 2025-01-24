/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>

namespace rosbag2_exporter
{

class IMUHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  IMUHandler(const std::string & output_dir, rclcpp::Logger logger)
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

    // Open file and write IMU data as CSV
    outfile_ = std::ofstream(filepath);
    if (!outfile_.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write IMU data: %s", filepath.c_str());
      return;
    }

    // Write first line
    outfile_ << "#timestamp [ns],"
            << "angular_velocity_x [rad s^-1],angular_velocity_y [rad s^-1],angular_velocity_z [rad s^-1],"
            << "linear_acceleration_x [m s^-2],linear_acceleration_y [m s^-2],linear_acceleration_z [m s^-2],"
            << std::endl;  // << "orientation_x,orientation_y,orientation_z,orientation_w," << std::endl;
  }

  // Destructor to close the logger
  ~IMUHandler()
  {
    outfile_.close();
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::Imu imu_data;
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    serializer.deserialize_message(&serialized_msg, &imu_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << imu_data.header.stamp.sec
                << std::setw(9) << std::setfill('0') << imu_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Write IMU data (angular velocity, linear acceleration, orientation)
    outfile_ << timestamp << ","
            << imu_data.angular_velocity.x << "," << imu_data.angular_velocity.y << "," << imu_data.angular_velocity.z << ","
            << imu_data.linear_acceleration.x << "," << imu_data.linear_acceleration.y << "," << imu_data.linear_acceleration.z
            << std::endl;
            // << imu_data.linear_acceleration.x << "," << imu_data.linear_acceleration.y << "," << imu_data.linear_acceleration.z << ","
            // << imu_data.orientation.x << "," << imu_data.orientation.y << "," << imu_data.orientation.z << "," << imu_data.orientation.w
            // << std::endl;

    RCLCPP_INFO(logger_, "Successfully wrote IMU data, timestamp = %s ns", timestamp.c_str());
  }

private:
  std::string output_dir_;
  std::ofstream outfile_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_
