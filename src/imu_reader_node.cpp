#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

using namespace std::chrono_literals;
using boost::asio::serial_port;
using boost::asio::io_service;

class ImuReaderNode : public rclcpp::Node {
public:
  ImuReaderNode() : Node("imu_reader_node"), io(), serial(io, "/dev/ttyACM0") {
    // Configure serial port parameters
    serial.set_option(serial_port::baud_rate(115200));
    serial.set_option(serial_port::character_size(8));
    serial.set_option(serial_port::parity(serial_port::parity::none));
    serial.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    serial.set_option(serial_port::flow_control(serial_port::flow_control::none));

    // Publisher for IMU data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 20);
    timer_ = this->create_wall_timer(100ms, std::bind(&ImuReaderNode::readSerialData, this));
  }

private:
  void readSerialData() {
    // Read data from serial port
    std::string line;
    boost::asio::streambuf buf;
    boost::asio::read_until(serial, buf, '\n');
    std::istream is(&buf);
    std::getline(is, line);

    // Parse IMU data from the Arduino
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    if (sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f", &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz) == 9) {
      // Populate the IMU message
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = this->now();
      imu_msg.header.frame_id = "imu_link";

      // Linear acceleration
      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;

      // Angular velocity
      imu_msg.angular_velocity.x = gx * M_PI / 180.0;  // Convert to radians
      imu_msg.angular_velocity.y = gy * M_PI / 180.0;
      imu_msg.angular_velocity.z = gz * M_PI / 180.0;

      // Magnetometer data can be added in a custom message or as covariance for orientation (if required)

      imu_publisher_->publish(imu_msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to parse IMU data.");
    }
  }

  io_service io;
  serial_port serial;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuReaderNode>());
  rclcpp::shutdown();
  return 0;
}
