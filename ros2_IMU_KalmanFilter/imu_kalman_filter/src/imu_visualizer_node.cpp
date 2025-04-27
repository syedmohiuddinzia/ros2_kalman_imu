#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"

class IMUVisualizer : public rclcpp::Node
{
public:
  IMUVisualizer() : Node("imu_visualizer_node")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&IMUVisualizer::imu_callback, this, std::placeholders::_1));

    filtered_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/filtered", 10, std::bind(&IMUVisualizer::filtered_imu_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto marker = create_arrow_marker(msg, 0, 1.0, 0.0, 0.0); // Red for raw
    marker_pub_->publish(marker);
  }

  void filtered_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto marker = create_arrow_marker(msg, 1, 0.0, 1.0, 0.0); // Green for filtered
    marker_pub_->publish(marker);
  }

  visualization_msgs::msg::Marker create_arrow_marker(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    int id,
    float r, float g, float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "imu_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "imu_arrows";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation = msg->orientation;
    marker.scale.x = 1.0; // arrow length
    marker.scale.y = 0.05; // arrow width
    marker.scale.z = 0.05; // arrow height
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUVisualizer>());
  rclcpp::shutdown();
  return 0;
}

