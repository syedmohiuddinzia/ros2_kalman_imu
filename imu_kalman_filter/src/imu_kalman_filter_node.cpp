// imu_kalman_filter_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "kalman_filter.hpp"

class ImuKalmanFilterNode : public rclcpp::Node
{
public:
    ImuKalmanFilterNode()
    : Node("imu_kalman_filter_node"),
      kf_(6, 6)  // 6 states: 3 accel + 3 gyro
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuKalmanFilterNode::imuCallback, this, std::placeholders::_1)
        );

        // (Optional) publisher for filtered data
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);

        RCLCPP_INFO(this->get_logger(), "IMU Kalman Filter Node has been started!");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Get the raw measurements
        Eigen::VectorXd measurement(6);
        measurement << 
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;

        // Predict and Update
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);

        kf_.predict(F);
        kf_.update(measurement, H);

        Eigen::VectorXd filtered = kf_.getState();

        // (Optional) Print filtered results
        RCLCPP_INFO(this->get_logger(), "Filtered IMU: Accel[%.3f, %.3f, %.3f] Gyro[%.3f, %.3f, %.3f]",
            filtered(0), filtered(1), filtered(2),
            filtered(3), filtered(4), filtered(5)
        );

        // (Optional) Publish filtered IMU message
        auto filtered_msg = *msg;  // Copy original msg
        filtered_msg.linear_acceleration.x = filtered(0);
        filtered_msg.linear_acceleration.y = filtered(1);
        filtered_msg.linear_acceleration.z = filtered(2);
        filtered_msg.angular_velocity.x = filtered(3);
        filtered_msg.angular_velocity.y = filtered(4);
        filtered_msg.angular_velocity.z = filtered(5);

        imu_publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    KalmanFilter kf_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}

