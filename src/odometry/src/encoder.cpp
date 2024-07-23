#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "odometry/encoder.hpp"

using namespace odometry;

Encoder::Encoder() : rclcpp::Node("encoder")
{
    _left_encoder_raw_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/sim/encoder/left", 10, std::bind(&Encoder::left_encoder_callback, this, std::placeholders::_1)
    );
    _right_encoder_raw_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/sim/encoder/right", 10, std::bind(&Encoder::right_encoder_callback, this, std::placeholders::_1)
    );

    _sim_status_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/sim/status", 10, std::bind(&Encoder::sim_status_callback, this, std::placeholders::_1)
    );
    _sim_time_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/sim/time", 10, std::bind(&Encoder::sim_time_callback, this, std::placeholders::_1)
    );

    _encoder_twist_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "encoder_twist", 10
    );

    _encoder_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Encoder::publish_encoder_twist, this)
    );
}

Encoder::~Encoder()
{
}

void Encoder::left_encoder_callback(const std_msgs::msg::UInt8 & msg)
{
    if (msg.data == 1)
    {
        _left_period = _sim_time - _left_last_read_time;
        _left_last_read_time = _sim_time;
    }
}

void Encoder::right_encoder_callback(const std_msgs::msg::UInt8 & msg)
{
    if (msg.data == 1)
    {
        _right_period = _sim_time - _right_last_read_time;
        _right_last_read_time = _sim_time;
    }
}

void Encoder::sim_status_callback(const std_msgs::msg::Int32 & msg)
{
    _sim_status = msg.data;
}

void Encoder::sim_time_callback(const std_msgs::msg::Float32 & msg)
{
    _sim_time = msg.data;
}

void Encoder::publish_encoder_twist()
{
    if (_sim_status == 1)
    {
        geometry_msgs::msg::TwistWithCovarianceStamped msg;

        // Header
        msg.header.frame_id = "Robot";
        msg.header.stamp = this->now();

        // Obtendo velocidade linear e angular
        float left_freq = 1.0 / _left_period;
        float right_freq = 1.0 / _right_period;

        float left_rpm = 60 * left_freq;
        float right_rpm = 60 * right_freq;

        // Twist
        msg.twist.twist.angular.z = (right_rpm - left_rpm) / _wheel_base;
        msg.twist.twist.linear.x = msg.twist.twist.angular.z * _radius;

        // TODO: Covariance (Como fazer?)
        // msg.twist.covariance;
        
        // TODO: Testar isso
        _encoder_twist_pub->publish(msg);
    }
}

int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Encoder>());
    rclcpp::shutdown();

    return 0;
}