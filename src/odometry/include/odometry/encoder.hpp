#ifndef ODOMETRY__ENCODER_H
#define ODOMETRY__ENCODER_H

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

namespace odometry
{
    class Encoder : public rclcpp::Node
    {
        public:
            Encoder();
            ~Encoder();

        private:
            void left_encoder_callback(const std_msgs::msg::UInt8 & msg);
            void right_encoder_callback(const std_msgs::msg::UInt8 & msg);

            void sim_status_callback(const std_msgs::msg::Int32 & msg);
            void sim_time_callback(const std_msgs::msg::Float32 & msg);

            void publish_encoder_twist();

            rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _left_encoder_raw_sub;
            rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _right_encoder_raw_sub;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _sim_status_sub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sim_time_sub;

            rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _encoder_twist_pub;

            rclcpp::TimerBase::SharedPtr _encoder_timer;

            int32_t _sim_status;
            float_t _sim_time;

            float_t _left_period{0};
            float_t _left_last_read_time{0};
            float_t _right_period{0};
            float_t _right_last_read_time{0};

            float_t _radius{0.025};
            float_t _wheel_base{0.10};
    };
}

#endif // ODOMETRY__ENCODER_H