#ifndef BEHAVIOR_REACTIVE__H
#define BEHAVIOR_REACTIVE__H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

namespace tcc {
    typedef enum {
        US_LEFT = 0,
        US_RIGHT,
        US_FRONT
    } ultrasonic_id_e;

    typedef enum {
        B_STOP = 0,
        B_FORWARD,
        B_LEFT,
        B_RIGHT
    } behavior_e;

    class Reactive : public rclcpp::Node
    {
        public:
            Reactive();
            ~Reactive();

        private:
            void left_ultrasonic_callback(const std_msgs::msg::Float32 & msg);
            void right_ultrasonic_callback(const std_msgs::msg::Float32 & msg);
            void front_ultrasonic_callback(const std_msgs::msg::Float32 & msg);
            bool turn(uint8_t angle);
            
            void do_behavior();

            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _left_ultrasonic_sub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _right_ultrasonic_sub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _front_ultrasonic_sub;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

            rclcpp::TimerBase::SharedPtr _behavior_trigger_timer;
            std::chrono::system_clock::time_point _behavior_trigger_time;

            std::vector<double> _distance_array = {0.0, 0.0, 0.0};

            behavior_e _behavior_state = behavior_e::B_STOP;

            // TODO: Turn the following variables into parameters
            double _left_distance_threshold = 0.5;
            double _right_distance_threshold = 0.5;
            double _front_distance_threshold = 0.5;

            double _min_distance_threshold = 0.01;

            double _max_linear_velocity = 0.07;
            double _max_angular_velocity = 0.28;
    };
}

#endif // BEHAVIOR_REACTIVE__H