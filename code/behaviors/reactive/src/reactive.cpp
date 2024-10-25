#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

#include "reactive/reactive.hpp"

using namespace tcc;

Reactive::Reactive() : Node("reactive")
{
    bool use_sim_param = this->declare_parameter<bool>("use_sim_param", true);

    this->_left_ultrasonic_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/sim/ultrasonic/left", 10, 
        std::bind(&Reactive::left_ultrasonic_callback, this, std::placeholders::_1)
    );
    this->_right_ultrasonic_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/sim/ultrasonic/right", 10, 
        std::bind(&Reactive::right_ultrasonic_callback, this, std::placeholders::_1)
    );
    this->_front_ultrasonic_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/sim/ultrasonic/front", 10, 
        std::bind(&Reactive::front_ultrasonic_callback, this, std::placeholders::_1)
    );

    std::string cmd_vel_topic = use_sim_param ? "/sim/cmd_vel" : "/cmd_vel";
    this->_cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    this->_behavior_trigger_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Reactive::do_behavior, this)
    );

}

Reactive::~Reactive()
{
}

void Reactive::left_ultrasonic_callback(const std_msgs::msg::Float32 & msg)
{
    this->_distance_array[ultrasonic_id_e::US_LEFT] = msg.data;
}

void Reactive::right_ultrasonic_callback(const std_msgs::msg::Float32 & msg)
{
    this->_distance_array[ultrasonic_id_e::US_RIGHT] = msg.data;
}

void Reactive::front_ultrasonic_callback(const std_msgs::msg::Float32 & msg)
{
    this->_distance_array[ultrasonic_id_e::US_FRONT] = msg.data;
}

bool change_direction(double distance, double threshold, double min_distance_threshold)
{
    return min_distance_threshold < distance && threshold > distance;
}

bool Reactive::turn(uint8_t angle)
{
    // Tempo estimado para rotação de 180 com velocidade angular de ~ 0.28
    float a = 5.0 / 180;
    float time = angle * a * 1000;

    // Evita o uso de função sleep(), que é blockante  
    int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - this->_behavior_trigger_time).count();
    
    return duration >= time;
}

void Reactive::do_behavior()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    if (behavior_e::B_STOP == this->_behavior_state) 
    {
        RCLCPP_INFO(this->get_logger(), "Behavior: STOP");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        rclcpp::sleep_for(std::chrono::milliseconds(3000));
        this->_behavior_state = behavior_e::B_FORWARD;
    }
    else if (behavior_e::B_FORWARD == this->_behavior_state)
    {
        RCLCPP_INFO(this->get_logger(), "Behavior: FORWARD - FRONT: %.2f | LEFT: %.2f | RIGHT: %.2f", 
            this->_distance_array[ultrasonic_id_e::US_FRONT], this->_distance_array[ultrasonic_id_e::US_LEFT],
            this->_distance_array[ultrasonic_id_e::US_RIGHT]);
        cmd_vel.linear.x = _max_linear_velocity;
        cmd_vel.angular.z = 0.0;

        // Verifica se tem obstáculo na frente (>0.01 e <0.5)
        if (change_direction(_distance_array[US_FRONT], _front_distance_threshold, _min_distance_threshold))
        {   
            // Escolhe a direção com maior distância

            // Caso ambos os sensores possuem leitura, e o da esqueda for maior que o da direita
            if (_min_distance_threshold < _distance_array[US_RIGHT] 
                && _distance_array[US_LEFT] > _distance_array[US_RIGHT])
            {
                this->_behavior_state = behavior_e::B_LEFT;
            }
            // Caso apenas o sensor da esquerda não possua leitura valida (provavelmente mais espaço livre)
            else if (_min_distance_threshold > _distance_array[US_LEFT]
                && _distance_array[US_RIGHT] > _distance_array[US_LEFT])
            {
                this->_behavior_state = behavior_e::B_LEFT;
            }
            else
            {
                this->_behavior_state = behavior_e::B_RIGHT;
            }
            
            // Registra tempo de quando o comportamento foi chamado
            this->_behavior_trigger_time = std::chrono::system_clock::now();
        }
    }
    else if (behavior_e::B_LEFT == this->_behavior_state)
    {
        RCLCPP_INFO(this->get_logger(), "Behavior: LEFT - FRONT: %.2f | LEFT: %.2f | RIGHT: %.2f", 
            this->_distance_array[ultrasonic_id_e::US_FRONT], this->_distance_array[ultrasonic_id_e::US_LEFT],
            this->_distance_array[ultrasonic_id_e::US_RIGHT]);
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = _max_angular_velocity;

        // Mantem a rotação até atingir 40, usando o tempo.
        if (turn(70))
        {
            this->_behavior_state = behavior_e::B_FORWARD;
        }
    }
    else if (behavior_e::B_RIGHT == this->_behavior_state)
    {
        RCLCPP_INFO(this->get_logger(), "Behavior: RIGHT - FRONT: %.2f | LEFT: %.2f | RIGHT: %.2f", 
            this->_distance_array[ultrasonic_id_e::US_FRONT], this->_distance_array[ultrasonic_id_e::US_LEFT],
            this->_distance_array[ultrasonic_id_e::US_RIGHT]);
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = -_max_angular_velocity;

        // Mantem a rotação até atingir 40, usando o tempo.
        if (turn(70))
        {
            this->_behavior_state = behavior_e::B_FORWARD;
        }
    }

    this->_cmd_vel_pub->publish(cmd_vel);

}

int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Reactive>());
    rclcpp::shutdown();

    return 0;
}