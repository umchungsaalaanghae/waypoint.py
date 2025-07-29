#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

double calculate_bearing(double lat1, double lon1, double lat2, double lon2);
double normalize_angle(double angle);

class PixhawkAutoBoat : public rclcpp::Node {
public:
    PixhawkAutoBoat() : Node("pixhawk_autoboat"),
        kp_(1.0), ki_(0.0), kd_(0.0),
        dt_(0.1), saturation_(1.0), standard_pwm_(1500.0),
        dead_min_(1464.0), dead_max_(1536.0),
        rc_mode_(2.0) // 기본: AUTO
    {

        rc_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/mavros/rc/override", 10);

        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        rc_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/rc_data", 10, std::bind(&PixhawkAutoBoat::rc_callback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", 10, std::bind(&PixhawkAutoBoat::gps_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", 10, std::bind(&PixhawkAutoBoat::imu_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&PixhawkAutoBoat::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚤 Pixhawk 자율 선박 제어 노드 시작됨");
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rc_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
   
    double current_lat_ = 0.0, current_lon_ = 0.0;
    double target_lat_ = 37.123456;
    double target_lon_ = 127.123456;
    double current_psi_ = 0.0;


    // 제어 파라미터
    double kp_, ki_, kd_, dt_, saturation_;
    double standard_pwm_, dead_min_, dead_max_;
    double rc_mode_; // 1 = RC, 2 = AUTO

    // PID 내부 상태
    double integral_ = 0.0;
    double prev_error_ = 0.0;

    void rc_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        rc_mode_ = msg->data;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        const auto& q = msg->orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_psi_ = std::atan2(siny_cosp, cosy_cosp);  // yaw = ψ
    }

    void control_loop() {
        double left_pwm = 1500.0;
        double right_pwm = 1500.0;
        double rudder_pwm = 1500.0;
        double mode_switch_pwm = 1500.0;

        if (rc_mode_ == 1.0) {
            RCLCPP_INFO_ONCE(this->get_logger(), "MODE: RC");
        } else if (rc_mode_ == 2.0) {
            RCLCPP_INFO_ONCE(this->get_logger(), "MODE: AUTO");

            // 예시: 현재 값과 목표 값 (실제 상황에서는 topic에서 받아와야 함)
            double desired_psi = calculate_bearing(current_lat_, current_lon_, target_lat_, target_lon_);
            double current_psi = current_psi_;
            double error_psi = normalize_angle(desired_psi - current_psi);
            double pwm_adjust = compute_pid(error_psi);

            left_pwm  = process_pwm(standard_pwm_ - pwm_adjust);
            right_pwm = process_pwm(standard_pwm_ + pwm_adjust);
            rudder_pwm = 1500;  // 기본값이지만 추후 조향값 반영 가능
            mode_switch_pwm = 1200; // RC 스위치가 AUTO 모드를 가리키게 설정
        
            if (mode_client_->wait_for_service(1s)) {
                auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                request->custom_mode = "MANUAL"; // 또는 "OFFBOARD"
                mode_client_->async_send_request(request);
            }
        }

        // RC override 퍼블리시
        std_msgs::msg::UInt16MultiArray msg;
        msg.data.resize(8, 0);
        msg.data[0] = left_pwm;
        msg.data[1] = mode_switch_pwm;
        msg.data[2] = rudder_pwm;
        msg.data[3] = right_pwm;  
        rc_pub_->publish(msg);
    }

    // PID 계산
    double compute_pid(double error) {
        integral_ += error * dt_;
        double derivative = (error - prev_error_) / dt_;
        prev_error_ = error;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 포화 제한
        if (output > saturation_) output = saturation_;
        if (output < -saturation_) output = -saturation_;
        return output;
    }

    // PWM 제한 및 deadzone 처리
    uint16_t process_pwm(double input) {
        if (input < 1100) input = 1100;
        if (input > 1900) input = 1900;
        if (input < dead_max_ && input > dead_min_) {
            input = standard_pwm_;
        }
        return static_cast<uint16_t>(input);
    }
};

double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 *= M_PI / 180.0;
    lon1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;

    double dlon = lon2 - lon1;
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    return atan2(y, x);  // 라디안 (−π ~ π)
}

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixhawkAutoBoat>());
    rclcpp::shutdown();
    return 0;
}
