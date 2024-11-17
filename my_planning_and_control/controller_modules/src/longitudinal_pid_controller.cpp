#include "longitudinal_pid_controller.h"

LongitudinalPIDController::LongitudinalPIDController(){
    _args_longitudinal.emplace("K_P", 3);
    _args_longitudinal.emplace("K_I", 0.0);
    _args_longitudinal.emplace("K_D", 0.8);

    _longitudinal_error_proportional = 0.0;
    _longitudinal_error_integral = 0.0;
    _longitudinal_error_derivative = 0.0;
    _longitudinal_previous_error_proportional = 0.0;
}

// double LongitudinalPIDController::run_step(const double &target_speed, const double &current_speed){
//     double K_P = _args_longitudinal.at("K_P");
//     double K_I = _args_longitudinal.at("K_I");
//     double K_D = _args_longitudinal.at("K_D");
//     _longitudinal_previous_error_proportional = _longitudinal_error_proportional;
//     _longitudinal_error_proportional = target_speed - current_speed;
//     _longitudinal_error_integral = std::min(std::max(_longitudinal_error_integral + _longitudinal_error_proportional, -40.0), 40.0);
//     _longitudinal_error_derivative = _longitudinal_error_proportional - _longitudinal_previous_error_proportional;

//     double throttle = std::min(std::max(K_P * _longitudinal_error_proportional + K_I * _longitudinal_error_integral + K_D * _longitudinal_error_derivative, 0.0), 1.0);

//     RCLCPP_INFO(rclcpp::get_logger("longitudinal_pid_controller"), "纵向PID数据:PID参数(%.2f,%.2f,%.2f),各项误差(%.2f,%.2f,%.2f),期望速度%.2f,实际速度%.2f,控制指令throttle:%.2f", K_P, K_I, K_D, _longitudinal_error_proportional, _longitudinal_error_integral, _longitudinal_error_derivative, target_speed, current_speed, throttle);

//     return throttle;
// }

double LongitudinalPIDController::run_step_throttle(const double &target_speed, const double &current_speed){
    double K_P = _args_longitudinal.at("K_P");
    double K_I = _args_longitudinal.at("K_I");
    double K_D = _args_longitudinal.at("K_D");
    _longitudinal_previous_error_proportional = _longitudinal_error_proportional;
    _longitudinal_error_proportional = target_speed - current_speed;
    _longitudinal_error_integral = std::min(std::max(_longitudinal_error_integral + _longitudinal_error_proportional, -40.0), 40.0);
    _longitudinal_error_derivative = _longitudinal_error_proportional - _longitudinal_previous_error_proportional;

    double contor_signal = K_P * _longitudinal_error_proportional + K_I * _longitudinal_error_integral + K_D * _longitudinal_error_derivative;

    double throttle = std::min(std::max(contor_signal, 0.0), 1.0);

    RCLCPP_INFO(rclcpp::get_logger("longitudinal_pid"), "contor_signal(%.2f),期望速度%.2f,实际速度%.2f,控制指令throttle:%.2f", contor_signal, target_speed, current_speed, throttle);

    return throttle;
}

double LongitudinalPIDController::run_step_brake(const double &target_speed, const double &current_speed)
{
    double K_P = _args_longitudinal.at("K_P");
    double K_I = _args_longitudinal.at("K_I");
    double K_D = _args_longitudinal.at("K_D");
    _longitudinal_previous_error_proportional = _longitudinal_error_proportional;
    _longitudinal_error_proportional = target_speed - current_speed;
    _longitudinal_error_integral = std::min(std::max(_longitudinal_error_integral + _longitudinal_error_proportional, -40.0), 40.0);
    _longitudinal_error_derivative = _longitudinal_error_proportional - _longitudinal_previous_error_proportional;

    double contor_signal = K_P * _longitudinal_error_proportional + K_I * _longitudinal_error_integral + K_D * _longitudinal_error_derivative;
    double brake;
    // TODO:待调试
    if (std::abs(contor_signal) >= 2.5){
        brake = std::min(std::abs(contor_signal)/5, 1.0);
    }
    else{
        brake = 0.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("longitudinal_pid"), "contor_signal(%.2f),期望速度%.2f,实际速度%.2f,控制指令brake:%.2f", contor_signal, target_speed, current_speed, brake);

    return brake;
}