#pragma once

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "common.h"


class LaterLQRController{
public:
    LaterLQRController();

    double run_step(const TrajectoryPoint &match_point, const VehicleState &current_ego_state, const double &control_time_step);

    bool SolveLQRFeedback(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                          const int &iter_max, const double &tolerance);

private:
    //车辆参数
    double _cf, _cr; //前后轮侧偏刚度系数
    double _a, _b; //质心到车辆前后轴距离
    double _m, _Iz; //质量和转动惯量
    double _steer_ratio; //方向盘转角到轮胎转交之间的比值系数
    double _vx; //沿车身轴线的速度

    //LQR参数
    int _matrix_size;
    Eigen::MatrixXd _matrix_A, _matrix_B; //状态矩阵和输入矩阵
    Eigen::MatrixXd _matrix_A_bar, _matrix_B_bar; // 离散化的状态矩阵和输入矩阵
    Eigen::MatrixXd _matrix_Q, _matrix_R; // 目标函数矩阵
    Eigen::MatrixXd _matrix_K; // 反馈矩阵
    Eigen::VectorXd _matrix_err; // 误差向量
    int _iter_max;
    double _tolerance;
};