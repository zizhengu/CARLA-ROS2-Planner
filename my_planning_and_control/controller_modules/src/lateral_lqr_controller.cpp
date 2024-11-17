# include "lateral_lqr_controller.h"

LaterLQRController::LaterLQRController(){

    //车身静止参数初始化
    _cf = -155494.663;
    _cr = -155494.663;
    _m = 1845.0;
    _a = 2.852 / 2.0;
    _b = 2.852 / 2.0;
    _Iz = _a * _a * (_m / 2.0) + _b * _b * (_m / 2.0);
    _steer_ratio = 16.0;

    //LQR矩阵初始化
    _matrix_size = 4;
    _matrix_A = Eigen::MatrixXd::Zero(_matrix_size, _matrix_size);
    _matrix_A_bar = Eigen::MatrixXd::Zero(_matrix_size, _matrix_size);
    _matrix_B = Eigen::MatrixXd::Zero(_matrix_size, 1);
    _matrix_B_bar = Eigen::MatrixXd::Zero(_matrix_size, 1);
    _matrix_Q = Eigen::MatrixXd::Identity(_matrix_size, _matrix_size); // 误差权重矩阵 _matrix_err.T * _matrix_Q * _matrix_err
    _matrix_R = Eigen::MatrixXd::Identity(1, 1);                       // 输入矩阵 _u.T * _matrix_R * _u
    _matrix_K = Eigen::MatrixXd::Zero(1, _matrix_size);
    _matrix_err = Eigen::VectorXd::Zero(_matrix_size); //4*1的向量
    _iter_max = 1500;
    _tolerance = 0.01;

    // LQR矩阵初始化赋值，见讲义
    _matrix_B(1, 0) = -_cf / _m; // 在 Eigen 库中，不可以使用数组下标（[]）直接访问和赋值矩阵的元素
    _matrix_B(3, 0) = -_cf * _a / _Iz;

    _matrix_Q(0, 0) = 5;   // 横向距离误差权重
    _matrix_Q(1, 1) = 1.5; // 横向距离误差导数权重
    _matrix_Q(2, 2) = 10;  // 横向角度误差权重
    _matrix_Q(3, 3) = 5;   // 横向角度误差导数权重

    _matrix_R(0, 0) = 4; // 输入权重
}

double LaterLQRController::run_step(const TrajectoryPoint &match_point, const VehicleState &current_ego_state, const double &control_time_step){
    auto LOG = rclcpp::get_logger("lateral_lqr_controller");

    //1.根据车身速度初始化状态矩阵A
    _vx = current_ego_state.v + 1e-4;
    _matrix_A(0, 1) = 1.0;
    _matrix_A(1, 1) = (_cf + _cr) / (_m * _vx);
    _matrix_A(1, 2) = -(_cf + _cr) / _m;
    _matrix_A(1, 3) = (_a * _cf - _b * _cr) / (_m * _vx);
    _matrix_A(2, 3) = 1.0;
    _matrix_A(3, 1) = (_a * _cf - _b * _cr) / (_Iz * _vx);
    _matrix_A(3, 2) = -(_a * _cf - _b *_cr) / _Iz;
    _matrix_A(3, 3) = (_a * _a * _cf + _b * _b * _cr) / (_Iz * _vx);

    //2.离散化，采用向前离散方法
    auto I = Eigen::MatrixXd::Identity(_matrix_size, _matrix_size);
    _matrix_A_bar = (I + _matrix_A * control_time_step);
    _matrix_B_bar = _matrix_B * control_time_step;

    //3.通过离散LQR求解反馈矩阵
    if (! SolveLQRFeedback(_matrix_A_bar, _matrix_B_bar, _matrix_Q, _matrix_R, _iter_max, _tolerance)){
        RCLCPP_ERROR(LOG, "反馈矩阵k求解失败");
        _matrix_K = Eigen::MatrixXd::Zero(1, _matrix_size);
    }

    //4.计算误差
    Eigen::Vector2d nor_match(-std::sin(match_point.heading), std::cos(match_point.heading));
    Eigen::Vector2d tau_match(std::cos(match_point.heading), std::sin(match_point.heading));
    Eigen::Vector2d match_to_ego(current_ego_state.x - match_point.x, current_ego_state.y - match_point.y);

    double err_d = match_to_ego.transpose() * nor_match;
    //double err_s = match_to_ego.transpose() * tau_match;
    //double theta_r = match_point.heading + match_point.kappa * err_s;
    double err_phi = current_ego_state.heading - match_point.heading; // 代码原作者做法
    // 修正角度误差取值区间(-pi,pi)
    if (err_phi > M_PI){
        err_phi = err_phi - 2 * M_PI;
    }
    else if (err_phi < -M_PI){
        err_phi = err_phi + 2 * M_PI;
    }
    // 为了避免角度多值问题，可以使用sin近似来避免
    err_phi = std::sin(err_phi);

    //double s_dot = current_ego_state.v * std::cos(err_phi) / (1 - match_point.kappa * err_d);
    //double err_phi_dot = current_ego_state.heading - match_point.kappa * s_dot;  up做法
    double err_phi_dot = current_ego_state.omega - match_point.kappa * match_point.v; // 代码原作者做法
    double err_d_dot = current_ego_state.v * std::sin(err_phi);

    _matrix_err[0] = err_d;
    _matrix_err[1] = err_d_dot;
    _matrix_err[2] = err_phi;
    _matrix_err[3] = err_phi_dot;

    //5.计算前馈
    double k3 = _matrix_K(0, 2);
    double delta_f = match_point.kappa * (_a + _b - _b * k3 - (_m * _vx * _vx / (_a + _b)) * (_b / _cf + _a * k3 / _cr - _a / _cr));

    //6.计算控制量u,即前轮转角
    double u = -(_matrix_K * _matrix_err)[0] + delta_f;
    double max_u = 20.0 * M_PI / 180.0;
    u = std::min(std::max(u, -max_u), max_u);

    //7.计算方向盘转角
    double steer = -u; // 前轮转角计算出负值应该是右转，所以有一个负号。

    return std::min(std::max(steer, -1.0), 1.0);
}

bool LaterLQRController::SolveLQRFeedback(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                                        const int &iter_max, const double &tolerance)
{
    auto LOG = rclcpp::get_logger("lateral_lqr_controller");
    //判断输入矩阵是否正确
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.cols() != B.cols())
    {
        RCLCPP_INFO(LOG, "输入矩阵维数不匹配");
        return false;
    }

    //初始化P
    auto matrix_size = A.rows();
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();

    // 迭代解算黎卡提方程
    for (int i = 0; i < iter_max; i++){
        P_next = Q + AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A;
        if (std::fabs((P_next-P).maxCoeff()) < tolerance){
            _matrix_K = (R + BT * P * B).inverse() * (BT * P * A);
            return true;
        }
        P = P_next;
    }
    return false;
}