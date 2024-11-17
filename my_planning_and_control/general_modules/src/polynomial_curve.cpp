#include "polynomial_curve.h"

void PolynomialCurve::curve_fitting(const double& start_s, const double& start_l, const double& start_l_prime, const double& start_l_prime_prime,
                                    const double& end_s, const double& end_l, const double& end_l_prime, const double& end_l_prime_prime){
    /**
     * 五次多项式
     * l = f(s) = a_0 + a_1 s + a_2 s^2 + a_3 s^3 + a_4 s^4 + a_5 s^5
     * 其中 s 是自变量，代表在某个范围内的插值点。通过选取合适的系数，可以确保多项式在给定的起始和结束条件下平滑过渡。
     */
    // auto A = Eigen::MatrixXd::Zero(6,6);
    // auto B = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd A;
    Eigen::VectorXd B;
    A.resize(6, 6);
    B.resize(6);
    double correct_l_prime_prime = start_l_prime_prime;
    // double correct_l_prime_prime = std::min(end_l_prime_prime, 10.0);

    // 使用初始化列表赋值
    A << 1, start_s,  std::pow(start_s,2),   std::pow(start_s,3),   std::pow(start_s,4),   std::pow(start_s,5),
         0,   1,    2*std::pow(start_s,1), 3*std::pow(start_s,2), 4*std::pow(start_s,3), 5*std::pow(start_s,4),
         0,   0,        2,                 6*std::pow(start_s,1),12*std::pow(start_s,2),20*std::pow(start_s,3),
         1, end_s,    std::pow(end_s,2),     std::pow(end_s,3),     std::pow(end_s,4),     std::pow(end_s,5),
         0,   1,    2*std::pow(end_s,1),   3*std::pow(end_s,2),   4*std::pow(end_s,3),   5*std::pow(end_s,4),
         0,   0,        2,                 6*std::pow(end_s,1),  12*std::pow(end_s,2),  20*std::pow(end_s,3);

    B << start_l, start_l_prime, correct_l_prime_prime, end_l, end_l_prime, end_l_prime_prime;

    // 全主元 LU 分解器(fullPivLu())
    _coefficients = A.fullPivLu().solve(B);

    // auto LOG = rclcpp::get_logger("curve_fitting");
    // RCLCPP_INFO(LOG, "_coefficients: (%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)", _coefficients[0], _coefficients[1], _coefficients[2], _coefficients[3], _coefficients[4], _coefficients[5]);
}

double PolynomialCurve::value_evaluation(const double &s, const int &order)
{
    // auto phi_s = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd phi_s;
    phi_s.resize(6);

    switch (order)
    {
    case 0: // 0阶导数，就是原函数求值
        phi_s << 1, s, std::pow(s, 2), std::pow(s, 3), std::pow(s, 4), std::pow(s, 5);
        break;

    case 1:
        phi_s << 0, 1, 2 * std::pow(s, 1), 3 * std::pow(s, 2), 4 * std::pow(s, 3), 5 * std::pow(s, 4);
        break;

    case 2:
        phi_s << 0, 0, 2, 6 * std::pow(s, 1), 12 * std::pow(s, 2), 20 * std::pow(s, 3);
        break;

    case 3:
        phi_s << 0, 0, 0, 6, 24 * s, 60 * std::pow(s, 2);
        break;

    default:
        break;
    }

    return phi_s.dot(_coefficients);
}
