#pragma once
#include "Eigen/Dense"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
class PolynomialCurve
{
public:
    PolynomialCurve() = default;

    void curve_fitting(const double &start_s, const double &start_l, const double &start_l_prime, const double &start_l_prime_prime,
                       const double &end_s, const double &end_l, const double &end_l_prime, const double &end_l_prime_prime);

    double value_evaluation(const double &s, const int &order = 0);

private:
    Eigen::VectorXd _coefficients;
};

