#pragma once

#include <vector>
#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "derived_object_msgs/msg/object.hpp"

#include "tf2_eigen/tf2_eigen.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//轨迹点，包含速度
struct TrajectoryPoint{
    //初始化列表。
    TrajectoryPoint() : x(0.0), y(0.0), heading(0.0), kappa(0.0), v(0.0),
                        ax(0.0), ay(0.0), a_tau(0.0), time_stamped(0.0) {};
    double x;
    double y;
    double heading;
    double kappa;
    double v;
    double ax;
    double ay;
    double a_tau; // 切向加速度
    double time_stamped;
};

//路径点，不包含速度
struct PathPoint{
    PathPoint() : x(0.0), y(0.0), heading(0.0), kappa(0.0) {};
    double x;
    double y;
    double heading;
    double kappa;
};

//Frenet坐标系,共八个坐标未知量待求，详情见决策规划讲义第一章第三讲
struct FrenetPoint{
    FrenetPoint() : s(0.0), l(0.0), s_dot(0.0), l_dot(0.0), l_prime(0.0),
                    s_dot_dot(0.0), l_dot_dot(0.0), l_prime_prime(0.0) {};
    double s;
    double l;
    double s_dot;
    double l_dot;
    double l_prime; //l_prime =  dl/ds
    double s_dot_dot;
    double l_dot_dot;
    double l_prime_prime;
};

struct STPoint{
    STPoint() : t(0.0), s(0.0), s_dot(0.0), s_dot_dot(0.0) {};
    double t, s, s_dot, s_dot_dot;

    // 常量引用能够提高效率并避免不必要的内存开销,内存管理。因此，在需要传递对象但又不希望修改它们时，使用常量引用是一个良好的选择
    bool operator == (const STPoint &other)
    {
        return (t == other.t) && (s == other.s) && (s_dot == other.s_dot) && (s_dot_dot == other.s_dot_dot);
    }
};

struct VehicleState{
    VehicleState() : x(0.0), y(0.0), z(0.0), heading(0.0), v(0.0), ax(0.0), ay(0.0),
                     omega(0.0), alpha(0.0), time_stamp(0.0), id(0),
                     flag_imu(false), flag_ode(false), flag_info(false) {};
    double x, y, z, heading, v, ax, ay, omega, alpha, time_stamp;
    uint32_t id;
    bool flag_imu, flag_ode, flag_info;
};

struct ControlCMD
{
    ControlCMD() : steer(0.0), throttle(0.0), brake(0.0) {}
    double steer;
    double throttle;
    double brake;
};
// 计算路径的偏航角与曲率
/**
 * 这是一个值传递（pass-by-value）的参数。
   函数接收到的是 path 的一个拷贝，因此在函数内部对 path 的任何修改不会影响外部的原始向量。
   如果 path 较大，拷贝的开销可能会比较高，因为需要将整个向量的数据复制到新的内存位置。
 */
// void Calculate_heading_and_kappa(std::vector<PathPoint> path);

/**
 * 这是一个智能指针类型（shared pointer）的参数。
    path 是一个指向 std::vector<PathPoint> 的共享智能指针，允许多个指针共享同一块内存。
    函数内部可以修改原始向量的内容，因为它操作的是智能指针所指向的对象。
    使用智能指针还可以避免手动管理内存（如使用 new 和 delete），并能自动处理引用计数，确保当最后一个使用该指针的对象被销毁时，内存能够被释放。
 */
void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path);

//计算投影点
/**
 * path 待被投影路径
 * point 待投影点
 * match_point_index path中的匹配影点index
 * project_point 投影点
 */
void Calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const Eigen::Vector2d point,
                                int &match_point_index, PathPoint &project_point);
// 针对trajectory_point进行重载
void Calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const TrajectoryPoint &trajectory_point,
                                int &match_point_index, PathPoint &match_point, PathPoint &project_point);

//计算index2s表
std::vector<double> calculate_index_to_s(std::shared_ptr<std::vector<PathPoint>> path, std::shared_ptr<VehicleState> vehicle_state);

//----------笛卡尔坐标到Frenet坐标转换----------
FrenetPoint cartesion_to_frenet(const TrajectoryPoint &host, const PathPoint &projection_point, const PathPoint &match_point,
                                std::vector<double> index2s, const int &math_point_index); // index2s是否需要修改，不修改应该定义为const
/**
 * const 引用用于输入参数，表示这些参数不会被修改并且避免拷贝。
    shared_ptr 用于需要共享或动态管理内存的对象。
    普通引用用于输出参数，表示函数将会修改这个变量的内容。
 */
// 将障碍物投影到trajectory的frenet坐标系上
void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object> &object_set,
                                 const std::vector<TrajectoryPoint> &trajectory, const TrajectoryPoint &original_point,
                                 std::vector<FrenetPoint> &frenet_set);
// 将障碍物投影到path的frenet坐标系上
void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object> &object_set,
                                 const std::vector<PathPoint> &path, std::shared_ptr<VehicleState> vehicle_state,
                                 std::vector<FrenetPoint> &frenet_set);
// 将trajectory转换到frenet坐标系上
/**
 * trajectory_point : planning_start_point
 * path: reference_line
 */
void cartesion_set_to_frenet_set(const TrajectoryPoint &trajectory_point, const std::vector<PathPoint> &path, 
                                 std::shared_ptr<VehicleState> vehicle_state,
                                 std::vector<FrenetPoint> &frenet_set);

void cartesion_set_to_frenet_set(const std::shared_ptr<VehicleState> &ego_state,
                                 const std::vector<TrajectoryPoint> &trajectory,
                                 std::vector<FrenetPoint> &frenet_set);
//----------Frenet坐标到Cartesion坐标转换----------
/**
 * 使用 const 修饰 std::shared_ptr 是为了防止函数内部改变指针指向的对象（即不允许 cartesian_path 指向其他对象）
 * 但仍然允许对指向的对象进行修改
 * (*cartesian_path)[0] = some_new_value; // 这是允许的
 */
std::vector<TrajectoryPoint> frenet_to_cartesion(const std::vector<FrenetPoint> &frenet_point_set,
                                                 const std::shared_ptr<std::vector<PathPoint>> cartesian_path,
                                                 const std::vector<double> cartesian_path_index2s);

//object转换为轨迹点
TrajectoryPoint object_to_trajectory_point(const derived_object_msgs::msg::Object object);
//车辆状态转换为轨迹点
TrajectoryPoint vehicle_state_to_trajectory_point(const std::shared_ptr<VehicleState> vehicle_state);
