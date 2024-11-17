#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "common.h"
#include "polynomial_curve.h"
#include "derived_object_msgs/msg/object.hpp"
#include "carla_waypoint_types/srv/get_waypoint.hpp"
#include "std_msgs/msg/float64.hpp"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include "matplot/matplot.h"

struct DpPathNode{
    /**
     * 为什么要有两个构造函数：
        灵活性：允许用户根据需要选择是否提供初始化参数。这样可以在不同的上下文中使用 DpPathNode。
        简化代码：在某些情况下，您可能只需要一个简单的、未初始化的对象，而在其他情况下，可能需要详细的初始化。
     * 在 C++ 中，结构体和类的定义是可以自我引用的。这意味着您可以在构造函数中使用结构体自身作为参数类型，尽管该结构体的定义尚未完全解析
     */
    DpPathNode();
    DpPathNode(const FrenetPoint &args_sl_point, const double &args_min_cost, const std::shared_ptr<DpPathNode> args_pre_node);
    FrenetPoint sl_point;
    double min_cost;
    std::shared_ptr<DpPathNode> pre_node;
};

struct DpSpeedNode{
    DpSpeedNode();
    DpSpeedNode(const STPoint &args_st_point, const double &args_min_cost, const std::shared_ptr<DpSpeedNode> args_pre_node);
    STPoint st_point;
    double min_cost;
    std::shared_ptr<DpSpeedNode> pre_node;
};

struct WeightCoefficients{
    //路径动态规划系数
    double path_dp_w_ref;
    double path_dp_w_dl;//平滑代价
    double path_dp_w_ddl;
    double path_dp_w_dddl;
    double path_dp_w_obs;

    //路径二次规划系数
    double path_qp_w_ref;
    double path_qp_w_dl; // 平滑代价
    double path_qp_w_ddl;
    double path_qp_w_dddl;
    double path_qp_w_mid;//凸空间中央代价

    double path_qp_w_l_end; // 末端代价，讲义没有，存疑
    double path_qp_w_dl_end;
    double path_qp_w_ddl_end;

    //速度动态规划系数
    double speed_dp_w_ref_speed;
    double speed_dp_w_a;
    double speed_dp_w_jerk;
    double speed_dp_w_obs;

    // 速度二次规划系数
    double speed_qp_w_ref_speed;
    double speed_qp_w_a;
    double speed_qp_w_jerk;
};

class EMPlanner : public rclcpp::Node{
public:
    EMPlanner();

    //-----------------------EMPlanner主流程--------------------
    // 执行规划
    void planning_run_step(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state,
                           const std::vector<derived_object_msgs::msg::Object> &objects, std::vector<TrajectoryPoint> &trajectory);
    // 区分静态和动态障碍物(源代码ego_state没加const)
    void obstacle_fileter(const std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object> &objects);

    // 确定规划起点(源代码ego_state没加const)
    TrajectoryPoint calculate_planning_start_point(std::shared_ptr<VehicleState> ego_state);

    //-----------------------路径规划--------------------
    //路径二次规划求解器
    OsqpEigen::Solver _path_qp_solver;

    //获取路径规划的采样点
    //为什么有两层vector嵌套？
    void get_path_dp_sample(std::vector<std::vector<FrenetPoint>> &path_dp_sample, const FrenetPoint &planning_start_sl,
                            const double &s_sample_distance, const size_t &s_sample_number,
                            const double &l_sample_distance, const size_t &l_sample_number);

    //计算动态规划代价
    double calculate_dp_cost(const FrenetPoint &start_point, const FrenetPoint &end_point, const std::vector<FrenetPoint> &static_obs_sl_set,
                             const WeightCoefficients &weight_coeff);

    //增密动态规划结果
    void increased_dp_path(const std::vector<FrenetPoint> &init_dp_path, const double &increased_distance, std::vector<FrenetPoint> &final_dp_path);

    //生成凸空间(也就是把路缘、静态障碍物都加上,结果储存在path_l_max 和path_l_min中)
    void generate_convex_space(const std::vector<FrenetPoint> &final_dp_path, const double &road_up_bound, const double &road_low_bound,
                               const std::vector<FrenetPoint> &static_obs_sl_set, std::vector<double> &path_l_max, std::vector<double> &path_l_min);

    // 在路径上寻找与障碍物S坐标最近路径点索引
    int find_index_for_obs_on_path(const std::vector<FrenetPoint> &final_dp_path, const double &static_obs_s);

    //路径QP规划，结果储存在init_qp_path里
    bool path_QP_planning(const std::vector<FrenetPoint> &final_dp_path, const std::vector<double> &final_dp_path_lmin,
                          const std::vector<double> &final_dp_path_lmax, const double &l_desire, const double &dl_desire, const double &ddl_desire,
                          const WeightCoefficients &weight_coeff, std::vector<FrenetPoint> &init_qp_path);

    //-----------------------速度规划--------------------
    OsqpEigen::Solver _speed_qp_solver;

    //生成st图,结果储存在st_graph中，是向量中嵌套哈希表的数据结构
    void generate_st_graph(const std::vector<FrenetPoint> &dynamic_obs_sl_set, const double &delta_l,
                           std::vector<std::unordered_map<std::string, double>> &st_graph);
    // 计算速度规划的起点
    STPoint calculate_speed_start_point(const TrajectoryPoint &planning_start_point);

    //获取采样点,结果储存在 speed_dp_sample中，同样是两层vector嵌套
    void get_speed_dp_sample(const STPoint &planning_start_st, const double &t_sample_distance, const double &t_end,
                             const std::vector<double> &index2s, std::vector<std::vector<STPoint>> &speed_dp_sample);

    //计算速度规划中动态规划的代价
    double calculate_speed_dp_cost(const STPoint &start_point, const STPoint &end_point, const double &reference_speed,
                                   const std::vector<std::unordered_map<std::string, double>> &dynamic_obs_st_graph,
                                   const WeightCoefficients &weight_coefficients);

    // 生成凸空间,加入速度的最大限制，结果储存在 s_lb, s_ub, s_dot_lb, s_dot_ub中
    void generate_convex_space(const std::vector<TrajectoryPoint> &init_trajectory, const std::vector<double> path_index2s,
                               const std::deque<STPoint> &speed_profile, const std::vector<std::unordered_map<std::string, double>> &dynamic_obs_st_graph,
                               const double &ay_max, std::vector<double> &s_lb, std::vector<double> &s_ub, std::vector<double> &s_dot_lb, std::vector<double> &s_dot_ub);

    // 在dp速度剖面上搜索obs的t对应的索引
    int find_t_index(const std::deque<STPoint> &speed_profile, const double &t);

    // 速度二次规划,结果储存在qp_speed_profile中
    bool speed_QP_planning(const std::deque<STPoint> &dp_speed_profile, const std::vector<double> &s_lb, const std::vector<double> &s_ub,
                           const double &reference_speed, const std::vector<double> &s_dot_lb, const std::vector<double> &s_dot_ub,
                           const WeightCoefficients &weight_coeff, std::vector<STPoint> &qp_speed_profile);

    // 增密二次规划所得到的速度剖面
    void increased_speed_profile(const std::vector<STPoint> &init_speed_profile, std::vector<STPoint> &final_speed_profile);

    //--------------------将速度-路径规划结果组装成一条轨迹 trajectory-----------------
    void generate_trajectory(const std::vector<STPoint> &final_speed_profile, const std::vector<TrajectoryPoint> &path_trajectory,
                             const std::vector<double> &path_index2s, const double &planning_start_point_time_stamped,
                             std::vector<TrajectoryPoint> &trajectory);

private:
    std::vector<derived_object_msgs::msg::Object> _static_obstacles;  // 静态障碍物
    std::vector<derived_object_msgs::msg::Object> _dynamic_obstacles; // 动态障碍物
    std::deque<TrajectoryPoint> _previous_trajectory;                 // 上一周期的轨迹
    std::deque<TrajectoryPoint> _switch_trajectory;                   // 拼接轨迹
    // std::vector<double> final_dp_path_l_min;
    // std::vector<double> final_dp_path_l_max;

    rclcpp::Client<carla_waypoint_types::srv::GetWaypoint>::SharedPtr _get_waypoint_client;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _reference_speed_subscriber;
    // void reference_speed_callback(std_msgs::msg::Float64::SharedPtr msg);
    double _reference_speed;
    double _current_time;

    WeightCoefficients _weight_coefficients; // 路径，速度规划的权重系数

    // 绘图
    matplot::figure_handle _path_plot_handle;
    size_t _plot_count;
    matplot::figure_handle _trajectory_plot_handle;
    matplot::figure_handle _STL_plot_handle;
};
