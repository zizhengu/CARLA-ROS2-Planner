#include "emplanner.h"

auto LOG = rclcpp::get_logger("emplanner");

EMPlanner::EMPlanner() : Node("emplanner"){
    /**
     * 这一行创建了一个客户端，用于与名为 /carla_waypoint_publisher/ego_vehicle/get_waypoint 的服务进行通信
     * create_client 是 ROS 2 的方法，用于创建一个服务客户端，允许该节点向指定服务发送请求
     * carla_waypoint_types::srv::GetWaypoint 表示要调用的服务的类型
     * 使用 this 可以清楚地表明 create_client 是当前对象的方法,避免调用的函数名与当前类中定义的成员函数重名的情况
     */
    _get_waypoint_client = this->create_client<carla_waypoint_types::srv::GetWaypoint>(
        "/carla_waypoint_publisher/ego_vehicle/get_waypoint");

    // 路径决策代价初始化
    _weight_coefficients.path_dp_w_ref = 200.0;
    _weight_coefficients.path_dp_w_dl = 300.0;
    _weight_coefficients.path_dp_w_ddl = 200.0;
    _weight_coefficients.path_dp_w_dddl = 1000.0;
    _weight_coefficients.path_dp_w_obs = 2e5;

    // 路径规划代价初始化
    _weight_coefficients.path_qp_w_ref = 100.0;
    _weight_coefficients.path_qp_w_dl = 200.0;
    _weight_coefficients.path_qp_w_ddl = 50.0;
    _weight_coefficients.path_qp_w_dddl = 10.0;
    _weight_coefficients.path_qp_w_mid = 20.0;
    _weight_coefficients.path_qp_w_l_end = 40.0;
    _weight_coefficients.path_qp_w_dl_end = 40.0;
    _weight_coefficients.path_qp_w_ddl_end = 40.0;

    // 速度决策代价初始化
    _weight_coefficients.speed_dp_w_ref_speed = 1000.0;
    _weight_coefficients.speed_dp_w_a = 400.0;
    _weight_coefficients.speed_dp_w_jerk = 400.0;
    // _weight_coefficients.speed_dp_w_obs = 2000;
    _weight_coefficients.speed_dp_w_obs = 1e5;
    // 速度规划代价初始化
    _weight_coefficients.speed_qp_w_ref_speed = 400.0;
    _weight_coefficients.speed_qp_w_a = 200.0;
    _weight_coefficients.speed_qp_w_jerk = 200.0;

    //绘图
    _path_plot_handle = matplot::figure();
    _trajectory_plot_handle = matplot::figure();
    _STL_plot_handle = matplot::figure();

    _plot_count = 0;

    //路径二次规划热启动
    _path_qp_solver.settings()->setWarmStart(true);
    _speed_qp_solver.settings()->setWarmStart(true);
    //TODO:
    //改动这里，看看能不能加速
    _reference_speed = 11.0;
}

// TODO:
// 弄清楚 reference_line、ego_state、obstacles是怎么传进来的
void EMPlanner::planning_run_step(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state,
                       const std::vector<derived_object_msgs::msg::Object> &obstacles, std::vector<TrajectoryPoint> &final_trajectory){
    /*
    planning_run_step 是 EMPlanner 类中的一个成员函数，它可以通过某个 EMPlanner 对象来调用。
    这里的 this 代表 EMPlanner 类的当前实例。当你在成员函数 planning_run_step 中使用 this 时，它指向调用该函数的具体 EMPlanner 对象。
     */
    _current_time = this->now().seconds();

    //-----------------------------------1.障碍物处理--------------------------------------
    // 1.1区别动态与静态障碍物，将其储存在成员变量 std::vector<derived_object_msgs::msg::Object> _static_/dynamic_obstacles中
    obstacle_fileter(ego_state, obstacles);

    // 1.2障碍物坐标转换
    //std::vector<double> reference_index2s,储存车辆投影点到reference_line中每个点的距离
    auto reference_index2s = calculate_index_to_s(reference_line, ego_state);
    std::vector<FrenetPoint> static_obstacles_frent_coords;
    if (!_static_obstacles.empty()){
        cartesion_set_to_frenet_set(_static_obstacles, *reference_line, ego_state, static_obstacles_frent_coords);
    }

    for (auto &&obs_frenet : static_obstacles_frent_coords){
        RCLCPP_INFO(LOG, "静态障碍物信息(%.3f,%.3f)", obs_frenet.s, obs_frenet.l);
    }

    //----------------------------2.确定规划起点并将其投影到frenet坐标系--------------------------
    TrajectoryPoint planning_start_point = calculate_planning_start_point(ego_state);
    std::vector<FrenetPoint> temp_set;
    cartesion_set_to_frenet_set(planning_start_point, *reference_line, ego_state, temp_set);
    FrenetPoint planning_start_point_frenet = temp_set.front();
    planning_start_point_frenet.l_prime_prime = 0.0;

    //-------------------------------------3.路径规划----------------------------------------
    // 3.1获取车道宽度
    // TODO:
    // 结合地图获取
    double lane_width = 4.0;

    // 3.2dp路径决策
    // 3.2.1获取采样点
    //  TODO:
    // 调整L方向的采样个数和采样间隔，查看有无效果
    std::vector<std::vector<FrenetPoint>> path_dp_sample;
    // get_path_dp_sample(path_dp_sample, planning_start_point_frenet, 10.0, 6, 1.0, 7);
    get_path_dp_sample(path_dp_sample, planning_start_point_frenet, 10.0, 8, 1.0, 7);

    // 3.2.2 获取每个采样点的最小代价及其前继点,储存在容器path_dp_node_table中
    // 下面这段代码只是创建了一个空的容器，没有任何内层向量
    std::vector<std::vector<DpPathNode>> path_dp_node_table;
    // 使用 emplace_back() 方法向 path_dp_node_table 中添加一个空的内层向量。这是在动态创建一个新的内层向量，以便后续可以向其中添加 DpPathNode 对象。
    path_dp_node_table.emplace_back();
    // path_dp_node_table.back() 返回 path_dp_node_table 中最后一个内层向量。
    // emplace_back()内的参数是结构体DpPathNode的构造函数的参数，无需直接给予它一整个DpPathNode对象
    path_dp_node_table.back().emplace_back(planning_start_point_frenet, 0.0, nullptr);
    auto path_dp_sample_front = path_dp_sample.front().front();
    for (size_t level = 1; level < path_dp_sample.size(); level++){
        path_dp_node_table.emplace_back();
        if (level == 1){
            for (auto &&current_sample_point : path_dp_sample[level]){
                double min_cost = calculate_dp_cost(path_dp_sample_front, current_sample_point, static_obstacles_frent_coords, _weight_coefficients);
                path_dp_node_table.back().emplace_back(current_sample_point, min_cost, std::make_shared<DpPathNode>(path_dp_node_table.front().front()));
            }
        }
        else{
            for (auto &&current_sample_point : path_dp_sample[level]){
                size_t index;
                double min_cost = std::numeric_limits<double>::max();
                for (size_t i = 0; i < path_dp_node_table[level - 1].size(); i++){
                    DpPathNode previous_node = path_dp_node_table[level - 1][i];
                    double current_cost = previous_node.min_cost + calculate_dp_cost(previous_node.sl_point, current_sample_point,static_obstacles_frent_coords, _weight_coefficients);
                    if (current_cost < min_cost){
                        min_cost = current_cost;
                        index = i;
                    }
                }
                path_dp_node_table.back().emplace_back(current_sample_point, min_cost, std::make_shared<DpPathNode>(path_dp_node_table[level - 1][index]));
            }
        }
    }

    // 3.2.3 利用path_dp_node_table找到最短路径
    size_t path_min_cost_index;
    double min_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_dp_node_table.back().size(); i++){
        if (path_dp_node_table.back()[i].min_cost < min_cost){
            min_cost = path_dp_node_table.back()[i].min_cost;
            path_min_cost_index = i;
        }
    }
    auto path_useful_node = path_dp_node_table.back()[path_min_cost_index];

    std::vector<FrenetPoint> dp_best_path;
    dp_best_path.emplace_back(path_useful_node.sl_point);
    while(true){
        path_useful_node = *(path_useful_node.pre_node);
        dp_best_path.emplace(dp_best_path.begin(), path_useful_node.sl_point);
        if (dp_best_path.front().l == planning_start_point_frenet.l && dp_best_path.front().s == planning_start_point_frenet.s){
            break;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "路径规划——利用DP找到最短路径完成");

    //3.2.4 对规划出的路径进行加密
    std::vector<FrenetPoint> final_dp_path;
    increased_dp_path(dp_best_path, 1.0, final_dp_path);

    // 3.2.5获取凸空间
    // 现在假定静态障碍物都是车，一般化情况应该是根据相应的障碍物类型获得相应的边框
    //  TODO:设置不同障碍物类型以生成凸空间
    std::vector<double> final_dp_path_l_max;
    std::vector<double> final_dp_path_l_min;
    double safe_distance = 0.0;
    double road_ub = 1.5 * lane_width - safe_distance;
    double road_lb = -(0.5 * lane_width - safe_distance);
    generate_convex_space(final_dp_path, road_ub, road_lb, static_obstacles_frent_coords, final_dp_path_l_max, final_dp_path_l_min);
    // RCLCPP_INFO(this->get_logger(), "路径规划——获取凸空间完成");

    // 3.3 QP路径规划
    //  TODO:
    // 为什么都是0.0？
    std::vector<FrenetPoint> init_qp_path;
    double l_desire = 0.0;
    double dl_desire = 0.0;
    double ddl_desire = 0.0;
    path_QP_planning(final_dp_path, final_dp_path_l_min, final_dp_path_l_max, l_desire, dl_desire, ddl_desire, _weight_coefficients, init_qp_path);

    //3.4将路径转换为轨迹
    auto path_trajectory = frenet_to_cartesion(init_qp_path, reference_line, reference_index2s);

    //-------------------------------------4.速度规划----------------------------------------

    //4.1获取路径规划结果的index2s表
    std::vector<double> path_index2s;
    path_index2s.emplace_back(0.0);
    for (size_t i = 1; i < path_trajectory.size(); i++){
        path_index2s.emplace_back(path_index2s.back() + std::hypot(path_trajectory[i].x - path_trajectory[i - 1].x, path_trajectory[i].y - path_trajectory[i - 1].y));
    }

    //4.2动态障碍物投影
    std::vector<FrenetPoint> dynamic_obstacles_frenet_coords;
    if (!_dynamic_obstacles.empty()){
        cartesion_set_to_frenet_set(_dynamic_obstacles, path_trajectory, path_trajectory.front(), dynamic_obstacles_frenet_coords);
    }

    for (auto &&obs_frenet : dynamic_obstacles_frenet_coords){
        RCLCPP_INFO(LOG, "动态障碍物信息(s: %.3f,l: %.3f,s_dot: %.3f,l_dot: %.3f)", obs_frenet.s, obs_frenet.l, obs_frenet.s_dot, obs_frenet.l_dot);
    }

    // 4.3生成ST图，数据结构为向量中嵌套哈希表
    std::vector<std::unordered_map<std::string, double>> dynamic_obs_st_graph;
    if (!dynamic_obstacles_frenet_coords.empty()){
        generate_st_graph(dynamic_obstacles_frenet_coords, 2.0, dynamic_obs_st_graph);
    }
    // for (auto &&st_graph : dynamic_obs_st_graph){
    //     RCLCPP_INFO(LOG, "动态障碍物st_graph(t_in:%.3f, t_out:%.3f, s_in:%.3f, s_out:%.3f)", st_graph["t_in"], st_graph["t_out"], st_graph["s_in"], st_graph["s_out"]);
    // }
    // 4.4计算速度规划起点信息

    STPoint planning_start_point_st_coords = calculate_speed_start_point(planning_start_point);

    // 4.5利用DP进行速度决策
    // 4.5.1 获取采样点
    // 除起点外，其它的STpoint 都只有s，t,无s_dot,s_dot_dot
    std::vector<std::vector<STPoint>> speed_dp_sample;
    get_speed_dp_sample(planning_start_point_st_coords, 0.5, 8.0, path_index2s, speed_dp_sample);

    //4.5.2计算代价
    std::vector<std::vector<DpSpeedNode>> speed_dp_node_table;
    speed_dp_node_table.emplace_back();
    auto speed_dp_sample_front = speed_dp_sample.front().front();
    speed_dp_node_table.back().emplace_back(speed_dp_sample_front, 0.0, nullptr);
    for (size_t level = 1; level < speed_dp_sample.size(); level++){
        if (level == 1){
            speed_dp_node_table.emplace_back();
            for (auto &&sample_point : speed_dp_sample[level]){
                STPoint pre_st_point = speed_dp_sample_front;
                STPoint cur_st_point = sample_point;
                double delta_t = cur_st_point.t - pre_st_point.t;
                cur_st_point.s_dot = (cur_st_point.s - pre_st_point.s) / delta_t;
                cur_st_point.s_dot_dot = (cur_st_point.s_dot - pre_st_point.s_dot) / delta_t;

                double min_cost = calculate_speed_dp_cost(speed_dp_sample_front, sample_point, _reference_speed, dynamic_obs_st_graph, _weight_coefficients);
                speed_dp_node_table.back().emplace_back(cur_st_point, min_cost, std::make_shared<DpSpeedNode>(speed_dp_node_table.front().front()));
            }
        }
        else{
            speed_dp_node_table.emplace_back();
            for (int i = 0; i < (int)speed_dp_sample[level].size(); i++){
                double min_speed_dp_cost = std::numeric_limits<double>::max();
                int pre_index = -1;
                for (int j = 0; j < (int)speed_dp_sample[level - 1].size(); j++)
                {
                    double cur_cost = calculate_speed_dp_cost(speed_dp_node_table[level - 1][j].st_point, speed_dp_sample[level][i], _reference_speed, dynamic_obs_st_graph, _weight_coefficients) + speed_dp_node_table[level - 1][j].min_cost; // 注意，这里面的start_point不能用speed_dp_sample里的点，因为那里面的没有赋值
                    if (cur_cost < min_speed_dp_cost)
                    {
                        min_speed_dp_cost = cur_cost;
                        pre_index = j;
                    }
                }
                STPoint pre_st_point(speed_dp_node_table[level - 1][pre_index].st_point);
                STPoint cur_st_point(speed_dp_sample[level][i]);
                double delat_t = cur_st_point.t - pre_st_point.t;
                cur_st_point.s_dot = (cur_st_point.s - pre_st_point.s) / delat_t;
                cur_st_point.s_dot_dot = (cur_st_point.s_dot - pre_st_point.s_dot) / delat_t;

                speed_dp_node_table.back().emplace_back(
                    cur_st_point,
                    min_speed_dp_cost,
                    std::make_shared<DpSpeedNode>(speed_dp_node_table[level - 1][pre_index]));
            }
        }
    }
    // 4.5.3回溯轨迹
    double trajectory_min_cost = std::numeric_limits<double>::max();
    int index_level = -1;
    int index_row = -1;
    // 搜索上边界
    for (int level = 1; level < (int)speed_dp_node_table.size(); level++){
        if (speed_dp_node_table[level].front().min_cost < trajectory_min_cost){
            trajectory_min_cost = speed_dp_node_table[level].front().min_cost;
            index_level = level;
            index_row = 0;
        }
    }
    // 搜索右边界
    for (int row = 0; row < (int)speed_dp_node_table.back().size(); row++)
    {
        if (speed_dp_node_table.back()[row].min_cost < trajectory_min_cost)
        {
            trajectory_min_cost = speed_dp_node_table.back()[row].min_cost;
            index_level = (int)speed_dp_node_table.size() - 1;
            index_row = row;
        }
    }

    DpSpeedNode trajectory_useful_node(speed_dp_node_table[index_level][index_row]);
    //建立速度剖面，数据结构为双端队列
    std::deque<STPoint> dp_speed_profile;
    dp_speed_profile.emplace_front(trajectory_useful_node.st_point);
    while(true){
        trajectory_useful_node = (*trajectory_useful_node.pre_node);
        dp_speed_profile.emplace_front(trajectory_useful_node.st_point);
        if (trajectory_useful_node.st_point == speed_dp_sample_front){
            break;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "速度规划——速度剖面建立完成");

    // 4.6速度二次规划
    // TODO:解决二次规划速度结果过慢的问题
    // 4.6.1 生成凸空间
    std::vector<double> s_lb, s_ub, s_dot_lb, s_dot_ub;
    // generate_convex_space(path_trajectory, path_index2s, dp_speed_profile, dynamic_obs_st_graph, 0.2 * 9.8, s_lb, s_ub, s_dot_lb, s_dot_ub);
    generate_convex_space(path_trajectory, path_index2s, dp_speed_profile, dynamic_obs_st_graph, 0.5 * 9.8, s_lb, s_ub, s_dot_lb, s_dot_ub);
    // RCLCPP_INFO(this->get_logger(), "速度规划——速度凸空间完成");
    //4.6.2 二次规划
    std::vector<STPoint> init_qp_speed_profile;
    speed_QP_planning(dp_speed_profile, s_lb, s_ub, _reference_speed, s_dot_lb, s_dot_ub, _weight_coefficients, init_qp_speed_profile);
    // RCLCPP_INFO(this->get_logger(), "速度规划——速度二次规划完成");
    //4.6.3 加密二次规划所得到的速度曲线
    std::vector<STPoint> final_qp_speed_profile;
    increased_speed_profile(init_qp_speed_profile, final_qp_speed_profile);
    // RCLCPP_INFO(this->get_logger(), "速度规划——加密速度曲线完成");
    //-------------------------------------5.生成最终轨迹----------------------------------------
    // 5.1将路径规划结果和速度规划结果拼成轨迹
    std::vector<TrajectoryPoint> init_trajectory;
    generate_trajectory(final_qp_speed_profile, path_trajectory, path_index2s, planning_start_point.time_stamped, init_trajectory);

    // 5.2轨迹拼接
    final_trajectory.clear();
    if( !_switch_trajectory.empty()){
        for (auto &&trajectory_point : _switch_trajectory){
            final_trajectory.emplace_back(trajectory_point);
        }
    }
    for (auto &&trajectory_point : init_trajectory){
        final_trajectory.emplace_back(trajectory_point);
    }
    // RCLCPP_INFO(this->get_logger(), "轨迹拼接完成");

    // 5.3上一周期轨迹赋值
    _previous_trajectory.clear();
    for (auto &&trajectory_point: final_trajectory){
        _previous_trajectory.emplace_back(trajectory_point);
    }

    //-------------------------------------6.绘制ST、SL图----------------------------------------
    // 6.1绘制SL图
    if (_plot_count % 5 == 0){
        matplot::figure(_path_plot_handle);
        matplot::cla();
        std::vector<double> init_dp_path_s, init_dp_path_l;
        std::vector<double> final_dp_path_s, final_dp_path_l;
        std::vector<double> init_qp_path_s, init_qp_path_l;
        //储存增密前的DP_PATH
        for (size_t i = 0; i < dp_best_path.size(); i++){
            init_dp_path_s.emplace_back(dp_best_path[i].s);
            init_dp_path_l.emplace_back(dp_best_path[i].l);
        }
        // 储存增密后的DP_PATH及QP_PATH
        for (size_t i = 0; i < final_dp_path.size(); i++){
            final_dp_path_s.emplace_back(final_dp_path[i].s);
            final_dp_path_l.emplace_back(final_dp_path[i].l);
            init_qp_path_s.emplace_back(init_qp_path[i].s);
            init_qp_path_l.emplace_back(init_qp_path[i].l);
        }
        // 增密后的DP_PATH，蓝色
        matplot::plot(final_dp_path_s, final_dp_path_l, "bo-")->line_width(4);

        // 只需调用一次的原因是，它的作用是保持当前图形，使后续的绘图命令不会清除之前的内容。直到你需要开始一个新的图形时，可以调用 matplot::hold(false); 来重置。
        matplot::hold(true);
        // 凸空间
        matplot::plot(final_dp_path_s, final_dp_path_l_min, "r*-")->line_width(2);
        matplot::plot(final_dp_path_s, final_dp_path_l_max, "ro-")->line_width(2);
        // QP_PATH
        matplot::plot(init_qp_path_s, init_qp_path_l, "go-")->line_width(2);
        // 静态障碍物
        for (auto &&static_obs_sl_point : static_obstacles_frent_coords){
            matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1)->line_width(2);
            matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1)->line_width(2);
            matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1)->line_width(2);
            matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1)->line_width(2);
        }
        matplot::title("SL Path and Obstacles");
    }
    // 6.2绘制ST图
    if (_plot_count % 5 == 0){
        matplot::figure(_trajectory_plot_handle);
        matplot::cla();
        std::vector<double> init_t_set, init_s_set;
        std::vector<double> init_qp_speed_profile_t_set, init_qp_speed_profile_s_set;
        matplot::hold(true);
        // 储存DP_SPEED
        for (auto &&st_point : dp_speed_profile){
            init_t_set.emplace_back(st_point.t);
            init_s_set.emplace_back(st_point.s);
        }
        matplot::plot(init_t_set, init_s_set, "bo-")->line_width(2);

        // 储存增密前QP_SPEED
        for (auto &&st_point : init_qp_speed_profile){
            init_qp_speed_profile_t_set.emplace_back(st_point.t);
            init_qp_speed_profile_s_set.emplace_back(st_point.s);
        }
        matplot::plot(init_qp_speed_profile_t_set, init_qp_speed_profile_s_set, "g*-")->line_width(4);

        // ST图中的动态障碍物
        for (auto &&st_graph_node : dynamic_obs_st_graph){
            matplot::line(st_graph_node.at("t_in"), st_graph_node.at("s_in"),st_graph_node.at("t_out"), st_graph_node.at("s_out"))->line_width(4);
        }
        matplot::xlim({0,8});  // 使用std::vector<double> 初始化
        matplot::ylim({0, 80}); // 使用std::vector<double> 初始化

        matplot::title("ST Path and Dynamic Obstacles");
    }

    _plot_count++;
    if (_plot_count == 1e10)
    {
        _plot_count = 0;
    }
    // RCLCPP_INFO(this->get_logger(), "轨迹包含点数:%d", final_trajectory.size());
    //RCLCPP_INFO(this->get_logger(), "emplanner绘图完成");
}

/**
 * 获取规划起点与拼接轨迹的基本思路：
 * 第一次运行时，以自车位姿作为起点
 * 之后，获取当前时刻在上一周期轨迹对应的时间索引，将_previous_trajectory[current_time_index]作为目标点，计算自车位姿与目标点之间的误差
 * 若主车实际位置与目标点差距不大，则获取当前时刻 + delta_t在上一周期轨迹对应的时间索引，将该时间索引的点作为起点，同时拼接上一段轨迹,向前拼20个点
 * 如果主车实际位置与目标点差距很大，用动力学方程外推,将外推结果作为规划起点
 */

TrajectoryPoint EMPlanner::calculate_planning_start_point(std::shared_ptr<VehicleState> ego_state){
    _switch_trajectory.clear();
    TrajectoryPoint planning_start_point;
    // 如果第一次运行
    double delta_T = 0.1;
    if (_previous_trajectory.empty()){
        planning_start_point.x = ego_state->x;
        planning_start_point.y = ego_state->y;
        planning_start_point.v = 0.0;
        planning_start_point.heading = ego_state->heading;
        planning_start_point.ax = 0.0;
        planning_start_point.ay = 0.0;
        planning_start_point.kappa = 0.0;
        planning_start_point.time_stamped = _current_time + delta_T;
    }
    // 不是第一次运行，已经有了上一周期规划的轨迹
    else {
        // 计算主车位置与目标点之间的误差
        size_t current_time_index = -1;
        if (_current_time <= _previous_trajectory[0].time_stamped){
            current_time_index = 0;
        }
        // 获取当前时刻在上一周期轨迹对应的时间索引
        for (size_t i = 0; i < _previous_trajectory.size() - 2; i++) {
            if (_current_time > _previous_trajectory[i].time_stamped && _current_time <= _previous_trajectory[i + 1].time_stamped){
                if ((_current_time - _previous_trajectory[i].time_stamped) <= (_previous_trajectory[i + 1].time_stamped - _current_time)){
                    current_time_index = i;
                }
                else{
                    current_time_index = i + 1;
                }
            }
        }
        auto target_point = _previous_trajectory[current_time_index];
        Eigen::Vector2d tau_target_point(std::cos(target_point.heading), std::sin(target_point.heading));
        Eigen::Vector2d nor_target_point(-std::sin(target_point.heading), std::cos(target_point.heading));
        Eigen::Vector2d host_to_target_point(target_point.x - ego_state->x, target_point.y - ego_state->y);
        double error_longitudional = std::abs(host_to_target_point.dot(tau_target_point)); // 纵向误差
        double error_lateral = std::abs(host_to_target_point.dot(nor_target_point));       // 横向误差

        // 主车实际位置与目标点差距不大
        if (error_lateral < 0.5 && error_longitudional < 1.5) {
            // 在上一周期轨迹上搜索规划起点
            size_t start_time_index = -1;
            double start_time = _current_time + delta_T;
            if (start_time <= _previous_trajectory[0].time_stamped){
                start_time_index = 0;
            }
            for (size_t i = 0; i < _previous_trajectory.size() - 2; i++){
                if (start_time > _previous_trajectory[i].time_stamped && start_time <= _previous_trajectory[i + 1].time_stamped){
                    if ((start_time - _previous_trajectory[i].time_stamped) <= (_previous_trajectory[i + 1].time_stamped - start_time)){
                        start_time_index = i;
                    }
                    else{
                        start_time_index = i + 1;
                    }
                }
            }

            planning_start_point.x = _previous_trajectory[start_time_index].x;
            planning_start_point.y = _previous_trajectory[start_time_index].y;
            planning_start_point.v = _previous_trajectory[start_time_index].v;
            planning_start_point.heading = _previous_trajectory[start_time_index].heading;
            planning_start_point.ax = _previous_trajectory[start_time_index].ax;
            planning_start_point.ay = _previous_trajectory[start_time_index].ay;
            // 这里还要一个值得商讨的问题，实际使用current_time+delta_T还是上一周期上start_time_index所对应轨迹点的时间
            planning_start_point.time_stamped = start_time;

            // 这个时候还要拼接上一段轨迹,向前拼20个点
            if (start_time_index >= 20){
                for (int i = start_time_index - 1; i >= 0 && (start_time_index - i) <= 20; i--){
                    _switch_trajectory.emplace_front(_previous_trajectory[i]); // 这个排列顺序是序号越大，时间越靠后的
                }
            }
            else if (start_time_index > 0){
                for (int i = start_time_index - 1; i >= 0; i--){
                    _switch_trajectory.emplace_front(_previous_trajectory[i]); // 这个排列顺序是序号越大，时间越靠后的
                }
            }
        }
        // 主车实际位置与目标点差距很大
        else {
            // 用动力学方程外推,认为在着100ms中加速度变化很小
            planning_start_point.ax = ego_state->ax;
            planning_start_point.ay = ego_state->ay;
            double ego_vx = ego_state->v * std::cos(ego_state->heading);
            double ego_vy = ego_state->v * std::sin(ego_state->heading);
            double planning_start_point_vx = ego_vx + ego_state->ax * delta_T;
            double planning_start_point_vy = ego_vy + ego_state->ay * delta_T;
            planning_start_point.v = std::hypot(planning_start_point_vx, planning_start_point_vy);
            planning_start_point.heading = std::atan2(planning_start_point_vy, planning_start_point_vx);
            planning_start_point.x = ego_state->x + ego_vx * delta_T + 0.5 * ego_state->ax * delta_T * delta_T;
            planning_start_point.y = ego_state->y + ego_vy * delta_T + 0.5 * ego_state->ay * delta_T * delta_T;
            planning_start_point.time_stamped = _current_time + delta_T;
        }
    }

    return planning_start_point;
}

void EMPlanner::obstacle_fileter(std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object> &obstacles)
{
    auto LOG = rclcpp::get_logger("emplanner");
    // 将障碍物分成动态与静态障碍物
    _static_obstacles.clear();
    _dynamic_obstacles.clear();
    if (obstacles.empty()){
        return;
    }
    for (auto &&obs : obstacles){
        if (obs.id == ego_state->id){
            continue;
        } // 障碍物不包含主车

        double v_obs = std::sqrt(std::pow(obs.twist.linear.x, 2.0) + std::pow(obs.twist.linear.y, 2.0) + std::pow(obs.twist.linear.z, 2.0)); // 障碍物速度
        Eigen::Vector2d host_to_obs(obs.pose.position.x - ego_state->x, obs.pose.position.y - ego_state->y); // 主车到障碍物的向量
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        double longitudinal_d = host_to_obs.dot(tau_host); // 纵向距离
        double lateral_d = host_to_obs.dot(nor_host);      // 横向距离
        // 静态障碍物，即使有加速度，一个规划周期是0.1s，障碍物以最大加速度加速也达不到很大的速度
        if (v_obs <= 0.1){
            //超出1.5个车长即变道
            if (longitudinal_d <= 60 && longitudinal_d >= -7.5 && lateral_d <= 10 && lateral_d >= -10){
                _static_obstacles.push_back(obs);
            }
            else{
                continue;
            }
        }
        else // 动态障碍物
        {
            if (longitudinal_d <= 60 && longitudinal_d > -10 && lateral_d <= 25 && lateral_d >= -25){
                _dynamic_obstacles.push_back(obs);
            }
            else{
                continue;
            }
        }
    }
}
