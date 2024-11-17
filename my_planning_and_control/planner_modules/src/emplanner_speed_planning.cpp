#include "emplanner.h"

// ST图中储存每个动态障碍物的s_in; s_out; t_in; t_out
void EMPlanner::generate_st_graph(const std::vector<FrenetPoint> &dynamic_obs_sl_set, const double &delta_l,
                                  std::vector<std::unordered_map<std::string, double>> &st_graph){
    if (dynamic_obs_sl_set.empty()){
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "正在计算动态障碍物st_graph");
    for (auto && obs:dynamic_obs_sl_set){
        double t_in, t_out;
        //TODO:对于速度较慢的障碍物如何解决？
        // 对于速度过慢的障碍物，且在上下边界内，应该考虑作为虚拟障碍物
        if (std::abs(obs.l_dot) <= 0.1 || obs.s <=0.0)
        {
            continue;
        }
        else{
            // 动态障碍物初始位于前方边界阈值外
            if (std::abs(obs.l) > delta_l){
                if (obs.l * obs.l_dot > 0){
                    continue;
                }
                else{
                    t_in = std::abs(obs.l - delta_l) / std::abs(obs.l_dot);
                    // TODO:
                    // t_out应该是两倍的delta_t，源代码可能有误
                    t_out = std::abs(obs.l + 2 * delta_l) / std::abs(obs.l_dot);
                }
            }
            // 动态障碍物初始位于前方边界阈值内
            else{
                t_in = 0.0;
                if (obs.l_dot * obs.l > 0){
                    t_out = std::abs(delta_l - obs.l) / std::abs(obs.l_dot);
                }
                else{
                    t_out = std::abs(delta_l + obs.l) / std::abs(obs.l_dot);
                }
            }
        }
        // 碰瓷或者太远，不处理
        RCLCPP_INFO(this->get_logger(), "动态障碍物st_graph计算中,(t_in:%.3f, t_out:%.3f)", t_in, t_out);
        // if (t_out >= 8 || t_out <= 1) {
        //     continue;
        // }
        if (t_out >= 8){
            continue;
        }
        else
        {
            st_graph.emplace_back();
            st_graph.back()["t_in"] = t_in;
            st_graph.back()["t_out"] = t_out;
            st_graph.back()["s_in"] = obs.s + obs.s_dot * t_in;
            st_graph.back()["s_out"] = obs.s + obs.s_dot * t_out;
        }
    }
}

STPoint EMPlanner::calculate_speed_start_point(const TrajectoryPoint &planning_start_point){
    STPoint planning_start_point_st;
    planning_start_point_st.s = 0.0;
    planning_start_point_st.t = 0.0;
    double ax = planning_start_point.ax;
    double ay = planning_start_point.ay;
    Eigen::Vector2d a(ax, ay);
    Eigen::Vector2d tau(std::cos(planning_start_point.heading), std::sin(planning_start_point.heading));
    planning_start_point_st.s_dot = planning_start_point.v;
    planning_start_point_st.s_dot_dot = a.dot(tau);

    return planning_start_point_st;
}

//TODO: 采样加密
// std::vector<std::vector<STPoint>> speed_dp_sample;
void EMPlanner::get_speed_dp_sample(const STPoint &planning_start_st, const double &t_sample_distance, const double &t_end,
                                    const std::vector<double> &index2s, std::vector<std::vector<STPoint>> &speed_dp_sample){
    int num_t_sample = ceil(t_end / t_sample_distance);
    //下面建立s的采样表
    double s_end = index2s.back();
    std::deque<double> s_sample;
    s_sample.emplace_front(0.0);
    // s的采样最开始0.5m采样一个，每隔10个采样点增长一倍
    int count = 1;
    double delta_s = 0.5;
    
    while(true){
        double next_s = s_sample.front() + delta_s;
        if (next_s > s_end){
            break;
        }
        s_sample.emplace_front(next_s);
        count++;
        if (count % 10 == 1){
            delta_s *= 2;
        }
    }
    // 将采样填入speed_dp_sample中
    speed_dp_sample.emplace_back();
    speed_dp_sample.back().emplace_back(planning_start_st);
    for (int level = 1; level <= num_t_sample; level++){
        speed_dp_sample.emplace_back();
        for (size_t i = 0; i < s_sample.size(); i++){
            STPoint sample_point;
            sample_point.s = s_sample[i];
            sample_point.t = level * t_sample_distance;
            speed_dp_sample.back().emplace_back(sample_point);
        }
    }
}

double EMPlanner::calculate_speed_dp_cost(const STPoint &start_point, const STPoint &end_point, const double &reference_speed,
                                          const std::vector<std::unordered_map<std::string, double>> &dynamic_obs_st_graph,
                                          const WeightCoefficients &weight_coefficients){
    double cost_ref = 0.0, cost_obs = 0.0, cost_acc = 0.0, cost_jerk = 0.0;
    //计算参考速度代价
    double delta_t = end_point.t - start_point.t;
    double s_dot = (end_point.s - start_point.s) / delta_t;
    cost_ref = weight_coefficients.speed_dp_w_ref_speed * std::pow(s_dot - reference_speed, 2);

    //加速度代价
    double s_dot_dot = (s_dot - start_point.s_dot) / delta_t;
    if (s_dot_dot >= 5 || s_dot_dot <= -6){
        cost_acc = 1e5;
    }
    else{
        cost_acc = weight_coefficients.speed_dp_w_a * std::pow(s_dot_dot, 2);
    }

    //jerk代价
    double s_dot_dot_dot = (s_dot_dot - start_point.s_dot_dot) / delta_t;
    cost_jerk = weight_coefficients.speed_dp_w_jerk * s_dot_dot_dot * s_dot_dot_dot;

    // 障碍物代价
    if (!dynamic_obs_st_graph.empty()){
        // 1.在start_point与end_point之间建立采样点
        auto linspace = Eigen::VectorXd::LinSpaced(6, start_point.t, end_point.t);
        std::vector<double> t_set, s_set;
        for (int i = 1; i < linspace.size(); i++)
        {
            t_set.emplace_back(linspace[i]);
            s_set.emplace_back(s_dot * (t_set.back() - start_point.t) + start_point.s);
        }
        // 2.逐个障碍物计算代价
        for (auto &&obs : dynamic_obs_st_graph){
            //对于每一个采样点，计算障碍物代价
            for (size_t i = 0; i < t_set.size(); i++){
                //首先计算采样点到障碍物距离
                double min_distance = 0.0;
                //其定义见讲义
                Eigen::Vector2d v1(obs.at("t_in") - t_set[i], obs.at("s_in") - s_set[i]); //point2in
                Eigen::Vector2d v3(obs.at("t_out") - t_set[i], obs.at("s_out") - s_set[i]); //point2out
                Eigen::Vector2d v2(obs.at("t_out") - obs.at("t_in"), obs.at("s_out") - obs.at("s_in")); //in2out
                if ((v1.dot(v2)) * (v3.dot(v2)) >= 0){
                    min_distance = std::min(v1.norm(), v3.norm());
                }
                else{
                    //二维向量不能直接用.cross叉乘
                    min_distance = std::abs(v1[0] * v3[1] - v1[1] * v3[0]) / v2.norm();
                }
                //由距离计算代价
                if (min_distance > 3){
                    cost_obs += 0;
                }
                else if (min_distance < 2){
                    cost_obs += weight_coefficients.speed_dp_w_obs;
                }
                else{
                    cost_obs += 1000 / std::pow(min_distance, 2);
                }
            }
        }
    }
    return cost_ref + cost_acc + cost_jerk + cost_obs;
}

void EMPlanner::generate_convex_space(const std::vector<TrajectoryPoint> &init_trajectory, const std::vector<double> path_index2s,
                                    const std::deque<STPoint> &speed_profile, 
                                    const std::vector<std::unordered_map<std::string, double>> &dynamic_obs_st_graph, const double &ay_max, std::vector<double> &s_lb, std::vector<double> &s_ub, 
                                    std::vector<double> &s_dot_lb, std::vector<double> &s_dot_ub){
    s_lb.emplace_back(speed_profile.front().s);
    s_ub.emplace_back(speed_profile.front().s);
    s_dot_lb.emplace_back(speed_profile.front().s_dot);
    s_dot_ub.emplace_back(speed_profile.front().s_dot);
    // 1.确定s_dot的上下界（即车辆速度的上下界，一般不能超过最大加速度和曲率的限制）
    for (size_t i = 0; i < speed_profile.size(); i++){
        // 1.1计算当前的曲率，为求出最大加速度做准备
        int obs_s_index = -1;
        double cur_s = speed_profile[i].s;
        double cur_kappa;
        if (cur_s == path_index2s.back()){
            obs_s_index = (int)init_trajectory.size() - 1;
            cur_kappa = init_trajectory.back().kappa;
        }
        else{
            // 因为speed_profile的点比较密，所以线性插值计算kappa
            for (int j = 0; j < (int)path_index2s.size()-1; j++){
                {
                    if (cur_s >= path_index2s[j] && cur_s < path_index2s[j+1]){
                        obs_s_index = j;
                        break;
                    }
                }
            }
            double k = (init_trajectory[obs_s_index + 1].kappa - init_trajectory[obs_s_index].kappa) / (path_index2s[obs_s_index + 1] - path_index2s[obs_s_index]);
            cur_kappa = init_trajectory[obs_s_index].kappa + k * (cur_s - path_index2s[obs_s_index]);
        }
        // 1.2由曲率计算速度上下界
        s_dot_ub.emplace_back(std::sqrt(std::abs(ay_max / cur_kappa)));
        s_dot_lb.emplace_back(0.0);
    }

    // 2确定s的上下界
    for (size_t i = 1; i < speed_profile.size(); i++){
        s_ub.emplace_back(path_index2s.back());
        s_lb.emplace_back(0.0);
    }

    for (auto &&cur_obs : dynamic_obs_st_graph)
    {   
        //TODO:验证是否正确
        // 就是近距离1.0s之内突然出现一个障碍物，那就急停或跟车。
        if (cur_obs.at("t_in") <= 1.0 && std::min(cur_obs.at("s_in"),cur_obs.at("s_out")) < 10.0){
            for (size_t i = 0; i < s_ub.size(); i++)
            {
                s_ub[i] = std::max(0.0,std::max(cur_obs.at("s_in"),cur_obs.at("s_out")));
                s_lb[i] = 0.0;
            }
            continue;
        }
        // 2.1判断超车还是减速
        // 直接使用st图的中点判断
        double obs_t_center = (cur_obs.at("t_in") + cur_obs.at("t_out")) / 2.0;
        double obs_s_center = (cur_obs.at("s_in") + cur_obs.at("s_out")) / 2.0;
        int obs_t_center_index = find_t_index(speed_profile, obs_t_center);
        if (obs_t_center_index == -1){
            continue;
        }
        // 2.2判断并修改相应的上下界
        int start_index = find_t_index(speed_profile, cur_obs.at("t_in"));
        int end_index = find_t_index(speed_profile, cur_obs.at("t_out"));
        // 对起始点和截至点做偏向于安全的放缩
        start_index = std::max(start_index - 2, 1); // 第一个不处理，因为是规划起点，是等式约束
        end_index = std::min(end_index + 2, (int)speed_profile.size() - 1);
        double k = (cur_obs.at("s_out") - cur_obs.at("s_in")) / (cur_obs.at("t_out") - cur_obs.at("t_in"));
        // 加速超车
        if (speed_profile[obs_t_center_index].s >= obs_s_center) {
            for (int i = start_index; i <= end_index; i++){
                s_lb[i] = std::max(s_lb[i], obs_s_center + k * (speed_profile[i].t) - obs_t_center);
            }
        }
        // 减速让行
        else {
            for (int i = start_index; i <= end_index; i++)
            {
                s_ub[i] = std::min(s_ub[i], obs_s_center + k * (speed_profile[i].t) - obs_t_center);
            }
        }
    }
}

int EMPlanner::find_t_index(const std::deque<STPoint> &speed_profile, const double &t){
    int index = -1;
    // 超界情况
    if (t >= speed_profile.back().t || t < speed_profile.front().t) {
        index = -1;
    }
    // 在界内,左边界在进入这个函数之前判断是否碰瓷里已经淘汰掉了
    else {
        for (int j = 0; j < (int)speed_profile.size() - 1; j++){
            if (t >= speed_profile[j].t && t < speed_profile[j + 1].t){
                if (std::abs(t - speed_profile[j].t) <= std::abs(t - speed_profile[j + 1].t)){
                    index = j;
                    break;
                }
                else{
                    index = j + 1;
                    break;
                }
            }
        }
    }
    return index;
}

bool EMPlanner::speed_QP_planning(const std::deque<STPoint> &dp_speed_profile, const std::vector<double> &s_lb, const std::vector<double> &s_ub,
                                  const double &reference_speed, const std::vector<double> &s_dot_lb, const std::vector<double> &s_dot_ub,
                                  const WeightCoefficients &weight_coeff, std::vector<STPoint> &qp_speed_profile){
    size_t point_num = dp_speed_profile.size();
    //----------------1.建立目标函数-------------------
    //1.1建立H矩阵
    Eigen::SparseMatrix<double> H_v, H_a, H_jerk, H_total;
    H_v.resize(point_num, 3 * point_num);
    H_a.resize(point_num, 3 * point_num);
    H_jerk.resize(point_num - 1, 3 * point_num);

    for (size_t i = 0; i < point_num; i++){
        H_v.insert(i, 3 * i + 1) = 1; // s_dot - ref_speed
        H_a.insert(i, 3 * i + 2) = 1; // s_dot_dot ^2
    }
    // (s2_dot_dot - s1_dot_dot)^2
    for (size_t i = 0; i < point_num - 1; i++){
        H_jerk.insert(i, 3 * i + 2) = -1;
        H_jerk.insert(i, 3 * i + 5) = 1;
    }

    //TODO:这是源代码的写法，应该会报错,但为什么没有
    //H_total.resize(point_num, point_num);
    H_total.resize(3 * point_num, 3 * point_num);
    H_total = (weight_coeff.speed_qp_w_ref_speed * H_v.transpose() * H_v +
               weight_coeff.speed_qp_w_a * H_a.transpose() * H_a +
               weight_coeff.speed_qp_w_jerk * H_jerk.transpose() * H_jerk);
    H_total = 2 * H_total;
    // RCLCPP_INFO(this->get_logger(), "速度二次规划——H_total完成");

    // 1.2建立f矩阵
    Eigen::VectorXd f;
    f.setZero(3 * point_num);
    for (size_t i = 0; i < point_num; i++){
        f[3 * i + 1] = -2 * weight_coeff.speed_qp_w_ref_speed * reference_speed; // -2 * ref_speed * s_dot
    }
    // RCLCPP_INFO(this->get_logger(), "速度二次规划——建立目标函数完成");
    //----------------2.建立约束-------------------
    // 2.1连续性约束
    Eigen::SparseMatrix<double> A_continuity;
    Eigen::VectorXd lb_continuity, ub_continuity;
    A_continuity.resize(2 * point_num - 2, 3 * point_num);
    double delta_t = dp_speed_profile[1].t - dp_speed_profile[0].t;
    for (size_t i = 0; i < point_num - 1; i++){
        A_continuity.insert(2 * i + 0, 3 * i + 0) = 1.0;
        A_continuity.insert(2 * i + 0, 3 * i + 1) = delta_t;
        A_continuity.insert(2 * i + 0, 3 * i + 2) = delta_t * delta_t / 3.0;
        A_continuity.insert(2 * i + 0, 3 * i + 3) = -1.0;
        A_continuity.insert(2 * i + 0, 3 * i + 4) = 0.0;
        A_continuity.insert(2 * i + 0, 3 * i + 5) = delta_t * delta_t / 6.0;
        A_continuity.insert(2 * i + 1, 3 * i + 0) = 0.0;
        A_continuity.insert(2 * i + 1, 3 * i + 1) = 1.0;
        A_continuity.insert(2 * i + 1, 3 * i + 2) = delta_t / 2.0;
        A_continuity.insert(2 * i + 1, 3 * i + 3) = 0.0;
        A_continuity.insert(2 * i + 1, 3 * i + 4) = -1.0;
        A_continuity.insert(2 * i + 1, 3 * i + 5) = delta_t / 2.0;
    }
    lb_continuity.setZero(2 * point_num - 2);
    ub_continuity.setZero(2 * point_num - 2);
    
    // 2.2不允许倒车约束
    Eigen::SparseMatrix<double> A_avoid_reverse;
    Eigen::VectorXd lb_avoid_reverse, ub_avoid_reverse;
    A_avoid_reverse.resize(point_num - 1, 3 * point_num);
    ub_avoid_reverse.setZero(point_num - 1);
    lb_avoid_reverse.setZero(point_num - 1);
    // s_1 - s_2 <= 0
    for (size_t i = 0; i < point_num - 1; i++)
    {
        A_avoid_reverse.insert(i, 3 * i + 2) = 1;
        A_avoid_reverse.insert(i, 3 * (i + 1) + 2) = -1;
        lb_avoid_reverse[i] = std::numeric_limits<double>::lowest();
    }

    // 2.3凸空间约束（凸空间中已经包括了最大加速度约束）
    Eigen::SparseMatrix<double> A_convex_space;
    Eigen::VectorXd lb_convex_space, ub_convex_space;

    A_convex_space.resize(2 * point_num, 3 * point_num);
    lb_convex_space.setZero(2 * point_num);
    ub_convex_space.setZero(2 * point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        A_convex_space.insert(2 * i + 0, 3 * i + 0) = 1;
        A_convex_space.insert(2 * i + 1, 3 * i + 1) = 1;
        lb_convex_space[2 * i + 0] = s_lb[i];
        lb_convex_space[2 * i + 1] = s_dot_lb[i];
        ub_convex_space[2 * i + 0] = s_ub[i];
        ub_convex_space[2 * i + 1] = s_dot_ub[i];
    }
    // 2.4组装矩阵
    Eigen::SparseMatrix<double> A_total, A_total_transpose;
    Eigen::VectorXd lb_total, ub_total;

    A_total_transpose.resize(3 * point_num, 5 * point_num - 3);
    A_total_transpose.middleCols(0, A_continuity.rows()) = A_continuity.transpose();
    A_total_transpose.middleCols(A_continuity.rows(), A_avoid_reverse.rows()) = A_avoid_reverse.transpose();
    A_total_transpose.middleCols(A_continuity.rows() + A_avoid_reverse.rows(), A_convex_space.rows()) = A_convex_space.transpose();
    A_total = A_total_transpose.transpose();

    lb_total.resize(5 * point_num - 3);
    lb_total << lb_continuity, lb_avoid_reverse, lb_convex_space;
    ub_total.resize(5 * point_num - 3);
    ub_total << ub_continuity, ub_avoid_reverse, ub_convex_space;
    // RCLCPP_INFO(this->get_logger(), "速度二次规划——建立约束完成");
    //----------------3.求解-------------------
    _speed_qp_solver.data()->setNumberOfVariables(3 * point_num);
    _speed_qp_solver.data()->setNumberOfConstraints(A_total.rows());
    if (!_speed_qp_solver.data()->setHessianMatrix(H_total))
    {
        return false;
    }
    if (!_speed_qp_solver.data()->setGradient(f))
    {
        return false;
    }
    if (!_speed_qp_solver.data()->setLinearConstraintsMatrix(A_total))
    {
        return false;
    }

    if (!_speed_qp_solver.data()->setLowerBound(lb_total))
    {
        return false;
    }
    if (!_speed_qp_solver.data()->setUpperBound(ub_total))
    {
        return false;
    }
    if (!_speed_qp_solver.initSolver())
    {
        return false;
    }
    if (_speed_qp_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        return false;
    }

    auto solution = _speed_qp_solver.getSolution();
    for (size_t i = 0; i < point_num; i++)
    {
        STPoint cur_st_point;
        cur_st_point.t = dp_speed_profile[i].t;
        cur_st_point.s = solution[3 * i + 0];
        cur_st_point.s_dot = solution[3 * i + 1];
        cur_st_point.s_dot_dot = solution[3 * i + 2];
        qp_speed_profile.emplace_back(cur_st_point);
   }

//    _speed_qp_solver.data()->clearHessianMatrix();
//    _speed_qp_solver.data()->clearLinearConstraintsMatrix();
//    _speed_qp_solver.clearSolverVariables();
//    _speed_qp_solver.clearSolver();
      _speed_qp_solver.clearSolverVariables();
      _speed_qp_solver.data()->clearLinearConstraintsMatrix();
      _speed_qp_solver.data()->clearHessianMatrix();
      _speed_qp_solver.clearSolver();
   return true;
}

// 加密二次规划所得速度剖面
void EMPlanner::increased_speed_profile(const std::vector<STPoint> &init_speed_profile, std::vector<STPoint> &final_speed_proflie)
{
    double dt = 0.02; // 加密到每隔0.02s一个点
    for (size_t i = 0; i < init_speed_profile.size() - 1; i++)
    {
        STPoint start_point(init_speed_profile[i]);
        double start_time = start_point.t;
        STPoint end_point(init_speed_profile[i + 1]);
        size_t num = floor((end_point.t - start_point.t) / dt);
        PolynomialCurve curve;
        curve.curve_fitting(start_point.t, start_point.s, start_point.s_dot, start_point.s_dot_dot,
                            end_point.t, end_point.s, end_point.s_dot, end_point.s_dot_dot);
        for (size_t j = 0; j < num; j++) // 包含起点不包含终点,避免终点重复包含
        {
            STPoint increased_point;
            double cur_t = start_time + j * dt;
            increased_point.t = cur_t;
            increased_point.s = curve.value_evaluation(cur_t, 0);
            increased_point.s_dot = curve.value_evaluation(cur_t, 1);
            increased_point.s_dot_dot = curve.value_evaluation(cur_t, 2);

            final_speed_proflie.emplace_back(increased_point);
        }
    }
}

void EMPlanner::generate_trajectory(const std::vector<STPoint> &final_speed_profile, const std::vector<TrajectoryPoint> &path_trajectory,
                                    const std::vector<double> &path_index2s, const double &planning_start_point_time_stamped,
                                    std::vector<TrajectoryPoint> &trajectory){
    for (size_t i = 0; i < final_speed_profile.size(); i++){
        using namespace std::chrono_literals;
        TrajectoryPoint trajectory_point;
        // 从final_speed_profile中读取 v,a,t
        trajectory_point.v = final_speed_profile[i].s_dot;
        trajectory_point.a_tau = final_speed_profile[i].s_dot_dot;
        trajectory_point.time_stamped = planning_start_point_time_stamped + final_speed_profile[i].t;

        // 从path_index2s中读取 x,y, heading, kappa
        double cur_s = final_speed_profile[i].s;
        double nearest_index;
        if (cur_s == path_index2s.back())
        {
            nearest_index = path_index2s.size() - 1;
        }
        for (size_t j = 0; j < path_index2s.size() - 1; j++){
            if (cur_s >= path_index2s[j] && cur_s < path_index2s[j + 1]){
                nearest_index = j;
            }
        }
        trajectory_point.x = path_trajectory[nearest_index].x +
                             ((path_trajectory[nearest_index + 1].x - path_trajectory[nearest_index].x) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);

        trajectory_point.y = path_trajectory[nearest_index].y +
                             ((path_trajectory[nearest_index + 1].y - path_trajectory[nearest_index].y) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);

        trajectory_point.heading = path_trajectory[nearest_index].heading +
                                   ((path_trajectory[nearest_index + 1].heading - path_trajectory[nearest_index].heading) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);

        trajectory_point.kappa = path_trajectory[nearest_index].kappa +
                                 ((path_trajectory[nearest_index + 1].kappa - path_trajectory[nearest_index].kappa) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);
        trajectory.emplace_back(trajectory_point);
    }
}
