#include "emplanner.h"

void EMPlanner::get_path_dp_sample(std::vector<std::vector<FrenetPoint>> &path_dp_sample, const FrenetPoint &planning_start_sl,
                                   const double &s_sample_distance, const size_t &s_sample_number,
                                   const double &l_sample_distance, const size_t &l_sample_number){
                                    //10 6 1 7
    path_dp_sample.emplace_back();
    path_dp_sample.back().emplace_back(planning_start_sl);
    for (size_t level = 1; level <= s_sample_number; level++){
        path_dp_sample.emplace_back();
        // TODO:l采样时取负数
        for (int i = 1; i <= (int)l_sample_number; i++){
            FrenetPoint current_point;
            current_point.s = planning_start_sl.s + level * s_sample_distance;

            //  因为参考线都是靠右，并且l采样时没有取复数，所以干脆让l只对参考线左侧采样，需要改进
            current_point.l = ((int)l_sample_number - i) * l_sample_distance;
            path_dp_sample.back().emplace_back(current_point);
        }
    }
}

double EMPlanner::calculate_dp_cost(const FrenetPoint &start_point, const FrenetPoint &end_point, const std::vector<FrenetPoint> &static_obs_sl_set,
                                    const WeightCoefficients &weight_coeff){
    //1.先将起终点用五次多项式连接
    //曲线拟合类实例化
    PolynomialCurve ploy_curve;
    ploy_curve.curve_fitting(start_point.s, start_point.l, start_point.l_prime, start_point.l_prime_prime,
                             end_point.s, end_point.l, end_point.l_prime, end_point.l_prime_prime);
    int num_sample_points = 11;
    auto s_sample_points = Eigen::VectorXd::LinSpaced(num_sample_points, start_point.s, end_point.s); // 先采样s，再根据s计算得到l

    // 创建计算cost需要的变量并赋值, dl = dl/ds, 不严谨，但此处简写（真正的dl = dl/dt）
    std::vector<double> s_set, l_set, dl_set, ddl_set, dddl_set; 
    // 因为每一次计算DP_cost的起点都相同，所以无需加入起点，节约计算量
    for (size_t i = 1; i < (size_t)s_sample_points.size(); i++){
        s_set.emplace_back(s_sample_points[i]);
        l_set.emplace_back(ploy_curve.value_evaluation(s_sample_points[i], 0));
        dl_set.emplace_back(ploy_curve.value_evaluation(s_sample_points[i], 1));
        ddl_set.emplace_back(ploy_curve.value_evaluation(s_sample_points[i], 2));
        dddl_set.emplace_back(ploy_curve.value_evaluation(s_sample_points[i], 3));
    }

    double cost_total = 0.0, cost_ref = 0.0, cost_smooth = 0.0, cost_obs = 0.0;
    // 参考线代价
    for (size_t i = 0; i < l_set.size(); i++){
        cost_ref += weight_coeff.path_dp_w_ref * std::pow(l_set[i], 2);
    }
    //平滑代价
    for (size_t i = 0; i < dl_set.size(); i++){
        cost_smooth += weight_coeff.path_dp_w_dl * std::pow(dl_set[i], 2) +
                       weight_coeff.path_dp_w_ddl * std::pow(ddl_set[i], 2) +
                       weight_coeff.path_dp_w_dddl * std::pow(dddl_set[i], 2);
    }

    //障碍物代价
    //TODO:正确设置
    if (!static_obs_sl_set.empty()){
        for (size_t i = 0; i < static_obs_sl_set.size(); i++){
            for (size_t j = 0; j < s_set.size(); j++){
                double distance = std::hypot(s_set[j] - static_obs_sl_set[i].s, l_set[j] - static_obs_sl_set[i].l);
                if (distance >= 4){
                    cost_obs += 0;
                }
                else if (distance >= 0.5){
                    cost_obs += weight_coeff.path_dp_w_obs / distance;
                }
                else{
                    cost_obs += weight_coeff.path_dp_w_obs / (distance * distance + 1e-6);
                }
            }
        }
    }
    // if (!static_obs_sl_set.empty())
    // {
    //     for (size_t i = 0; i < static_obs_sl_set.size(); i++)
    //     {
    //         for (size_t j = 0; j < s_set.size(); j++)
    //         {
    //             double distance = std::hypot(s_set[j] - static_obs_sl_set[i].s, l_set[j] - static_obs_sl_set[i].l);
    //             if (distance >= 4)
    //             {
    //                 cost_obs += 0;
    //             }
    //             else if (distance <= 3)
    //             {
    //                 cost_obs += weight_coeff.path_dp_w_obs;
    //             }
    //             else
    //             {
    //                 cost_obs += 1000.0 / (distance * distance + 1e-6);
    //             }
    //         }
    //     }
    // }
    cost_total = cost_ref + cost_smooth + cost_obs;
    return cost_total;
}

//增密动态规划结果
void EMPlanner::increased_dp_path(const std::vector<FrenetPoint> &init_dp_path, const double &increased_distance, std::vector<FrenetPoint> &final_dp_path){
    //计算每相邻两个DP路径点的五次多项式及采样个数
    double sample_s = init_dp_path[1].s - init_dp_path[0].s;
    size_t sample_num = (size_t)(sample_s / increased_distance);
    for (size_t i = 0; i < init_dp_path.size() - 1; i++){
        PolynomialCurve poly_curve;
        poly_curve.curve_fitting(init_dp_path[i].s, init_dp_path[i].l, init_dp_path[i].l_prime, init_dp_path[i].l_prime_prime,
                                 init_dp_path[i + 1].s, init_dp_path[i + 1].l, init_dp_path[i + 1].l_prime, init_dp_path[i + 1].l_prime_prime);
        // 不包括相邻两个DP路径点中的后一个点，因为该点会在下一次作为起点被纳入
        for (size_t j = 0; j < sample_num; j++){
            double cur_s = init_dp_path[i].s + j * increased_distance;
            FrenetPoint cur_point;
            cur_point.s = cur_s;
            cur_point.l = poly_curve.value_evaluation(cur_point.s, 0);
            cur_point.l_prime = poly_curve.value_evaluation(cur_point.s, 1);
            cur_point.l_prime_prime = poly_curve.value_evaluation(cur_point.s, 2);
            final_dp_path.emplace_back(cur_point);
        }
    }
    //整条DP路径的最后一个点单独纳入
    final_dp_path.emplace_back(init_dp_path.back());
}

// 生成凸空间(也就是把路缘、静态障碍物都加上,结果储存在path_l_max 和path_l_min中)
void EMPlanner::generate_convex_space(const std::vector<FrenetPoint> &final_dp_path, const double &road_up_bound, const double &road_low_bound,
                                    const std::vector<FrenetPoint> &static_obs_sl_set, std::vector<double> &path_l_max, std::vector<double> &path_l_min){
    for (size_t i = 0; i < final_dp_path.size(); i++){
        path_l_max.emplace_back(road_up_bound);
        path_l_min.emplace_back(road_low_bound);
    }

    double obs_length = 5.0; // 障碍物(车辆)长度
    double obs_width = 3.0;  // 障碍物(车辆)宽度

    if (!static_obs_sl_set.empty()){
        for (size_t i = 0; i < static_obs_sl_set.size(); i++){
            int center_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_set[i].s);
            if (center_index == -1){
                continue;
            }
            // 找到障碍物的起始index
            int start_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_set[i].s - obs_length / 2.0);

            // int end_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_set[i].s + obs_length / 2.0);
            // TODO:静态障碍物的凸空间尾部太宽了，控制未能及时跟上导致阻碍了车返回原车道，因此缩短
            int end_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_set[i].s + obs_length / 2.0);
            if (start_index == -1 || end_index == -1){
                continue;
            }
            //左侧绕行
            if (final_dp_path[center_index].l > static_obs_sl_set[i].l){
                for (int j = start_index; j <= end_index; j++){
                    path_l_min[j] = std::max(path_l_min[j], static_obs_sl_set[i].l + obs_width / 2.0);
                }
            }
            else{
                for (int j = start_index; j <= end_index; j++){
                    path_l_max[j] = std::min(path_l_max[j], static_obs_sl_set[i].l - obs_width / 2.0);
                }
            }
        }
    }
}


// 在路径上寻找与障碍物S坐标最近路径点索引
int EMPlanner::find_index_for_obs_on_path(const std::vector<FrenetPoint> &final_dp_path, const double &static_obs_s){
    int index = -1;
    if (static_obs_s < final_dp_path[0].s){
        index = 0;
    }
    else if (static_obs_s > final_dp_path.back().s){
        index = (int)final_dp_path.size() - 1;
    }
    else{
        for (size_t i = 0; i < final_dp_path.size() - 1; i++){
            if (static_obs_s > final_dp_path[i].s && static_obs_s < final_dp_path[i + 1].s){
                if (std::abs(static_obs_s - final_dp_path[i].s) > std::abs(static_obs_s - final_dp_path[i + 1].s)){
                    index = i + 1;
                    break;
                }
                else{
                    index = i;
                    break;
                }
            }
        }
    }
    return index;
}

// 路径QP规划，结果储存在init_qp_path里，自变量为l_n,dl_n,ddl_n, n=1...final_dp_path.size()
bool EMPlanner::path_QP_planning(const std::vector<FrenetPoint> &final_dp_path, const std::vector<double> &final_dp_path_lmin,
                                 const std::vector<double> &final_dp_path_lmax, const double &l_desire, const double &dl_desire, const double &ddl_desire,
                                 const WeightCoefficients &weight_coeff, std::vector<FrenetPoint> &init_qp_path){
    size_t point_num = final_dp_path.size();
    //------------------1. 建立目标函数------------------
    //1.1 H矩阵，即求解器中的海森矩阵，包括参考线代价矩阵、平滑性代价矩阵
    Eigen::SparseMatrix<double> H_ref, H_dl, H_ddl, H_dddl, H_mid, H_l_end, H_dl_end, H_ddl_end, H;
    H_ref.resize(point_num, 3 * point_num);
    H_dl.resize(point_num, 3 * point_num);
    H_ddl.resize(point_num, 3 * point_num);
    H_mid.resize(point_num, 3 * point_num);
    for (size_t i = 0; i < point_num; i++){
        //稀疏矩阵用insert方法，别忘了
        H_ref.insert(i, 3 * i) = 1;
        H_dl.insert(i, 3 * i + 1) = 1;
        H_ddl.insert(i, 3 * i + 2) = 1;
        H_mid.insert(i, 3 * i) = 1;
    }
    H_dddl.resize(point_num - 1, 3 * point_num);
    for (size_t i = 0; i < point_num - 1; i++){
        // 稀疏矩阵用insert方法，别忘了
        H_dddl.insert(i, 3 * i + 2) = -1;
        H_dddl.insert(i, 3 * (i + 1) + 2) = 1;
    }
    //TODO:
    //下面矩阵为何要如此初始化，存疑
    H_l_end.resize(1, 3 * point_num);
    H_dl_end.resize(1, 3 * point_num);
    H_ddl_end.resize(1, 3 * point_num);
    H_l_end.insert(0, 3 * (point_num - 1)) = 1;
    H_dl_end.insert(0, 3 * (point_num - 1) + 1) = 1;
    H_ddl_end.insert(0, 3 * (point_num - 1) + 2) = 1;

    //组合为H矩阵
    H = weight_coeff.path_qp_w_ref * H_ref.transpose() * H_ref +
        weight_coeff.path_qp_w_dl * H_dl.transpose() * H_dl +
        weight_coeff.path_qp_w_ddl * H_ddl.transpose() * H_ddl +
        weight_coeff.path_qp_w_dddl * H_dddl.transpose() * H_dddl +
        weight_coeff.path_qp_w_mid * H_mid.transpose() * H_mid +
        weight_coeff.path_qp_w_l_end * H_l_end.transpose() * H_l_end +
        weight_coeff.path_qp_w_dl_end * H_dl_end.transpose() * H_dl_end +
        weight_coeff.path_qp_w_ddl_end * H_ddl_end.transpose() * H_ddl_end;
    H = 2 * H;

    // 1.2 f矩阵，即求解器中的梯度
    Eigen::VectorXd f_mid, f_l_end, f_dl_end, f_ddl_end, f;
    f_mid.setZero(3 * point_num);
    f_l_end.setZero(3 * point_num);
    f_dl_end.setZero(3 * point_num);
    f_ddl_end.setZero(3 * point_num);
    //TODO:
    //尝试两个版本的中心约束
    for (size_t i = 0; i < point_num; i++){
        // 凸空间中央约束 版本1： 几何稳定，但不平滑
        f_mid[3 * i] = -2.0 * (final_dp_path_lmax[i] + final_dp_path_lmin[i]) / 2.0;
        // 车道中心线约束 版本2： 几何不稳定，但平滑
        // f_mid[3*i] = -2.0 * final_dp_path[i].l;
    }
    f_l_end[3 * (point_num - 1)] = -2.0 * l_desire;
    f_dl_end[3 * (point_num - 1) + 1] = -2.0 * dl_desire;
    f_ddl_end[3 * (point_num - 1) + 2] = -2.0 * ddl_desire;

    f.resize(3 * point_num);
    f = weight_coeff.path_qp_w_mid * f_mid +
        weight_coeff.path_qp_w_l_end * f_l_end +
        weight_coeff.path_qp_w_dl_end * f_dl_end +
        weight_coeff.path_qp_w_ddl_end * f_ddl_end;

    //------------------2. 建立约束------------------
    // 2.1等式约束(连续性约束),由于osqp-eigen的约束形式是统一的，l <= Ax <= u,所以等式约束要做出相应改变
    // 由原来的Ax = b变为 b <= Ax <= b
    Eigen::SparseMatrix<double> A_continuity;
    A_continuity.resize(2 * point_num - 2, 3 * point_num);
    double delta_s = final_dp_path[1].s - final_dp_path[0].s;
    // 填充A_continuity矩阵
    for (size_t i = 0; i < point_num - 1; i++){
        A_continuity.insert(2 * i, 3 * i) = 1.0;
        A_continuity.insert(2 * i, 3 * i + 1) = delta_s;
        A_continuity.insert(2 * i, 3 * i + 2) = 1 / 3.0 * delta_s * delta_s;
        A_continuity.insert(2 * i, 3 * i + 3) = -1.0;
        A_continuity.insert(2 * i, 3 * i + 5) = 1.0 / 6 * delta_s * delta_s;
        A_continuity.insert(2 * i + 1, 3 * i + 1) = 1.0;
        A_continuity.insert(2 * i + 1, 3 * i + 2) = 1 / 2.0 * delta_s;
        A_continuity.insert(2 * i + 1, 3 * i + 4) = - 1.0;
        A_continuity.insert(2 * i + 1, 3 * i + 5) = 1 / 2.0 * delta_s;
    }
    // 建立连续性约束，也即A_continuity * x = b 中的 b
    Eigen::VectorXd lb_continuity, ub_continuity;
    // 等价于：
    // lb_continuity.resize(2 * point_num - 2);
    // lb_continuity.setZero();
    lb_continuity.setZero(2 * point_num - 2);
    ub_continuity.setZero(2 * point_num - 2);

    // 2.2不等式约束(凸空间上下界约束)
    Eigen::SparseMatrix<double> A_collision;
    // 因为写成统一的 l <= Ax <= u，所以是A_sub是4行，而不是讲义中的8行
    A_collision.resize(4 * point_num, 3 * point_num);
    double d1 = 3.0, d2 = 2.0, car_width = 3.0;
    for (size_t i = 0; i < point_num; i++){
        A_collision.insert(4 * i, 3 * i) = 1;
        A_collision.insert(4 * i, 3 * i + 1) = d1;
        A_collision.insert(4 * i + 1, 3 * i) = 1;
        A_collision.insert(4 * i + 1, 3 * i + 1) = d1;
        A_collision.insert(4 * i + 2, 3 * i) = 1;
        A_collision.insert(4 * i + 2, 3 * i + 1) = -d2;
        A_collision.insert(4 * i + 3, 3 * i) = 1;
        A_collision.insert(4 * i + 3, 3 * i + 1) = -d2;
    }
    Eigen::VectorXd lb_collision, ub_collision;
    lb_collision.setZero(4 * point_num);
    ub_collision.setZero(4 * point_num);
    for (size_t i = 0; i < point_num; i++){
        int start_index = (int)std::max(0.0, floor(i - d2));
        int end_index = std::min((int)ceil(i + d1), (int)point_num - 1);
        double lbi = std::numeric_limits<double>::lowest(); // 在start_index到end_index中lmin里最大的
        double ubi = std::numeric_limits<double>::max(); // 在start_index到end_index中lmax里最小的
        for (int j = start_index; j <= end_index; j++){
            if (final_dp_path_lmin[j] > lbi){
                lbi = final_dp_path_lmin[j];
            }
            if (final_dp_path_lmax[j] < ubi){
                ubi = final_dp_path_lmax[j];
            }
        }
        lb_collision[4 * i] = lbi - car_width / 2.0;
        lb_collision[4 * i + 1] = lbi + car_width / 2.0;
        lb_collision[4 * i + 2] = lbi - car_width / 2.0;
        lb_collision[4 * i + 3] = lbi + car_width / 2.0;
        ub_collision[4 * i] = ubi - car_width / 2.0;
        ub_collision[4 * i + 1] = ubi + car_width / 2.0;
        ub_collision[4 * i + 2] = ubi - car_width / 2.0;
        ub_collision[4 * i + 3] = ubi + car_width / 2.0;
    }
    //放松起点的碰撞约束
    lb_collision[0] = std::numeric_limits<double>::lowest();
    lb_collision[1] = std::numeric_limits<double>::lowest();
    lb_collision[2] = std::numeric_limits<double>::lowest();
    lb_collision[3] = std::numeric_limits<double>::lowest();
    ub_collision[0] = std::numeric_limits<double>::max();
    ub_collision[1] = std::numeric_limits<double>::max();
    ub_collision[2] = std::numeric_limits<double>::max();
    ub_collision[3] = std::numeric_limits<double>::max();

    // 2.3规划起点约束
    Eigen::SparseMatrix<double> A_start_point;
    Eigen::Vector3d lb_start_point, ub_start_point;
    A_start_point.resize(3, 3 * point_num);
    A_start_point.insert(0, 0) = 1.0;
    A_start_point.insert(1, 1) = 1.0;
    A_start_point.insert(2, 2) = 1.0;
    ub_start_point[0] = final_dp_path.front().l;
    ub_start_point[1] = final_dp_path.front().l_prime;
    ub_start_point[2] = final_dp_path.front().l_prime_prime;
    lb_start_point = ub_start_point;

    // 2.4把所有约束拼起来
    Eigen::SparseMatrix<double> A_total, A_toal_transpose;
    Eigen::VectorXd lb_total, ub_total;
    A_total.resize(A_continuity.rows() + A_collision.rows() + A_start_point.rows(), 3 * point_num);
    A_toal_transpose.resize(3 * point_num, A_continuity.rows() + A_collision.rows() + A_start_point.rows());
    // 对于主列（默认）的稀疏矩阵，是不允许用middlerows来赋值的，所以咱们只能先转置，在赋值，再转置回来
    A_toal_transpose.middleCols(0, A_continuity.rows()) = A_continuity.transpose();
    A_toal_transpose.middleCols(A_continuity.rows(), A_collision.rows()) = A_collision.transpose();
    A_toal_transpose.middleCols(A_continuity.rows() + A_collision.rows(), A_start_point.rows()) = A_start_point.transpose();
    A_total = A_toal_transpose.transpose();
    ub_total.resize(ub_continuity.rows() + ub_collision.rows() + ub_start_point.rows());
    lb_total.resize(lb_continuity.rows() + lb_collision.rows() + lb_start_point.rows());
    ub_total << ub_continuity, ub_collision, ub_start_point;
    lb_total << lb_continuity, lb_collision, lb_start_point;

    //------------------3. 求解------------------
    //3.1初始化
    _path_qp_solver.data()->setNumberOfVariables(3 * point_num);
    _path_qp_solver.data()->setNumberOfConstraints(6 * point_num + 1);
    if (!_path_qp_solver.data()->setHessianMatrix(H)){
            return false;
        }
    if (!_path_qp_solver.data()->setGradient(f)){
        return false;
    }
    if (!_path_qp_solver.data()->setLinearConstraintsMatrix(A_total)){
        return false;
    }
    if (!_path_qp_solver.data()->setBounds(lb_total, ub_total)){
        return false;
    }
    if (!_path_qp_solver.initSolver()){
        return false;
    }
    //3.2求解并赋初值
    if (!(_path_qp_solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError)){
        return false;
    }
    auto solution = _path_qp_solver.getSolution();
    for (size_t i = 0; i < point_num; i++){
        FrenetPoint qp_path_point;
        qp_path_point.s = final_dp_path[i].s;
        qp_path_point.l = solution[3 * i + 0];
        qp_path_point.l_prime = solution[3 * i + 1];
        qp_path_point.l_prime_prime = solution[3 * i + 2];
        init_qp_path.emplace_back(qp_path_point);
    }
    _path_qp_solver.clearSolverVariables();
    _path_qp_solver.data()->clearLinearConstraintsMatrix();
    _path_qp_solver.data()->clearHessianMatrix();
    _path_qp_solver.clearSolver();


    return true;
}
