#include "reference_line.h"

ReferenceLine::ReferenceLine(){
    // 初始化参考线
    _forward_num = 80;
    _backward_num = 15;
    _is_first_run = true;
    _previous_match_point_index = 0;

    // 参考线平滑
    _cost_smooth = 5.0;
    _cost_geometry = 10.0;
    _cost_compact = 50.0;
    _max_x_offset = 0.5;
    _max_y_offset = 0.5;

    /**
    * 不能写为如下形式：
     _smooth_solver = std::shared_ptr<OsqpEigen::Solver>(); // 重置为 nullptr
     * 热启动加快收敛速度：
     * 当你在相似的优化问题上多次运行求解器时，热启动可以使求解器从上一次的解出发，而不是从初始猜测开始。这通常会显著减少迭代次数，从而加快收敛。
    */
    _smooth_solver = std::make_shared<OsqpEigen::Solver>();//创建求解器
    _smooth_solver->settings()->setWarmStart(true);//求解器 热启动
}

bool ReferenceLine::run_step(std::shared_ptr<VehicleState> current_ego_state, std::shared_ptr<std::vector<PathPoint>> global_path, std::shared_ptr<std::vector<PathPoint>> reference_line){
    //1.在global_path上搜索匹配点的位置
    int match_point_index = 0;
    double min_distance = 1e10;
    int num_count = 0;
    if (_is_first_run)
    {
        _is_first_run = false;
        for (int i = 0; i < (int)global_path->size(); i++){
            double current_distance = std::hypot(current_ego_state->x - global_path->at(i).x, current_ego_state->y - global_path->at(i).y);
            if (current_distance < min_distance){
                min_distance = current_distance;
                match_point_index = i;
                num_count = 0;
            }
            num_count += 1;
            if (num_count > 50)
            {
                break;
            }
        }
    }
    else{
        int start_index = _previous_match_point_index;
        PathPoint start_point = global_path->at(start_index);
        Eigen::Vector2d tau(std::cos(start_point.heading), std::sin(start_point.heading));
        Eigen::Vector2d start_to_ego(current_ego_state->x - start_point.x, current_ego_state->y - start_point.y);
        //不能直接点积，因为要判断方向
        double dot = start_to_ego.normalized().dot(tau);

        if (dot >1e-2){
            for (int i = start_index; i < (int)global_path->size(); i++){
                double current_distance = std::hypot(current_ego_state->x - global_path->at(i).x, current_ego_state->y - global_path->at(i).y);
                if (current_distance < min_distance)
                {
                    min_distance = current_distance;
                    match_point_index = i;
                    num_count = 0;
                }
                num_count += 1;
                if (num_count > 30)
                {
                    break;
                }
            }
        }
        else if (dot < -1e-2){
            for (int i = start_index; i >= 0 ; i--)
            {
                double current_distance = std::hypot(current_ego_state->x - global_path->at(i).x, current_ego_state->y - global_path->at(i).y);
                if (current_distance < min_distance)
                {
                    min_distance = current_distance;
                    match_point_index = i;
                    num_count = 0;
                }
                num_count += 1;
                if (num_count > 30)
                {
                    break;
                }
            }
        }
        else{
            match_point_index = start_index;
        }
    }
    _previous_match_point_index = match_point_index;

    //2.将匹配点前后的点组成初始参考线
    /**
     * 为什么使用 std::deque
     * std::deque 是一个双端队列，它可以在两端高效地插入和删除元素。这与 std::vector 相比，后者在头部插入时效率较低，因为它需要移动所有现有元素。
     * std::deque 可以动态调整大小，适合在不确定元素数量的情况下使用。
     * std::deque 提供了在头尾进行操作的灵活性，特别适合需要从两端添加或移除元素的场景。
     *                   head |------------------------------|  back
     * global_path(10),global_path(11),global_path(12),...,global_path(math_point_index),...,global_path(90),global_path(91),global_path(92)
     */
    auto init_reference_line = std::make_shared<std::deque<PathPoint>>();
    for (int i = 0; i <= _forward_num && i + match_point_index < (int)global_path->size(); i++){
        init_reference_line->push_back(global_path->at(i + match_point_index));
    }

    for (int i = 1; i <= _backward_num && match_point_index - i >= 0; i++)
    {
        init_reference_line->push_front(global_path->at(match_point_index - i));
    }

    //3.对初始参考线进行平滑
    //二次规划所需的稀疏H矩阵
    Eigen::SparseMatrix<double> H_smooth, H_geometry, H_compact, H;
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd f, lowbound, upbound;
    int var_num = init_reference_line->size();

    //3.1 H矩阵初始化
    H_smooth.resize(2 * var_num - 4, 2 * var_num);
    H_geometry.resize(2 * var_num, 2 * var_num);
    H_compact.resize(2 * var_num - 2, 2 * var_num);
    H.resize(2 * var_num, 2 * var_num);

    // 3.2 H矩阵赋值
    // 因为稀疏矩阵不支持(2*i,2*i)直接访问方式,需要用insert()方法
    for (int i = 0; i < var_num - 2; i++){
        H_smooth.insert(2 * i, 2 * i) = 1;
        H_smooth.insert(2 * i + 1, 2 * i + 1) = 1;
        H_smooth.insert(2 * i, 2 * i + 2) = -2;
        H_smooth.insert(2 * i + 1, 2 * i + 3) = -2;
        H_smooth.insert(2 * i, 2 * i + 4) = 1;
        H_smooth.insert(2 * i + 1, 2 * i + 5) = 1;
    }

    for (int i = 0; i < 2 * var_num; i++){
        H_geometry.insert(i, i) = 1;
    }

    //源代码：
    // for (int i = 0; i < var_num - 1; i++)
    // {
    //     H_compact.insert(2 * i, 2 * i) = -1;
    //     H_compact.insert(2 * i, 2 * i + 2) = 1;
    //     H_compact.insert(2 * i + 1, 2 * i + 1) = -1;
    //     H_compact.insert(2 * i + 1, 2 * i + 3) = 1;
    // }
    for (int i = 0; i < var_num - 1; i++)
    {
        H_compact.insert(2 * i, 2 * i) = 1;
        H_compact.insert(2 * i, 2 * i + 2) = -1;
        H_compact.insert(2 * i + 1, 2 * i + 1) = 1;
        H_compact.insert(2 * i + 1, 2 * i + 3) = -1;
    }

    H = 2 * (_cost_smooth * H_smooth.transpose() * H_smooth + _cost_geometry * H_geometry.transpose() * H_geometry + _cost_compact * H_compact.transpose() * H_compact);

    //3.3 f向量赋值
    f.resize(2 * var_num);
    for (int i = 0; i < var_num; i++){
        f(2 * i) = init_reference_line->at(i).x;
        f(2 * i + 1) = init_reference_line->at(i).y;
    }
    f = -2 * _cost_geometry * f;

    //3.4 不等式约束条件初始化
    A.resize(2 * var_num, 2 * var_num);
    lowbound.resize(2 * var_num);
    upbound.resize(2 * var_num);

    for (int i = 0; i < var_num; i++){
        A.insert(2 * i, 2 * i) = 1;
        A.insert(2 * i + 1, 2 * i + 1) = 1;
        lowbound(2 * i) = init_reference_line->at(i).x - _max_x_offset;
        lowbound(2 * i + 1) = init_reference_line->at(i).y - _max_y_offset;
        upbound(2 * i) = init_reference_line->at(i).x + _max_x_offset;
        upbound(2 * i + 1) = init_reference_line->at(i).y + _max_y_offset;
    }

    //3.5 设置求解器矩阵
    _smooth_solver->data()->setNumberOfVariables(2 * var_num);
    _smooth_solver->data()->setNumberOfConstraints(2 * var_num);
    if (!_smooth_solver->data()->setHessianMatrix(H)){
        return false;
    }
    if (!_smooth_solver->data()->setGradient(f)){
        return false;
    }
    if (!_smooth_solver->data()->setLinearConstraintsMatrix(A)){
        return false;
    }
    if (!_smooth_solver->data()->setUpperBound(upbound)){
        return false;
    }
    if (!_smooth_solver->data()->setLowerBound(lowbound)){
        return false;
    }
    /**
     * _smooth_solver->initSolver() 的作用是初始化求解器。具体来说，它执行以下任务：
        设置内部状态：准备求解器以便进行优化计算，包括分配必要的内存和资源。
        加载数据：将之前定义的 Hessian 矩阵、梯度、约束矩阵以及上下界等数据传递给求解器。
        检查一致性：确认所有设置的数据结构都是有效的，确保没有错误或不一致的参数。
     */
    if(!_smooth_solver->initSolver()){
        return false;
    }

    // 3.6 求解
    /*
    一般而言，qp_solution 是一个向量或数组，包含了优化后的变量值
    Eigen::VectorXd qp_solution; // 假设这是从求解器获取的解决方案
    qp_solution[0] 是第一个点的 x 坐标
    qp_solution[1] 是第一个点的 y 坐标
    qp_solution[2] 是第二个点的 x 坐标
    qp_solution[3] 是第二个点的 y 坐标
    ...
    */
    _smooth_solver->solveProblem();
    auto qp_solution = _smooth_solver->getSolution();

    //4.将求解结果导入到referenceline中
    reference_line->resize(var_num);
    for (int i = 0; i < var_num; i++){
        reference_line->at(i).x = qp_solution[2 * i];
        reference_line->at(i).y = qp_solution[2 * i + 1];
    }
    Calculate_heading_and_kappa(reference_line);

    // 5.清空求解器
    _smooth_solver->data()->clearHessianMatrix();
    _smooth_solver->data()->clearLinearConstraintsMatrix();
    _smooth_solver->clearSolverVariables();
    _smooth_solver->clearSolver();
    return true;
}