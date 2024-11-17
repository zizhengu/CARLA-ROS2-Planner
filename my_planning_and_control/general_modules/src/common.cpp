#include "common.h"


void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path){
    int n = path->size();
    // 它的内部存储方式是一维的线性结构,所有元素会被初始化为 0.0
    std::vector<double> dx(n), dy(n), ds(n), dtheta(n);

    for (int i = 0; i < n; i++){
        if (i == 0){
            dx[i] = (*path)[i + 1].x - (*path)[i].x;
            dy[i] = (*path)[i + 1].y - (*path)[i].y;
        }
        else if (i == (n-1)){
            dx[i] = (*path)[i].x - (*path)[i - 1].x;
            dy[i] = (*path)[i].y - (*path)[i - 1].y;
        }
        else{
            dx[i] = ((*path)[i + 1].x - (*path)[i - 1].x) / 2.0;
            dy[i] = ((*path)[i + 1].y - (*path)[i - 1].y) / 2.0;
        }
        ds[i] = std::hypot(dx[i], dy[i]);
        (*path)[i].heading = std::atan2(dy[i], dx[i]);
    }

    for (int i = 0; i < n; i++){
        if (i == 0){
            dtheta[i] = (*path)[i + 1].heading - (*path)[i].heading;
        }
        else if (i == (n - 1)){
            dtheta[i] = (*path)[i].heading - (*path)[i - 1].heading;
        }
        else{
            dtheta[i] = ((*path)[i + 1].heading - (*path)[i - 1].heading) / 2.0;
        }
        (*path)[i].kappa = dtheta[i] / ds[i];
    }
}

void Calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const Eigen::Vector2d point,
                                int &match_point_index, PathPoint &project_point){
    /**
     * 1.找到匹配点（距离最近的点）
     */
    PathPoint match_point;
    double min_distance = std::numeric_limits<double>::max();
    size_t count_num = 0;
    for (size_t i = 0; i < path->size(); i++){
        double cur_distance = std::hypot((*path)[i].x - point[0], (*path)[i].y - point[1]);
        if (cur_distance < min_distance){
            min_distance = cur_distance;
            match_point_index = i;
            count_num = 0;
        }
        count_num ++;
        if (count_num > 30){
            break;
        }
    }
    match_point = (*path)[match_point_index];
    
    /**
     * 2.由匹配点计算投影点
     * 注意计算heading：
     * apollo方法 heading = match_point.heading; 
     * up方法 heading = match_point.heading + kappa*tau_m
     */
    Eigen::Vector2d d; //由匹配点指向待投影点的向量
    Eigen::Vector2d match_point_vector(match_point.x, match_point.y); //匹配点向量
    Eigen::Vector2d tau_m(std::cos(match_point.heading), std::sin(match_point.heading)); // 匹配点切线向量
    d = point - match_point_vector;
    auto project_point_vector = match_point_vector + d.dot(tau_m) * tau_m;
    project_point.x = project_point_vector[0];
    project_point.y = project_point_vector[1]; 
    project_point.heading = match_point.heading + match_point.kappa * d.dot(tau_m);
    project_point.kappa = match_point.kappa;
}

// 针对trajectory_point进行重载
void Calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const TrajectoryPoint &trajectory_point,
                                int &match_point_index, PathPoint &match_point, PathPoint &project_point){
    /**
     * 1.找到匹配点（距离最近的点）
     */
    double min_distance = std::numeric_limits<double>::max();
    size_t count_num = 0;
    for (size_t i = 0; i < path->size(); i++){
        double cur_distance = std::hypot(((*path)[i].x - trajectory_point.x), ((*path)[i].y - trajectory_point.y));
        if (cur_distance < min_distance){
            count_num = 0;
            match_point_index = i;
            min_distance = cur_distance;
        }
        count_num++;
        if (count_num > 30){
            break;
        }
    }
    //匹配点赋值
    match_point = (*path)[match_point_index];

    /**
     * 2.由匹配点计算投影点
     * 注意计算heading：
     * apollo方法 heading = match_point.heading;
     * up方法 heading = match_point.heading + kappa*tau_m
     */
    Eigen::Vector2d match_point_vector(match_point.x, match_point.y);
    Eigen::Vector2d trajectory_point_vector(trajectory_point.x, trajectory_point.y);
    Eigen::Vector2d d;
    Eigen::Vector2d tau_m(std::cos(match_point.heading), std::sin(match_point.heading));
    d = trajectory_point_vector - match_point_vector;
    auto project_point_vector = match_point_vector + d.dot(tau_m) * tau_m;
    //投影点赋值
    project_point.x = project_point_vector[0];
    project_point.x = project_point_vector[1];
    project_point.heading = match_point.heading + match_point.kappa * d.dot(tau_m);
    project_point.kappa = match_point.kappa;
}

/**
 * 计算车辆投影点到path中每个点的距离
 */
std::vector<double> calculate_index_to_s(std::shared_ptr<std::vector<PathPoint>> path, std::shared_ptr<VehicleState> vehicle_state)
{   
    //1.获取投影点索引
    Eigen::Vector2d host_point(vehicle_state->x, vehicle_state->y);
    PathPoint project_point, match_point;
    int match_point_index;
    Calculate_projection_point(path, host_point, match_point_index, project_point);
    match_point = (*path)[match_point_index];

    // 2.计算初始表，起点为path的第一个点
    std::vector<double> index2s;
    double distance = 0.0;
    for (size_t i = 1; i < path->size(); i++){
        distance += std::hypot((*path)[i].x - (*path)[i - 1].x, (*path)[i].y - (*path)[i - 1].y);
        index2s.push_back(distance);
    }

    //3.将原点平移至投影点
    Eigen::Vector2d match_to_pro(project_point.x - match_point.x, project_point.y - match_point.y);
    Eigen::Vector2d tau_match(std::cos(match_point.heading), std::sin(match_point.heading));

    double delta_s = 0.0;
    if (match_to_pro.dot(tau_match) > 0){
        delta_s = index2s[match_point_index] + match_to_pro.norm();
    }
    else if (match_to_pro.dot(tau_match) < 0){
        delta_s = index2s[match_point_index] - match_to_pro.norm();
    }
    else{
        delta_s = index2s[match_point_index];
    }

    //4.整体pingyi
    for(auto && element : index2s){
        element -= delta_s;
    }

    return index2s;
}


/**
 * 计算自车的笛卡尔->自然坐标系
 * 
 */
FrenetPoint cartesion_to_frenet(const TrajectoryPoint &host, const PathPoint &projection_point, const PathPoint &match_point,
                                std::vector<double> index2s, const int &math_point_index){
    FrenetPoint frenet_point;
    //待转换的向量
    Eigen::Vector2d r_host(host.x, host.y);
    Eigen::Vector2d tau_host(std::cos(host.heading), std::sin(host.heading));
    Eigen::Vector2d nor_host(-std::sin(host.heading), std::cos(host.heading));
    Eigen::Vector2d a_host(host.ax, host.ay);
    double v_host = host.v;

    //投影点相关向量
    Eigen::Vector2d r_pro(projection_point.x, projection_point.y);
    Eigen::Vector2d tau_pro(std::cos(projection_point.heading), std::sin(projection_point.heading));
    Eigen::Vector2d nor_pro(-std::sin(projection_point.heading), std::cos(projection_point.heading));
    double kr = projection_point.kappa;

    //计算frenet坐标，具体公式见讲义
    double l = (r_host - r_pro).dot(nor_host);
    double s_dot = v_host * (tau_host.dot(tau_pro)) / (1 - kr * l);
    double l_dot = v_host * (tau_host.dot(nor_pro));
    double l_prime = l_dot / (s_dot + 1e-10);
    double s_dot_dot = (a_host.dot(tau_pro) + kr * s_dot * s_dot * l_prime + s_dot * s_dot * kr * l_prime) / (1 - kr * l);
    double l_dot_dot = a_host.dot(nor_pro) - kr * (1 - kr * l) * s_dot * s_dot;
    double l_prime_prime = (l_dot_dot - l_prime * s_dot_dot) / (s_dot * s_dot + 1e-4);
    double s = index2s[math_point_index];

    //针对s，需要根据投影点和匹配点之间的位置关系进行加减 
    //(存疑，S的起点到底在哪？)
    Eigen::Vector2d match_point_to_host(host.x - match_point.x, host.y - match_point.y);
    Eigen::Vector2d tau_match(std::cos(match_point.heading), std::sin(match_point.heading));
    if (match_point_to_host.dot(tau_match) > 0){
        s += std::hypot(match_point.x - projection_point.x, match_point.y - projection_point.y);
    }
    else if (match_point_to_host.dot(tau_match) < 0){
        s = std::hypot(match_point.x - projection_point.x, match_point.y - projection_point.y);
    }

    frenet_point.s = s;
    frenet_point.l = l;
    frenet_point.s_dot = s_dot;
    frenet_point.l_dot = l_dot;
    frenet_point.l_prime = l_prime;
    frenet_point.s_dot_dot = s_dot_dot;
    frenet_point.l_dot_dot = l_dot_dot;
    frenet_point.l_prime_prime = l_prime_prime;

    return frenet_point;
}

std::vector<TrajectoryPoint> frenet_to_cartesion(const std::vector<FrenetPoint> &frenet_point_set,
                                                 const std::shared_ptr<std::vector<PathPoint>> cartesian_path,
                                                 const std::vector<double> cartesian_path_index2s){
    std::vector<TrajectoryPoint> trajectory_point_set;

    for (size_t i = 0; i < frenet_point_set.size(); i++){
        FrenetPoint frenet_point_host = frenet_point_set[i];
        //1.寻找匹配点
        //处理边界情况
        int match_point_index = -1;
        if (frenet_point_host.s < cartesian_path_index2s.front()){
            match_point_index = 0;
        }
        else if (frenet_point_host.s > cartesian_path_index2s.back()){
            match_point_index = frenet_point_set.size() - 1 ;
        }
        else{
            for (size_t j = 0; j < cartesian_path_index2s.size(); j++){
                if (frenet_point_host.s >= cartesian_path_index2s[j] && frenet_point_host.s < cartesian_path_index2s[j+1]){
                    if (std::abs(frenet_point_host.s - cartesian_path_index2s[j]) > std::abs(frenet_point_host.s - cartesian_path_index2s[j+1])){
                        match_point_index = j + 1;
                        break;
                    }
                    else{
                        match_point_index = j;
                        break;
                    }
                }
            }
        }
        PathPoint match_point = (*cartesian_path)[match_point_index];

        //2匹配点的位置向量、切向量
        Eigen::Vector2d p_match_point(match_point.x, match_point.y);
        Eigen::Vector2d tau_m(std::cos(match_point.heading), std::sin(match_point.heading));

        //3投影点
        double delta_s = frenet_point_host.s - cartesian_path_index2s[match_point_index];
        Eigen::Vector2d p_project_point = p_match_point + delta_s * tau_m;
        double kappa_project_point = match_point.kappa;
        double heading_project_point = match_point.heading + delta_s * kappa_project_point;
        Eigen::Vector2d nor_project_point(-std::sin(heading_project_point), std::cos(heading_project_point));

        //4坐标转换，公式见讲义
        TrajectoryPoint trajectory_point_host;
        Eigen::Vector2d p_host = p_project_point + frenet_point_host.l * nor_project_point;
        trajectory_point_host.x = p_host[0];
        trajectory_point_host.y = p_host[1];
        double c = 1.0 - kappa_project_point * frenet_point_host.l;
        double dealta_heading = std::atan2(frenet_point_host.l_prime, c);
        trajectory_point_host.heading = dealta_heading + heading_project_point;
        trajectory_point_host.kappa = ((frenet_point_host.l_prime_prime + kappa_project_point * frenet_point_host.l_prime * std::tan(dealta_heading)) * std::pow(std::cos(dealta_heading), 2) / c + kappa_project_point) * std::cos(dealta_heading) / c;

        trajectory_point_set.emplace_back(trajectory_point_host);
    }
    return trajectory_point_set;
}

// 物体对象转换至轨迹点
TrajectoryPoint object_to_trajectory_point(const derived_object_msgs::msg::Object object){
    TrajectoryPoint trajectory_point;
    trajectory_point.x = object.pose.position.x;
    trajectory_point.y = object.pose.position.y;
    // “twist”是一个重要的概念，通常用于描述物体的线速度和角速度
    trajectory_point.v = std::sqrt(std::pow(object.twist.linear.x, 2) + 
                                   std::pow(object.twist.linear.y, 2) + 
                                   std::pow(object.twist.linear.z, 2));
    trajectory_point.kappa = 0;
    trajectory_point.ax = object.accel.linear.x;
    trajectory_point.ax = object.accel.linear.y;

    //利用tf2读取四元数，提取yaw角
    tf2::Quaternion tf2_q;
    tf2::fromMsg(object.pose.orientation, tf2_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    trajectory_point.heading = yaw;

    return trajectory_point;
}

// 车辆对象转换至轨迹点
TrajectoryPoint vehicle_state_to_trajectory_point(const std::shared_ptr<VehicleState> vehicle_state){
    TrajectoryPoint trajectory_point;
    trajectory_point.x = (*vehicle_state).x;
    trajectory_point.y = (*vehicle_state).y;
    trajectory_point.heading = (*vehicle_state).heading;
    trajectory_point.v = (*vehicle_state).v;
    trajectory_point.ax = (*vehicle_state).ax;
    trajectory_point.ay = (*vehicle_state).ay;

    return trajectory_point;
}

// TODO:如下的三个重载函数有重复的地方，尚未优化
// 用于将动态障碍物投影至路径规划的trajectory下
void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object> &object_set,
                                 const std::vector<TrajectoryPoint> &trajectory, const TrajectoryPoint &original_point,
                                 std::vector<FrenetPoint> &frenet_set){
    // 1.若为空，直接退出
    if (object_set.empty())
    {
        return;
    }

    // 2.计算index2s表
    // 2.1原始index2s表
    std::vector<double> index2s;
    index2s.emplace_back(0.0);
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        index2s.emplace_back((std::hypot(trajectory[i].x - trajectory[i - 1].x, trajectory[i].y - trajectory[i - 1].y)) + index2s.back());
    }
    // 2.2搜索匹配点
    int match_point_index = -1;
    double min_distace = std::numeric_limits<double>::max();
    for (int j = 0; j < (int)trajectory.size(); j++)
    {
        double current_distace = hypot(trajectory[j].x - original_point.x, trajectory[j].y - original_point.y);
        if (current_distace < min_distace)
        {
            min_distace = current_distace;
            match_point_index = j;
        }
    }
    // 2.3由匹配点计算投影点
    // 匹配点信息：位置向量，航向角，曲率，切向量，法向量
    Eigen::Vector2d match_point_p(trajectory[match_point_index].x, trajectory[match_point_index].y);
    double match_point_heading = trajectory[match_point_index].heading;
    double match_point_kappa = trajectory[match_point_index].kappa;
    Eigen::Vector2d match_point_tau(std::cos(match_point_heading), std::sin(match_point_heading));
    Eigen::Vector2d match_point_nor(-std::sin(match_point_heading), std::cos(match_point_heading));

    Eigen::Vector2d match_point_to_original_point(original_point.x - match_point_p[0], original_point.y - match_point_p[1]);
    // 计算投影点信息：位置向量，航向角，曲率，切向量，法向量
    Eigen::Vector2d projection_point_p = match_point_p + match_point_tau * (match_point_tau.dot(match_point_to_original_point));
    double projection_point_heading = match_point_heading + match_point_kappa * (match_point_tau.dot(match_point_to_original_point));
    double projection_point_kappa = match_point_kappa;
    Eigen::Vector2d projection_point_tau(std::cos(projection_point_heading), std::sin(projection_point_heading));
    Eigen::Vector2d projection_point_nor(-std::sin(projection_point_heading), std::cos(projection_point_heading));

    // 2.4修正index2s表
    Eigen::Vector2d match_point_to_projection_point(projection_point_p[0] - match_point_p[0],
                                                    projection_point_p[1] - match_point_p[1]); // 匹配点指向投影点的向量
    double delta_s = index2s[match_point_index];                                               // 修正量
    if (match_point_to_projection_point.dot(match_point_tau) > 0)                              // 判断投影点在匹配点的前面还是后面
    {
        delta_s += match_point_to_projection_point.norm();
    }
    else
    {
        delta_s -= match_point_to_projection_point.norm();
    }

    for (size_t i = 0; i < index2s.size(); i++) // 遍历每一个元素，逐个修正
    {
        index2s[i] -= delta_s;
    }

    //3.逐个坐标转化
    for (auto && object : object_set){

        TrajectoryPoint host = object_to_trajectory_point(object);

        // 带转化对象相关信息：位置向量，航向角，曲率，切向量，方向向量
        Eigen::Vector2d host_p(host.x, host.y);
        double host_heading = host.heading;
        double host_v = host.v;
        Eigen::Vector2d host_a(host.ax, host.ay);
        Eigen::Vector2d host_tau(std::cos(host_heading), std::sin(host_heading));
        Eigen::Vector2d host_nor(-std::sin(host_heading), std::cos(host_heading));

        // 搜索投影点
        match_point_index = -1;
        min_distace = std::numeric_limits<double>::max();
        for (int j = 0; j < (int)trajectory.size(); j++)
        {
            double current_distace = hypot(trajectory[j].x - host.x, trajectory[j].y - host.y);
            if (current_distace < min_distace)
            {
                min_distace = current_distace;
                match_point_index = j;
            }
        }

        // 匹配点信息：位置向量，航向角，曲率，切向量，法向量
        match_point_p << trajectory[match_point_index].x, trajectory[match_point_index].y;
        match_point_heading = trajectory[match_point_index].heading;
        match_point_kappa = trajectory[match_point_index].kappa;
        match_point_tau << std::cos(match_point_heading), std::sin(match_point_heading);
        match_point_nor << -std::sin(match_point_heading), std::cos(match_point_heading);
        // 匹配点到待转化点向量
        Eigen::Vector2d match_point_to_host(host_p[0] - match_point_p[0], host_p[1] - match_point_p[1]);
        // 投影点信息：位置向量，航向角，曲率，切向量，法向量
        projection_point_p = match_point_p + match_point_tau * (match_point_tau.dot(match_point_to_host));
        projection_point_heading = match_point_heading + match_point_kappa * (match_point_tau.dot(match_point_to_host));
        projection_point_kappa = match_point_kappa;
        projection_point_tau << std::cos(projection_point_heading), std::sin(projection_point_heading);
        projection_point_nor << -std::sin(projection_point_heading), std::cos(projection_point_heading);
        // 坐标转化
        double l = (host_p - projection_point_p).dot(projection_point_nor);
        double c = 1 - projection_point_kappa * l;
        double s_dot = host_v * (host_tau.dot(projection_point_tau)) / c;
        double l_dot = host_v * (host_tau.dot(projection_point_nor));
        double l_prime = l_dot / (s_dot + 1e-10);
        double s_dot_dot = host_a.dot(projection_point_tau) / c + projection_point_kappa * s_dot * s_dot * l_prime / c + s_dot * s_dot * projection_point_kappa * l_prime / c;
        double l_dot_dot = host_a.dot(projection_point_nor) - projection_point_kappa * c * s_dot * s_dot;
        double l_prime_prime = (l_dot_dot - l_prime * s_dot_dot) / (s_dot * s_dot + 1e-3);
        double s = index2s[match_point_index];

        match_point_to_projection_point << projection_point_p[0] - match_point_p[0], projection_point_p[1] - match_point_p[1]; // 匹配点指向投影点的向量
        if (match_point_to_projection_point.dot(match_point_tau) > 0)                                                          // 判断投影点在匹配点的前面还是后面
        {
            s += match_point_to_projection_point.norm();
        }
        else
        {
            s -= match_point_to_projection_point.norm();
        }

        FrenetPoint host_frenet;
        host_frenet.s = s;
        host_frenet.l = l;
        host_frenet.s_dot = s_dot;
        host_frenet.l_dot = l_dot;
        host_frenet.l_prime = l_prime;
        host_frenet.s_dot_dot = s_dot_dot;
        host_frenet.l_dot_dot = l_dot_dot;
        host_frenet.l_prime_prime = l_prime_prime;

        frenet_set.emplace_back(host_frenet);
    }
}

// 用于将静态障碍物投影至参考线下
void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object> &object_set,
                                 const std::vector<PathPoint> &path, std::shared_ptr<VehicleState> vehicle_state,
                                 std::vector<FrenetPoint> &frenet_set){
    std::vector<TrajectoryPoint> trajectory;
    for (auto && path_point : path){
        TrajectoryPoint trajectory_point;
        trajectory_point.x = path_point.x;
        trajectory_point.y = path_point.y;
        trajectory_point.heading = path_point.heading;
        trajectory_point.kappa = path_point.kappa;

        trajectory.emplace_back(trajectory_point);
    }

    TrajectoryPoint original_point = vehicle_state_to_trajectory_point(vehicle_state);

    cartesion_set_to_frenet_set(object_set, trajectory, original_point, frenet_set);
}

void cartesion_set_to_frenet_set(const TrajectoryPoint &trajectory_point, const std::vector<PathPoint> &path, std::shared_ptr<VehicleState> vehicle_state, std::vector<FrenetPoint> &frenet_set){
    derived_object_msgs::msg::Object object;
    object.pose.position.x = trajectory_point.x;
    object.pose.position.y = trajectory_point.y;
    object.pose.position.z = 0.0;
    object.accel.linear.x = trajectory_point.ax;
    object.accel.linear.y = trajectory_point.ay;
    tf2::Quaternion tf_q;
    tf_q.setEuler(0.0, 0.0, trajectory_point.heading);
    object.pose.orientation.x = tf_q.x();
    object.pose.orientation.y = tf_q.y();
    object.pose.orientation.z = tf_q.z();
    object.pose.orientation.w = tf_q.w();

    std::vector<derived_object_msgs::msg::Object> object_set;
    object_set.emplace_back(object);

    cartesion_set_to_frenet_set(object_set, path, vehicle_state, frenet_set);
}

void cartesion_set_to_frenet_set(const std::shared_ptr<VehicleState> &ego_state,
                                 const std::vector<TrajectoryPoint> &trajectory,
                                 std::vector<FrenetPoint> &frenet_set)
{
    derived_object_msgs::msg::Object ego_obj;
    ego_obj.pose.position.x = ego_state->x;
    ego_obj.pose.position.y = ego_state->y;
    ego_obj.pose.position.z = 0.0;
    ego_obj.twist.linear.x = ego_state->v * std::cos(ego_state->heading);
    ego_obj.twist.linear.y = ego_state->v * std::sin(ego_state->heading);
    ego_obj.twist.linear.z = 0.0;
    ego_obj.accel.linear.x = ego_state->ax;
    ego_obj.accel.linear.y = ego_state->ay;
    ego_obj.accel.linear.z = 0.0;
    tf2::Quaternion tf_q;
    tf_q.setEuler(0.0, 0.0, ego_state->heading);
    ego_obj.pose.orientation.x = tf_q.x();
    ego_obj.pose.orientation.y = tf_q.y();
    ego_obj.pose.orientation.z = tf_q.z();
    ego_obj.pose.orientation.w = tf_q.w();

    std::vector<derived_object_msgs::msg::Object> obj_set;
    obj_set.emplace_back(ego_obj);

    cartesion_set_to_frenet_set(obj_set, trajectory, trajectory.front(), frenet_set);
}