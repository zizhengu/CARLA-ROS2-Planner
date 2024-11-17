#include "main_function.h"
#include <random>
#include <cstdlib>
class MyPlanningANDControl : public rclcpp::Node
{
public:
    // 这是类的构造函数。当实例化 MyPlanningANDControl 时，会调用这个构造函数。
    //通过初始化列表调用父类的构造函数，并给节点命名为 "my_planning_and_control"。这个名称在 ROS 网络中唯一标识该节点。
    MyPlanningANDControl() : Node("my_planning_and_control"), MIN_DISTANCE_PERCENTAGE(0.5){
        // 主车名字
        role_name = "ego_vehicle";

        // 控制器时钟，按照控制时间步来执行控制
        //TODO:更改控制和规划的频率，目前好像是20ms
        _control_time_step = 0.02; // 20ms执行一次控制
        /**
         * _control_timer：创建一个壁钟定时器，用于定期调用控制函数。
         * std::chrono::milliseconds(int(_control_time_step * 1000))：将控制时间步转换为毫秒（20 毫秒）。
         * std::bind(&MyPlanningANDControl::control_run_step, this)：使用 std::bind 绑定当前对象 (this) 和 control_run_step 函数，这样定时器到期时会调用该
         */
        _control_timer = this->create_wall_timer(std::chrono::milliseconds(int(_control_time_step * 1000)), std::bind(&MyPlanningANDControl::control_run_step, this));
        // 控制器的实例化
        _longitudinal_pid_controller = std::make_shared<LongitudinalPIDController>();
        _lateral_pid_controller = std::make_shared<LateralPIDController>();
        _lateral_lqr_controller = std::make_shared<LaterLQRController>();
        _lon_cascade_pid_controller = std::make_shared<LonCascadePIDController>();
        _mpc_controller = std::make_shared<MPCController>();
        // 规划时钟
        _planning_time_step = 0.1;                                     // 100ms执行一次规划
        _reference_line_generator = std::make_shared<ReferenceLine>(); // 参考线生成器
        _reference_line = std::make_shared<std::vector<PathPoint>>();
        _planning_timer = this->create_wall_timer(std::chrono::milliseconds(int(_planning_time_step * 1000)), std::bind(&MyPlanningANDControl::planning_run_step, this));
        _emplanner = std::make_shared<EMPlanner>();

        // 订阅方
        // 创建里程计订阅方，订阅车辆当前位姿消息
        /**
         * std::placeholders::_1 是占位符，表示订阅者接收到的消息将作为参数传递给 odometry_cb 函数。
         * 10表示队列大小
         */
        _current_ego_state = std::make_shared<VehicleState>();
        _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            // 话题名称
            "/carla/" + role_name + "/odometry",
            10, 
            std::bind(&MyPlanningANDControl::odometry_cb, this, std::placeholders::_1));
        // 创建惯性导航订阅方，订阅车辆当前加速度和角速度消息
        _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/carla/" + role_name + "/imu",
            10,
            std::bind(&MyPlanningANDControl::imu_cb, this, std::placeholders::_1));
        // 创建车辆信息订阅方，订阅车辆id号
        _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
            "/carla/ego_vehicle/vehicle_info",
            10,
            std::bind(&MyPlanningANDControl::ego_info_cb, this, std::placeholders::_1));

        // 创建期望速度订阅方
        _target_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/carla/" + role_name + "/speed_command",
            10,
            std::bind(&MyPlanningANDControl::target_speed_cb, this, std::placeholders::_1));

        // 创建路径订阅方
        _global_path = std::make_shared<std::vector<PathPoint>>();
        _path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
            "/carla/" + role_name + "/waypoints",
            10,
            std::bind(&MyPlanningANDControl::path_cb, this, std::placeholders::_1));
        _object_array_subscriber = this->create_subscription<derived_object_msgs::msg::ObjectArray>(
            "/carla/ego_vehicle/objects",
            10,
            std::bind(&MyPlanningANDControl::objects_cb, this, std::placeholders::_1));

        // 发布方
        _control_cmd_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/" + role_name + "/vehicle_control_cmd",
            10);

        _targer_pose_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
            "/carla/" + role_name + "/next_target",
            10);
        _reference_line_publisher = this->create_publisher<nav_msgs::msg::Path>(
            "/carla/" + role_name + "/reference_line",
            10);

        _current_ego_state = std::make_shared<VehicleState>();
        _current_ego_state->flag_imu = false;
        _current_ego_state->flag_ode = false;
        _current_ego_state->flag_info = false;
    // 绘图
    #ifdef PLOT
        _reference_line_figure_handle = matplot::figure();
        _final_path_figure_handle = matplot::figure();
    #endif
    }

private:
    // 参数
    double MIN_DISTANCE_PERCENTAGE;
    std::string role_name;

    // 订阅方以及订阅的数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;              // 里程计订阅方，订阅本车当前位姿与速度
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;                     // 惯性导航订阅方，订阅加速度与角速度
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber; // 定于车辆的车道信息
    std::shared_ptr<VehicleState> _current_ego_state;
    rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr _object_array_subscriber; // 对象序列，包含本车和障碍物
    std::vector<derived_object_msgs::msg::Object> _object_arrry;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _target_speed_subscriber; // 期望速度订阅方
    double _reference_speed;                                                          // 期望速度,单位为km/h

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscriber; // 路径订阅方
    std::shared_ptr<std::vector<PathPoint>> _global_path;                  // 全局路径存储器

    // 发布方
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _control_cmd_publisher; // 控制指令发布方
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _targer_pose_publisher;         // 目标点发布方
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _reference_line_publisher;

    // 控制环节
    std::shared_ptr<LongitudinalPIDController> _longitudinal_pid_controller; // 纵向控制器
    std::shared_ptr<LateralPIDController> _lateral_pid_controller;           // 横向PID控制器
    std::shared_ptr<LaterLQRController> _lateral_lqr_controller;             // 横向LQR控制器
    std::shared_ptr<LonCascadePIDController> _lon_cascade_pid_controller;
    std::shared_ptr<MPCController> _mpc_controller;
    rclcpp::TimerBase::SharedPtr _control_timer; // 控制器时钟
    double _control_time_step;

    // 规划环节
    double _planning_time_step;
    rclcpp::TimerBase::SharedPtr _planning_timer;
    std::shared_ptr<ReferenceLine> _reference_line_generator; // 参考线生成器
    std::shared_ptr<std::vector<PathPoint>> _reference_line;
    std::shared_ptr<EMPlanner> _emplanner;
    std::vector<TrajectoryPoint> _trajectory;

    // 计算指标
    double _total_frame = 0.0;
    double _dangerous_frame = 0.0;
    double _uncomfortable_acc_frame = 0.0;
    double _large_curvature_changing_frame = 0.0;
    double _total_v = 0.0;
    double Safety, Efficiency, UD, LCC;
    double _previous_heading;
    double _current_omega, _previous_omega;
    rclcpp::Time _previous_time;
    bool _is_fisrt_evaluation = true;
    int _add_localiation_noise_type = 1;

// 绘图
#ifdef PLOT
        size_t _count_plot = 0;
    matplot::figure_handle _reference_line_figure_handle;
    matplot::figure_handle _final_path_figure_handle;
#endif

public:
  //*******************************************************
  //回调函数
  //里程计订阅方回调函数，获取当前本车位姿与速度
  void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
      std::default_random_engine generator;
      std::normal_distribution<double> distribution(0.0, 0.5);
      switch (static_cast<int>(_add_localiation_noise_type)){
      case 0:
            _current_ego_state->x = msg->pose.pose.position.x;
            _current_ego_state->y = msg->pose.pose.position.y;
            _current_ego_state->z = msg->pose.pose.position.z;
            break;

      case 1:
          _current_ego_state->x = msg->pose.pose.position.x + distribution(generator);
          _current_ego_state->y = msg->pose.pose.position.y + distribution(generator);
          _current_ego_state->z = msg->pose.pose.position.z;
          RCLCPP_INFO(this->get_logger(), "adding localization noise");
          break;
      case 2:
          if ((msg->pose.pose.position.y >= -165.8 && msg->pose.pose.position.y <= -100.5 && msg->pose.pose.position.x >= 91 && msg->pose.pose.position.x <= 93) || (msg->pose.pose.position.y >= -80.0 && msg->pose.pose.position.y <= -58.0 && msg->pose.pose.position.x >= 290 && msg->pose.pose.position.x <= 340))
          {
              _current_ego_state->x = msg->pose.pose.position.x + distribution(generator);
              _current_ego_state->y = msg->pose.pose.position.y + distribution(generator);
              _current_ego_state->z = msg->pose.pose.position.z;
              RCLCPP_INFO(this->get_logger(), "adding localization noise");
              break;
          }
          else{
              _current_ego_state->x = msg->pose.pose.position.x;
              _current_ego_state->y = msg->pose.pose.position.y;
              _current_ego_state->z = msg->pose.pose.position.z;
              break;
          }
        }
    // RCLCPP_INFO(this->get_logger(), "current_x : %5.f,  current_y: %5.f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    _current_ego_state->v = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2) + std::pow(msg->twist.twist.linear.z, 2));
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg->pose.pose.orientation, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    _current_ego_state->heading = tf2NormalizeAngle(yaw);
    _current_ego_state->flag_ode = true;
  }

  void imu_cb(sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    if (imu_msg->linear_acceleration.x >= 10 || imu_msg->linear_acceleration.y >= 10)
    {
      return;
    }
    
    _current_ego_state->ax = imu_msg->linear_acceleration.x;
    _current_ego_state->ay = imu_msg->linear_acceleration.y;
    _current_ego_state->flag_imu = true;
  }

  void ego_info_cb(carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
  {
    _current_ego_state->id = ego_info->id;
    _current_ego_state->flag_info = true;
  }

  //速度指令订阅方回调函数，获取有ad_agent发布的速度指令
  void target_speed_cb(std_msgs::msg::Float64::SharedPtr msg)
  {
    _reference_speed = msg->data;
  }

  void path_cb(nav_msgs::msg::Path::SharedPtr waypoints)
  {
    RCLCPP_INFO(this->get_logger(),"接收到全局路径信息......");
    for (auto &&pose : waypoints->poses)
    {

      PathPoint temp_path_point;
      temp_path_point.x = pose.pose.position.x;
      temp_path_point.y = pose.pose.position.y;
      _global_path->push_back(temp_path_point);

    }
    Calculate_heading_and_kappa(_global_path); 
  }//全局路径订阅回调


  void objects_cb(derived_object_msgs::msg::ObjectArray::SharedPtr object_array)
  {
    _object_arrry.clear();
    for (auto &&object : object_array->objects)
    {
      _object_arrry.push_back(object);
    }
    
  }

  void planning_run_step()
  {
      if (_global_path->empty()) // 等待接受到路径
      {
        //   RCLCPP_INFO(this->get_logger(), "等待全局路径信息......");
          return;
      }
      // 等待接受到主车信息
      if (_current_ego_state->flag_imu == false || _current_ego_state->flag_ode == false || _current_ego_state->flag_info == false)
      {
          RCLCPP_INFO(this->get_logger(), "等待主车信息......");
          return;
      }

      // 获取参考线
      _reference_line->clear();
      if (_reference_line_generator->run_step(_current_ego_state, _global_path, _reference_line))
      {
#ifdef MAIN_DEBUG
        //   RCLCPP_INFO(this->get_logger(), "参考线生成成功！");
#endif
      }
      else
      {
        //   RCLCPP_INFO(this->get_logger(), "参考线生成失败！");
      }

      // 将参考线发布出去
      nav_msgs::msg::Path nav_reference_line;
      for (auto &&path_point : *_reference_line)
      {
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header.frame_id = "map";
          pose_stamped.pose.position.x = path_point.x;
          pose_stamped.pose.position.y = path_point.y;
          pose_stamped.pose.position.z = 0.0;
          nav_reference_line.poses.push_back(pose_stamped);
          // RCLCPP_INFO(this->get_logger(),"参考线(%.2f,%.2f)",path_point.x,path_point.y);
      }

      _reference_line_publisher->publish(nav_reference_line);
    //   RCLCPP_INFO(this->get_logger(), "mian debug1");
      // 调用emplanner获取轨迹
      auto current_time = this->get_clock()->now();
      _emplanner->planning_run_step(_reference_line, _current_ego_state, _object_arrry, _trajectory);

    //   RCLCPP_INFO(this->get_logger(), "mian debug2");
#ifdef PLOT
      // 绘图，因为rviz的可视化没整明白
      if (_count_plot % 20 == 0)
      {
          matplot::figure(_reference_line_figure_handle);
          matplot::cla();
          std::vector<double> global_x, global_y, reference_x, reference_y, trajectory_x, trajectory_y;
          for (auto &&path_point : *_global_path)
          {
              global_x.push_back(path_point.x);
              global_y.push_back(path_point.y);
          }

          // matplot::hold(false);
          matplot::plot(global_x, global_y, "go");
          matplot::hold(true);
          for (auto &&path_point : *_reference_line)
          {
              reference_x.push_back(path_point.x);
              reference_y.push_back(path_point.y);
          }
          matplot::plot(reference_x, reference_y, "--rx");

          int plot_index = 0;
          for (auto &&trajectory_point : _trajectory)
          {
              if (plot_index % 10 == 0)
              {
                  trajectory_x.emplace_back(trajectory_point.x);
                  trajectory_y.emplace_back(trajectory_point.y);
              }
              plot_index++;
          }

          matplot::plot(trajectory_x, trajectory_y, "-b*");

          matplot::plot({_current_ego_state->x}, {_current_ego_state->y}, "ro");
          matplot::hold(false);
          // h->draw();
      }
      _count_plot++;

#endif
      if (std::abs(_current_ego_state->v) > 0.01){
          _total_v += std::abs(_current_ego_state->v);
          _total_frame += 1.0;
      }
      //计算平均车速
      Efficiency = _total_v / _total_frame;

      //计算UD
      if (std::sqrt(std::pow(_current_ego_state->ax, 2) + std::pow(_current_ego_state->ay, 2)) > 5.0){
          _uncomfortable_acc_frame += 1.0;
      }
      UD = _uncomfortable_acc_frame / _total_frame;

      // 计算Safety
      for (auto &&obs : _object_arrry){
          // 障碍物不包含主车
          if (obs.id == _current_ego_state->id){
              continue;
          } 
          // 主车到障碍物的向量
          Eigen::Vector2d host_to_obs(obs.pose.position.x - _current_ego_state->x, obs.pose.position.y -_current_ego_state->y);
          double length = host_to_obs.norm();
          if (length < 3.0){
              _dangerous_frame += 1.0;
              break;
          }
      }
      Safety = _dangerous_frame / _total_frame;

      //计算LCC
      if (_is_fisrt_evaluation){
          _is_fisrt_evaluation = false;
          _previous_heading = _current_ego_state->heading;
          _previous_time = this->get_clock()->now();
          _previous_omega = 0.0;
      }
      else{
          auto current_time = this->get_clock()->now();
          auto duration = current_time - _previous_time;
          double seconds = duration.seconds();
          double delta_heading = std::abs(_current_ego_state->heading - _previous_heading);
          _current_omega = delta_heading;
          RCLCPP_INFO(this->get_logger(), "_previous_heading: %.3f)", _previous_heading);
          //单位：rad
        //   if (std::abs(_current_omega - _previous_omega / seconds) > 1.0)
        if (std::abs(_current_omega - _previous_omega / seconds) > 0.2){
                  _large_curvature_changing_frame += 1.0;
          }
          _previous_heading = _current_ego_state->heading;
          _previous_time = this->get_clock()->now();
          _previous_omega = _current_omega;
      }
      LCC = _large_curvature_changing_frame / _total_frame;

      RCLCPP_INFO(this->get_logger(), "计算指标完成! Efficiency: %.5f Safety: %.5f UD: %.5f LCC: %.5f total_frame: %.5f", Efficiency, Safety, UD, LCC, _total_frame);

      if (_current_ego_state->x <= 340 && _current_ego_state->x >= 338 && _current_ego_state->y <= -81 && _current_ego_state->y >= -82)
      {
          exit(0); // 0表示正常退出
      }
  }

  void control_run_step()//单步控制
  {
    //没收到路径信息
    if(_current_ego_state->flag_imu == false || _current_ego_state->flag_ode == false ||_current_ego_state->flag_info == false){
      RCLCPP_INFO(this->get_logger(),"控制模块等待主车信息......");
      return;
    }
    if(_reference_line->empty()){
        // RCLCPP_INFO(this->get_logger(), "控制模块等待参考线信息......");
        return;
    }
    if (_trajectory.empty()){
        RCLCPP_INFO(this->get_logger(), "控制模块等待轨迹信息......");
        return;
    }
    
    //目标速度为0
    if(_reference_speed <= 0.5)
    {
      emergency_stop();
      return;
    }

    //由轨迹搜索出目标点
    TrajectoryPoint target_point;
    double cur_time = this->now().seconds();
    double predicted_time = cur_time + 0.2;
    int target_point_index = -1;
    // RCLCPP_INFO(this->get_logger(),"当前时刻:%.3f",cur_time.seconds());
    // int index = 1;
    // for (size_t i = 0; i < _trajectory->size(); i++)
    // {
    //   RCLCPP_INFO(this->get_logger(), "(序号%d:,x:%.3f, y:%.3f, heading:%.3f, v:%.3f, a_tau:%.3f, time_stampe:%.3f)",
    //   index,_trajectory->at(i).x,_trajectory->at(i).y,_trajectory->at(i).heading,_trajectory->at(i).v,_trajectory->at(i).a_tau,_trajectory->at(i).time_stamped.seconds());
    //   index++;
    // }
    
    for (int i = 0; i < (int)_trajectory.size() - 1; i++)
    {
      if (predicted_time >= _trajectory.at(i).time_stamped && predicted_time < _trajectory.at(i+1).time_stamped)
      {
        target_point_index = i;
        break;
      }
    }
    double delta_t = (_trajectory.at(target_point_index+1).time_stamped - _trajectory.at(target_point_index).time_stamped);
    double dt = predicted_time - _trajectory.at(target_point_index).time_stamped;

    double k_x = (_trajectory.at(target_point_index+1).x - _trajectory.at(target_point_index).x)/delta_t;
    target_point.x = _trajectory.at(target_point_index).x + k_x*dt;

    double k_y = (_trajectory.at(target_point_index+1).y - _trajectory.at(target_point_index).y)/delta_t;
    target_point.y = _trajectory.at(target_point_index).y + k_y*dt;

    double k_v = (_trajectory.at(target_point_index+1).v - _trajectory.at(target_point_index).v)/delta_t;
    target_point.v = _trajectory.at(target_point_index).v + k_v*dt;

    double k_heading = (_trajectory.at(target_point_index+1).heading - _trajectory.at(target_point_index).heading)/delta_t;
    target_point.heading = _trajectory.at(target_point_index).heading + k_heading*dt;

    double k_a_tau = (_trajectory.at(target_point_index+1).a_tau - _trajectory.at(target_point_index).a_tau)/delta_t;    
    target_point.a_tau = _trajectory.at(target_point_index).a_tau + k_a_tau*dt;

    #ifdef MAIN_DEBUG
    // RCLCPP_INFO(this->get_logger(),"当前位姿(%.3f,%.3f,%.3f,%.3f),期望位姿(%.3f,%.3f,%.3f,%.3f)",
                // _current_ego_state->x,_current_ego_state->y,_current_ego_state->heading,_current_ego_state->v,
                // target_point.x,target_point.y,target_point.heading, target_point.v);
    // RCLCPP_INFO(this->get_logger(),"参考线起点位姿(%.3f,%.3f,%.3f),参考线终点位姿(%.3f,%.3f,%.3f)",
    //             _reference_line->at(0).x,_reference_line->at(0).y,_reference_line->at(0).heading,
    //             _reference_line->back().x,_reference_line->back().y,_reference_line->back().heading);

    #endif
    
    //RCLCPP_INFO(this->get_logger(),"目标位置(%.2f,%.2f)",target_pose.pose.position.x,target_pose.pose.position.y);
    //****发布标记
    visualization_msgs::msg::Marker marker;
    marker.type = 0;
    marker.header.frame_id = "map";
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 255;
    marker.color.a = 1.0;
    _targer_pose_publisher->publish(marker);
    

    //计算并发布控制指令
    carla_msgs::msg::CarlaEgoVehicleControl control_msg;
    control_msg.hand_brake = false;
    control_msg.manual_gear_shift = false;
    control_msg.reverse = false;

    //TODO:单级别PID
    // 添加刹车逻辑
    double speed_difference = target_point.v - _current_ego_state->v;
    if (speed_difference < 0){
        control_msg.brake = _longitudinal_pid_controller->run_step_brake(target_point.v, _current_ego_state->v);
        control_msg.throttle = 0.0;
    }
    else{
        control_msg.throttle = _longitudinal_pid_controller->run_step_throttle(target_point.v, _current_ego_state->v);
        control_msg.brake = 0.0;
    }
    control_msg.steer = _lateral_lqr_controller->run_step(target_point, *_current_ego_state, _control_time_step);

    // double cur_t = this->now().seconds();
    // ControlCMD cmd;
    // // 串联PID纵向控制
    // _lon_cascade_pid_controller->set_station_controller(1.0, 0.1, 1.0);
    // _lon_cascade_pid_controller->set_speed_controller(4.0, 0.1, 1.0);

    //原参数
    // _lon_cascade_pid_controller->set_station_controller(0.1, 0.0, 0.0);
    // _lon_cascade_pid_controller->set_speed_controller(2.0, 0.2, 0.0);
    // _lon_cascade_pid_controller->set_station_controller(0.1, 0.0, 0.0);
    // _lon_cascade_pid_controller->set_speed_controller(2.0, 0.2, 0.0);
    // _lon_cascade_pid_controller->set_speed_integral_saturation_boundary(0.3, -0.3);
    // _lon_cascade_pid_controller->compute_control_cmd(_trajectory, _current_ego_state, cur_t, _control_time_step, cmd);



    // 横纵向mpc
    // std::vector<double> q_vector = {50.0, 0.0, 220.0, 0.0, 70.0, 20.0};
    // std::vector<double> r_vector = {70.0, 10.0};
    // _mpc_controller->set_matrix_Q(q_vector);
    // _mpc_controller->set_matrix_R(r_vector);
    // _mpc_controller->compute_control_cmd(_trajectory, _current_ego_state, cur_t, _control_time_step, cmd);

    // control_msg.steer = _lateral_lqr_controller->run_step(target_point, *_current_ego_state, _control_time_step);
    // control_msg.brake = cmd.brake;
    // control_msg.throttle = cmd.throttle;

    _control_cmd_publisher->publish(control_msg);
  }

  //发布紧急制动指令
  void emergency_stop()
  {
    carla_msgs::msg::CarlaEgoVehicleControl control_msg ;
    control_msg.throttle = 0.0;
    control_msg.steer = 0.0;
    control_msg.brake = 1.0;
    control_msg.hand_brake = false;
    control_msg.manual_gear_shift = false;
    control_msg.reverse = false;

    _control_cmd_publisher->publish(control_msg);

  }

};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<MyPlanningANDControl>());

  rclcpp::shutdown();
  
  return 0;
}
