# 基于ROS2-CARLA的自动驾驶规控算法仿真
复现了[开源项目](https://github.com/GoGoGo13579/Planning-and-Control-based-on-Carla-and-ROS2)，实现了Carla和ros2环境下Apollo EMplanner的C++版本。实现控制算法包括：纵向串级PID，横向LQR，横纵向MPC；规划算法包括：参考线平滑，路径DP+QP，速度DP+QP。再次基础上进行了代码检查和改进优化，对动态障碍物及静态障碍物分别进行了测试

## 环境配置
本项目使用Windows系统+WSL2(Ubuntu20.04).其中Carla软件装在Windows下，其余代码全部在Ubuntu系统下，运行时通过Carla和ROS2远程通信联合仿真
PS：使用Windows系统+WSL2是本人迫不得已，最好是Carla也装在Ubuntu20.04系统中，避免双系统通讯问题
'Ubuntu20.04'
'ros2-foxy'
'Carla-0.9.13'
'osqp0.6.3'
'osqp-eigen0.8.0'
'matplot++'

## 编译及运行

## 实现效果

## 参考链接
