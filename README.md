# 基于ROS2-CARLA的自动驾驶规控算法仿真
复现[开源项目](https://github.com/GoGoGo13579/Planning-and-Control-based-on-Carla-and-ROS2)，实现了Carla和ros2环境下Apollo EMplanner的C++版本

控制算法包括：纵向串级PID，横向LQR，横纵向MPC

规划算法包括：参考线平滑，路径DP+QP，速度DP+QP

在此基础上进行了代码检查和改进优化，对动态障碍物及静态障碍物分别进行了测试

## 环境配置
本项目使用Windows系统+WSL2(Ubuntu20.04).其中Carla软件装在Windows下，其余代码全部在Ubuntu系统下，运行时通过Carla和ROS2远程通信联合仿真

硬件需要6GB以上显存的独立显卡 + 200GB硬盘空间

PS：使用Windows系统+WSL2是本人迫不得已，最好是Carla也装在Ubuntu20.04系统中，避免双系统通讯问题
```bash
Ubuntu20.04
ros2-foxy
Carla-0.9.13
osqp0.6.3
osqp-eigen0.8.0
matplot++
```
软件版本最好一致，不然编译容易出问题

## 编译及运行
决策规划相关源代码在my_planning_and_control这个文件夹里，后续也会在这个文件夹里进行修改

运行案列我是借助的carla_ad_demo(这是carla-ros-bridge里自带的案例)，在它的场景里运行自己规控算法

```bash
colcon build
. install/setup.bash
ros2 launch carla_ad_demo carla_ad_demo.launch.py
(windows + ubuntu 情况下： ros2 launch carla_ad_demo carla_ad_demo.launch.py host:=172.xx.xxx.x)
```

## 实现效果
参见[B站视频](https://www.bilibili.com/video/BV1bT421e7CM/?share_source=copy_web&vd_source=c6672cbc9ac3f70466950e7ef1e8855a)

## 参考链接
[Ubuntu20.04安装Carla](https://blog.csdn.net/m0_61772308/article/details/131590593#:~:text=%E8%AF%A5%E6%96%87%E4%BB%B6%E8%AE%B0%E5%BD%95Ubunt)
[Windows从源码编译Carla0.9.13](https://zhuanlan.zhihu.com/p/668593156#:~:text=%E6%9C%AC%E6%96%87%E4%BB%8B%E7%BB%8D%E4%BA%86%E5%A6%82%E4%BD%95%E5%9C%A8Wi)
[ROS Bridge for ROS 2](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/#:~:text=Learn%20how%20to%20install%20the%20ROS%20bridge%20on%20Linux%20for)
[WSL2 && carla-win11跨系统通讯](https://blog.csdn.net/steven_ysh/article/details/125994191#:~:text=%E6%96%87%E7%AB%A0%E6%B5%8F%E8%A7%88%E9%98%85%E8%AF%BB2.6k)
[二次规划osqp库](https://blog.csdn.net/qjj18776858511/article/details/125963379)

## 致谢
感谢B站UP主忠厚老实的老王关于自动驾驶决策规划[系列课程](https://space.bilibili.com/287989852)
感谢项目原作者的无私[开源](https://github.com/GoGoGo13579/Planning-and-Control-based-on-Carla-and-ROS2)
感谢本项目用到所有开源系统、框架、求解库的作者

