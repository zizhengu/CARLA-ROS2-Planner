#pragma once

#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "common.h"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"

class ReferenceLine{
public:
    ReferenceLine();

    /**
     * 疑惑：为函数传递指针的话，不能直接修改指针所指的变量，必须解引用。
     * 解答：
     * 传递智能指针的原因：
     * 通过->运算符，我们可以直接访问指针所指向的对象的成员。
     * 使用 -> 可以隐式地解引用。
     * 解引用操作（*init_reference_line）会返回指针所指向的对象，但通常并不需要这样做，除非你要进行一些复杂的操作或传递该对象的引用。
     */
    bool run_step(std::shared_ptr<VehicleState> current_ego_state, std::shared_ptr<std::vector<PathPoint>> global_path, std::shared_ptr<std::vector<PathPoint>> reference_line);


private:
    //初始化参考线
    double _forward_num, _backward_num;
    bool _is_first_run;
    int _previous_match_point_index;

    //参考线平滑
    double _cost_smooth, _cost_geometry, _cost_compact;
    double _max_x_offset, _max_y_offset;
    std::shared_ptr<OsqpEigen::Solver> _smooth_solver;
};