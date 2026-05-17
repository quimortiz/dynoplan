#pragma once

#include <bits/stdc++.h>
#include <string>
// #include "dynobench/motions.hpp"
#include <dynobench/multirobot_trajectory.hpp>

bool execute_optimizationMultiRobot(const std::string &env_file,
                                    const std::string &initial_guess_file,
                                    const std::string &output_file,
                                    const std::string &base,
                                    bool sum_robots_cost);

bool execute_optimizationMetaRobot(
    dynobench::Problem &problem,
    MultiRobotTrajectory &multi_robot_initial_guess,
    MultiRobotTrajectory &multi_robot_out, const std::string &dynobench_base,
    bool sum_robots_cost);
