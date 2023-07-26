#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>

#include "Eigen/Core"

#include "dynobench/croco_macros.hpp"
#include <boost/program_options.hpp>
// #include "croco_models.hpp"
#include "dynobench/general_utils.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"

#include "crocoddyl/core/action-base.hpp"

#include "dynoplan/optimization/generate_ocp.hpp"
#include "dynoplan/optimization/options.hpp"

namespace dynoplan {

void print_data(
    boost::shared_ptr<crocoddyl::ActionDataAbstractTpl<double>> data);

struct Result_opti {
  // Note: success is not the same as feasible.
  // Feasible=1 means that the trajectory fulfil all the constraints.
  // Success=1 means that the trajectory fulfil the constraints imposed by
  // the current formulation (e.g. some formulations will solve
  // first without bound constraints).
  bool feasible = false;
  bool success = false;
  double cost = -1;
  std::map<std::string, std::string> data;
  std::string name;
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;

  void write_yaml(std::ostream &out);
  void write_yaml_db(std::ostream &out);
};

void __trajectory_optimization(
    const dynobench::Problem &problem,
    std::shared_ptr<dynobench::Model_robot> &model_robot,
    const dynobench::Trajectory &init_guess,
    const Options_trajopt &options_trajopt, dynobench::Trajectory &traj,
    Result_opti &opti_out);

void trajectory_optimization(const dynobench::Problem &problem,
                             const dynobench::Trajectory &init_guess,
                             const Options_trajopt &opti_parms,
                             dynobench::Trajectory &traj,
                             Result_opti &opti_out);

std::vector<Eigen::VectorXd>
smooth_traj2(const std::vector<Eigen::VectorXd> &xs_init,
             const dynobench::StateDyno &state);

} // namespace dynoplan
