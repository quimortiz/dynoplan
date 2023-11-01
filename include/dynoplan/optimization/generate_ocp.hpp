#pragma once

#include "dynobench/robot_models_base.hpp"
#include "dynoplan/optimization/croco_models.hpp"
#include "dynoplan/optimization/options.hpp"
#include <Eigen/Core>
#include <crocoddyl/core/fwd.hpp>
#include <memory>
#include <string>

namespace dynoplan {

template <typename Derived>
boost::shared_ptr<crocoddyl::ActionModelAbstract>
to_am_base(boost::shared_ptr<Derived> am) {
  return boost::static_pointer_cast<crocoddyl::ActionModelAbstract>(am);
};

struct Generate_params {
  bool free_time = false;
  bool free_time_linear = false;
  std::string name;
  size_t N;
  Eigen::VectorXd goal;
  Eigen::VectorXd start;
  std::shared_ptr<dynobench::Model_robot> model_robot;
  std::vector<Eigen::VectorXd> states = {};
  std::vector<Eigen::VectorXd> states_weights = {};
  std::vector<Eigen::VectorXd> actions = {};
  bool contour_control = false;
  ptr<dynobench::Interpolator> interpolator = nullptr;
  double max_alpha = 100.;
  bool linear_contour = true;
  bool goal_cost = true;
  bool collisions = true;
  double penalty = 1; // penalty for the constraints
  void print(std::ostream &out) const;
};

ptr<crocoddyl::ShootingProblem>
generate_problem(const Generate_params &gen_args,
                 const Options_trajopt &options_trajopt);

std::vector<ReportCost> report_problem(ptr<crocoddyl::ShootingProblem> problem,
                                       const std::vector<Eigen::VectorXd> &xs,
                                       const std::vector<Eigen::VectorXd> &us,
                                       const char *file_name);

bool check_problem(ptr<crocoddyl::ShootingProblem> problem,
                   ptr<crocoddyl::ShootingProblem> problem2,
                   const std::vector<Eigen::VectorXd> &xs,
                   const std::vector<Eigen::VectorXd> &us);

}; // namespace dynoplan
