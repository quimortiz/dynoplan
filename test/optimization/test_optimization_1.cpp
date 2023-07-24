#include "idbastar/optimization/ocp.hpp"

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include "Eigen/Core"
#include <boost/program_options.hpp>

// #include "collision_checker.hpp"

// save data without the cluster stuff

#include <filesystem>
#include <random>
#include <regex>
#include <type_traits>

#include <filesystem>
#include <regex>

#include "dynobench/motions.hpp"
#include <Eigen/Dense>
#include <iostream>

#define dynobench_base "../../dynobench/"

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(t_method_time_opti) {

  Options_trajopt options_mpcc, options_mpc, options_search, options_dt;

  options_mpc.window_optimize = 100;
  options_mpc.window_shift = 40;
  options_mpc.max_iter = 50;
  options_mpc.weight_goal = 100;
  options_mpc.soft_control_bounds = false; // use true for quad2d
  options_mpc.solver_id = 10;
  options_mpc.name = "mpc";

  options_dt.solver_id = 1;
  options_dt.max_iter = 100;
  options_dt.weight_goal = 100;
  options_dt.name = "dt";

  options_search.name = "search";
  options_search.max_iter = 200;
  options_search.weight_goal = 200;
  options_search.linear_search = true;
  options_search.solver_id = 9;

  options_mpcc.name = "mpcc";
  options_mpcc.solver_id = 8;
  options_mpcc.window_optimize = 100;
  options_mpcc.window_shift = 40;
  options_mpcc.max_iter = 50;
  options_mpcc.k_linear = 100;
  options_mpcc.soft_control_bounds = false;
  options_mpcc.weight_goal = 100;

  std::vector<std::pair<Problem, Trajectory>> problem_with_init_guess;
  std::vector<Options_trajopt> solvers{options_mpc, options_dt, options_search,
                                       options_mpcc};

  problem_with_init_guess.push_back(std::make_pair(
      Problem(dynobench_base "envs/quadrotor_v0/recovery.yaml"),
      Trajectory(
          "../../benchmark_initguess/quadrotor_v0/recovery/delta_05_v0.yaml")));

  problem_with_init_guess.push_back(std::make_pair(
      Problem(dynobench_base "envs/quadrotor_v0/window.yaml"),
      Trajectory(
          "../../benchmark_initguess/quadrotor_v0/window/delta_05_v0.yaml")));

  for (auto &p : problem_with_init_guess) {
    p.first.models_base_path = dynobench_base + std::string("models/");
  }

  for (size_t i = 0; i < problem_with_init_guess.size(); i++) {

    for (size_t j = 0; j < solvers.size(); j++) {

      auto &solver = solvers.at(j);
      auto &problem = problem_with_init_guess[i].first;
      auto &init_guess = problem_with_init_guess[i].second;
      std::string experiment_id = std::to_string(i) + ":" + std::to_string(j) +
                                  ":" + solver.name + ":" + problem.name + ":" +
                                  init_guess.filename;

      if (solver.name == "mpcc" && problem.name == "quadrotor_0-recovery") {
        BOOST_TEST_WARN(false, "i skip mpcc in quadrotor_0-recovery");
        continue;
      }

      std::cout << "experiment id" << std::endl;
      std::cout << experiment_id << std::endl;
      Result_opti result;
      Trajectory sol;
      trajectory_optimization(problem, init_guess, solver, sol, result);

      BOOST_TEST_CHECK(result.feasible, experiment_id);
      std::cout << "cost is " << result.cost << std::endl;
      // BOOST_TEST_CHECK(result.cost <= 5., experiment_id);
    }
  }
}

BOOST_AUTO_TEST_CASE(t_method_time_opti2) {
  // do the same on bugtrap

  Options_trajopt options_mpcc, options_mpc, options_search, options_dt;

  options_mpc.window_optimize = 40;
  options_mpc.window_shift = 20;
  options_mpc.max_iter = 30;
  options_mpc.weight_goal = 50;
  options_mpc.name = "mpc";
  options_mpc.solver_id = 10;

  options_dt.solver_id = 1;
  options_dt.max_iter = 100;
  options_dt.weight_goal = 100;
  options_dt.name = "dt";

  options_search.name = "search";
  options_search.max_iter = 100;
  options_search.weight_goal = 100;
  options_search.linear_search = true;
  options_search.solver_id = 9;

  options_mpcc.name = "mpcc";
  options_mpcc.solver_id = 8;
  options_mpcc.window_optimize = 50;
  options_mpcc.window_shift = 20;
  options_mpcc.max_iter = 30;
  options_mpcc.weight_goal = 50;
  options_mpcc.k_linear = 50;
  options_mpcc.k_contour = 10;

  std::vector<std::pair<Problem, Trajectory>> problem_with_init_guess;
  std::vector<Options_trajopt> solvers{options_mpc, options_dt, options_search,
                                       options_mpcc};

  problem_with_init_guess.push_back(
      std::make_pair(Problem(dynobench_base "envs/unicycle1_v0/bugtrap_0.yaml"),
                     Trajectory("../../benchmark_initguess/unicycle1_v0/"
                                "bugtrap_0/delta_03_v0.yaml")));

  problem_with_init_guess.push_back(
      std::make_pair(Problem(dynobench_base "envs/unicycle2_v0/bugtrap_0.yaml"),
                     Trajectory("../../benchmark_initguess/unicycle2_v0/"
                                "bugtrap_0/delta_02_v0.yaml")));

  for (auto &p : problem_with_init_guess) {
    p.first.models_base_path = dynobench_base + std::string("models/");
  }

  for (size_t i = 0; i < problem_with_init_guess.size(); i++) {

    for (size_t j = 0; j < solvers.size(); j++) {
      auto &solver = solvers.at(j);

      auto &problem = problem_with_init_guess[i].first;
      auto &init_guess = problem_with_init_guess[i].second;

      Result_opti result;
      Trajectory sol;
      trajectory_optimization(problem, init_guess, solver, sol, result);
      std::string experiment_id = std::to_string(i) + ":" + std::to_string(j) +
                                  ":" + solver.name + ":" + problem.name + ":" +
                                  init_guess.filename;

      if (solver.name == "mpcc" && problem.name == "quadrotor_0-recovery") {
        BOOST_TEST_WARN(result.feasible, experiment_id);
      } else {
        BOOST_TEST_CHECK(result.feasible, experiment_id);
        std::cout << "cost is " << result.cost << std::endl;
      }
      // BOOST_TEST_CHECK(result.cost <= 5., experiment_id);
    }
  }
}
