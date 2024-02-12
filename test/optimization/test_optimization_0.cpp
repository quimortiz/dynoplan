#include "dynoplan/optimization/ocp.hpp"

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

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"
#define DYNOBENCH_BASE_DATA "../../dynobench/"

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(second_order_park_traj_opt) {

  Options_trajopt options_trajopt;
  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle2_v0/parallelpark_0.yaml"));

  Trajectory init_guess(
      DYNOBENCH_BASE_DATA +
      std::string("data/unicycle2_0_parallelark_guess_0.yaml"));

  options_trajopt.solver_id = static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Result_opti result;
  Trajectory sol;
  trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
  BOOST_TEST_CHECK(result.feasible);
  std::cout << "cost is " << result.cost << std::endl;
  BOOST_TEST_CHECK(result.cost <= 10.);
}
