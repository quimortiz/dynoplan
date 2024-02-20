#include "dynoplan/optimization/ocp.hpp"

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
#include <boost/test/tools/old/interface.hpp>
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
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include <Eigen/Dense>
#include <iostream>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"

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

  // std::vector<Options_trajopt> solvers{options_dt};

  problem_with_init_guess.push_back(std::make_pair(
      Problem(DYNOBENCH_BASE "envs/quadrotor_v0/recovery.yaml"),
      Trajectory(
          "../../benchmark_initguess/quadrotor_v0/recovery/delta_05_v0.yaml")));

  problem_with_init_guess.push_back(std::make_pair(
      Problem(DYNOBENCH_BASE "envs/quadrotor_v0/window.yaml"),
      Trajectory(
          "../../benchmark_initguess/quadrotor_v0/window/delta_05_v0.yaml")));

  for (auto &p : problem_with_init_guess) {
    p.first.models_base_path = DYNOBENCH_BASE + std::string("models/");
  }

  for (size_t i = 0; i < problem_with_init_guess.size(); i++) {

    for (size_t j = 0; j < solvers.size(); j++) {

      auto &solver = solvers.at(j);
      auto &problem = problem_with_init_guess[i].first;
      auto &init_guess = problem_with_init_guess[i].second;
      std::string experiment_id = std::to_string(i) + ":" + std::to_string(j) +
                                  ":" + solver.name + ":" + problem.name + ":" +
                                  init_guess.filename;

      std::cout << "experiment id" << std::endl;
      std::cout << experiment_id << std::endl;
      Result_opti result;
      Trajectory sol;

      // TODO: not clear with mpcc sometimes fail and sometimes not.
      // it complains about quaternion norm...
      if (solver.name == "mpcc" && (problem.name == "quadrotor_0-recovery" ||
                                    problem.name == "quadrotor_0-window")) {
        BOOST_TEST_WARN(false,
                        "i skip mpcc in quadrotor_0-recovery and window");
        continue;
      }

      BOOST_CHECK_NO_THROW(
          trajectory_optimization(problem, init_guess, solver, sol, result));

      BOOST_TEST_CHECK(result.feasible, experiment_id);
      std::cout << "cost is " << result.cost << std::endl;
      // BOOST_TEST_CHECK(result.cost <= 5., experiment_id);
    }
  }
}

BOOST_AUTO_TEST_CASE(t_method_time_opti2) {
  // do the same on bugtrap

  srand(0);

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
  options_mpcc.weight_goal = 100;
  // options_mpcc.k_linear = 50;
  // options_mpcc.k_contour = 50;
  // options_mpcc.noise_level = 0;

  std::vector<std::pair<Problem, Trajectory>> problem_with_init_guess;
  std::vector<Options_trajopt> solvers{options_mpc, options_dt, options_search,
                                       options_mpcc};

  // std::vector<Options_trajopt> solvers{options_mpcc};

  problem_with_init_guess.push_back(
      std::make_pair(Problem(DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml"),
                     Trajectory("../../benchmark_initguess/unicycle1_v0/"
                                "bugtrap_0/delta_03_v0.yaml")));

  problem_with_init_guess.push_back(
      std::make_pair(Problem(DYNOBENCH_BASE "envs/unicycle2_v0/bugtrap_0.yaml"),
                     Trajectory("../../benchmark_initguess/unicycle2_v0/"
                                "bugtrap_0/delta_02_v0.yaml")));

  for (auto &p : problem_with_init_guess) {
    p.first.models_base_path = DYNOBENCH_BASE + std::string("models/");
  }

  for (size_t i = 0; i < problem_with_init_guess.size(); i++) {

    for (size_t j = 0; j < solvers.size(); j++) {
      auto &solver = solvers.at(j);

      auto &problem = problem_with_init_guess[i].first;
      auto &init_guess = problem_with_init_guess[i].second;

      Result_opti result;
      Trajectory sol;

      std::string experiment_id = std::to_string(i) + ":" + std::to_string(j) +
                                  ":" + solver.name + ":" + problem.name + ":" +
                                  init_guess.filename;

      BOOST_CHECK_NO_THROW(
          trajectory_optimization(problem, init_guess, solver, sol, result));

      if (solver.name == "mpcc" && (problem.name == "quadrotor_0-recovery" ||
                                    "unicycle2_v0-bugtrap_0")) {
        BOOST_TEST_WARN(result.feasible, experiment_id);
      } else {
        BOOST_TEST_CHECK(result.feasible, experiment_id);
        // std::cout << "cost is " << result.cost << std::endl;
        // if (!result.feasible) {
        //   throw -1;
        // }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(t_opti_integrator2) {

  Options_trajopt options;
  Problem problem(DYNOBENCH_BASE "envs/integrator2_2d_v0/park.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Trajectory init_guess, traj_out;
  init_guess.num_time_steps = 50;
  Result_opti opti_out;
  trajectory_optimization(problem, init_guess, options, traj_out, opti_out);
  BOOST_TEST(opti_out.feasible);

  // write down the generated trajectory

  std::string filename = "/tmp/dynoplan/traj_t_opti_integrator2.yaml";
  create_dir_if_necessary(filename.c_str());
  std::ofstream out(filename);
  traj_out.to_yaml_format(out);
}

BOOST_AUTO_TEST_CASE(t_opti_integrator1) {

  Options_trajopt options;
  Problem problem(DYNOBENCH_BASE "envs/integrator1_2d_v0/empty.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Trajectory init_guess, traj_out;
  init_guess.num_time_steps = 50;
  // init_guess.num_time_steps = 5; use this to see how it saturates control
  // limits
  Result_opti opti_out;
  trajectory_optimization(problem, init_guess, options, traj_out, opti_out);
  BOOST_TEST(opti_out.feasible);

  // write down the generated trajectory

  std::string filename = "/tmp/dynoplan/traj_t_opti_integrator1_2d.yaml";
  create_dir_if_necessary(filename.c_str());
  std::ofstream out(filename);
  traj_out.to_yaml_format(out);
}

static Eigen::VectorXd default_vector;

// TODO: move to bench
BOOST_AUTO_TEST_CASE(t_dyn) {

  double tol = 1e-7;
  double margin_rate = 100;
  {
    auto dyn = mk<Dynamics>(mks<dynobench::Model_acrobot>());
    check_dyn(dyn, tol, default_vector, default_vector, margin_rate);
  }

  {
    auto dyn = mk<Dynamics>(mks<dynobench::Model_acrobot>(),
                            Control_Mode::default_mode);
    check_dyn(dyn, tol, default_vector, default_vector, margin_rate);
  }

  {
    double tol = 1e-7;
    {
      auto dyn = mk<Dynamics>(mks<Model_quad2d>());
      check_dyn(dyn, tol);
    }
    {
      auto dyn_free_time =
          mk<Dynamics>(mks<Model_quad2d>(), Control_Mode::free_time);

      Eigen::VectorXd x(6);
      x.setRandom();

      Eigen::VectorXd u(3);
      u.setRandom();
      u(2) = std::fabs(u(2));

      check_dyn(dyn_free_time, tol, x, u);
    }
  }

  {
    ptr<Dynamics> dyn = mk<Dynamics>(mks<Model_quad2dpole>());
    check_dyn(dyn, 1e-6, Eigen::VectorXd(), Eigen::VectorXd(), 200);
  }
}

BOOST_AUTO_TEST_CASE(t_multirotor_pole) {

  {
    // solving without initial guess
    Problem problem(DYNOBENCH_BASE "envs/quad2dpole_v0/move_with_up.yaml");

    problem.models_base_path = DYNOBENCH_BASE "models/";

    Trajectory traj_in, traj_out;
    traj_in.num_time_steps = 300;

    Options_trajopt options_trajopt;
    options_trajopt.solver_id = 0;
    options_trajopt.smooth_traj = true;
    options_trajopt.weight_goal = 200;
    options_trajopt.max_iter = 100;

    Result_opti opti_out;

    BOOST_CHECK_NO_THROW(trajectory_optimization(
        problem, traj_in, options_trajopt, traj_out, opti_out));

    BOOST_CHECK(opti_out.feasible);
  }

  {
    Problem problem(DYNOBENCH_BASE "envs/quad2dpole_v0/window_hard.yaml");
    problem.models_base_path = DYNOBENCH_BASE "models/";

    Trajectory traj_in, traj_out;

    traj_in.read_from_yaml(DYNOBENCH_BASE "envs/quad2dpole_v0/window_hard/"
                                          "idbastar_v0_db_solution_v0.yaml");

    Options_trajopt options;
    options.solver_id = 0;
    options.weight_goal = 200;
    Result_opti opti_out;
    BOOST_CHECK_NO_THROW(
        trajectory_optimization(problem, traj_in, options, traj_out, opti_out));
    BOOST_TEST(opti_out.feasible);

    options.solver_id = 1;

    BOOST_CHECK_NO_THROW(
        trajectory_optimization(problem, traj_in, options, traj_out, opti_out));
    BOOST_TEST(opti_out.feasible);
  }
}
