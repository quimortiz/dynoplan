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
#include "dynobench/multirobot_trajectory.hpp"
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(t_multi_robot_cli) {

  std::vector<std::string> run_cmd_new = {
      "../main_multirobot_optimization",
      "--env",
      "../../dynobench/envs/multirobot/straight.yaml",
      "--init",
      "../../dynobench/envs/multirobot/guess_indiv_straight.yaml",
      "--base",
      "../../dynobench/",
      "--out",
      "buu.yaml",
      "--s",
      "1",
      ">",
      "/tmp/db_log.txt"};

  std::string cmd = "";

  for (auto &s : run_cmd_new) {
    cmd += s + " ";
  }

  std::cout << "running:\n" << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 1);

  // TODO: @akmaral, Can you add the other test in test_standalone?
}

BOOST_AUTO_TEST_CASE(t_multi_robot) {

  // TODO: @akmaral, Can you add the other test in  test_standalone?

  // 0: optimize the Max time of arrival.
  // 1: optimize the sum of the time of arrival of all robots.
  bool sum_robots_cost = 1;
  std::string dynobench_base = "../../dynobench/";
  std::string env_file = dynobench_base + "envs/multirobot/straight.yaml";
  std::string initial_guess_file =
      dynobench_base + "envs/multirobot/guess_indiv_straight.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1; 
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = dynobench_base + std::string("models/");

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check2.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi.yaml");
}
