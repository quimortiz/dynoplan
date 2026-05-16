#include "dynobench/motions.hpp"
#include <dynobench/multirobot_trajectory.hpp>
#include <dynoplan/optimization/multirobot_optimization.hpp>
#include <dynoplan/optimization/ocp.hpp>
#include <string>
#include <vector>

bool execute_optimizationMultiRobot(const std::string &env_file,
                                    const std::string &initial_guess_file,
                                    const std::string &output_file,
                                    const std::string &base,
                                    bool sum_robots_cost) {

  using namespace dynoplan;
  using namespace dynobench;

  Options_trajopt options_trajopt;
  Problem problem(env_file);

  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  Trajectory init_guess;

  std::cout << "goal times are " << std::endl;

  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

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

  options_trajopt.solver_id = 1; // static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = base + std::string("robot_types/");

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);
  if (!result.feasible) {
    std::cout << "optimization infeasible" << std::endl;
    return false;
  }

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

  multi_out.to_yaml_format("/tmp/check5.yaml");
  multi_out.to_yaml_format(output_file.c_str());

  return true;
}

bool execute_optimizationMetaRobot(
    dynobench::Problem &problem,
    MultiRobotTrajectory &init_guess_multi_robot, // discrete search
    MultiRobotTrajectory &multi_robot_out,        // output
    const std::string &base, bool sum_robots_cost = true) {

  using namespace dynoplan;
  using namespace dynobench;

  Options_trajopt options_trajopt;
  // Problem problem(env_file);

  std::vector<int> goal_times;
  size_t index = 0;
  for (size_t i = 0; i < init_guess_multi_robot.trajectories.size(); i++) {
    goal_times.push_back(
        init_guess_multi_robot.trajectories.at(i).states.size());
    ++index;
  }

  Trajectory init_guess;

  std::cout << "goal times are " << std::endl;

  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

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

  options_trajopt.solver_id = 1; // time-optimal trajectory
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  options_trajopt.collision_weight = 100; 
  problem.models_base_path = base + std::string("robot_types/");

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);
  if (!result.feasible) {
    std::cout << "optimization infeasible" << std::endl;
    return false;
  }

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals; // for all robots
  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  multi_robot_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);
  return true;
}
