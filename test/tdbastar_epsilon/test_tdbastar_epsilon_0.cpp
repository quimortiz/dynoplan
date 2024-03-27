#include "Eigen/Core"
#include "dynobench/motions.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/tdbastar_epsilon.hpp"
#include "dynobench/multirobot_trajectory.hpp"

#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>
#include <filesystem>
#include <iostream>
#include <random>
#include <regex>
#include <type_traits>

#define DYNOBENCH_BASE "../../dynobench/"
#define BASE_PATH_MOTIONS "../../dynomotions/"

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(t_check_traj_drone1c) {

  std::string env = DYNOBENCH_BASE "envs/multirobot/example/drone1c.yaml";

  Problem problem(env);
  std::string robot_type = problem.robotType;

  std::string _base_path = DYNOBENCH_BASE "models/";

  problem.models_base_path = _base_path;
  std::unique_ptr<Model_robot> joint_robot = joint_robot_factory(
      problem.robotTypes, _base_path, problem.p_lb, problem.p_ub);

  load_env(*joint_robot, problem);

  {
    std::string result_file =
        DYNOBENCH_BASE "envs/multirobot/results/drone1c_solution.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible);
  }

  {

    std::string result_file =
        DYNOBENCH_BASE "envs/multirobot/results/drone1c_db.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible == false);
  }
}
