
#include "Eigen/Core"
#include "dynobench/motions.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>
#include <filesystem>
#include <iostream>
#include <random>
#include <regex>
#include <type_traits>

#define DYNOBENCH_BASE "../dynobench/"
#define BASE_PATH_MOTIONS "../dynomotions/"

using namespace dynoplan;
using namespace dynobench;
// Run from dynoplan build
BOOST_AUTO_TEST_CASE(test_eval_multiple) {

  std::vector<Problem> problems{
      DYNOBENCH_BASE "envs/unicycle1_v0/swap/swap1_unicycle.yaml",
      DYNOBENCH_BASE "envs/unicycle2_v0/swap/swap1_unicycle2.yaml",
      DYNOBENCH_BASE "envs/car1_v0/swap/swap1_trailer.yaml"};

  for (auto &p : problems) {
    p.models_base_path = DYNOBENCH_BASE "models/";
  }

  Options_tdbastar o_uni1, o_uni2, o_car;

  o_uni1.max_motions = 100;
  o_uni1.delta = .5;
  o_uni1.motionsFile =
      BASE_PATH_MOTIONS "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin."
                        "im.bin.small5000.msgpack";

  o_uni2.max_motions = 100;
  o_uni2.delta = .5;
  o_uni2.motionsFile =
      BASE_PATH_MOTIONS "unicycle2_v0__ispso__2023_04_03__15_36_01.bin.im.bin."
                        "im.bin.small5000.msgpack";

  o_car.max_motions = 400;
  o_car.delta = .75;
  o_car.motionsFile =
      BASE_PATH_MOTIONS "car1_v0_all.bin.sp.bin.small5000.msgpack";

  std::vector<Options_tdbastar> options{o_uni1, o_uni2, o_car};

  for (auto &o : options) {
    o.search_timelimit = 40 * 10e3;
  }

  // you can choose the heuristic here!!

  DYNO_CHECK_EQ(options.size(), problems.size(), AT);
  size_t robot_id = 0;
  std::vector<dynobench::Trajectory> expanded_trajs_tmp;
  for (size_t j = 0; j < options.size(); j++) {

    auto &problem = problems[j];
    auto &option = options[j];

    std::string msg = problem.name;

    Trajectory traj_out;
    Out_info_tdb info_out;

    // load motions

    std::vector<Motion> motions;

    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotType + ".yaml").c_str(),
        problem.p_lb, problem.p_ub);

    load_motion_primitives_new(option.motionsFile, *robot, motions,
                               option.max_motions, option.cut_actions, false,
                               option.check_cols);

    option.motions_ptr = &motions;
    expanded_trajs_tmp.clear();
    BOOST_REQUIRE_NO_THROW(tdbastar(
        problem, option, traj_out, /*constraints*/ {}, info_out, robot_id,
        /*reverse_search*/ false, expanded_trajs_tmp, nullptr, nullptr));

    BOOST_TEST(info_out.solved, msg);
  }
}
// for reverse search
BOOST_AUTO_TEST_CASE(test_eval_reverse) {

  Problem problem(DYNOBENCH_BASE "envs/unicycle1_v0/swap/swap1_unicycle.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";
  std::string msg = problem.name;
  Options_tdbastar o_uni1;

  o_uni1.max_motions = 100;
  o_uni1.delta = .5;
  o_uni1.motionsFile =
      BASE_PATH_MOTIONS "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin."
                        "im.bin.small5000.msgpack";
  o_uni1.search_timelimit = 40 * 10e3;

  size_t robot_id = 0;
  size_t robot_num = 1;
  std::vector<dynobench::Trajectory> expanded_trajs_tmp;
  // save expanded nodes
  // std::string output_folder = "../reverse_expansion_vis";
  // create_folder_if_necessary(output_folder);
  std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *>
      all_heuristics(robot_num, nullptr);
  Trajectory traj_out;
  Out_info_tdb info_out;
  std::vector<Motion> motions;
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_motion_primitives_new(o_uni1.motionsFile, *robot, motions,
                             o_uni1.max_motions, o_uni1.cut_actions, false,
                             o_uni1.check_cols);

  o_uni1.motions_ptr = &motions;
  BOOST_REQUIRE_NO_THROW(tdbastar(problem, o_uni1, traj_out, /*constraints*/ {},
                                  info_out, robot_id, /*reverse_search*/ true,
                                  expanded_trajs_tmp, nullptr,
                                  &all_heuristics[robot_id]));

  // save and visualize expanded nodes
  // std::ofstream out2(output_folder + "/exp_trajs_reverse_" +
  // problem.robotType + ".yaml"); // assumes different robot types
  // export_node_expansion(expanded_trajs_tmp, &out2);
  BOOST_REQUIRE_NO_THROW(tdbastar(problem, o_uni1, traj_out, /*constraints*/ {},
                                  info_out, robot_id, /*reverse_search*/ false,
                                  expanded_trajs_tmp, all_heuristics[robot_id],
                                  nullptr));
  BOOST_TEST(info_out.solved, msg);
}

BOOST_AUTO_TEST_CASE(test_eval_multiple_with_constraints) {

  std::vector<Problem> problems{
      DYNOBENCH_BASE "envs/unicycle1_v0/swap/swap1_unicycle.yaml",
      DYNOBENCH_BASE "envs/unicycle2_v0/swap/swap1_unicycle2.yaml",
      DYNOBENCH_BASE "envs/car1_v0/swap/swap1_trailer.yaml"};

  for (auto &p : problems) {
    p.models_base_path = DYNOBENCH_BASE "models/";
  }
  std::vector<std::string> all_constraints{
      DYNOBENCH_BASE "envs/unicycle1_v0/constraints/constraints_unicycle.yaml",
      DYNOBENCH_BASE "envs/unicycle2_v0/constraints/constraints_unicycle2.yaml",
      DYNOBENCH_BASE "envs/car1_v0/constraints/constraints_trailer.yaml"};

  Options_tdbastar o_uni1, o_uni2, o_integrator2, o_car;

  o_uni1.max_motions = 100;
  o_uni1.delta = .5;
  o_uni1.motionsFile =
      BASE_PATH_MOTIONS "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin."
                        "im.bin.small5000.msgpack";

  o_uni2.max_motions = 100;
  o_uni2.delta = .5;
  o_uni2.motionsFile =
      BASE_PATH_MOTIONS "unicycle2_v0__ispso__2023_04_03__15_36_01"
                        ".bin.im.bin.im.bin.small5000.msgpack";

  o_car.max_motions = 400;
  o_car.delta = .75;
  o_car.motionsFile =
      BASE_PATH_MOTIONS "car1_v0_all.bin.sp.bin.small5000.msgpack";

  std::vector<Options_tdbastar> options{o_uni1, o_uni2, o_car};

  for (auto &o : options) {
    o.search_timelimit = 40 * 10e3;
  }

  DYNO_CHECK_EQ(options.size(), problems.size(), AT);
  size_t robot_id = 0;
  std::vector<dynobench::Trajectory> expanded_trajs_tmp;
  for (size_t j = 0; j < options.size(); j++) {

    auto &problem = problems[j];
    auto &option = options[j];
    auto &constraints_file = all_constraints[j];
    std::string msg = problem.name;

    Trajectory traj_out;
    Out_info_tdb info_out;

    // load motions
    std::vector<Motion> motions;

    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotType + ".yaml").c_str(),
        problem.p_lb, problem.p_ub);

    load_motion_primitives_new(option.motionsFile, *robot, motions,
                               option.max_motions, option.cut_actions, false,
                               option.check_cols);

    option.motions_ptr = &motions;
    expanded_trajs_tmp.clear();
    // read constraints
    YAML::Node robot_constraints = YAML::LoadFile(constraints_file);
    size_t i = 0;
    std::vector<Constraint> constraints = {};
    for (const auto &c : robot_constraints["constraints"]) {
      std::vector<double> tmp_constraint;
      const auto tmp_state = c["states"];
      for (const auto &v : tmp_state) {
        tmp_constraint.push_back(v.as<double>());
      }
      Eigen::VectorXd constrained_state =
          Eigen::VectorXd::Map(tmp_constraint.data(), tmp_constraint.size());
      double constrained_time = c["time"].as<float>();
      constraints.push_back({constrained_time, constrained_state});
      i++;
    }
    BOOST_REQUIRE_NO_THROW(tdbastar(
        problem, option, traj_out, constraints, info_out, robot_id,
        /*reverse_search*/ false, expanded_trajs_tmp, nullptr, nullptr));

    BOOST_TEST(info_out.solved, msg);
  }
}
