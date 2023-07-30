
#include "dynoplan/dbastar/dbastar.hpp"

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

#define DYNOBENCH_BASE "../../dynobench/"

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(test_heu_map) {

  Problem problem(DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_env(*robot, problem);

  Options_dbastar options;
  std::vector<Heuristic_node> heu_map;
  generate_heuristic_map(problem, robot, options, heu_map);

  auto filename = "/tmp/dynoplan/tmp_heu_map.yaml";
  create_dir_if_necessary(filename);
  write_heu_map(heu_map, filename);

  // YOU CAN VISUALIZE WITH:
  // python3 ../utils/show_heuristic_map.py --file
  // /tmp/dynoplan/tmp_heu_map.yaml --max 30

  // check a couple of points

  auto hh = std::make_shared<Heu_roadmap>(robot, heu_map, problem.goal,
                                          problem.robotType);

  hh->connect_radius_h = options.connect_radius_h;

  double h1 = hh->h(problem.goal);
  double h2 = hh->h(problem.start);
  double h3 = hh->h(Eigen::Vector3d(4, 5, 0));
  double h4 = hh->h(Eigen::Vector3d(4, 5.5, 1.57));
  double h5 = hh->h(Eigen::Vector3d(1, 5, 0));

  std::vector<double> order_h = {h1, h3, h4, h5, h2};
  std::vector<double> order_expected = {h1, h3, h4, h5, h2};

  std::sort(order_h.begin(), order_h.end());
  BOOST_TEST(order_h == order_expected);
}

BOOST_AUTO_TEST_CASE(test_bugtrap_heu) {
  // TODO: PLOT the HEU MAP

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle1_v0/bugtrap_0.yaml"));
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbastar options_dbastar;
  options_dbastar.search_timelimit = 1e5; // in ms
  options_dbastar.max_motions = 100;
  options_dbastar.motionsFile =
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin";

  // DEBUG THIS!! -- add tools for debugging :)

  Out_info_db out_info_db;
  Trajectory traj_out;

  std::vector<Motion> motions;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_motion_primitives_new(
      options_dbastar.motionsFile, *robot, motions, options_dbastar.max_motions,
      options_dbastar.cut_actions, false, options_dbastar.check_cols);

  options_dbastar.motions_ptr = &motions;

  std::vector<int> heus;
  // heus = {0, 1, -1};
  heus = {0};
  // heus = {1};
  for (auto &heu : heus) {
    options_dbastar.heuristic = heu;
    Trajectory traj_out;
    Out_info_db info_out;
    BOOST_REQUIRE_NO_THROW(
        dbastar(problem, options_dbastar, traj_out, out_info_db));
    std::string message = std::string("heu: ") + std::to_string(heu);
    BOOST_TEST(out_info_db.solved, heu);
    BOOST_TEST(out_info_db.cost < 100., heu);
  }
}
// ADD the test that i use for  the paper heuristic evaluation!

BOOST_AUTO_TEST_CASE(test_eval_multiple) {

  // "tmp_motions_unicycle1_v1.bin.sp.bin.small5000.msgpack"
  // "tmp_motions_unicycle1_v2.bin.sp.bin.small5000.msgpack"
  // "unicycle2_v0__ispso__2023_04_03__15_36_01.bin.im.bin.im.bin.small5000."
  // "msgpack"
  // "car1_v0_all.bin.sp.bin.small5000.msgpack"
  // "quad2d_v0_all_im.bin.sp.bin.ca.bin.small5000.msgpack"
  // "quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack"
  // "acrobot_v0_all2.bin.sp.bin.small5000.msgpack"
  // "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack"
  // "quad3dompl_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack"

  std::vector<Problem> problems{
      DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml",
      DYNOBENCH_BASE "envs/unicycle2_v0/bugtrap_0.yaml",
      DYNOBENCH_BASE "envs/car1_v0/bugtrap_0.yaml",
      DYNOBENCH_BASE "envs/quad2d_v0/quad_bugtrap.yaml",
      DYNOBENCH_BASE "envs/quadrotor_v0/quad_one_obs.yaml",
      DYNOBENCH_BASE "envs/quadrotor_v0/window.yaml",
      DYNOBENCH_BASE "envs/quad2dpole_v0/window.yaml"};

  for (auto &p : problems) {
    p.models_base_path = DYNOBENCH_BASE "models/";
  }

  Options_dbastar o_uni1, o_uni2, o_car, o_quad2d, o_quad3d, o_quad2dpole;

#define BASE_PATH "../../dynomotions/"

  // TODO: use less primitive so that the test runs faster!!

  o_uni1.max_motions = 300;
  o_uni1.delta = .3;
  o_uni1.motionsFile = BASE_PATH
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin.im.bin.small5000."
      "msgpack";

  o_uni2.max_motions = 400;
  o_uni2.delta = .4;
  o_uni2.motionsFile = BASE_PATH "unicycle2_v0__ispso__2023_04_03__15_36_01."
                                 "bin.im.bin.im.bin.small5000.msgpack";

  o_car.max_motions = 400;
  o_car.delta = .3;
  o_car.motionsFile = BASE_PATH "car1_v0_all.bin.sp.bin.small5000.msgpack";

  o_quad2d.max_motions = 400;
  o_quad2d.delta = .5;
  o_quad2d.motionsFile = BASE_PATH "quad2d_v0_all_im.bin.sp.bin.ca.bin."
                                   "small5000.msgpack";
  o_quad2d.new_invariance = true;

  o_quad3d.delta = .8;
  o_quad3d.max_motions = 3000;
  o_quad3d.motionsFile = BASE_PATH "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin."
                                   "small5000.msgpack";
  o_quad3d.new_invariance = true;

  o_quad2dpole.delta = .8;
  o_quad2dpole.max_motions = 4000;
  o_quad2dpole.motionsFile =
      BASE_PATH "quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack";
  o_quad2dpole.new_invariance = true;

  std::vector<Options_dbastar> options{
      o_uni1, o_uni2, o_car, o_quad2d, o_quad3d, o_quad3d, o_quad2dpole};

  // std::vector<Options_dbastar> options{o_quad3d, o_quad3d};

  for (auto &o : options) {
    o.search_timelimit = 40 * 10e3;
  }

  // you can choose the heuristic here!!

  DYNO_CHECK_EQ(options.size(), problems.size(), AT);

  for (size_t j = 0; j < options.size(); j++) {

    auto &problem = problems[j];
    auto &option = options[j];

    std::string msg = problem.name;

    Trajectory traj_out;
    Out_info_db info_out;

    // load motions

    std::vector<Motion> motions;

    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotType + ".yaml").c_str(),
        problem.p_lb, problem.p_ub);

    load_motion_primitives_new(option.motionsFile, *robot, motions,
                               option.max_motions, option.cut_actions, false,
                               option.check_cols);

    option.motions_ptr = &motions;

    BOOST_REQUIRE_NO_THROW(dbastar(problem, option, traj_out, info_out));

    BOOST_TEST(info_out.solved, msg);
  }

  // #-
  // #- quadrotor_v0 / quad_one_obs
  // #- quadrotor_v0 / window

  // continue here!!
}
