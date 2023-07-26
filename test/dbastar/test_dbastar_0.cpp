
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

BOOST_AUTO_TEST_CASE(test_bugtrap_heu) {

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle1_v0/bugtrap_0.yaml"));
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbastar options_dbastar;
  options_dbastar.search_timelimit = 1e5; // in ms
  options_dbastar.max_motions = 30;
  options_dbastar.motionsFile =
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin";

  Out_info_db out_info_db;
  Trajectory traj_out;

  std::vector<int> heus;
  heus = {0, 1, -1};
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

#define BASE_PATH "../../"

  // TODO: use less primitive so that the test runs faster!!

  o_uni1.max_motions = 300;
  o_uni1.delta = .3;
  o_uni1.motionsFile =
      BASE_PATH "cloud/motionsV2/good/unicycle1_v0/"
                "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin.im.bin";

  o_uni2.max_motions = 400;
  o_uni2.delta = .4;
  o_uni2.motionsFile =
      BASE_PATH "cloud/motionsV2/good/unicycle2_v0/"
                "unicycle2_v0__ispso__2023_04_03__15_36_01.bin.im.bin.im.bin";

  o_car.max_motions = 400;
  o_car.delta = .3;
  o_car.motionsFile =

      BASE_PATH "cloud/motionsV2/good/car1_v0/car1_v0_all.bin.sp.bin";

  o_quad2d.max_motions = 400;
  o_quad2d.delta = .5;
  o_quad2d.motionsFile = BASE_PATH
      "cloud/motionsV2/good/quad2d_v0/quad2d_v0_all_im.bin.sp.bin.ca.bin";
  o_quad2d.new_invariance = true;

  o_quad3d.delta = .8;
  o_quad3d.max_motions = 3000;
  o_quad3d.motionsFile = BASE_PATH "cloud/motionsV2/good/quad3d_v0/"
                                   "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin";
  o_quad3d.new_invariance = true;

  o_quad2dpole.delta = .8;
  o_quad2dpole.max_motions = 4000;
  o_quad2dpole.motionsFile =
      BASE_PATH "cloud/motionsV2/good/quad2dpole_v0/"
                "quad2dpole_all.bin.im.bin.sp1.bin.ca.bin";
  o_quad2dpole.new_invariance = true;

  std::vector<Options_dbastar> options{
      o_uni1, o_uni2, o_car, o_quad2d, o_quad3d, o_quad3d, o_quad2dpole};

  for (auto &o : options) {
    o.search_timelimit = 40 * 10e3;
  }

  // you can choose the heuristic here!!

  CHECK_EQ(options.size(), problems.size(), AT);

  for (size_t j = 0; j < options.size(); j++) {

    auto &problem = problems[j];
    auto &option = options[j];

    std::string msg = problem.name;

    Trajectory traj_out;
    Out_info_db info_out;

    BOOST_REQUIRE_NO_THROW(dbastar(problem, option, traj_out, info_out));

    BOOST_TEST(info_out.solved, msg);
  }

  // #-
  // #- quadrotor_v0 / quad_one_obs
  // #- quadrotor_v0 / window

  // continue here!!
}
