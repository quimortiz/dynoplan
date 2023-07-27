#include "dynoplan/idbastar/idbastar.hpp"

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

BOOST_AUTO_TEST_CASE(t_uni1_bugtrap) {

  Problem problem(DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml");

  problem.models_base_path = DYNOBENCH_BASE "models/";

  Options_idbAStar options_idbas;
  options_idbas.timelimit = 50;
  Options_dbastar options_dbastar;
  Options_trajopt options_trajopt;

  options_dbastar.motionsFile =
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin";
  options_trajopt.solver_id = 1;
  options_dbastar.cost_delta_factor = 1;
  options_idbas.num_primitives_0 = 30;

  Trajectory traj_out;
  Info_out_idbastar out_info_idbas;

  idbA(problem, options_idbas, options_dbastar, options_trajopt, traj_out,
       out_info_idbas);

  BOOST_TEST(out_info_idbas.solved);
  CSTR_(out_info_idbas.cost);
  BOOST_TEST(out_info_idbas.cost < 60.);
}
