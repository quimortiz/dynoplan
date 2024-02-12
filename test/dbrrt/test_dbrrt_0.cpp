#include "dynoplan/dbrrt/dbrrt.hpp"

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
#include <boost/test/detail/throw_exception.hpp>
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

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(test_quad3d_connect) {

  // Problem problem(DYNOBENCH_BASE +
  //                 std::string("envs/quadrotor_v0/recovery_with_obs.yaml"));

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/quadrotor_v0/window.yaml"));

  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbrrt options_dbrrt;
  options_dbrrt.choose_first_motion_valid = true;
  options_dbrrt.timelimit = 1e5; // in ms
  options_dbrrt.max_motions = 3000;
  options_dbrrt.debug = false;
  options_dbrrt.max_expands = 50000;
  options_dbrrt.cost_bound = 1e6;
  options_dbrrt.delta = .6;       // when expanding
  options_dbrrt.goal_region = .7; // when connecting the trees
  options_dbrrt.timelimit = 300000;
  options_dbrrt.goal_bias = .1;
  options_dbrrt.use_nigh_nn = true;
  options_dbrrt.seed = 0;
  options_dbrrt.do_optimization = 0;
  options_dbrrt.prob_expand_forward = .5; // 0 is working

  options_dbrrt.motionsFile =
      "../../dynomotions/"
      "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack";

  std::vector<Motion> motions;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_env(*robot, problem);

  load_motion_primitives_new(
      options_dbrrt.motionsFile, *robot, motions, options_dbrrt.max_motions,
      options_dbrrt.cut_actions, false, options_dbrrt.check_cols);

  options_dbrrt.motions_ptr = &motions;
  Trajectory traj_out;
  Info_out out_info;

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 0;

  try {
    dbrrtConnect(problem, robot, options_dbrrt, options_trajopt, traj_out,
                 out_info);
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    BOOST_TEST(false, "caught exception");
  }

  BOOST_TEST(out_info.trajs_raw.size() == 1);
  BOOST_TEST(out_info.solved_raw == 1);
}

BOOST_AUTO_TEST_CASE(test_quad2d_connect) {

  int argc = boost::unit_test::framework::master_test_suite().argc;
  char **argv = boost::unit_test::framework::master_test_suite().argv;

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/quad2d_v0/quad_bugtrap.yaml"));
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbrrt options_dbrrt;
  options_dbrrt.choose_first_motion_valid = true;
  options_dbrrt.timelimit = 1e5; // in ms
  options_dbrrt.max_motions = 3000;
  options_dbrrt.debug = true;
  options_dbrrt.max_expands = 50000;
  options_dbrrt.cost_bound = 1e6;
  options_dbrrt.delta = .5;
  options_dbrrt.goal_bias = .1;
  options_dbrrt.use_nigh_nn = true;
  options_dbrrt.seed = 0;
  options_dbrrt.do_optimization = 0;
  options_dbrrt.prob_expand_forward = .5; // 0 is working

  options_dbrrt.motionsFile =
      "../../dynomotions/quad2d_v0_all_im.bin.sp.bin.ca.bin.small5000.msgpack";

  po::options_description desc("Allowed options");
  options_dbrrt.add_options(desc);

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
  }

  std::vector<Motion> motions;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_env(*robot, problem);

  load_motion_primitives_new(
      options_dbrrt.motionsFile, *robot, motions, options_dbrrt.max_motions,
      options_dbrrt.cut_actions, false, options_dbrrt.check_cols);

  options_dbrrt.motions_ptr = &motions;
  Trajectory traj_out;
  Info_out out_info;

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 0;

  try {
    dbrrtConnect(problem, robot, options_dbrrt, options_trajopt, traj_out,
                 out_info);
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    BOOST_TEST(false, "caught exception");
  }

  BOOST_TEST(out_info.trajs_raw.size() == 1);
  BOOST_TEST(out_info.solved_raw == 1);
  // BOOST_TEST(out_info.trajs_opt.size() == 1);
}

BOOST_AUTO_TEST_CASE(test_uni1_bugtrap_connect) {

  int argc = boost::unit_test::framework::master_test_suite().argc;
  char **argv = boost::unit_test::framework::master_test_suite().argv;

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle1_v0/bugtrap_0.yaml"));
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbrrt options_dbrrt;
  options_dbrrt.choose_first_motion_valid = true;
  options_dbrrt.timelimit = 1e5; // in ms
  options_dbrrt.max_motions = 30;
  options_dbrrt.debug = true;
  options_dbrrt.max_expands = 30000;
  options_dbrrt.cost_bound = 1e6;
  options_dbrrt.delta = .3;
  options_dbrrt.goal_bias = .1;
  options_dbrrt.use_nigh_nn = true;
  options_dbrrt.seed = 0;
  options_dbrrt.do_optimization = 0;
  options_dbrrt.prob_expand_forward = .5; // 0 is working

  options_dbrrt.motionsFile = "../../dynomotions/"
                              "unicycle1_v0__ispso__2023_04_03__14_56_57.bin."
                              "im.bin.im.bin.small5000.msgpack";

  po::options_description desc("Allowed options");
  options_dbrrt.add_options(desc);

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
  }

  std::vector<Motion> motions;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_env(*robot, problem);

  load_motion_primitives_new(
      options_dbrrt.motionsFile, *robot, motions, options_dbrrt.max_motions,
      options_dbrrt.cut_actions, false, options_dbrrt.check_cols);

  options_dbrrt.motions_ptr = &motions;
  Trajectory traj_out;
  Info_out out_info;

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 0;
  try {
    dbrrtConnect(problem, robot, options_dbrrt, options_trajopt, traj_out,
                 out_info);
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    BOOST_TEST(false, "caught exception");
  }

  BOOST_TEST(out_info.trajs_raw.size() == 1);
  BOOST_TEST(out_info.solved_raw == 1);
  // BOOST_TEST(out_info.trajs_opt.size() == 1);
}

BOOST_AUTO_TEST_CASE(test_uni1_bugtrap_with_opt) {

  int argc = boost::unit_test::framework::master_test_suite().argc;
  char **argv = boost::unit_test::framework::master_test_suite().argv;

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle1_v0/bugtrap_0.yaml"));

  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbrrt options_dbrrt;
  options_dbrrt.choose_first_motion_valid = true;
  options_dbrrt.timelimit = 1e5; // in ms
  options_dbrrt.max_motions = 30;
  options_dbrrt.debug = false;
  options_dbrrt.max_expands = 30000;
  options_dbrrt.cost_bound = 1e6;
  options_dbrrt.delta = .3;
  options_dbrrt.goal_bias = .1;
  options_dbrrt.use_nigh_nn = true;
  options_dbrrt.seed = 0;
  options_dbrrt.do_optimization = 1;

  options_dbrrt.motionsFile =
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin";

  // "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/cloud/"
  // "motionsV2/good/unicycle1_v0/"
  // "unicycle1_v0__ispso__2023_04_03__14_56_57.bin";

  po::options_description desc("Allowed options");
  options_dbrrt.add_options(desc);

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
  }

  std::vector<Motion> motions;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_env(*robot, problem);
  load_motion_primitives_new(
      options_dbrrt.motionsFile, *robot, motions, options_dbrrt.max_motions,
      options_dbrrt.cut_actions, false, options_dbrrt.check_cols);

  options_dbrrt.motions_ptr = &motions;
  Trajectory traj_out;
  Info_out out_info;

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 0;
  dbrrt(problem, robot, options_dbrrt, options_trajopt, traj_out, out_info);

  BOOST_TEST(out_info.cost < 60);
  BOOST_TEST(out_info.solved);
  BOOST_TEST(out_info.cost_raw);
  BOOST_TEST(out_info.trajs_raw.size() == 1);
  BOOST_TEST(out_info.trajs_opt.size() == 1);

  // now with aorrt
  options_dbrrt.ao_rrt = true;

  Info_out out_info2;
  dbrrt(problem, robot, options_dbrrt, options_trajopt, traj_out, out_info2);
  BOOST_TEST(out_info2.cost < 50);
  BOOST_TEST(out_info2.solved);
  BOOST_TEST(out_info2.solved_raw);
  BOOST_TEST(out_info2.cost_raw);
  BOOST_TEST(out_info2.trajs_raw.size() > 1);
  BOOST_TEST(out_info2.trajs_opt.size() > 1);
}

// BOOST_AUTO_TEST_CASE(t_0) {
//
//   Problem problem(DYNOBENCH_BASE +
//                   std::string("envs/unicycle1_v0/bugtrap_0.yaml"));
//   Options_trajopt options_trajopt;
//   Options_dbrrt options_dbrrt;
//
//   Trajectory traj_out;
//   Info_out out_info_db;
//   dbrrt(problem, options_dbrrt, options_trajopt, traj_out, out_info_db);
//   // dbastar(problem, options_dbastar, traj_out, out_info_db);
//
//   // Options_dbastar options_dbastar;
//   // options_dbastar.models_base_path = DYNOBENCH_BASE +
//   std::string("models/");
//   // options_dbastar.search_timelimit = 1e5; // in ms
//   // options_dbastar.max_motions = 30;
//   // options_dbastar.heuristic = 0;
//   // options_dbastar.motionsFile =
//   // "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/cloud/"
//   //     "motionsV2/good/unicycle1_v0/"
//   //     "unicycle1_v0__ispso__2023_04_03__14_56_57.bin";
//   // options_dbastar.use_nigh_nn = 1;
//   // Out_info_db out_info_db;
//   // Trajectory traj_out;
//   // dbastar(problem, options_dbastar, traj_out, out_info_db);
//   // BOOST_TEST(out_info_db.solved);
//   // CSTR_(out_info_db.cost);
//   // BOOST_TEST(out_info_db.cost < 60.);
// }
