
#include <boost/test/unit_test.hpp>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"

BOOST_AUTO_TEST_CASE(ompl_geo_cli) {

  std::vector<std::string> _cmd = {
      "../main_rrt_to",
      "--env_file",
      DYNOBENCH_BASE + std::string("envs/unicycle1_v0/parallelpark_0.yaml"),
      "--models_base_path",
      DYNOBENCH_BASE + std::string("models/"),
      "--geo_use_nigh",
      "1",
      "--solver_id",
      "9",
      "--timelimit",
      "3"};

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }
  std::cout << "Running cpp command: \n" << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}

// BOOST_AUTO_TEST_CASE(t_cli) {
//
//   std::vector<std::string> _cmd = {
//       "../main_dbastar",
//       "--env_file",
//       DYNOBENCH_BASE + std::string("envs/unicycle1_v0/bugtrap_0.yaml"),
//       "--max_motions",
//       "30",
//       "--motionsFile",
//       "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/cloud/"
//       "motionsV2/good/unicycle1_v0/"
//       "unicycle1_v0__ispso__2023_04_03__14_56_57.bin",
//       "--use_nigh_nn",
//       "1",
//       "--models_base_path",
//       DYNOBENCH_BASE + std::string("models/")};
//
//   std::string cmd = "";
//
//   for (auto &c : _cmd) {
//     cmd += c + " ";
//   }
//
//   std::cout << "Command is: " << cmd << std::endl;
//   int out = std::system(cmd.c_str());
//   BOOST_TEST(out == 0);
// }
