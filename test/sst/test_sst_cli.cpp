
#include <boost/test/unit_test.hpp>

#define DYNOBENCH_BASE "../../dynobench/dynobench/"

BOOST_AUTO_TEST_CASE(t_cli) {

  //     std::vector<std::string> _cmd = {
  //
  //         std::string cmd =
  //             "make main_ompl && ./main_ompl --env_file "
  //             "../benchmark/unicycle_first_order_0/parallelpark_0.yaml "
  //             "--results_file results_sst.yaml --timelimit 10";
  //
  // std::cout << "Running cpp command: \n" << cmd << std::endl;
  // int out = std::system(cmd.c_str());
  // BOOST_TEST(out == 0);
  // }

  std::vector<std::string> _cmd = {
      "../main_sst",
      "--env_file",
      DYNOBENCH_BASE + std::string("envs/unicycle1_v0/parallelpark_0.yaml"),
      "--sst_use_nigh",
      "1",
      "--models_base_path",
      DYNOBENCH_BASE + std::string("models/"),
      "--timelimit",
      "5"};

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }

  std::cout << "Command is: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}
