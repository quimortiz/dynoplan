
#include <boost/test/unit_test.hpp>
#define dynobench_base "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_main_optimization) {

  std::vector<std::string> _cmd = {
      "../main_optimization",
      "--solver_id",
      "0",
      "--env_file",
      dynobench_base + std::string("envs/unicycle2_v0/parallelpark_0.yaml"),
      "--models_base_path",
      dynobench_base + std::string("models/"),
      "--init_file",
      dynobench_base +
          std::string("data/unicycle2_0_parallelark_guess_0.yaml")};

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }

  std::cout << "Command is: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}
