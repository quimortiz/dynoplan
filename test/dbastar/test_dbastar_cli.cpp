
#include <boost/test/unit_test.hpp>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"
#define BASE_PATH_MOTIONS "../../dynomotions/"

BOOST_AUTO_TEST_CASE(t_cli) {

  std::vector<std::string> _cmd = {
      "../main_dbastar",
      "--env_file",
      DYNOBENCH_BASE + std::string("envs/unicycle1_v0/bugtrap_0.yaml"),
      "--max_motions",
      "30",
      "--motionsFile",
      BASE_PATH_MOTIONS "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin."
                        "im.bin.small5000.msgpack",
      "--models_base_path",
      DYNOBENCH_BASE + std::string("models/")};

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }

  std::string cmd0 = "make ../main_dbastar";
  std::cout << "Command is: " << cmd0 << std::endl;
  int out0 = std::system(cmd0.c_str());
  BOOST_TEST(out0 == 0);

  std::cout << "Command is: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}
