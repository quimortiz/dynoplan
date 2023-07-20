
#include <boost/test/unit_test.hpp>

#define DYNOBENCH_BASE "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_cli_generate) {
  std::vector<std::string> _cmd = {"../main_primitives",
                                   "--mode_gen_id",
                                   "0",
                                   "--max_num_primitives",
                                   "10",
                                   "--out_file",
                                   "/tmp/primitives.bin",
                                   "--dynamics",
                                   "unicycle1_v0",
                                   "--models_base_path",
                                   DYNOBENCH_BASE "models/"};

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }

  std::cout << "Command is: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}
