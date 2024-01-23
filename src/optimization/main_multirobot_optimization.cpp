#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include "dynobench/multirobot_trajectory.hpp"

// OMPL headers

#include <dynoplan/optimization/ocp.hpp>
#include <dynoplan/optimization/multirobot_optimization.hpp>




int main(int argc, char *argv[]) {

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string envFile;
  std::string initFile;
  std::string outFile;
  std::string dynobench_base;
  bool sum_robots_cost = true;
  desc.add_options()("help", "produce help message")(
      "env,e", po::value<std::string>(&envFile)->required())(
      "init,i", po::value<std::string>(&initFile)->required())(
      "out,o", po::value<std::string>(&outFile)->required())
(
      "base,b", po::value<std::string>(&dynobench_base)->required())

    (
      "sum,s", po::value<bool>(&sum_robots_cost)->required());

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  
  execute_optimizationMultiRobot(envFile, initFile, outFile,
                                 dynobench_base, sum_robots_cost);
}
