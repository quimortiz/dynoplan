#include <algorithm>
#include <boost/graph/graphviz.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

#include <flann/flann.hpp>
#include <msgpack.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>

// #include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/datastructures/NearestNeighbors.h>
// #include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include "idbastar/ompl/robots.h"
#include "ompl/base/ScopedState.h"

#include "dynobench/general_utils.hpp"
#include "idbastar/dbastar/dbastar.hpp"

int main(int argc, char *argv[]) {

  Options_dbastar options_dbastar;
  po::options_description desc("Allowed options");
  options_dbastar.add_options(desc);
  std::string cfg_file, results_file, env_file, models_base_path;
  set_from_boostop(desc, VAR_WITH_NAME(models_base_path));
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
  set_from_boostop(desc, VAR_WITH_NAME(results_file));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));

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
  // print options

  if (cfg_file != "") {
    options_dbastar.read_from_yaml(cfg_file.c_str());
  }

  Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;
  Trajectory traj;
  Out_info_db out_db;

  std::cout << "*** options_dbastar ***" << std::endl;
  options_dbastar.print(std::cout);
  std::cout << "***" << std::endl;

  dbastar(problem, options_dbastar, traj, out_db);

  std::cout << "*** inout_db *** " << std::endl;
  out_db.write_yaml(std::cout);
  std::cout << "***" << std::endl;

  CSTR_(results_file);
  std::ofstream results(results_file);
  results << "alg: dbastar" << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;
  results << "env_file: " << env_file << std::endl;
  results << "cfg_file: " << cfg_file << std::endl;
  results << "results_file: " << results_file << std::endl;
  results << "options dbastar:" << std::endl;
  options_dbastar.print(results, "  ");
  out_db.write_yaml(results);

  if (out_db.solved) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}
