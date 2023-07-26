#include <algorithm>
#include <boost/graph/graphviz.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

// #include <flann/flann.hpp>
// #include <msgpack.hpp>
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

#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"

#include "dynobench/general_utils.hpp"
#include "dynoplan/dbrrt/dbrrt.hpp"

using namespace dynobench;
using namespace dynoplan;

int main(int argc, char *argv[]) {

  Options_trajopt options_trajopt;
  Options_dbrrt options_dbrrt;
  po::options_description desc("Allowed options");
  std::string cfg_file, results_file, env_file, models_base_path;
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
  set_from_boostop(desc, VAR_WITH_NAME(results_file));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(models_base_path));
  options_trajopt.add_options(desc);
  options_dbrrt.add_options(desc);

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
    options_dbrrt.read_from_yaml(cfg_file.c_str());
  }

  Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;
  Trajectory traj;
  Info_out out_db;

  std::cout << "*** options_dbrrt ***" << std::endl;
  options_dbrrt.print(std::cout);
  std::cout << "***" << std::endl;

  std::vector<Motion> motions;
  load_motion_primitives_new(
      options_dbrrt.motionsFile, *robot_factory_ompl(problem), motions,
      options_dbrrt.max_motions, options_dbrrt.cut_actions, false,
      options_dbrrt.check_cols);

  options_dbrrt.motions_ptr = &motions;

  dbrrt(problem, options_dbrrt, options_trajopt, traj, out_db);

  std::cout << "*** inout_db *** " << std::endl;
  out_db.to_yaml(std::cout);
  std::cout << "***" << std::endl;

  CSTR_(results_file);
  std::ofstream results(results_file);
  results << "alg: dbastar" << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;
  results << "env_file: " << env_file << std::endl;
  results << "cfg_file: " << cfg_file << std::endl;
  results << "results_file: " << results_file << std::endl;
  results << "options dbastar:" << std::endl;
  options_dbrrt.print(results, "  ");
  out_db.to_yaml(results);

  if (out_db.solved) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}
