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
#include "dynoplan/optimization/ocp.hpp"
#include "ompl/base/ScopedState.h"

#include "dynobench/general_utils.hpp"
#include "dynoplan/dbastar/dbastar.hpp"
#include "dynoplan/idbastar/idbastar.hpp"

// this is the complete algorithm: idbA*

using namespace dynoplan;
int main(int argc, char *argv[]) {

  srand(time(0));

  Options_idbAStar options_idbastar;
  Options_dbastar options_dbastar;
  Options_trajopt options_trajopt;
  std::string env_file = "";
  std::string cfg_file = "";
  std::string results_file = "";
  std::string models_base_path = "";

  po::options_description desc("Allowed options");
  options_idbastar.add_options(desc);
  options_dbastar.add_options(desc);
  options_trajopt.add_options(desc);
  set_from_boostop(desc, VAR_WITH_NAME(models_base_path));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
  set_from_boostop(desc, VAR_WITH_NAME(results_file));

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
    options_idbastar.read_from_yaml(cfg_file.c_str());
    options_trajopt.read_from_yaml(cfg_file.c_str());
  }

  std::cout << "*** options_idbastar ***" << std::endl;
  options_idbastar.print(std::cout);
  std::cout << "***" << std::endl;

  std::cout << "*** options_dbastar ***" << std::endl;
  options_dbastar.print(std::cout);
  std::cout << "***" << std::endl;

  std::cout << "*** options_trajopt ***" << std::endl;
  options_trajopt.print(std::cout);
  std::cout << "***" << std::endl;

  dynobench::Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;
  dynobench::Trajectory traj_out;
  Info_out_idbastar info_out_idbastar;

  idbA(problem, options_idbastar, options_dbastar, options_trajopt, traj_out,
       info_out_idbastar);

  std::cout << "*** info_out_dbastar   *** " << std::endl;
  info_out_idbastar.print(std::cout);
  std::cout << "***" << std::endl;

  write_results_idbastar(results_file.c_str(), problem, options_idbastar,
                         options_dbastar, options_trajopt, info_out_idbastar);

  info_out_idbastar.print_trajs(results_file.c_str());

  if (traj_out.states.size() && traj_out.actions.size()) {
    std::string file = results_file + ".traj-sol.yaml";
    std::ofstream out(file);
    traj_out.to_yaml_format(out);
  }

  if (info_out_idbastar.solved) {
    return 0;
  } else {
    return 1;
  }
}
