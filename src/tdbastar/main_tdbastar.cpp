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
#include "dynoplan/tdbastar/tdbastar.hpp"

using namespace dynoplan;
int main(int argc, char *argv[]) {

  Options_tdbastar options_tdbastar;
  po::options_description desc("Allowed options");
  options_tdbastar.add_options(desc);
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
  std::cout << results_file << std::endl;
  if (cfg_file != "") {
    options_tdbastar.read_from_yaml(cfg_file.c_str());
  }

  dynobench::Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;
  dynobench::Trajectory traj;
  Out_info_tdb out_tdb;

  std::cout << "*** options_tdbastar ***" << std::endl;
  options_tdbastar.print(std::cout);
  std::cout << "***" << std::endl;

  // load motions primitives

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  std::vector<Motion> motions;
  load_motion_primitives_new(
      options_tdbastar.motionsFile, *robot, motions, options_tdbastar.max_motions,
      options_tdbastar.cut_actions, false, options_tdbastar.check_cols);

  options_tdbastar.motions_ptr = &motions;
  int fake_id = 0;
  auto filename = "/tmp/dynoplan/main_tdgbastar_out.yaml";
  create_dir_if_necessary(filename);
  std::ofstream out(filename);
  tdbastar(problem, options_tdbastar, traj, out_tdb, fake_id, out);

  std::cout << "*** inout_tdb *** " << std::endl;
  out_tdb.write_yaml(std::cout);
  std::cout << "***" << std::endl;

  CSTR_(results_file);
  std::ofstream results(results_file);
  results << "alg: tdbastar" << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;
  results << "env_file: " << env_file << std::endl;
  results << "cfg_file: " << cfg_file << std::endl;
  results << "results_file: " << results_file << std::endl;
  results << "options tdbastar:" << std::endl;
  options_tdbastar.print(results, "  ");
  out_tdb.write_yaml(results);

  if (out_tdb.solved) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}
