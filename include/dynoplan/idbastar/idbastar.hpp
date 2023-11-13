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

namespace dynoplan {

enum class EXIT_CRITERIA {
  max_it,
  time_limit,
  max_solution,
  none,
};

struct Info_out_idbastar : dynobench::Info_out {
  EXIT_CRITERIA exit_criteria = EXIT_CRITERIA::none;
};

struct Options_idbAStar {
  double delta_0 = .3;              // initial delta
  size_t num_primitives_0 = 1000;   // initial number of primitives
  double delta_rate = .9;           // rate to decrease delta
  double num_primitives_rate = 1.5; // rate to increase number of primitives
  double timelimit = 10;            // Time limit in seconds
  size_t max_it = 10;               // Maximum number of iterations
  size_t max_num_sol = 5; //  Stop when you reach this number of solutions
  size_t max_motions_primitives =
      1e4; // Maximum number of primitives to load frol file
  bool new_schedule =
      true; // Schedule for delta and number of primitives of the TRO paper.
  bool add_primitives_opt = true; // Add primitives after optimization

  void add_options(po::options_description &desc) {

    set_from_boostop(desc, VAR_WITH_NAME(delta_0));
    set_from_boostop(desc, VAR_WITH_NAME(num_primitives_0));
    set_from_boostop(desc, VAR_WITH_NAME(delta_rate));
    set_from_boostop(desc, VAR_WITH_NAME(num_primitives_rate));
    set_from_boostop(desc, VAR_WITH_NAME(timelimit));
    set_from_boostop(desc, VAR_WITH_NAME(max_it));
    set_from_boostop(desc, VAR_WITH_NAME(new_schedule));
    set_from_boostop(desc, VAR_WITH_NAME(max_motions_primitives));
    set_from_boostop(desc, VAR_WITH_NAME(add_primitives_opt));
  }
  void print(std::ostream &out, const std::string be = "",
             const std::string af = ": ") const {

    out << be << STR(delta_0, af) << std::endl;
    out << be << STR(num_primitives_0, af) << std::endl;
    out << be << STR(delta_rate, af) << std::endl;
    out << be << STR(num_primitives_rate, af) << std::endl;
    out << be << STR(timelimit, af) << std::endl;
    out << be << STR(max_it, af) << std::endl;
    out << be << STR(new_schedule, af) << std::endl;
    out << be << STR(max_motions_primitives, af) << std::endl;
    out << be << STR(add_primitives_opt, af) << std::endl;
  }

  void __read_from_node(const YAML::Node &node) {

    set_from_yaml(node, VAR_WITH_NAME(delta_0));
    set_from_yaml(node, VAR_WITH_NAME(num_primitives_0));
    set_from_yaml(node, VAR_WITH_NAME(delta_rate));
    set_from_yaml(node, VAR_WITH_NAME(num_primitives_rate));
    set_from_yaml(node, VAR_WITH_NAME(timelimit));
    set_from_yaml(node, VAR_WITH_NAME(max_it));
    set_from_yaml(node, VAR_WITH_NAME(new_schedule));
    set_from_yaml(node, VAR_WITH_NAME(max_motions_primitives));
    set_from_yaml(node, VAR_WITH_NAME(add_primitives_opt));
  }

  void read_from_yaml(YAML::Node &node) {
    if (node["options_idbastar"]) {
      __read_from_node(node["options_idbastar"]);
    } else {
      __read_from_node(node);
    }
  }

  void read_from_yaml(const char *file) {
    std::cout << "loading file: " << file << std::endl;
    YAML::Node node = YAML::LoadFile(file);
    read_from_yaml(node);
  }
};

// TODO
// give options to load primitives and heuristic map only once.
// cli to create and store a heuristic map for a robot in an environment.

void idbA(const dynobench::Problem &problem,
          const Options_idbAStar &options_idbas,
          const Options_dbastar &options_dbastar,
          const Options_trajopt &options_trajopt,
          dynobench::Trajectory &traj_out,
          Info_out_idbastar &info_out_idbastar);

// are you here?

void write_results_idbastar(const char *results_file,
                            const dynobench::Problem &problem,
                            const Options_idbAStar &options_idbastar,
                            const Options_dbastar &options_dbastar,
                            const Options_trajopt &options_trajopt,
                            const Info_out_idbastar &info_out_idbastar);

} // namespace dynoplan
