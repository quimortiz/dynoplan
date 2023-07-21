#pragma once
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

// #include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sst/SST.h>

#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models.hpp"
#include "idbastar/ompl/robots.h"
#include "idbastar/optimization/ocp.hpp"

namespace dynoplan {

struct Options_geo {
  std::string planner = "rrt*";
  double timelimit = 10; // TODO: which unit?
  // double goalregion = .1;
  double goalBias = -1;
  double range = -1;
  std::string outFile = "out.yaml";
  bool geo_use_nigh = false;

  void add_options(po::options_description &desc) {

    set_from_boostop(desc, VAR_WITH_NAME(geo_use_nigh));
    set_from_boostop(desc, VAR_WITH_NAME(range));
    set_from_boostop(desc, VAR_WITH_NAME(goalBias));
    set_from_boostop(desc, VAR_WITH_NAME(planner));
    set_from_boostop(desc, VAR_WITH_NAME(timelimit));
    set_from_boostop(desc, VAR_WITH_NAME(outFile));
  }

  void read_from_yaml(const char *file) {
    std::cout << "loading file: " << file << std::endl;
    YAML::Node node = YAML::LoadFile(file);
    read_from_yaml(node);
  }

  void read_from_yaml(YAML::Node &node) {
    if (node["options_geo"]) {
      __read_from_node(node["options_geo"]);

    } else {
      __read_from_node(node);
    }
  }
  void __read_from_node(const YAML::Node &node) {

    set_from_yaml(node, VAR_WITH_NAME(geo_use_nigh));
    set_from_yaml(node, VAR_WITH_NAME(range));
    set_from_yaml(node, VAR_WITH_NAME(goalBias));
    set_from_yaml(node, VAR_WITH_NAME(planner));
    set_from_yaml(node, VAR_WITH_NAME(timelimit));
    set_from_yaml(node, VAR_WITH_NAME(outFile));
  }

  void print(std::ostream &out, const std::string &be = "",
             const std::string &af = ": ") {

    out << be << STR(geo_use_nigh, af) << std::endl;
    out << be << STR(goalBias, af) << std::endl;
    out << be << STR(range, af) << std::endl;
    out << be << STR(planner, af) << std::endl;
    out << be << STR(timelimit, af) << std::endl;
    STRY(outFile, out, be, af);
  }
};

void solve_ompl_geometric(const dynobench::Problem &problem,
                          const Options_geo &options_geo,
                          const Options_trajopt &options_trajopt,
                          dynobench::Trajectory &traj_out,
                          dynobench::Info_out &info_out_omplgeo);

} // namespace dynoplan
