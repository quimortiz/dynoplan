#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>

// BOOST
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
// DYNOPLAN
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"

#include "dynoplan/ompl/robots.h"

using namespace dynoplan;
namespace fs = std::filesystem;
#define DYNOBENCH_BASE "../dynobench/"
// Run from dynoplan/build

void export_final_solution(dynobench::Trajectory &trajectory,
                           std::ofstream *out) {
  float cost = trajectory.cost;
  *out << "cost: " << cost << std::endl;
  *out << "result:" << std::endl;
  std::vector<Eigen::VectorXd> tmp_states = trajectory.states;
  std::vector<Eigen::VectorXd> tmp_actions = trajectory.actions;
  *out << "  - states:" << std::endl;
  for (size_t j = 0; j < tmp_states.size(); ++j) {
    *out << "      - ";
    *out << tmp_states.at(j).format(dynobench::FMT) << std::endl;
  }
  *out << "    actions:" << std::endl;
  for (size_t j = 0; j < tmp_actions.size(); ++j) {
    *out << "      - ";
    *out << tmp_actions.at(j).format(dynobench::FMT) << std::endl;
  }
}

int main(int argc, char *argv[]) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string constraintsFile;
  std::string cfgFile;
  double timeLimit;

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (yaml)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (yaml)")(
      "constraints,cnst", po::value<std::string>(&constraintsFile),
      "constraints file (yaml)")("cfg,c",
                                 po::value<std::string>(&cfgFile)->required(),
                                 "configuration file (yaml)")(
      "time_limit,t", po::value<double>(&timeLimit)->required(),
      "time limit for search");

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
  // read files
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  cfg = cfg["db-cbs"]["default"];
  // tdbstar options
  Options_tdbastar options_tdbastar;
  options_tdbastar.outFile = outputFile;
  options_tdbastar.search_timelimit = timeLimit;
  options_tdbastar.cost_delta_factor = 0;
  options_tdbastar.delta = cfg["delta_0"].as<float>();
  options_tdbastar.fix_seed = 1;
  options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
  std::cout << "*** options_tdbastar ***" << std::endl;
  options_tdbastar.print(std::cout);
  std::cout << "***" << std::endl;
  // tdbastar problem
  dynobench::Problem problem(inputFile);
  std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
  problem.models_base_path = models_base_path;
  // tdb_out info
  Out_info_tdb out_tdb;
  // trajectory for the final solution
  dynobench::Trajectory trajectory;
  // load problem description
  YAML::Node env = YAML::LoadFile(inputFile);
  size_t robot_id = 0; // for a single robot case
  std::vector<Constraint> constraints = {};
  if (constraintsFile != "") {
    YAML::Node cnst = YAML::LoadFile(constraintsFile);
    // std::vector<Constraint> constraints;
    robot_id = cnst["robot_id"].as<size_t>();
    size_t i = 0;
    for (const auto &c : cnst["constraints"]) {
      std::vector<double> tmp_constraint;
      const auto tmp_state = c["states"];
      for (const auto &v : tmp_state) {
        tmp_constraint.push_back(v.as<double>());
      }
      Eigen::VectorXd constrained_state =
          Eigen::VectorXd::Map(tmp_constraint.data(), tmp_constraint.size());
      double constrained_time = c["time"].as<float>();
      constraints.push_back({constrained_time, constrained_state});
      i++;
    }
  }
  bool save_expanded_trajs = false;
  const auto robot_type = problem.robotTypes[robot_id];
  std::string motionsFile;
  std::vector<Motion> motions;
  if (robot_type == "unicycle1_v0" || robot_type == "unicycle1_sphere_v0") {
    motionsFile = "../../new_format_motions/unicycle1_v0/unicycle1_v0.msgpack";
  } else if (robot_type == "unicycle2_v0") {
    motionsFile = "../../new_format_motions/unicycle2_v0/unicycle2_v0.msgpack";
  } else if (robot_type == "car1_v0") {
    motionsFile =
        "../../new_format_motions/car_with_trailers/car_with_trailers.msgpack";
  } else if (robot_type == "integrator2_2d_v0") {
    motionsFile =
        "../../new_format_motions/integrator2_2d_v0/integrator2_2d_v0.msgpack";
  } else if (robot_type == "integrator2_3d_v0") {
    motionsFile = "../../new_format_motions/integrator2_3d_v0/"
                  "integrator2_3d_v0.bin.im.bin.sp.bin";
  } else {
    throw std::runtime_error("Unknown motion filename for this robottype!");
  }
  options_tdbastar.motionsFile = motionsFile;
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + robot_type + ".yaml").c_str(), problem.p_lb,
      problem.p_ub);

  load_motion_primitives_new(options_tdbastar.motionsFile, *robot, motions,
                             options_tdbastar.max_motions,
                             options_tdbastar.cut_actions, true,
                             options_tdbastar.check_cols);
  options_tdbastar.motions_ptr = &motions;
  std::vector<dynobench::Trajectory> expanded_trajs_tmp;
  tdbastar(problem, options_tdbastar, trajectory, constraints, out_tdb,
           robot_id, /*reverse_search*/ false, expanded_trajs_tmp, nullptr,
           nullptr);
  if (save_expanded_trajs) {
    fs::path output_path(outputFile);
    std::string output_folder = output_path.parent_path().string();
    std::ofstream out2(output_folder + "/expanded_trajs.yaml");
    out2 << "trajs:" << std::endl;
    for (auto traj : expanded_trajs_tmp) {
      out2 << "  - " << std::endl;
      traj.to_yaml_format(out2, "    ");
    }
  }

  CSTR_(outputFile);
  std::ofstream results(outputFile);
  results << "alg: tdbastar" << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;
  results << "env_file: " << inputFile << std::endl;
  results << "cfg_file: " << cfgFile << std::endl;
  results << "results_file: " << outputFile << std::endl;
  results << "options dbastar:" << std::endl;
  options_tdbastar.print(results, "  ");
  out_tdb.write_yaml(results);

  if (out_tdb.solved) {
    export_final_solution(trajectory, &results);
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
  return 0;
}
