#pragma once

#include <string>
#include <vector>

#include "dynobench/general_utils.hpp"
#include <boost/program_options.hpp>

namespace dynoplan {

struct Heuristic_node; // forward declaration
struct Motion;         // forward declaration

namespace po = boost::program_options;

/**
 * @brief Options for the dbastar algorithm
 *
 * It is used to load the options from a yaml file/boost options and to pass
 * them to the the function dbastar. It also prints all the options to a file.
 *
 */
struct Options_tdbastar {

  bool fix_seed = 0; // use 1 to fix the seed of srand to 0
  float delta = .3;  // discontinuity bound
  float alpha =
      .5; // How discontinuity bound is shared between expansion and reaching
  float connect_radius_h = .5; // Connection radius for heuristic (only ROADMAP)
  std::string motionsFile = "";               // file with motion primitives
  std::vector<Motion> *motions_ptr = nullptr; // Pointer to loaded motions
  std::string outFile =
      "/tmp/dynoplan/out_db.yaml"; // output file to write some results
  float maxCost =
      std::numeric_limits<float>::infinity(); // Cost bound during search
  int heuristic = 0;                          // which heuristic to use
  size_t max_motions = 1e4;   //  Max number of motions to use in the search
  double heu_resolution = .5; // used only for ROADMAP heuristic
  double delta_factor_goal =
      1; // Acceptable distance to goal (in terms of delta)
  double cost_delta_factor = 1;    // Rate to add cost to the discontinuity
  size_t num_sample_trials = 3000; // Used in Roadmap heuristic
  size_t max_size_heu_map = 1000;  // Used in Roadmap heuristic
  size_t max_expands =
      1e6; // Max expands -- used as one of the stopping criteria
  bool cut_actions = false; // Cut input primitives to have more primitives
  bool debug = false;
  int limit_branching_factor =
      20; // Limit on branching factor to encourage more deep search
  bool use_collision_shape =
      true; // Use collision shapes (if available for the dynamics)
  bool new_invariance = false; // If true, allow for system dependent invariance
  std::vector<Heuristic_node> *heu_map_ptr =
      nullptr;                   // Pointer to a loaded heuristic map
  std::string heu_map_file;      // File that contains the heuristic map
  bool add_after_expand = false; // this does not improve cost of closed

  double search_timelimit = 1e4;    // in ms
  double heu_connection_radius = 1; // connection radius for ROADMAP heuristic
  bool use_nigh_nn = true;          // use nigh for nearest neighbor.
  bool check_cols = true;
  bool rewire = true; // to allow rewiring during the search
  double w = 1.0;     // omega for ecbs
  bool always_add_node = false;

  void add_options(po::options_description &desc);

  void __load_data(void *source, bool boost, bool write = false,
                   const std::string &be = "");

  void print(std::ostream &out, const std::string &be = "",
             const std::string &af = ": ") const;

  void __read_from_node(const YAML::Node &node);
  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);
};
} // namespace dynoplan
