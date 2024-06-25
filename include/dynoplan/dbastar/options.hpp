#pragma once

#include <string>
#include <vector>

#include "dynobench/general_utils.hpp"
#include <boost/program_options.hpp>

namespace dynoplan {

struct Heuristic_node; // forward declaration
struct Motion;         // forward declaration

namespace po = boost::program_options;

struct Time_benchmark {

  double extra_time = 0.;
  double check_bounds = 0.0;
  double time_lazy_expand = 0.0;
  double time_alloc_primitive = 0.0;
  double build_heuristic = 0.0;
  double time_transform_primitive = 0.0;
  double time_queue = 0.0;
  double time_hfun = 0.0;
  double time_nearestMotion = 0.0;
  double time_nearestNode = 0.0;
  double time_nearestNode_add = 0.0;
  double time_nearestNode_search = 0.0;
  double time_collisions = 0.0;
  double prepare_time = 0.0;
  double total_time = 0.;
  int expands = 0;
  int num_nn_motions = 0;
  int num_nn_states = 0;
  int num_col_motions = 0;
  int motions_tree_size = 0;
  int states_tree_size = 0;
  double time_search = 0;
  double time_collision_heuristic = 0;
  double time_check_constraints = 0;

  void inline write(std::ostream &out) {

    std::string be = "";
    std::string af = ": ";

    out << be << STR(extra_time, af) << std::endl;
    out << be << STR(check_bounds, af) << std::endl;
    out << be << STR(time_lazy_expand, af) << std::endl;
    out << be << STR(time_alloc_primitive, af) << std::endl;
    out << be << STR(time_transform_primitive, af) << std::endl;
    out << be << STR(build_heuristic, af) << std::endl;
    out << be << STR(time_queue, af) << std::endl;
    out << be << STR(time_search, af) << std::endl;
    out << be << STR(time_collision_heuristic, af) << std::endl;
    out << be << STR(time_check_constraints, af) << std::endl;
    out << be << STR(time_nearestMotion, af) << std::endl;
    out << be << STR(time_nearestNode, af) << std::endl;
    out << be << STR(time_nearestNode_add, af) << std::endl;
    out << be << STR(time_nearestNode_search, af) << std::endl;
    out << be << STR(time_collisions, af) << std::endl;
    out << be << STR(prepare_time, af) << std::endl;
    out << be << STR(total_time, af) << std::endl;
    out << be << STR(expands, af) << std::endl;
    out << be << STR(num_nn_motions, af) << std::endl;
    out << be << STR(num_nn_states, af) << std::endl;
    out << be << STR(num_col_motions, af) << std::endl;
    out << be << STR(motions_tree_size, af) << std::endl;
    out << be << STR(states_tree_size, af) << std::endl;
    out << be << STR(time_hfun, af) << std::endl;
  };

  inline std::map<std::string, std::string> to_data() const {
    std::map<std::string, std::string> out;

    out.insert(NAME_AND_STRING(check_bounds));
    out.insert(NAME_AND_STRING(time_lazy_expand));
    out.insert(NAME_AND_STRING(time_alloc_primitive));
    out.insert(NAME_AND_STRING(time_transform_primitive));
    out.insert(NAME_AND_STRING(build_heuristic));
    out.insert(NAME_AND_STRING(time_queue));
    out.insert(NAME_AND_STRING(time_search));
    out.insert(NAME_AND_STRING(time_nearestMotion));
    out.insert(NAME_AND_STRING(time_nearestNode));
    out.insert(NAME_AND_STRING(time_nearestNode_add));
    out.insert(NAME_AND_STRING(time_nearestNode_search));
    out.insert(NAME_AND_STRING(time_collisions));
    out.insert(NAME_AND_STRING(prepare_time));
    out.insert(NAME_AND_STRING(total_time));
    out.insert(NAME_AND_STRING(expands));
    out.insert(NAME_AND_STRING(num_nn_motions));
    out.insert(NAME_AND_STRING(num_nn_states));
    out.insert(NAME_AND_STRING(num_col_motions));
    out.insert(NAME_AND_STRING(motions_tree_size));
    out.insert(NAME_AND_STRING(states_tree_size));
    out.insert(NAME_AND_STRING(time_hfun));
    return out;
  };
};

/**
 * @brief Options for the dbastar algorithm
 *
 * It is used to load the options from a yaml file/boost options and to pass
 * them to the the function dbastar. It also prints all the options to a file.
 *
 */
struct Options_dbastar {

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
