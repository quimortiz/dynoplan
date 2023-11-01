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

struct Options_dbastar {

  bool fix_seed = 0;
  float delta = .3;
  float epsilon = 1.;
  float alpha = .5;
  float connect_radius_h = .5;
  std::string motionsFile = "";
  std::vector<Motion> *motions_ptr = nullptr; // pointer to loaded motions
  std::string outFile = "/tmp/dynoplan/out_db.yaml";
  bool filterDuplicates = false; // very expensive in high dim systems!
  float maxCost = std::numeric_limits<float>::infinity();
  int heuristic = 0;
  size_t max_motions = 1e4;
  double heu_resolution = .5;
  double delta_factor_goal = 1;
  double cost_delta_factor = 1;
  int rebuild_every = 5000;
  size_t num_sample_trials = 3000;
  size_t max_size_heu_map = 1000;
  size_t max_expands = 1e6;
  bool cut_actions = false;
  int duplicate_detection_int = 0;
  bool use_landmarks = false;
  double factor_duplicate_detection = 2;
  double epsilon_soft_duplicate = 1.5;
  bool add_node_if_better = false;
  bool debug = false;
  int limit_branching_factor = 20;
  bool use_collision_shape = true;
  bool always_add = false;
  bool new_invariance = false;
  double col_resolution = .01; // in meters

  std::vector<Heuristic_node> *heu_map_ptr = nullptr;
  std::string heu_map_file;
  bool add_after_expand = false; // this does not improve cost of closed
                                 // nodes. it is fine if heu is admissible
  bool propagate_controls =
      false; // TODO: check what happens, in the style of Raul Shome

  double search_timelimit = 1e4; // in ms
  double heu_connection_radius = 1;
  bool use_nigh_nn = true;
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
