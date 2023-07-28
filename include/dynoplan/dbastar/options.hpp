#pragma once

#include <string>
#include <vector>

#include "dynobench/general_utils.hpp"
#include <boost/program_options.hpp>

namespace dynoplan {

struct Heuristic_node; // forward declaration
struct Motion;         // forward declaration

namespace po = boost::program_options;

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
