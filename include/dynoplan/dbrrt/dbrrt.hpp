#include "dynobench/for_each_macro.hpp"
#include "dynoplan/dbastar/dbastar.hpp"
#include "dynoplan/optimization/ocp.hpp"
#include <string>
#include <vector>

namespace dynoplan {

struct Options_dbrrt {

  double prob_expand_forward = 0.5;
  bool extract_primitives = false;
  bool add_to_search_tree = false;
  bool do_optimization = false;
  bool fix_seed = false;
  double cost_jump = 1;
  double best_cost_prune_factor = .9;
  double cost_weight = .01;
  bool ao_rrt = false;
  bool ao_rrt_rebuild_tree = true;
  double max_step_size = 3;
  bool choose_first_motion_valid = true;
  double goal_region = .3;
  double cost_bound = 200;
  double goal_bias = .1;
  bool debug = false;
  bool new_invariance = true;
  double delta = .3;
  int max_expands = 10000;
  double search_timelimit = 10000;
  bool use_nigh_nn = true;
  int max_motions = 1000;
  std::vector<Motion> *motions_ptr = nullptr; // pointer to loaded motions
  std::string motionsFile = "";
  bool cut_actions = false;
  bool check_cols = true;
  bool use_collision_shape = false;

#define INOUTARGS_dbrrt                                                        \
  do_optimization, cost_jump, best_cost_prune_factor, cost_weight, cost_bound, \
      ao_rrt_rebuild_tree, ao_rrt

  void __load_data(void *source, bool boost, bool write = false,
                   const std::string &be = "") {

    Loader loader;
    loader.use_boost = boost;
    loader.print = write;
    loader.source = source;
    loader.be = be;

#define X(a) loader.set(VAR_WITH_NAME(a));
    APPLYXn(INOUTARGS_dbrrt);
#undef X

    loader.set(VAR_WITH_NAME(prob_expand_forward));
    loader.set(VAR_WITH_NAME(add_to_search_tree));
    loader.set(VAR_WITH_NAME(extract_primitives));
    loader.set(VAR_WITH_NAME(max_step_size));
    loader.set(VAR_WITH_NAME(choose_first_motion_valid));
    loader.set(VAR_WITH_NAME(goal_region));
    loader.set(VAR_WITH_NAME(debug));
    loader.set(VAR_WITH_NAME(new_invariance));
    loader.set(VAR_WITH_NAME(search_timelimit));
    loader.set(VAR_WITH_NAME(max_expands));
    loader.set(VAR_WITH_NAME(delta));
    loader.set(VAR_WITH_NAME(max_motions));
    loader.set(VAR_WITH_NAME(use_nigh_nn));

    loader.set(VAR_WITH_NAME(fix_seed));
    loader.set(VAR_WITH_NAME(motionsFile));
    loader.set(VAR_WITH_NAME(cut_actions));
    loader.set(VAR_WITH_NAME(check_cols));
    loader.set(VAR_WITH_NAME(use_collision_shape));
  }

  void add_options(po::options_description &desc) { __load_data(&desc, true); }

  void print(std::ostream &out, const std::string &be = "",
             const std::string &af = ": ") const {

    auto ptr = const_cast<Options_dbrrt *>(this);
    ptr->__load_data(&out, false, true, be);
  }

  void read_from_yaml(const char *file) {
    std::cout << "loading file: " << file << std::endl;
    YAML::Node node = YAML::LoadFile(file);
    read_from_yaml(node);
  }

  void read_from_yaml(YAML::Node &node) {

    if (node["options_dbastar"]) {
      __read_from_node(node["options_dbastar"]);
    } else {
      __read_from_node(node);
    }
  }
  void __read_from_node(const YAML::Node &node) {
    __load_data(&const_cast<YAML::Node &>(node), false);
  }
};

void dbrrt(const dynobench::Problem &problem,
           const Options_dbrrt &options_dbrrt,
           const Options_trajopt &options_trajopt,
           dynobench::Trajectory &traj_out, dynobench::Info_out &out_info_db);

void from_solution_to_yaml_and_traj(dynobench::Model_robot &robot,
                                    const std::vector<Motion> &motions,
                                    AStarNode *solution,
                                    const dynobench::Problem &problem,
                                    dynobench::Trajectory &traj_out,
                                    std::ofstream *out = nullptr);

void dbrrtConnect(const dynobench::Problem &problem,
                  const Options_dbrrt &options_dbrrt,
                  const Options_trajopt &options_trajopt,
                  dynobench::Trajectory &traj_out,
                  dynobench::Info_out &info_out);

} // namespace dynoplan
