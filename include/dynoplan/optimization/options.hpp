#pragma once

#include <boost/program_options.hpp>
#include <string>
#include <yaml-cpp/node/node.h>

namespace dynoplan {
namespace po = boost::program_options;

struct Options_trajopt {


  double time_ref = .5;
  double time_weight = .7;
  bool check_with_finite_diff = false;

  bool soft_control_bounds = false;
  bool CALLBACKS = true;
  std::string solver_name;
  bool use_finite_diff = false;
  bool use_warmstart = true;
  bool rollout_warmstart = false;
  bool repair_init_guess = true;
  bool control_bounds = true;
  bool states_reg = false;
  int solver_id = 0;
  double disturbance = 1e-5;
  int num_threads = 1;

  double th_stop = 1e-2;
  double init_reg = 1e2;
  double th_acceptnegstep = .3;
  double noise_level = 1e-5; // factor on top of [-1., 1.]
  double k_linear = 10.;
  double k_contour = 10.;
  double u_bound_scale = 1;

  size_t max_iter = 50;
  size_t window_optimize = 20;
  size_t window_shift = 10;
  size_t max_mpc_iterations = 50;
  std::string debug_file_name = "/tmp/debug_file.yaml";
  double weight_goal = 200.;
  double collision_weight = 100.;
  bool smooth_traj = true;
  bool shift_repeat = false;

  double tsearch_max_rate = 2;
  double tsearch_min_rate = .3;
  int tsearch_num_check = 10;
  bool ref_x0 = false;
  bool interp = false;
  bool welf_format = false;
  bool linear_search = false;
  std::string name = "";
  bool use_mim_solvers = false;
  bool use_mim_filter = false;

  void add_options(po::options_description &desc);

  void __read_from_node(const YAML::Node &node);

  void print(std::ostream &out, const std::string &be = "",
             const std::string &af = ": ") const;

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);
};

void PrintVariableMap(const boost::program_options::variables_map &vm,
                      std::ostream &out);

// TODO: check which methods are deprecated!!
enum class SOLVER {
  traj_opt = 0,
  traj_opt_free_time = 1,
  traj_opt_smooth_then_free_time = 2,
  mpc = 3,
  mpcc = 4,
  mpcc2 = 5,
  traj_opt_mpcc = 6,
  mpc_nobound_mpcc = 7,
  mpcc_linear = 8,
  time_search_traj_opt = 9,
  mpc_adaptative = 10,
  traj_opt_free_time_proxi = 11,
  traj_opt_no_bound_bound = 12,
  traj_opt_free_time_proxi_linear = 13,
  traj_opt_free_time_linear = 14,
  first_fixed_then_free_time = 15,
  none = 16
};

} // namespace dynoplan
