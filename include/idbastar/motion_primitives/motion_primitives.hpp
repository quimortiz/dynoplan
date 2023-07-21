#pragma once
#include "crocoddyl/core/utils/timer.hpp"
#include "dynobench/croco_macros.hpp"
#include "dynobench/motions.hpp"
#include "idbastar/optimization/ocp.hpp"

namespace dynoplan {

struct Options_primitives {

  double time_limit = 1000;    // in seconds
  int max_num_primitives = -1; // use -1 to say MAX
  size_t max_attempts = 1e8;
  std::string dynamics = "unicycle1_v0";
  bool adapt_infeas_primitives = false;
  size_t ref_time_steps = 50;
  size_t min_length_cut = 5;
  size_t max_length_cut = 20;
  size_t max_splits = 10;
  size_t num_threads = 1;
  bool use_random_displacemenet = false;
  std::string models_base_path = "";

  void print(std::ostream &out, const std::string &be = "",
             const std::string &af = ": ") {
    STRY(models_base_path, out, be, af);
    STRY(use_random_displacemenet, out, be, af);
    STRY(num_threads, out, be, af);
    STRY(time_limit, out, be, af);
    STRY(max_num_primitives, out, be, af);
    STRY(dynamics, out, be, af);
    STRY(adapt_infeas_primitives, out, be, af);
    STRY(max_attempts, out, be, af);
    STRY(min_length_cut, out, be, af);
    STRY(max_length_cut, out, be, af);
    STRY(max_splits, out, be, af);
    STRY(ref_time_steps, out, be, af);
  };

  void add_options(po::options_description &desc) {
    set_from_boostop(desc, VAR_WITH_NAME(models_base_path));
    set_from_boostop(desc, VAR_WITH_NAME(use_random_displacemenet));
    set_from_boostop(desc, VAR_WITH_NAME(num_threads));
    set_from_boostop(desc, VAR_WITH_NAME(time_limit));
    set_from_boostop(desc, VAR_WITH_NAME(max_num_primitives));
    set_from_boostop(desc, VAR_WITH_NAME(dynamics));
    set_from_boostop(desc, VAR_WITH_NAME(adapt_infeas_primitives));
    set_from_boostop(desc, VAR_WITH_NAME(max_attempts));
    set_from_boostop(desc, VAR_WITH_NAME(min_length_cut));
    set_from_boostop(desc, VAR_WITH_NAME(max_length_cut));
    set_from_boostop(desc, VAR_WITH_NAME(max_splits));
    set_from_boostop(desc, VAR_WITH_NAME(ref_time_steps));
  }
};

void sort_motion_primitives(
    const dynobench::Trajectories &trajs, dynobench::Trajectories &trajs_out,
    std::function<double(const Eigen::VectorXd &, const Eigen::VectorXd &)>
        distance_fun,
    int top_k = -1, bool naive = false);

void sort_motion_primitives_rand_config(
    const dynobench::Trajectories &__trajs, dynobench::Trajectories &trajs_out,
    std::shared_ptr<dynobench::Model_robot> robot, int top_k);

void split_motion_primitives(const dynobench::Trajectories &in,
                             const std::string &dynamics,
                             dynobench::Trajectories &out,
                             const Options_primitives &options_primitives);

void improve_motion_primitives(const Options_trajopt &options_trajopt,
                               const dynobench::Trajectories &trajs_in,
                               const std::string &dynamics,
                               dynobench::Trajectories &trajs_out,
                               const Options_primitives &options_primitives);

void generate_primitives(const Options_trajopt &options_trajopt,
                         const Options_primitives &options_primitives,
                         dynobench::Trajectories &trajectories);

void generate_primitives_random(const Options_primitives &options_primitives,
                                dynobench::Trajectories &trajectories);
void make_canonical(const dynobench::Trajectories &trajectories,
                    dynobench::Trajectories &trajectories_out,
                    const std::string &dynamics);

} // namespace dynoplan
