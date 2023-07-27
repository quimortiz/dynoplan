#pragma once
#include <algorithm>
// // #include <boost/graph/graphviz.hpp>
#include "Eigen/Core"
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
//
// #include <flann/flann.hpp>
// #include <msgpack.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
//
// // #include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
//
// // OMPL headers

// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
// #include <ompl/datastructures/NearestNeighborsFLANN.h>
// #include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
// #include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
//
// #include "fclHelper.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"
#include <fcl/fcl.h>

#include "dynoplan/dbastar/heuristics.hpp"
#include "dynoplan/dbastar/options.hpp"

namespace dynoplan {

namespace ob = ompl::base;

using Sample = std::vector<double>;
using Sample_ = ob::State;

struct AStarNode;
struct compareAStarNode {
  bool operator()(const AStarNode *a, const AStarNode *b) const;
};
typedef typename boost::heap::d_ary_heap<AStarNode *, boost::heap::arity<2>,
                                         boost::heap::compare<compareAStarNode>,
                                         boost::heap::mutable_<true>>
    open_t;

// Node type (used for open and explored states)
struct AStarNode {
  const ob::State *state;
  Eigen::VectorXd state_eig;

  float fScore;
  float gScore;
  float hScore;

  double get_cost() const { return gScore; }

  const AStarNode *came_from;
  fcl::Vector3d used_offset;
  size_t used_motion;
  int intermediate_state =
      -1; // checking intermediate states for reaching the goal.

  open_t::handle_type handle;
  bool is_in_open = false;
  bool valid = true;

  const ob::State *getState() { return state; }
  const Eigen::VectorXd &getStateEig() { return state_eig; }

  void write(std::ostream &out) {
    out << state_eig.format(dynobench::FMT) << std::endl;
    out << "fScore: " << fScore << " gScore: " << gScore
        << " hScore: " << hScore << std::endl;
    out << " used_motion: " << used_motion
        << " intermediate_state: " << intermediate_state
        << " is_in_open: " << is_in_open << " valid: " << valid << std::endl;
  }
};

float heuristic(std::shared_ptr<RobotOmpl> robot, const ob::State *s,
                const ob::State *g);

using Edge = std::pair<int, int>;
void backward_tree_with_dynamics(
    const std::vector<std::vector<double>> &data,
    std::vector<Motion> &primitives, std::vector<Edge> &edge_list,
    std::vector<double> &distance_list,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env,
    double delta_sq);

enum class Duplicate_detection {
  NO = 0,
  HARD = 1,
  SOFT = 2,
};

enum class Terminate_status {
  SOLVED = 0,
  MAX_EXPANDS = 1,
  EMPTY_QUEUE = 2,
  MAX_TIME = 3,
  UNKNOWN = 4,
  SOLVED_RAW = 5,
  SOLVED_OPT = 6,
};

static const char *terminate_status_str[] = {
    "SOLVED",  "MAX_EXPANDS", "EMPTY_QUEUE", "MAX_TIME",
    "UNKNOWN", "SOLVED_RAW",  "SOLVED_OPT",

};

struct Out_info_db {

  double cost = -1;
  bool solved = 0;
  double cost_with_delta_time = -1;
  // void print(std::ostream &out);
  double time_search = -1;
  std::map<std::string, std::string> data;

  void write_yaml(std::ostream &out) {

    out << STR_(cost) << std::endl;
    out << STR_(solved) << std::endl;
    out << STR_(cost_with_delta_time) << std::endl;
    out << STR_(time_search) << std::endl;
    out << "data:" << std::endl;
    for (auto &[k, v] : data) {
      out << "  " << k << ": " << v << std::endl;
    }
  }
};

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

// void generate_env(YAML::Node &env,
//                   std::vector<fcl::CollisionObjectf *> &obstacles,
//                   fcl::BroadPhaseCollisionManagerf *bpcm_env);

double automatic_delta(double delta_in, double alpha, RobotOmpl &robot,
                       ompl::NearestNeighbors<Motion *> &T_m);

void filte_duplicates(std::vector<Motion> &motions, double delta, double alpha,
                      RobotOmpl &robot, ompl::NearestNeighbors<Motion *> &T_m);

void dbastar(const dynobench::Problem &problem, Options_dbastar options_dbastar,
             dynobench::Trajectory &traj_out, Out_info_db &out_info_db);

struct LazyTraj {

  Eigen::VectorXd offset;
  dynobench::Model_robot *robot;
  Motion *motion;

  void compute(dynobench::Trajectory &tmp_traj, bool forward = true) {
    assert(offset.size());
    assert(robot);
    assert(motion);
    if (forward) {
      robot->transform_primitive(offset, motion->traj.states,
                                 motion->traj.actions, tmp_traj.states,
                                 tmp_traj.actions);
    } else {
      // TODO: change this outside translation invariance
      robot->transform_primitive(offset, motion->traj.states,
                                 motion->traj.actions, tmp_traj.states,
                                 tmp_traj.actions);
    }
  }
};

struct Expander {

  dynobench::Model_robot *robot;
  ompl::NearestNeighbors<Motion *> *T_m;
  Eigen::VectorXd canonical_state;
  Eigen::VectorXd offset;
  Motion fakeMotion;
  std::vector<Motion *> neighbors_m;
  double delta = -1;
  bool random = true;
  std::mt19937 g;
  size_t max_k = std::numeric_limits<size_t>::max();
  double time_in_nn = 0;
  bool verbose = false;

  Expander(dynobench::Model_robot *robot, ompl::NearestNeighbors<Motion *> *T_m,
           double delta)
      : robot(robot), T_m(T_m), delta(delta) {
    canonical_state.resize(robot->nx);
    offset.resize(robot->get_offset_dim());
    fakeMotion.idx = -1;
    fakeMotion.traj.states.push_back(Eigen::VectorXd(robot->nx));
    std::random_device rd;
    g = std::mt19937{rd()};
  }

  void seed(int seed) { g.seed(seed); }

  void expand_lazy(Eigen::Ref<const Eigen::VectorXd> x,
                   std::vector<LazyTraj> &lazy_trajs) {

    robot->canonical_state(x, canonical_state);
    robot->offset(x, offset);
    fakeMotion.traj.states.at(0) = canonical_state;
    assert(delta > 0);

    Stopwatch sw;
    T_m->nearestR(&fakeMotion, delta, neighbors_m);
    time_in_nn += sw.elapsed_ms();

    if (!neighbors_m.size() && verbose) {

      std::cout << "no neighours for state " << x.format(dynobench::FMT)
                << std::endl;

      std::cout << "close state is  " << std::endl;
      auto close_motion = T_m->nearest(&fakeMotion);
      CSTR_V(close_motion->getStateEig());
      std::cout << std::endl;

      std::cout << "close distance is:  "
                << robot->distance(close_motion->getStateEig(),
                                   fakeMotion.getStateEig())
                << std::endl;

      std::cout << "R is " << delta << std::endl;
    }

    if (random)
      std::shuffle(neighbors_m.begin(), neighbors_m.end(), g);

    assert(lazy_trajs.size() == 0);
    lazy_trajs.reserve(std::min(neighbors_m.size(), max_k));
    for (size_t i = 0; i < std::min(neighbors_m.size(), max_k); i++) {
      auto &m = neighbors_m.at(i);
      LazyTraj lazy_traj;
      lazy_traj.offset = offset;
      lazy_traj.robot = robot;
      lazy_traj.motion = m;
      lazy_trajs.push_back(lazy_traj);
    }
  }
};

void plot_search_tree(std::vector<AStarNode *> nodes,
                      std::vector<Motion> &motions,
                      dynobench::Model_robot &robot, const char *filename);

void from_solution_to_yaml_and_traj(dynobench::Model_robot &robot,
                                    const std::vector<Motion> &motions,
                                    AStarNode *solution,
                                    const dynobench::Problem &problem,
                                    dynobench::Trajectory &traj_out,
                                    std::ofstream *out = nullptr);

} // namespace dynoplan
