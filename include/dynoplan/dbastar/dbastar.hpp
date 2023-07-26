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
#include "dynobench/motions.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"
#include <fcl/fcl.h>

namespace dynoplan {

namespace ob = ompl::base;
namespace po = boost::program_options;

using Sample = std::vector<double>;
using Sample_ = ob::State;

#include "dynobench/croco_macros.hpp"

void copyToArray(const ompl::base::StateSpacePtr &space, double *reals,
                 const ompl::base::State *source);

double _distance_angle(double a, double b);

double distance_squared_se2(const double *x, const double *y, const double *s);

// a + s * b
void add(const double *a, const double *b, size_t n, double s, double *out);

// a + sb * b + sc * c
void add2(const double *a, const double *b, const double *c, size_t n,
          double sb, double sc, double *out);

#if 0
template <class T> struct L2Q {
  typedef bool is_kdtree_distance;

  typedef T ElementType;
  typedef typename flann::Accumulator<T>::Type ResultType;
  L2Q(std::function<double(const double *, const double *, size_t)> fun)
      : fun(fun) {}
  std::function<double(const double *, const double *, size_t)> fun;
  /**
   *  Compute the squared Euclidean distance between two vectors.
   *
   *	This is highly optimised, with loop unrolling, as it is one
   *	of the most expensive inner loops.
   *
   *	The computation of squared root at the end is omitted for
   *	efficiency.
   */
  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size,
                        ResultType worst_dist = -1) const {
    (void)worst_dist;
    return fun(a, b, size);
  }

  /**
   *	Partial euclidean distance, using just one dimension. This is used by
   *the kd-tree when computing partial distances while traversing the tree.
   *
   *	Squared root is omitted for efficiency.
   */
  template <typename U, typename V>
  inline ResultType accum_dist(const U &a, const V &b, int) const {
    return std::sqrt((a - b) * (a - b));
  }
};

template <typename _T> class FLANNDistanceQ {
public:
  using ElementType = _T;
  using ResultType = double;
  typedef bool is_kdtree_distance;

  FLANNDistanceQ(
      const typename ompl::NearestNeighbors<_T>::DistanceFunction &distFun)
      : distFun_(distFun) {}

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t /*size*/,
                        ResultType /*worst_dist*/ = -1) const {
    return distFun_(*a, *b);
  }

protected:
  const typename ompl::NearestNeighbors<_T>::DistanceFunction &distFun_;
};
#endif

struct HeuNodeWithIndex {

  int index;
  const ob::State *state;
  double dist;
  const ob::State *getState() const { return state; }
};

struct HeuNode {
  const ob::State *state;
  double dist;

  const ob::State *getState() const { return state; }
};

// forward declaration
struct AStarNode;

struct compareAStarNode {
  bool operator()(const AStarNode *a, const AStarNode *b) const;
};

// open type
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

using Ei = std::pair<int, double>;
using EdgeList = std::vector<std::pair<int, int>>;
using DistanceList = std::vector<double>;

void get_distance_all_vertices(const EdgeList &edge_list,
                               const DistanceList &distance_list,
                               double *dist_out, int *parents_out, int n,
                               int goal);

using Edge = std::pair<int, int>;
void backward_tree_with_dynamics(
    const std::vector<std::vector<double>> &data,
    std::vector<Motion> &primitives, std::vector<Edge> &edge_list,
    std::vector<double> &distance_list,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env,
    double delta_sq);

// std::vector<double>;
// std::vector<double>;

void generate_batch(std::function<void(double *, size_t)> free_sampler,
                    std::function<bool(Sample_ *)> checker,
                    size_t num_samples_trials, size_t dim,
                    std::shared_ptr<ompl::control::SpaceInformation> si,
                    std::vector<Sample_ *> &out);

struct SampleNode {
  Sample_ *x;
  double dist;
  int parent;
};

using HeuristicMap = std::vector<SampleNode>;

void compute_heuristic_map(const EdgeList &edge_list,
                           const DistanceList &distance_list,
                           const std::vector<Sample_ *> &batch_samples,
                           std::vector<SampleNode> &heuristic_map);

#if 0
void build_heuristic_motions(
    const std::vector<Sample> &batch_samples /* goal should be last */,
    std::vector<SampleNode> &heuristic_map, std::vector<Motion> &motions,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env,
    double delta_sq) {

  EdgeList edge_list;
  DistanceList distance_list;

  auto out = timed_fun([&] {
    backward_tree_with_dynamics(batch_samples, motions, edge_list,
                                distance_list, bpcm_env, delta_sq);
    return 0;
  });
  std::cout << "Motions Backward time: " << out.second << std::endl;

  compute_heuristic_map(edge_list, distance_list, batch_samples, heuristic_map);
}
#endif

void write_heuristic_map(
    const std::vector<SampleNode> &heuristic_map,
    const std::shared_ptr<ompl::control::SpaceInformation> si,
    const char *filename = "heu_map.txt");

// bool check_edge_at_resolution(
//     const double *start, const double *goal, int n,
//     std::function<bool(const double *, size_t n)> check_fun,
//     double resolution = .2) {

bool check_edge_at_resolution(const Sample_ *start, const Sample_ *goal,
                              std::shared_ptr<RobotOmpl> robot,
                              double resolution = .2);

void build_heuristic_distance(const std::vector<Sample_ *> &batch_samples,
                              std::shared_ptr<RobotOmpl> robot,
                              std::vector<SampleNode> &heuristic_map,
                              double distance_threshold, double resolution);

double euclidean_distance_squared(const double *x, const double *y, size_t n);

double euclidean_distance_scale_squared(const double *x, const double *y,
                                        const double *s, size_t n);

double euclidean_distance(const double *x, const double *y, size_t n);

#if 0
double query_heuristic_map(
    const HeuristicMap &map, const std::vector<double> &x,
    std::function<double(const double *, const double *, size_t n)>
        distance_fun = euclidean_distance_squared) {

  std::vector<double> distances(map.size());
  std::transform(map.begin(), map.end(), distances.begin(), [&](const auto &y) {
    // TODO: if it is too far: then infinity!
    return distance_fun(y.x.data(), x.data(), x.size());
  });
  auto it = std::min_element(distances.begin(), distances.end());

  size_t index = std::distance(distances.begin(), it);
  return map[index].dist;
}
#endif

// dbastar:
// /home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/src/fclHelper.hpp:38:
// void
// ShiftableDynamicAABBTreeCollisionManager<S>::shift_recursive(fcl::detail::NodeBase<fcl::AABB<S>
// >*, fcl::Vector3<S_>&) [with S = floa t; fcl::Vector3<S_> =
// Eigen::Matrix<float, 3, 1>]: Assertion `node->bv.equal(obj->getAABB())'
// failed. fish: Job 1, './dbastar -i ../benchmark/unicy…' terminated by
// signal SIGABRT (Abort)

// TODO: add the angular velocity also!!

// T

// change the nearest neighbour search!!
// the sqrt is even slower
// check the implementation of RRT.

void print_matrix(std::ostream &out,
                  const std::vector<std::vector<double>> &data);

double heuristicCollisionsTree(ompl::NearestNeighbors<HeuNode *> *T_heu,
                               const ob::State *s,
                               std::shared_ptr<RobotOmpl> robot,
                               double connect_radius_h);

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

  // void add_options(po::options_description &desc);
  // void read_from_yaml(YAML::Node &node);
  // void read_from_yaml(const char *file);
};

struct Heuristic_node {
  Eigen::VectorXd x;
  double d; // distance
  int p;    // parent
};

void load_heu_map(const char *file, std::vector<Heuristic_node> &heu_map);

struct Options_dbastar {

  float delta = .3;
  float epsilon = 1.;
  float alpha = .5;
  float connect_radius_h = .5;
  std::string motionsFile = "";
  std::vector<Motion> *motions_ptr = nullptr; // pointer to loaded motions
  std::string outFile = "/tmp/dbastar/out_db.yaml";
  bool filterDuplicates = false;     // very expensive in high dim systems!
  bool primitives_new_format = true; // (false=Format of IROS 22)
  float maxCost = std::numeric_limits<float>::infinity();
  int heuristic = 0;
  size_t max_motions = 1e4;
  double heu_resolution = .5;
  double delta_factor_goal = 1;
  double cost_delta_factor = 0;
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
  bool add_after_expand = false; // this does not improve cost of closed nodes.
                                 // it is fine if heu is admissible
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

struct Result_db {

  bool feasible = false;
  double cost = -1;
  double cost_with_delta_time = -1;
  void print(std::ostream &out) {
    std::string be = "";
    std::string af = ": ";
    out << be << STR(feasible, af) << std::endl;
    out << be << STR(cost, af) << std::endl;
    out << be << STR(cost_with_delta_time, af) << std::endl;
  }
};

struct Time_benchmark {

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

struct Heu_fun {
  virtual double h(const ompl::base::State *x) = 0;
  virtual ~Heu_fun() = default;
};

struct Heu_euclidean : Heu_fun {

  Heu_euclidean(std::shared_ptr<RobotOmpl> robot, ob::State *goal)
      : robot(robot), goal(goal) {}

  std::shared_ptr<RobotOmpl> robot;
  ob::State *goal;

  virtual double h(const ompl::base::State *x) override {
    return robot->cost_lower_bound(x, goal);
  }

  virtual ~Heu_euclidean() override{};
};

struct Heu_blind : Heu_fun {

  Heu_blind() {}

  virtual double h(const ompl::base::State *x) override {
    (void)x;
    return 0;
  }

  virtual ~Heu_blind() override{};
};

struct Heu_roadmap : Heu_fun {

  std::shared_ptr<RobotOmpl> robot;
  std::shared_ptr<ompl::NearestNeighbors<HeuNode *>> T_heu;
  double connect_radius_h = 0.5;
  ob::State *goal;
  ob::State *__x_zero_vel;
  // ob::State *__tmp_vel;

  Heu_roadmap(std::shared_ptr<RobotOmpl> robot,
              const std::vector<Heuristic_node> &heu_map, ob::State *goal,
              const std::string &robot_type = "");

  virtual double h(const ompl::base::State *x) override {
    CHECK(T_heu, AT);

    auto si = robot->getSpaceInformation()->getStateSpace();
    const auto &locations = si->getValueLocations();

    CHECK_EQ(locations.size(), robot->nx, AT);
    CHECK_LEQ(robot->nx_pr, locations.size(), AT);

    for (std::size_t i = 0; i < robot->nx_pr; ++i) {
      *si->getValueAddressAtLocation(__x_zero_vel, locations[i]) =
          *si->getValueAddressAtLocation(x, locations[i]);
    }

    double pos_h = heuristicCollisionsTree(T_heu.get(), __x_zero_vel, robot,
                                           connect_radius_h);
    double vel_h = robot->cost_lower_bound_vel(x, goal);

    return std::max(vel_h, pos_h);
  }

  virtual ~Heu_roadmap() override {

    robot->getSpaceInformation()->freeState(__x_zero_vel);

    // TODO: memory leak
    // I have to delete more stuff!!!
  };
};

void build_heuristic_distance_new(
    const std::vector<Eigen::VectorXd> &batch_samples,
    std::shared_ptr<dynobench::Model_robot> &robot,
    std::vector<Heuristic_node> &heuristic_map, double distance_threshold,
    double resolution);

bool check_edge_at_resolution_new(
    const Eigen::VectorXd &start, const Eigen::VectorXd &goal,
    std::shared_ptr<dynobench::Model_robot> &robot, double resolution);

void dbastar(const dynobench::Problem &problem,
             const Options_dbastar &options_dbastar,
             dynobench::Trajectory &traj_out, Out_info_db &out_info_db);

void write_heu_map(const std::vector<Heuristic_node> &heu_map, const char *file,
                   const char *header = nullptr);

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
  double total_times_ms = 0;
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
    total_times_ms += sw.elapsed_ms();

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

inline void plot_search_tree(std::vector<AStarNode *> nodes,
                             std::vector<Motion> motions,
                             dynobench::Model_robot &robot,
                             const char *filename) {

  std::cout << "plotting search tree to: " << filename << std::endl;
  std::ofstream out(filename);
  out << "nodes:" << std::endl;
  const std::string indent2 = "  ";
  const std::string indent4 = "    ";
  const std::string indent6 = "      ";
  for (auto &n : nodes) {
    out << indent2 << "-" << std::endl;
    out << indent4 << "x: " << n->state_eig.format(dynobench::FMT) << std::endl;
    out << indent4 << "fScore: " << n->fScore << std::endl;
    out << indent4 << "gScore: " << n->gScore << std::endl;
    out << indent4 << "hScore: " << n->hScore << std::endl;
  }
  // out << "edges_reduced:" << std::endl;
  //
  // for (auto &n : nodes) {
  //   if (n->came_from) {
  //     std::cout << indent2 << "-" << std::endl;
  //     out << indent4 << "from:" <<
  //     n->came_from->state_eig.format(FMT)
  //         << std::endl;
  //     out << indent4 << "to:" << n->state_eig.format(FMT) <<
  //     std::endl;
  //   }
  // }
  out << "edges:" << std::endl;

  for (auto &n : nodes) {
    if (n->came_from) {
      out << indent2 << "-" << std::endl;
      out << indent4
          << "from: " << n->came_from->state_eig.format(dynobench::FMT)
          << std::endl;
      out << indent4 << "to: " << n->state_eig.format(dynobench::FMT)
          << std::endl;
      // get the motion

      LazyTraj lazy_traj;
      lazy_traj.offset.resize(robot.get_offset_dim());
      robot.offset(n->came_from->state_eig, lazy_traj.offset);
      lazy_traj.robot = &robot;
      lazy_traj.motion = &motions.at(n->used_motion);

      dynobench::Trajectory traj;
      traj.states = lazy_traj.motion->traj.states;
      traj.actions = lazy_traj.motion->traj.actions;
      lazy_traj.compute(traj);
      out << indent4 << "traj:" << std::endl;
      for (auto &s : traj.states) {
        out << indent6 << "- " << s.format(dynobench::FMT) << std::endl;
      }
    }
  }
}

void generate_heuristic_map(const dynobench::Problem &problem,
                            std::shared_ptr<RobotOmpl> robot_ompl,
                            const Options_dbastar &options_dbastar,
                            std::vector<Heuristic_node> &heu_map);

} // namespace dynoplan
