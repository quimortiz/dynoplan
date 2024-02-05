#pragma once
#include <boost/graph/graphviz.hpp>

// #include <flann/flann.hpp>
// #include <msgpack.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>

// #include <boost/functional/hash.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/datastructures/NearestNeighbors.h>
// #include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"

// boost stuff for the graph
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include "dynobench/general_utils.hpp"

#include "dynoplan/dbastar/options.hpp"

namespace dynoplan {

namespace ob = ompl::base;
using Sample_ = ob::State;

struct SampleNode {
  Sample_ *x;
  double dist;
  int parent;
};

struct HeuNodeWithIndex {

  int index;
  const ob::State *state;
  double dist;
  const ob::State *getState() const { return state; }
};

struct Heuristic_node {
  Eigen::VectorXd x;
  double d; // distance
  int p;    // parent
  const Eigen::VectorXd &getStateEig() const { return x; }
  const double get_cost() const { ERROR_WITH_INFO("should not be here!"); }
};

void load_heu_map(const char *file, std::vector<Heuristic_node> &heu_map);

struct HeuNode {
  const ob::State *state;
  double dist;
  Eigen::VectorXd x;

  const ob::State *getState() const { return state; }
  const Eigen::VectorXd &getStateEig() const { return x; }
};

using HeuristicMap = std::vector<SampleNode>;

using Ei = std::pair<int, double>;
using EdgeList = std::vector<std::pair<int, int>>;
using DistanceList = std::vector<double>;

void compute_heuristic_map(const EdgeList &edge_list,
                           const DistanceList &distance_list,
                           const std::vector<Sample_ *> &batch_samples,
                           std::vector<SampleNode> &heuristic_map);

void get_distance_all_vertices(const EdgeList &edge_list,
                               const DistanceList &distance_list,
                               double *dist_out, int *parents_out, int n,
                               int goal);

void write_heuristic_map(
    const std::vector<SampleNode> &heuristic_map,
    const std::shared_ptr<ompl::control::SpaceInformation> si,
    const char *filename = "heu_map.txt");

void build_heuristic_distance(const std::vector<Sample_ *> &batch_samples,
                              std::shared_ptr<RobotOmpl> robot,
                              std::vector<SampleNode> &heuristic_map,
                              double distance_threshold, double resolution);

double heuristicCollisionsTree(ompl::NearestNeighbors<Heuristic_node *> *T_heu,
                               const Eigen::VectorXd &x,
                               std::shared_ptr<dynobench::Model_robot> robot,
                               double connect_radius_h);

void write_heu_map(const std::vector<Heuristic_node> &heu_map, const char *file,
                   const char *header = nullptr);

struct Heu_fun {
  virtual double h(const Eigen::VectorXd &x) = 0;
  virtual ~Heu_fun() = default;
};

struct Heu_euclidean : Heu_fun {

  Heu_euclidean(std::shared_ptr<dynobench::Model_robot> robot,
                const Eigen::VectorXd &goal)
      : robot(robot), goal(goal) {}

  std::shared_ptr<dynobench::Model_robot> robot;
  Eigen::VectorXd goal;

  virtual double h(const Eigen::VectorXd &x) override {
    assert(x.size() == robot->nx);
    return robot->lower_bound_time(x, goal);
  }

  virtual ~Heu_euclidean() override{};
};

struct Heu_blind : Heu_fun {

  Heu_blind() {}

  virtual double h(const Eigen::VectorXd &x) override {
    (void)x;
    return 0;
  }

  virtual ~Heu_blind() override{};
};

struct Heu_roadmap : Heu_fun {

  std::shared_ptr<dynobench::Model_robot> robot;
  std::shared_ptr<ompl::NearestNeighbors<Heuristic_node *>> T_heu;
  double connect_radius_h = 0.5;
  Eigen::VectorXd goal;
  Eigen::VectorXd __x_zero_vel;
  std::vector<Heuristic_node> heu_map;

  Heu_roadmap(std::shared_ptr<dynobench::Model_robot> robot,
              const std::vector<Heuristic_node> &heu_map,
              const Eigen::VectorXd &goal, const std::string &robot_type = "");

  virtual double h(const Eigen::VectorXd &x) override {
    assert(T_heu);
    assert(x.size() == robot->nx);
    __x_zero_vel = x;
    robot->set_0_velocity(__x_zero_vel);

    // auto si = robot->getSpaceInformation()->getStateSpace();
    // const auto &locations = si->getValueLocations();
    //
    // DYNO_CHECK_EQ(locations.size(), robot->nx, AT);
    // DYNO_CHECK_LEQ(robot->nx_pr, locations.size(), AT);
    //
    // for (std::size_t i = 0; i < robot->nx_pr; ++i) {
    //   *si->getValueAddressAtLocation(__x_zero_vel, locations[i]) =
    //       *si->getValueAddressAtLocation(x, locations[i]);
    // }

    // NOT_IMPLEMENTED;
    // refactor this!!
    double pos_h = heuristicCollisionsTree(T_heu.get(), __x_zero_vel, robot,
                                           connect_radius_h);

    double vel_h = robot->lower_bound_time_vel(x, goal);

    return std::max(vel_h, pos_h);
  }

  virtual ~Heu_roadmap() override{

      // robot->getSpaceInformation()->freeState(__x_zero_vel);

      // TODO: memory leak
      // I have to delete more stuff!!!
  };
};

template<typename _T, typename _Node>
struct Heu_roadmap_bwd : Heu_fun {

  Heu_roadmap_bwd(std::shared_ptr<dynobench::Model_robot> robot,
                  const _T heuristic_nn,
                  const Eigen::VectorXd &goal)
      : robot(robot), heuristic_nn(heuristic_nn), goal(goal) {}

  std::shared_ptr<dynobench::Model_robot> robot;
  const _T heuristic_nn;
  Eigen::VectorXd goal;

  virtual double h(const Eigen::VectorXd &x) override {
    assert(x.size() == robot->nx);
    if (heuristic_nn){
      _Node fake_node;
      fake_node.state_eig = x;
      // std::cout << "nearest gScore: " << heuristic_nn->nearest(&fake_node)->gScore << std::endl;
      // std::cout << "nearest hScore: " << heuristic_nn->nearest(&fake_node)->hScore << std::endl;
      return heuristic_nn->nearest(&fake_node)->gScore;
    }
    else {
      return robot->lower_bound_time(x, goal);
    }
  }

  virtual ~Heu_roadmap_bwd() override{};
};

void build_heuristic_distance_new(
    const std::vector<Eigen::VectorXd> &batch_samples,
    std::shared_ptr<dynobench::Model_robot> &robot,
    std::vector<Heuristic_node> &heuristic_map, double distance_threshold,
    double resolution);

void generate_heuristic_map(const dynobench::Problem &problem,
                            std::shared_ptr<dynobench::Model_robot> robot,
                            const Options_dbastar &options_dbastar,
                            std::vector<Heuristic_node> &heu_map);

} // namespace dynoplan
