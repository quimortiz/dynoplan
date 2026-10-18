#pragma once
#include <boost/graph/graphviz.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include "dynobench/general_utils.hpp"
#include "dynoplan/dbastar/options.hpp"

namespace dynoplan
{

  namespace ob = ompl::base;
  using Sample_ = ob::State;

  struct SampleNode
  {
    Sample_ *x;
    double dist;
    int parent;
  };

  struct HeuNodeWithIndex
  {

    int index;
    const ob::State *state;
    double dist;
    const ob::State *getState() const { return state; }
  };

  struct Heuristic_node
  {
    Eigen::VectorXd x;
    double d; // distance
    int p;    // parent
    const Eigen::VectorXd &getStateEig() const { return x; }
    const double get_cost() const { ERROR_WITH_INFO("should not be here!"); }
  };

  void load_heu_map(const char *file, std::vector<Heuristic_node> &heu_map);

  struct HeuNode
  {
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

  struct Heu_fun
  {
    virtual double h(const Eigen::VectorXd &x) = 0;
    virtual ~Heu_fun() = default;
  };

  struct Heu_euclidean : Heu_fun
  {

    Heu_euclidean(std::shared_ptr<dynobench::Model_robot> robot,
                  const Eigen::VectorXd &goal)
        : robot(robot), goal(goal) {}

    std::shared_ptr<dynobench::Model_robot> robot;
    Eigen::VectorXd goal;

    virtual double h(const Eigen::VectorXd &x) override
    {
      assert(x.size() == robot->nx);
      return robot->lower_bound_time(x, goal);
    }

    virtual ~Heu_euclidean() override {};
  };

  struct Heu_blind : Heu_fun
  {

    Heu_blind() {}

    virtual double h(const Eigen::VectorXd &x) override
    {
      (void)x;
      return 0;
    }

    virtual ~Heu_blind() override {};
  };

  struct Heu_roadmap : Heu_fun
  {

    std::shared_ptr<dynobench::Model_robot> robot;
    std::shared_ptr<ompl::NearestNeighbors<Heuristic_node *>> T_heu;
    double connect_radius_h = 0.5;
    Eigen::VectorXd goal;
    Eigen::VectorXd __x_zero_vel;
    std::vector<Heuristic_node> heu_map;

    Heu_roadmap(std::shared_ptr<dynobench::Model_robot> robot,
                const std::vector<Heuristic_node> &heu_map,
                const Eigen::VectorXd &goal, const std::string &robot_type = "");

    virtual double h(const Eigen::VectorXd &x) override
    {
      assert(T_heu);
      assert(x.size() == robot->nx);
      __x_zero_vel = x;
      robot->set_0_velocity(__x_zero_vel);

      double pos_h = heuristicCollisionsTree(T_heu.get(), __x_zero_vel, robot,
                                             connect_radius_h);

      double vel_h = robot->lower_bound_time_vel(x, goal);

      return std::max(vel_h, pos_h);
    }

    virtual ~Heu_roadmap() override {
    };
  };

  template <typename _T, typename _Node>
  struct Heu_roadmap_bwd : Heu_fun
  {

    Heu_roadmap_bwd(std::shared_ptr<dynobench::Model_robot> robot,
                    ompl::NearestNeighbors<_T> *heuristic_nn,
                    const Eigen::VectorXd &goal)
        : robot(robot), heuristic_nn(heuristic_nn), goal(goal) {}

    std::shared_ptr<dynobench::Model_robot> robot;
    ompl::NearestNeighbors<_T> *heuristic_nn;
    Eigen::VectorXd goal;

    virtual double h(const Eigen::VectorXd &x) override
    {
      assert(x.size() == robot->nx);
      if (heuristic_nn)
      {
        auto fake_node = std::make_shared<_Node>();
        fake_node->state_eig = x;
        auto nearest_node = heuristic_nn->nearest(fake_node);
        return nearest_node->gScore;
      }
      else
      {
        return robot->lower_bound_time(x, goal);
      }
    }

    virtual ~Heu_roadmap_bwd() override {};
  };
  // more precise for pibt
  template <typename _T, typename _Node>
  struct Heu_roadmap_bwd2 : Heu_fun
  {

    Heu_roadmap_bwd2(std::shared_ptr<dynobench::Model_robot> robot,
                     ompl::NearestNeighbors<_T> *heuristic_nn,
                     const Eigen::VectorXd &goal, bool nn)
        : robot(robot), heuristic_nn(heuristic_nn), goal(goal), nn(nn) {}

    std::shared_ptr<dynobench::Model_robot> robot;
    ompl::NearestNeighbors<_T> *heuristic_nn;
    Eigen::VectorXd goal;
    bool nn;

    virtual double h(const Eigen::VectorXd &x) override
    {
      assert(x.size() == robot->nx);
      if (heuristic_nn && !nn)
      {
        auto fake_node = std::make_shared<_Node>();
        fake_node->state_eig = x;
        auto nearest_node = heuristic_nn->nearest(fake_node);
        return (nearest_node->gScore + robot->lower_bound_time(fake_node->state_eig, nearest_node->state_eig));
      }
      if (heuristic_nn && nn)
      {
        auto fake_node = std::make_shared<_Node>();
        fake_node->state_eig = x;

        const int K = 4; // forest, wall
        std::vector<std::shared_ptr<_Node>> neighbors;
        heuristic_nn->nearestK(fake_node, K, neighbors);

        if (neighbors.empty())
        {
          // fallback to direct heuristic
          return robot->lower_bound_time(x, goal);
        }

        double weighted_sum = 0.0;
        double weight_total = 0.0;

        for (auto &n : neighbors)
        {
          double dist = robot->lower_bound_time(n->state_eig, x);
          double w = 1.0 / (dist + 1e-6); // inverse distance weighting
          double h_val = n->gScore;
          // robot->lower_bound_time(x, n->state_eig);
          weighted_sum += w * h_val;
          weight_total += w;
        }
        return weighted_sum / weight_total; // weighted average
      }
      else
      {
        return robot->lower_bound_time(x, goal);
      }
    }

    virtual ~Heu_roadmap_bwd2() override {};
  };
  // for EST
  template <typename _T, typename _Node>
  class HeuRoadmapBwdNearestR : Heu_fun
  {
  public:
    HeuRoadmapBwdNearestR(std::shared_ptr<dynobench::Model_robot> robot,
                          ompl::NearestNeighbors<_T> *heuristic_nn,
                          const Eigen::VectorXd &goal,
                          bool nn,
                          double distance = 1.0) // distance for neighbor search
        : robot(robot), heuristic_nn(heuristic_nn), goal(goal), nn(nn), search_distance(distance)
    {
    }

    std::shared_ptr<dynobench::Model_robot> robot;
    ompl::NearestNeighbors<_T> *heuristic_nn; // raw pointe
    Eigen::VectorXd goal;
    bool nn;
    double search_distance; // distance for nearest neighbors

    void addNode(const _T &node)
    {
      if (heuristic_nn)
        heuristic_nn->add(node);
    }

    virtual double h(const Eigen::VectorXd &x)
    {
      assert(x.size() == robot->nx);
      if (heuristic_nn->size() == 0)
        return -1.0;
      auto fake_node = std::make_shared<_Node>();
      fake_node->state_eig = x;
      auto nearest_node = heuristic_nn->nearest(fake_node);
      // std::cout << "from N: " << (nearest_node->hScore + robot->lower_bound_time(fake_node->state_eig, nearest_node->state_eig)) << std::endl;
      if (!nearest_node || robot->distance(fake_node->state_eig, nearest_node->state_eig) > search_distance /*threshold*/) // 1.0 - unicycle, 0.5 - single integrator
        return -1.0;

      return (nearest_node->hScore + robot->lower_bound_time(fake_node->state_eig, nearest_node->state_eig));
    }

    virtual ~HeuRoadmapBwdNearestR() override = default;
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
