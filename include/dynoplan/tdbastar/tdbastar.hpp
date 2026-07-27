#pragma once
#include <algorithm>
#include "Eigen/Core"
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
// OMPL
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"
#include <fcl/fcl.h>

#include "dynobench/planar_rotor.hpp"
#include "dynobench/quadrotor.hpp"
#include "dynoplan/dbastar/heuristics.hpp"
#include "dynoplan/tdbastar/options.hpp"

namespace dynoplan
{

  namespace ob = ompl::base;

  using Sample = std::vector<double>;
  using Sample_ = ob::State;

  struct AStarNode;
  struct compareAStarNode
  {
    bool operator()(const std::shared_ptr<AStarNode> a,
                    const std::shared_ptr<AStarNode> b) const;
  };

  typedef typename boost::heap::d_ary_heap<
      std::shared_ptr<AStarNode>, boost::heap::arity<2>,
      boost::heap::compare<compareAStarNode>, boost::heap::mutable_<true>>
      open_t;
  // Node type (used for open and explored states)
  struct AStarNode
  {
    const ob::State *state;
    Eigen::VectorXd state_eig;

    double fScore;
    double gScore;
    double hScore;
    std::shared_ptr<AStarNode> came_from;
    double out_degree;          // est needs it
    std::vector<int> motions{}; // list of applicable motions
    // std::vector<int> conflicts{};

    double get_cost() const { return gScore; }

    fcl::Vector3d used_offset;
    // size_t used_motion;
    int intermediate_state =
        -1; // checking intermediate states for reaching the goal.

    open_t::handle_type handle;
    bool is_in_open = false;
    bool valid = true;
    bool reaches_goal;
    // can arrive at this node at time gScore, starting from came_from, using
    // motion used_motion
    struct arrival
    {
      double gScore;
      double fScore;
      int focalHeuristic;
      // AStarNode* came_from;
      std::shared_ptr<AStarNode> came_from;
      size_t used_motion;
      size_t arrival_idx;
    };
    std::vector<arrival> arrivals;
    size_t current_arrival_idx;

    // These two values are updated when we reconstruct the FOCAL list from OPEN
    int bestFocalHeuristic; // ecbs
    size_t best_focal_arrival_idx;

    const ob::State *getState() { return state; }
    const Eigen::VectorXd &getStateEig() { return state_eig; }

    void write(std::ostream &out)
    {
      out << state_eig.format(dynobench::FMT) << std::endl;
      out << "fScore: " << fScore << " gScore: " << gScore
          << " hScore: " << hScore << std::endl;
      // out << " used_motion: " << used_motion
      out << " intermediate_state: " << intermediate_state
          << " is_in_open: " << is_in_open << " valid: " << valid << std::endl;
    }
  };

  using Edge = std::pair<int, int>;

  enum class Duplicate_detection
  {
    NO = 0,
    HARD = 1,
    SOFT = 2,
  };

  enum class Terminate_status
  {
    SOLVED = 0,
    MAX_EXPANDS = 1,
    EMPTY_QUEUE = 2,
    MAX_TIME = 3,
    UNKNOWN = 4,
    SOLVED_RAW = 5,
    SOLVED_OPT = 6,
  };

  static const char *terminate_status_str[] = {
      "SOLVED",
      "MAX_EXPANDS",
      "EMPTY_QUEUE",
      "MAX_TIME",
      "UNKNOWN",
      "SOLVED_RAW",
      "SOLVED_OPT",

  };

  struct Out_info_tdb
  {

    double cost = -1;
    bool solved = 0;
    double cost_with_delta_time = -1;
    // void print(std::ostream &out);
    double time_search = -1;
    std::map<std::string, std::string> data;

    void write_yaml(std::ostream &out)
    {

      out << STR_(cost) << std::endl;
      out << STR_(solved) << std::endl;
      out << STR_(cost_with_delta_time) << std::endl;
      out << STR_(time_search) << std::endl;
      out << "data:" << std::endl;
      for (auto &[k, v] : data)
      {
        out << "  " << k << ": " << v << std::endl;
      }
    }
    void reset()
    {
      cost = -1;
      solved = 0;
      cost_with_delta_time = -1;
      time_search = -1;
      data.clear();
    }
  };

  double automatic_delta(double delta_in, double alpha, RobotOmpl &robot,
                         ompl::NearestNeighbors<Motion *> &T_m);

  struct Constraint
  {
    double time;
    Eigen::VectorXd constrained_state;
  };

  void tdbastar(
      dynobench::Problem &problem, Options_tdbastar options_dbastar,
      dynobench::Trajectory &traj_out, const std::vector<Constraint> &constraints,
      Out_info_tdb &out_info_tdb, size_t &robot_id, bool reverse_search,
      ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *heuristic_nn = nullptr,
      ompl::NearestNeighbors<std::shared_ptr<AStarNode>> **heuristic_result =
          nullptr);

  struct LazyTraj
  {

    Eigen::VectorXd *offset;
    dynobench::Model_robot *robot;
    Motion *motion;

    void compute(
        dynobench::TrajWrapper &tmp, bool forward = true,
        std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state = nullptr,
        int *num_valid_states = nullptr)
    {
      assert(offset);
      assert(robot);
      assert(motion);
      assert(offset->size());
      if (forward)
      {
        robot->transform_primitive(*offset, motion->traj.states,
                                   motion->traj.actions, tmp, check_state,
                                   num_valid_states);
      }
      // reverse
      else
      {
        robot->transform_primitive(*offset - motion->traj.states.back().head(
                                                 robot->translation_invariance),
                                   motion->traj.states, motion->traj.actions, tmp,
                                   check_state, num_valid_states);
      }
    }

    void compute(
        dynobench::Trajectory &tmp, bool forward = true,
        std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state = nullptr,
        int *num_valid_states = nullptr)
    {
      assert(offset);
      assert(robot);
      assert(motion);

      assert(offset->size());

      dynobench::TrajWrapper __tmp = dynobench::Trajectory_2_trajWrapper(tmp);
      compute(__tmp, forward, check_state, num_valid_states);
      tmp = dynobench::trajWrapper_2_Trajectory(__tmp);
    }
  };

  struct Expander
  {

    dynobench::Model_robot *robot;
    ompl::NearestNeighbors<Motion *> *T_m;
    Eigen::VectorXd canonical_state;
    Eigen::VectorXd offset;
    Motion fakeMotion;
    std::vector<Motion *> neighbors_m;
    double delta = -1;
    bool random = false; // determenistic beh. DEBUG
    std::mt19937 g;
    size_t max_k = std::numeric_limits<size_t>::max();
    double time_in_nn = 0;
    bool verbose = false;
    bool add_static_motions = false;
    // bool reverse_search = false;
    Expander(dynobench::Model_robot *robot, ompl::NearestNeighbors<Motion *> *T_m,
             double delta, bool add_static_motions)
        : robot(robot), T_m(T_m), delta(delta), add_static_motions(add_static_motions)
    {
      canonical_state.resize(robot->nx);
      offset.resize(robot->get_offset_dim());
      fakeMotion.idx = -1;
      fakeMotion.traj.states.push_back(Eigen::VectorXd(robot->nx));
      std::random_device rd;
      g = std::mt19937{rd()};
    }

    void seed(int seed) { g.seed(seed); }
    double wrap_angle(double angle)
    {
      while (angle >= M_PI)
        angle -= 2 * M_PI;
      while (angle < -M_PI)
        angle += 2 * M_PI;
      return angle;
    }
    void expand_lazy(Eigen::Ref<const Eigen::VectorXd> x,
                     std::vector<LazyTraj> &lazy_trajs)
    {

      robot->canonical_state(x, canonical_state); // same as state size
      robot->offset(x, offset);                   // only position part
      fakeMotion.traj.states.at(0) = canonical_state;
      assert(delta > 0);

      Stopwatch sw;
      // CSTR_(x);

      T_m->nearestR(&fakeMotion, delta, neighbors_m);
      time_in_nn += sw.elapsed_ms();

      if (!neighbors_m.size() && verbose)
      {

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
      for (size_t i = 0; i < std::min(neighbors_m.size(), max_k); i++)
      {
        auto &m = neighbors_m.at(i);
        LazyTraj lazy_traj{.offset = &offset, .robot = robot, .motion = m};
        lazy_trajs.push_back(lazy_traj);
      }
      if (add_static_motions)
      {
        int num_steps = 12; // benchmarking
        Eigen::VectorXd zero_action(robot->nu);
        zero_action.setZero();
        if (robot->name == "Integrator2_3d")
          num_steps = 6; // for short motions
        Eigen::VectorXd fixed_state(robot->nx);
        fixed_state.setZero();
        if (robot->name == "unicycle1")
        {
          std::random_device rd;
          std::mt19937 gen(rd()); // gen(42);
          std::uniform_real_distribution<> dis(-M_PI, M_PI);
          // better sample, otherwise sensitive to goal theta value
          for (size_t i = 0; i < 1; i++)
          {
            double theta = dis(gen);
            theta = wrap_angle(theta);
            // std::cout << "Adding sampled theta: " << theta << "(" << theta * 180.0 / M_PI << " deg.)" << std::endl;
            Motion *m = new Motion();
            fixed_state(2) = theta; // deterministically set orientation
            m->traj.states.resize(num_steps, fixed_state);
            m->traj.actions.resize(num_steps - 1, zero_action);
            LazyTraj tmp_lazy_traj{.offset = &offset, .robot = robot, .motion = m};
            lazy_trajs.push_back(tmp_lazy_traj);
          }
        }
        if (robot->name == "Integrator2_3d")
        {
          std::random_device rd;
          std::mt19937 gen(rd());                          // gen(42);
          std::uniform_real_distribution<> dis(-0.5, 0.5); // velocity limit
          // better sample, otherwise sensitive to goal theta value
          for (size_t i = 0; i < 4; i++)
          {
            Motion *m = new Motion();
            // sample velocity components
            fixed_state(3) = dis(gen);
            fixed_state(4) = dis(gen);
            fixed_state(5) = dis(gen);
            m->traj.states.resize(num_steps, fixed_state);
            m->traj.actions.resize(num_steps - 1, zero_action);
            LazyTraj tmp_lazy_traj{.offset = &offset, .robot = robot, .motion = m};
            lazy_trajs.push_back(tmp_lazy_traj);
          }
        }
        // single motion primitive
        else
        {
          Motion *m = new Motion();
          m->traj.states.resize(num_steps, fixed_state);
          m->traj.actions.resize(num_steps - 1, zero_action);
          LazyTraj tmp_lazy_traj{.offset = &offset, .robot = robot, .motion = m};
          lazy_trajs.push_back(tmp_lazy_traj);
        }
      }
    }
  };

  //
  // @param:nodes
  //
  void plot_search_tree(std::vector<AStarNode *> nodes,
                        std::vector<Motion> &motions,
                        dynobench::Model_robot &robot, const char *filename);

  void from_solution_to_yaml_and_traj(dynobench::Model_robot &robot,
                                      const std::vector<Motion> &motions,
                                      // AStarNode *solution,
                                      std::shared_ptr<AStarNode> solution,
                                      const dynobench::Problem &problem,
                                      dynobench::Trajectory &traj_out,
                                      std::ofstream *out = nullptr);

  void check_goal(dynobench::Model_robot &robot, Eigen::Ref<Eigen::VectorXd> x,
                  const Eigen::Ref<const Eigen::VectorXd> &goal,
                  dynobench::TrajWrapper &traj_wrapper, double distance_bound,
                  size_t num_check_goal, int &chosen_index, bool forward);

  bool check_lazy_trajectory(
      LazyTraj &lazy_traj, dynobench::Model_robot &robot,
      const Eigen::Ref<const Eigen::VectorXd> &goal, Time_benchmark &time_bench,
      dynobench::TrajWrapper &tmp_traj,
      const std::vector<Constraint> &constraints, const float best_node_gscore,
      float delta, Eigen::Ref<Eigen::VectorXd> aux_last_state,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state = nullptr,
      int *num_valid_states = nullptr, bool forward = true);

  void disable_motions(std::shared_ptr<dynobench::Model_robot> &robot,
                       std::string &robot_name, float delta,
                       bool filterDuplicates, float alpha, size_t num_max_motions,
                       std::vector<Motion> &motions);

  void filter_motions(std::shared_ptr<dynobench::Model_robot> &robot,
                     std::string &robot_name,
                     float delta,
                     bool filterDuplicates,
                     float alpha,
                     size_t num_max_motions,
                     bool filterVelocityMagnitude,
                     std::vector<Motion> &motions);

} // namespace dynoplan
