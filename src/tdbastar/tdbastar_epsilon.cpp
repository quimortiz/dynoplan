#include <boost/graph/graphviz.hpp>

#include <map>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
// #include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>

// OMPL headers
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

// boost stuff for the graph
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/property_map/property_map.hpp>
// fcl
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>

#include "dynobench/general_utils.hpp"
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/tdbastar_epsilon.hpp"

#define REBUILT_FOCAL_LIST

namespace dynoplan {

using dynobench::Trajectory;

using dynobench::FMT;

namespace ob = ompl::base;

using Sample = std::vector<double>;
using Sample_ = ob::State;

// nigh interface for OMPL

bool compareFocalHeuristic::operator()(const open_t::handle_type &h1,
                                       const open_t::handle_type &h2) const {
  if ((*h1)->focalHeuristic != (*h2)->focalHeuristic) {
    return (*h1)->focalHeuristic > (*h2)->focalHeuristic;
  }
  return (*h1)->gScore > (*h2)->gScore; // cost
}

// focal heuristic based on shape
// this function counts the number of conflicts between the motion (being considered as applicable for expansion) with the 
// solution of other neighbor robots. Here each neighbors solution is given in &results vector, and robot_motions are used
// to retrieve the exact motion primitive from &results vector.
int lowLevelfocalHeuristicShape(std::vector<std::vector<std::pair<std::shared_ptr<AStarNode>, size_t>>> &results,
                                std::map<std::string, std::vector<Motion>> &robot_motions,
                                const dynobench::Problem &problem,
                                LazyTraj &current_lazy_traj, size_t &current_robot_idx,
                                const float current_gscore,
                                const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots) {
    int numConflicts = 0;
    int time_index = 0;
    Eigen::VectorXd node_state;
    // motion primitve being considered as applicable for expansion
    auto &current_motion = current_lazy_traj.motion;
    Eigen::VectorXd offset = *current_lazy_traj.offset;
    assert(offset.size() == 2 || offset.size() == 3);
    Eigen::Vector3d __offset;
    if (offset.size() == 2) {
      __offset = Eigen::Vector3d(offset(0), offset(1), 0);
    } else {
      __offset = offset.head<3>();
    }
    assert(current_motion);
    assert(current_motion->collision_manager);
    current_motion->collision_manager->shift(__offset);
    size_t idx = 0;
    size_t motion_idx = 0;
    size_t max_size = 0;
    Eigen::Vector3d __offset_tmp;
    // check the motion primitive for collision with each robots final solution
    for (auto &r : results) { // for each neighbor
      Eigen::VectorXd offset_tmp(all_robots[idx]->get_offset_dim()); 
      max_size = r.size() - 1;
      if (idx != current_robot_idx && !r.empty()){
        fcl::DefaultCollisionData<double> collision_data;
        // get the time index for other robots solution trajectory
        time_index = std::lround(current_gscore / all_robots[idx]->ref_dt);
        if (time_index >= int(max_size - 1)){
          time_index = max_size - 1;
        }
        node_state = r[time_index].first->state_eig;
        motion_idx = r[time_index + 1].first->arrivals[r[time_index + 1].second].used_motion;
        // get the motion primitive corresponding to the time_index from results[current_robot]
        auto &motion_to_check = robot_motions[problem.robotTypes[idx]].at(motion_idx);
        all_robots[idx]->offset(node_state, offset_tmp);
        if (offset_tmp.size() == 2) {
          __offset_tmp = Eigen::Vector3d(offset_tmp(0), offset_tmp(1), 0);
        } else {
          __offset_tmp = offset_tmp.head<3>();
        }
        motion_to_check.collision_manager->shift(__offset_tmp);
        // check for collision two motion primitives
        current_motion->collision_manager->collide(motion_to_check.collision_manager.get(), &collision_data,
                                         fcl::DefaultCollisionFunction<double>);
        motion_to_check.collision_manager->shift(-__offset_tmp);
        if(collision_data.result.isCollision()){
          numConflicts++;
        }
      }
    }
    current_motion->collision_manager->shift(-__offset);
    return numConflicts;
  }

// focal heuristic based on state
int highLevelfocalHeuristic(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs) {
  size_t max_t = 0;
  int numConflicts = 0;
  for (const auto &sol : solution) {
    max_t = std::max(max_t, sol.trajectory.states.size() - 1);
  }
  Eigen::VectorXd node_state;
  for (size_t t = 0; t <= max_t; ++t) {
    size_t robot_idx = 0;
    size_t obj_idx = 0;
    std::vector<fcl::Transform3d> ts_data;
    for (auto &robot : all_robots) {
      if (t >= solution[robot_idx].trajectory.states.size()) {
        node_state = solution[robot_idx].trajectory.states.back();
      } else {
        node_state = solution[robot_idx].trajectory.states[t];
      }
      std::vector<fcl::Transform3d> tmp_ts(1);
      if (robot->name == "car_with_trailers") {
        tmp_ts.resize(2);
      }
      robot->transformation_collision_geometries(node_state, tmp_ts);
      ts_data.insert(ts_data.end(), tmp_ts.begin(), tmp_ts.end());
      ++robot_idx;
    }
    for (size_t i = 0; i < ts_data.size(); i++) {
      fcl::Transform3d &transform = ts_data[i];
      robot_objs[obj_idx]->setTranslation(transform.translation());
      robot_objs[obj_idx]->setRotation(transform.rotation());
      robot_objs[obj_idx]->computeAABB();
      ++obj_idx;
    }
    col_mng_robots->update(robot_objs);
    fcl::DefaultCollisionData<double> collision_data;
    col_mng_robots->collide(&collision_data,
                            fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision()) {
      numConflicts++;
    }
  }
  return numConflicts;
}
// state heuristic
int lowLevelfocalHeuristic(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::shared_ptr<AStarNode> current_node, size_t &current_robot_idx,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs) {
  size_t max_t = 0;
  int numConflicts = 0;
  // for the state we are checking
  std::vector<fcl::Transform3d> current_tmp_ts(1);
  if (all_robots[current_robot_idx]->name == "car_with_trailers") {
    current_tmp_ts.resize(2);
  }
  all_robots[current_robot_idx]->transformation_collision_geometries(
      current_node->state_eig, current_tmp_ts);

  for (size_t i = 0; i < solution.size(); ++i) {
    if (i != current_robot_idx && !solution[i].trajectory.states.empty()) {
      max_t = std::max(max_t, solution[i].trajectory.states.size() - 1);
    }
  }
  Eigen::VectorXd node_state;
  for (size_t t = 0; t < max_t; ++t) {
    size_t robot_idx = 0;
    size_t obj_idx = 0;
    std::vector<fcl::Transform3d> ts_data;
    for (auto &robot : all_robots) {
      if (robot_idx == current_robot_idx) {
        ++robot_idx;
        continue;
      }
      if (t >= solution[robot_idx].trajectory.states.size()) {
        node_state = solution[robot_idx].trajectory.states.back();
      } else {
        node_state = solution[robot_idx].trajectory.states[t];
      }
      std::vector<fcl::Transform3d> tmp_ts(1);
      if (robot->name == "car_with_trailers") {
        tmp_ts.resize(2);
      }
      robot->transformation_collision_geometries(node_state, tmp_ts);
      ts_data.insert(ts_data.end(), tmp_ts.begin(), tmp_ts.end());
      ++robot_idx;
    }
    // add the state we are checking
    ts_data.insert(ts_data.end(), current_tmp_ts.begin(), current_tmp_ts.end());
    for (size_t i = 0; i < ts_data.size(); i++) {
      fcl::Transform3d &transform = ts_data[i];
      robot_objs[obj_idx]->setTranslation(transform.translation());
      robot_objs[obj_idx]->setRotation(transform.rotation());
      robot_objs[obj_idx]->computeAABB();
      ++obj_idx;
    }
    col_mng_robots->update(robot_objs);
    fcl::DefaultCollisionData<double> collision_data;
    col_mng_robots->collide(&collision_data,
                            fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision()) {
      numConflicts++;
    }
  }
  return numConflicts;
}

void tdbastar_epsilon(
    dynobench::Problem &problem, Options_tdbastar options_tdbastar,
    Trajectory &traj_out, const std::vector<Constraint> &constraints,
    Out_info_tdb &out_info_tdb, size_t &robot_id, bool reverse_search,
    std::vector<dynobench::Trajectory> &expanded_trajs,
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    std::vector<std::vector<std::pair<std::shared_ptr<AStarNode>, size_t>>>
        &results,
    std::map<std::string, std::vector<Motion>> &robot_motions,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs,
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *heuristic_nn,
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>> **heuristic_result,
    float w, std::string focal_heuristic_name) {

  // #ifdef DBG_PRINTS
  std::cout << "Running tdbA* for robot " << robot_id << std::endl;
  for (const auto &constraint : constraints) {
    std::cout << "constraint at time: " << constraint.time << " ";
    std::cout << constraint.constrained_state.format(dynobench::FMT)
              << std::endl;
  }
  // #endif
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotTypes[robot_id] + ".yaml")
          .c_str(),
      problem.p_lb, problem.p_ub);
  load_env(*robot, problem);
  const int nx = robot->nx;
  // clean
  traj_out.states.clear();
  traj_out.actions.clear();
  traj_out.cost = 0;
  traj_out.fmin = 0;
  CHECK(options_tdbastar.motions_ptr,
        "motions should be loaded before calling dbastar");
  std::vector<Motion> &motions = *options_tdbastar.motions_ptr;

  auto check_motions = [&] {
    for (size_t idx = 0; idx < motions.size(); ++idx) {
      if (motions[idx].idx != idx) {
        return false;
      }
    }
    return true;
  };

  assert(check_motions());

  Time_benchmark time_bench;
  // build kd-tree for motion primitives
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *T_n = nullptr;

  if (options_tdbastar.use_nigh_nn) {
    T_n = nigh_factory2<std::shared_ptr<AStarNode>>(
        problem.robotTypes[robot_id], robot);
  } else {
    NOT_IMPLEMENTED;
  }
  // // for the initial heuristics
  if (heuristic_result) {
    *heuristic_result = T_n;
  }
  if (options_tdbastar.use_nigh_nn) {
    T_m = nigh_factory_t<Motion *>(problem.robotTypes[robot_id], robot,
                                   reverse_search);
  } else {
    NOT_IMPLEMENTED;
  }

  time_bench.time_nearestMotion += timed_fun_void([&] {
    for (size_t i = 0;
         i < std::min(motions.size(), options_tdbastar.max_motions); ++i) {
      T_m->add(&motions.at(i));
    }
  });

  Expander expander(robot.get(), T_m,
                    options_tdbastar.alpha * options_tdbastar.delta);
  if (options_tdbastar.alpha <= 0 || options_tdbastar.alpha >= 1) {
    ERROR_WITH_INFO("Alpha needs to be between 0 and 1!");
  }

  if (options_tdbastar.delta < 0) {
    NOT_IMPLEMENTED;
  }

  std::shared_ptr<Heu_fun> h_fun = nullptr;
  std::vector<Heuristic_node> heu_map;

  if (reverse_search) {
    h_fun = std::make_shared<Heu_blind>();
  } else {
    h_fun = std::make_shared<
        Heu_roadmap_bwd<std::shared_ptr<AStarNode>, AStarNode>>(
        robot, heuristic_nn, problem.goals[robot_id]);
  }
  std::vector<std::shared_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_shared<AStarNode>());

  auto start_node = all_nodes.at(0);
  start_node->gScore = 0;
  start_node->state_eig = problem.starts[robot_id];
  start_node->hScore = h_fun->h(problem.starts[robot_id]);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->is_in_open = true;
  start_node->reaches_goal =
      (robot->distance(problem.starts[robot_id], problem.goals[robot_id]) <=
       options_tdbastar.delta);
  start_node->arrivals.push_back({.gScore = 0,
                                  .came_from = nullptr,
                                  .used_motion = (size_t)-1,
                                  .arrival_idx = (size_t)-1});
  start_node->current_arrival_idx = 0;
  DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
  DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");

  auto goal_node = std::make_shared<AStarNode>();
  goal_node->state_eig = problem.goals[robot_id];
  open_t open;
  focal_t focal; // subset of open nodes that are within suboptimality bound
  start_node->handle = open.push(start_node);
  focal.push(start_node->handle);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.traj.states.push_back(Eigen::VectorXd::Zero(robot->nx));

  auto tmp_node = std::make_shared<AStarNode>();
  tmp_node->state_eig = Eigen::VectorXd::Zero(robot->nx);

  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goals[robot_id]);

  std::mt19937 g = std::mt19937{std::random_device()()};

  double cost_bound = options_tdbastar.maxCost;

  if (options_tdbastar.fix_seed) {
    expander.seed(0);
    g = std::mt19937{0};
    srand(0);
  } else {
    srand(time(0));
  }

  time_bench.time_nearestNode_add +=
      timed_fun_void([&] { T_n->add(start_node); });
  const size_t print_every = 1000;

  double last_f_score = start_node->fScore;
  auto print_search_status = [&] {
    std::cout << "expands: " << time_bench.expands << " open: " << open.size()
              << " best distance: " << best_distance_to_goal
              << " fscore: " << last_f_score << std::endl;
  };

  Stopwatch watch;
  Terminate_status status = Terminate_status::UNKNOWN;

  auto stop_search = [&] {
    if (static_cast<size_t>(time_bench.expands) >=
        options_tdbastar.max_expands) {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }

    if (watch.elapsed_ms() > options_tdbastar.search_timelimit) {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      return true;
    }

    if (open.empty()) {
      status = Terminate_status::EMPTY_QUEUE;
      std::cout << "BREAK search:"
                << "EMPTY_QUEUE OPEN" << std::endl;
      return true;
    }

    return false;
  };

  std::shared_ptr<AStarNode> best_node;
  auto best_fScore = start_node->fScore;

  std::vector<std::shared_ptr<AStarNode>> neighbors_n;

  const bool debug = false;

  const bool check_intermediate_goal = true;
  const size_t num_check_goal = 0;

  std::function<bool(Eigen::Ref<Eigen::VectorXd>)> ff =
      [&](Eigen::Ref<Eigen::VectorXd> state) {
        return robot->is_state_valid(state);
      };

  // we allocate a trajectory for the largest motion primitive

  dynobench::TrajWrapper traj_wrapper;
  {
    std::vector<Motion *> motions;
    T_m->list(motions);
    size_t max_traj_size = (*std::max_element(motions.begin(), motions.end(),
                                              [](Motion *a, Motion *b) {
                                                return a->traj.states.size() <
                                                       b->traj.states.size();
                                              }))
                               ->traj.states.size();

    traj_wrapper.allocate_size(max_traj_size, robot->nx, robot->nu);
  }

  Eigen::VectorXd aux_last_state(robot->nx);
  int focalHeuristic = 0;
  while (!stop_search()) {
#ifdef REBUILT_FOCAL_LIST
    focal.clear();
    double best_cost = open.top()->fScore;
    auto iter = open.ordered_begin();
    auto iterEnd = open.ordered_end();
    for (; iter != iterEnd; ++iter) {
      auto cost = (*iter)->fScore;
      if (cost <= best_cost * w) {
        std::shared_ptr<AStarNode> n = *iter;
        focal.push(n->handle);
      } else {
        break;
      }
    }
#else
    {
      auto oldbest_fScore = best_fScore;
      best_fScore = open.top()->fScore;
      if (best_fScore > oldbest_fScore) {
        auto iter = open.ordered_begin();
        auto iterEnd = open.ordered_end();
        for (; iter != iterEnd; ++iter) {
          auto cost = (*iter)->fScore;
          if (cost > oldbest_fScore * w && cost <= best_fScore * w) {
            std::shared_ptr<AStarNode> n = *iter;
            focal.push(n->handle);
          }
          if (cost > best_fScore * w) {
            break;
          }
        }
      }
    }
#endif
    auto best_handle = focal.top();
    best_node = *best_handle;
    last_f_score = best_node->fScore;
    best_node->is_in_open = false;

    if (time_bench.expands % print_every == 0) {
      print_search_status();
    }

    time_bench.expands++;

    // CHECK if best node is close ENOUGH to goal
    double distance_to_goal =
        robot->distance(best_node->state_eig, problem.goals[robot_id]);

    if (distance_to_goal < best_distance_to_goal) {
      best_distance_to_goal = distance_to_goal;
    }
    bool is_at_goal_no_constraints = false;
    if (distance_to_goal <
        options_tdbastar.delta_factor_goal * options_tdbastar.delta) {
      is_at_goal_no_constraints = true;
      for (const auto &constraint : constraints) {
        if (constraint.time >= best_node->gScore - 1e-6) {
          bool violation = robot->distance(best_node->state_eig,
                                           constraint.constrained_state) <=
                           options_tdbastar.delta;
          if (violation) {
            is_at_goal_no_constraints = false;
            break;
          }
        }
      }
    }
    if (is_at_goal_no_constraints) {
      std::cout << "FOUND SOLUTION" << std::endl;
      std::cout << "COST: " << best_node->gScore + best_node->hScore
                << std::endl;
      std::cout << "x: " << best_node->state_eig.format(FMT) << std::endl;
      std::cout << "d: " << distance_to_goal << std::endl;
      status = Terminate_status::SOLVED;
      break;
    }
    // No solution yet, continue the search
    focal.pop();
    open.erase(best_handle);
    // EXPAND the best node
    size_t num_expansion_best_node = 0;
    std::vector<LazyTraj> lazy_trajs;
    time_bench.time_lazy_expand += timed_fun_void(
        [&] { expander.expand_lazy(best_node->state_eig, lazy_trajs); });
    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];

      int num_valid_states = -1;
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());

      bool motion_valid = check_lazy_trajectory(
          lazy_traj, *robot, problem.goals[robot_id], time_bench, traj_wrapper,
          constraints, best_node->gScore, options_tdbastar.delta,
          aux_last_state, &ff, &num_valid_states, !reverse_search);
      if (!motion_valid) {
        continue;
      }

      int chosen_index = -1;
      check_goal(*robot, tmp_node->state_eig, problem.goals[robot_id],
                 traj_wrapper,
                 options_tdbastar.delta_factor_goal * options_tdbastar.delta,
                 num_check_goal, chosen_index, !reverse_search);

      // for the Node, if it reaches after the motion being transferred
      bool reachesGoal =
          robot->distance(tmp_node->state_eig, problem.goals[robot_id]) <=
          options_tdbastar.delta;
      // Tentative hScore, gScore
      double hScore;
      time_bench.time_hfun +=
          timed_fun_void([&] { hScore = h_fun->h(tmp_node->state_eig); });
      assert(hScore >= 0);
      double cost_motion = chosen_index != -1
                               ? chosen_index * robot->ref_dt
                               : (traj_wrapper.get_size() - 1) * robot->ref_dt;

      assert(cost_motion >= 0);

      double gScore = best_node->gScore + cost_motion +
                      options_tdbastar.cost_delta_factor *
                          robot->lower_bound_time(best_node->state_eig,
                                                  traj_wrapper.get_state(0));

      if (focal_heuristic_name == "volume_wise")
        focalHeuristic = best_node->focalHeuristic +
                         lowLevelfocalHeuristicShape(
                             results, robot_motions, problem, lazy_traj,
                             robot_id, best_node->gScore, all_robots);
      else
        focalHeuristic =
            best_node->focalHeuristic +
            lowLevelfocalHeuristic(solution, tmp_node, robot_id, all_robots,
                                   col_mng_robots, robot_objs);
      auto tmp_traj = dynobench::trajWrapper_2_Trajectory(traj_wrapper);
      tmp_traj.cost = best_node->gScore;
      expanded_trajs.push_back(tmp_traj);
      // CHECK if new State is NOVEL
      time_bench.time_nearestNode_search += timed_fun_void([&] {
        T_n->nearestR(tmp_node,
                      (1. - options_tdbastar.alpha) * options_tdbastar.delta,
                      neighbors_n);
      });
      if (!neighbors_n.size() || chosen_index != -1) {
        // STATE is NOVEL, we add the node
        num_expansion_best_node++;
        all_nodes.push_back(std::make_shared<AStarNode>());
        auto __node = all_nodes.back();
        __node->state_eig = tmp_node->state_eig;
        __node->gScore = gScore;
        __node->hScore = hScore;
        __node->fScore = gScore + hScore;
        __node->focalHeuristic = focalHeuristic;
        if (chosen_index != -1)
          __node->intermediate_state = chosen_index;
        __node->is_in_open = true;
        __node->reaches_goal = reachesGoal;
        __node->arrivals.push_back(
            {.gScore = gScore,
             .came_from = best_node,
             .used_motion = lazy_traj.motion->idx,
             .arrival_idx = best_node->current_arrival_idx});
        __node->current_arrival_idx = 0;
        time_bench.time_queue +=
            timed_fun_void([&] { __node->handle = open.push(__node); });
        time_bench.time_nearestNode_add +=
            timed_fun_void([&] { T_n->add(__node); });
        if (__node->fScore <= best_fScore * w) {
          focal.push(__node->handle);
        }
      } else {
        if (options_tdbastar.rewire) {
          for (auto &n : neighbors_n) {
            // STATE is not novel, we udpate
            if (float tentative_g =
                    gScore + options_tdbastar.cost_delta_factor *
                                 robot->lower_bound_time(tmp_node->state_eig,
                                                         n->state_eig);
                tentative_g < n->gScore) {
              bool update_valid = true;
              if (n->reaches_goal) {
                for (const auto &constraint : constraints) {
                  if (constraint.time >= best_node->gScore - 1e-6) {
                    bool violation =
                        robot->distance(n->state_eig,
                                        constraint.constrained_state) <=
                        options_tdbastar.delta;
                    if (violation) {
                      update_valid = false;
                      break;
                    }
                  }
                }
              }
              if (update_valid) {
                n->gScore = tentative_g;
                n->fScore = tentative_g + n->hScore;
                n->intermediate_state = -1; // reset intermediate state.
                n->arrivals.push_back(
                    {.gScore = tentative_g,
                     .came_from = best_node,
                     .used_motion = lazy_traj.motion->idx,
                     .arrival_idx = best_node->current_arrival_idx});
                ++n->current_arrival_idx;
                if (n->is_in_open) {
                  time_bench.time_queue +=
                      timed_fun_void([&] { open.increase(n->handle); });
                } else {
                  n->is_in_open = true;
                  time_bench.time_queue +=
                      timed_fun_void([&] { n->handle = open.push(n); });
                }
                if (n->fScore <= best_fScore * w) {
                  focal.push(n->handle);
                }
              }
            }
          }
        }
      }
      if (num_expansion_best_node >= options_tdbastar.limit_branching_factor) {
        break;
      }
    } // end of lazy_trajs loop
  }   // out of while loop

  time_bench.time_search = watch.elapsed_ms();

  time_bench.time_nearestMotion += expander.time_in_nn;
  time_bench.time_nearestNode =
      time_bench.time_nearestNode_add + time_bench.time_nearestNode_search;

  time_bench.extra_time =
      time_bench.time_search - time_bench.time_collisions -
      time_bench.time_nearestNode_add - time_bench.time_nearestNode_search -
      time_bench.time_lazy_expand - time_bench.time_alloc_primitive -
      time_bench.time_transform_primitive - time_bench.time_queue -
      time_bench.check_bounds - time_bench.time_hfun;

  assert(time_bench.extra_time >= 0);
  // assert(time_bench.extra_time / time_bench.time_search * 100 <
  //        20.); // sanity check -- could this fail?

  std::cout << "extra time " << time_bench.extra_time << " "
            << time_bench.extra_time / time_bench.time_search * 100 << "%"
            << std::endl;

  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  std::shared_ptr<AStarNode> solution_node;

  if (status == Terminate_status::SOLVED) {
    solution_node = best_node;
    out_info_tdb.solved = true;
  } else {
    if (!reverse_search) {
      auto nearest = T_n->nearest(goal_node);
      std::cout << "Close distance T_n to goal: "
                << robot->distance(goal_node->getStateEig(),
                                   nearest->getStateEig())
                << std::endl;
      solution_node = nearest;
    }
    out_info_tdb.solved = false;
  }

  if (status == Terminate_status::SOLVED) {
    from_solution_to_yaml_and_traj(*robot, motions, solution_node, problem,
                                   traj_out);
    traj_out.start = problem.starts[robot_id];
    traj_out.goal = problem.goals[robot_id];
    traj_out.check(robot, false);
    traj_out.cost = traj_out.actions.size() * robot->ref_dt;
    traj_out.fmin = open.top()->fScore;
    dynobench::Feasibility_thresholds thresholds;
    thresholds.col_tol =
        5 * 1e-2; // NOTE: for the systems with 0.01 s integration step,
    thresholds.goal_tol = options_tdbastar.delta;
    thresholds.traj_tol = options_tdbastar.delta;
    traj_out.update_feasibility(thresholds, false);

    // Sanity check here, that verifies that we obey all constraints
    std::cout << "checking constraints for the final solution " << std::endl;
    for (const auto &constraint : constraints) {
      int time_index = std::lround(constraint.time / robot->ref_dt);
      assert(time_index >= 0);
      if (time_index > (int)traj_out.states.size() - 1) {
        continue;
      }
      assert(time_index < (int)traj_out.states.size());
      float dist = robot->distance(traj_out.states.at(time_index),
                                   constraint.constrained_state);
      if (dist <= options_tdbastar.delta) {
        std::cout << "VIOLATION in solution  " << dist << " "
                  << "time index in solution: " << time_index << " "
                  << std::endl;
        std::cout << traj_out.states.at(time_index).format(dynobench::FMT)
                  << std::endl;
        break;
      }
      assert(dist > options_tdbastar.delta);
    }
    // get results filled with updated new solution
    results[robot_id].clear();
    size_t arrival_idx = solution_node->current_arrival_idx;
    while (solution_node != nullptr) {
      results[robot_id].push_back(std::make_pair(solution_node, arrival_idx));
      const auto &arrival = solution_node->arrivals[arrival_idx];
      solution_node = arrival.came_from;
      arrival_idx = arrival.arrival_idx;
    }
    std::reverse(results[robot_id].begin(), results[robot_id].end());
  }

  out_info_tdb.solved = status == Terminate_status::SOLVED;
  out_info_tdb.cost = traj_out.cost;
  out_info_tdb.time_search = time_bench.time_search;
  out_info_tdb.data = time_bench.to_data();
  out_info_tdb.data.insert(std::make_pair(
      "terminate_status", terminate_status_str[static_cast<int>(status)]));
  out_info_tdb.data.insert(
      std::make_pair("solved", std::to_string(bool(out_info_tdb.solved))));
  out_info_tdb.data.insert(
      std::make_pair("delta", std::to_string(options_tdbastar.delta)));
  out_info_tdb.data.insert(
      std::make_pair("num_primitives", std::to_string(motions.size())));
}

} // namespace dynoplan
