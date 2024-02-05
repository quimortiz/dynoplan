// #include "dynoplan/dbastar/dbastar.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"

#include <boost/graph/graphviz.hpp>

// #include <flann/flann.hpp>
// #include <msgpack.hpp>
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

#include "dynoplan/nigh_custom_spaces.hpp"

namespace dynoplan {

using dynobench::Trajectory;

using dynobench::FMT;

namespace ob = ompl::base;

using Sample = std::vector<double>;
using Sample_ = ob::State;

// nigh interface for OMPL

const char *duplicate_detection_str[] = {"NO", "HARD", "SOFT"};

bool compareAStarNode::operator()(const AStarNode *a,
                                  const AStarNode *b) const {
  // Sort order
  // 1. lowest fScore
  // 2. highest gScore

  // Our heap is a maximum heap, so we invert the comperator function here
  if (a->fScore != b->fScore) {
    return a->fScore > b->fScore;
  } else {
    return a->gScore < b->gScore;
  }
}
void disable_motions(std::shared_ptr<dynobench::Model_robot>& robot,
    std::string &robot_name,
    float delta,
    bool filterDuplicates,
    float alpha,
    size_t num_max_motions,
    std::vector<Motion>& motions) {
    ompl::NearestNeighbors<Motion *> *T_m = nullptr;
    T_m = nigh_factory_t<Motion *>(robot_name, robot, /*reverse search*/false);
    // enable all motions
    for (size_t i = 0; i < motions.size(); ++i) {
      motions[i].disabled = false;
      T_m->add(&motions.at(i));
    } 
    if(filterDuplicates){
      size_t num_duplicates = 0;
      Motion fakeMotion;
      fakeMotion.idx = -1;
      fakeMotion.traj.states.push_back(Eigen::VectorXd(robot->nx));
      std::vector<Motion *> neighbors_m;
      for (const auto& m : motions) {
        if (m.disabled) {
          continue;
        }
        fakeMotion.traj.states.at(0) = m.traj.states.at(0);
        T_m->nearestR(&fakeMotion, delta*alpha, neighbors_m);
        for (Motion* nm : neighbors_m){
          if (nm == &m || nm->disabled) { 
            continue;
          }
          float goal_delta = robot->distance(m.traj.states.back(), nm->traj.states.back());
          if (goal_delta < delta*(1-alpha)) {
          nm->disabled = true;
          ++num_duplicates;
          }
        }
      }
      // std::cout << "There are " << num_duplicates << " duplicate motions!" << std::endl;
    }
    // limit to num_max_motions
    size_t num_enabled_motions = 0;
    for (size_t i = 0; i < motions.size(); ++i){
      if (!motions[i].disabled) {
        if (num_enabled_motions >= num_max_motions) {
          motions[i].disabled = true;
        } else {
          ++num_enabled_motions;
        }
      }
    }
    std::cout << "There are " << num_enabled_motions << " motions enabled." << std::endl;
  }

void from_solution_to_yaml_and_traj(dynobench::Model_robot &robot,
                                    const std::vector<Motion> &motions,
                                    AStarNode *solution,
                                    const dynobench::Problem &problem,
                                    dynobench::Trajectory &traj_out,
                                    std::ofstream *out) {
  std::vector<std::pair<AStarNode *, size_t>> result;
  CHECK(solution, AT);
  // TODO: check what happens if a solution is a single state?

  AStarNode *n = solution;
  size_t arrival_idx = n->current_arrival_idx;
  while (n != nullptr) {
    result.push_back(std::make_pair(n, arrival_idx));
    const auto& arrival = n->arrivals[arrival_idx];
    n = arrival.came_from;
    arrival_idx = arrival.arrival_idx;
  }

  std::reverse(result.begin(), result.end());

  std::cout << "result size " << result.size() << std::endl;

  auto space6 = std::string(6, ' ');
  if (result.size() == 1) {
    // eg. if start is closest state to the goal

    if (out) {
      *out << "  - states:" << std::endl;
      *out << space6 << "- ";
      *out << result.front().first->state_eig.format(FMT) << std::endl;
      *out << "    actions: []" << std::endl;
    }
    traj_out.states.push_back(result.front().first->state_eig);
  }

  if (out) {
    *out << "  - states:" << std::endl;
  }

  Eigen::VectorXd __tmp(robot.nx);
  Eigen::VectorXd __offset(robot.get_offset_dim());
  // get states
  for (size_t i = 0; i < result.size() - 1; ++i) {
    const auto node_state = result[i].first->state_eig;
    const auto &motion = motions.at(result[i+1].first->arrivals[result[i+1].second].used_motion);
    int take_until = result[i+1].first->intermediate_state;
    if (take_until != -1) {
      if (out) {
        *out << std::endl;
        *out << space6 + "# (note: we have stopped at intermediate state) "
             << std::endl;
      }
    }
    if (out) {
      *out << space6 + "# (node_state) " << node_state.format(FMT) << std::endl;
      *out << std::endl;
      *out << space6 + "# motion " << motion.idx << " with cost " << motion.cost
           << std::endl; // debug
      *out << space6 + "# motion first state "
           << motion.traj.states.front().format(FMT) << std::endl;
      *out << space6 + "# motion last state "
           << motion.traj.states.back().format(FMT) << std::endl;
    }
 
    // transform the motion to match the state

    // get the motion
    robot.offset(node_state, __offset);
    if (out) {
      *out << space6 + "# (tmp) " << __tmp.format(FMT) << std::endl;
      *out << space6 + "# (offset) " << __offset.format(FMT) << std::endl;
    };

    auto &traj = motion.traj;
    Trajectory __traj = motion.traj;
    dynobench::TrajWrapper traj_wrap =
        dynobench::Trajectory_2_trajWrapper(__traj);
        
    robot.transform_primitive(__offset, traj.states, traj.actions, traj_wrap);
    std::vector<Eigen::VectorXd> xs = traj_wrap.get_states();
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();

    // TODO: missing additional offset, if any

    double jump = robot.lower_bound_time(node_state, xs.front());
    // CSTR_V(node_state);
    // CSTR_V(xs.front());
    // std::cout << "jump " << jump << std::endl;

    if (out) {
      *out << space6 + "# (traj.states.front) "
           << traj.states.front().format(FMT) << std::endl;
      *out << space6 + "# (xs.front) " << xs.front().format(FMT) << std::endl;
    }

    size_t take_num_states = xs.size();
    if (take_until != -1)
      take_num_states = take_until + 1;

    DYNO_CHECK_LEQ(take_num_states, xs.size(), AT);
    for (size_t k = 0; k < take_num_states; ++k) {
      if (k < take_num_states - 1) {
        // print the state

        if (out) {
          *out << space6 << "- ";
        }
        traj_out.states.push_back(xs.at(k));
      } else if (i == result.size() - 2) {
        if (out) {
          *out << space6 << "- ";
        }
        // traj_out.states.push_back(result.at(i + 1)->state_eig);
        traj_out.states.push_back(result[i+1].first->state_eig);
        // traj_out.states.push_back(xs.at(k)); This was before, fails if I have
        // change the parent of the last node before it goes out of the queue.
      } else {
        if (out) {
          *out << space6 << "# (last state) ";
        }
      }
      if (out) {
        *out << xs.at(k).format(FMT) << std::endl;
      }
    }

    // Continue here!!
    // Just get state + motion
    // skip last, then state... and so on!!!
  }
  if (out) {
    *out << space6 << "# goal state is " << problem.goal.format(FMT)
         << std::endl;
    *out << "    actions:" << std::endl;
  }

  for (size_t i = 0; i < result.size() - 1; ++i) {
    const auto &motion = motions.at(result[i+1].first->arrivals[result[i+1].second].used_motion);
    int take_until = result[i+1].first->intermediate_state;
    if (take_until != -1) {
      if (out) {
        *out << space6 + "# (note: we have stop at intermediate state) "
             << std::endl;
      }
    }

    if (out) {
      *out << space6 + "# motion " << motion.idx << " with cost " << motion.cost
           << std::endl;
    }

    size_t take_num_actions = motion.actions.size();

    if (take_until != -1) {
      take_num_actions = take_until;
    }
    DYNO_CHECK_LEQ(take_num_actions, motion.actions.size(), AT);
    if (out) {
      *out << space6 + "# "
           << "take_num_actions " << take_num_actions << std::endl;
    }

    for (size_t k = 0; k < take_num_actions; ++k) {
      const auto &action = motion.traj.actions.at(k);
      if (out) {
        *out << space6 + "- ";
        *out << action.format(FMT) << std::endl;
        *out << std::endl;
      }
      Eigen::VectorXd x;
      traj_out.actions.push_back(action);
    }
    if (out)
      *out << std::endl;
  }
  DYNO_CHECK_LEQ((result.back().first->state_eig - traj_out.states.back()).norm(),
                 1e-6, "");
};

void __add_state_timed(AStarNode *node,
                       ompl::NearestNeighbors<AStarNode *> *T_n,
                       Time_benchmark &time_bench) {
  assert(node);
  assert(T_n);
  auto out = timed_fun([&] {
    T_n->add(node);
    return 0;
  });
  time_bench.time_nearestNode += out.second;
  time_bench.time_nearestNode_add += out.second;
};

bool check_lazy_trajectory(
    LazyTraj &lazy_traj, dynobench::Model_robot &robot,
    const Eigen::Ref<const Eigen::VectorXd> &goal,
    Time_benchmark &time_bench, dynobench::TrajWrapper &tmp_traj,
    const std::vector<Constraint>& constraints,
    const float best_node_gScore,
    float delta,
    Eigen::Ref<Eigen::VectorXd> aux_last_state,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state,
    int *num_valid_states, bool forward) {

  time_bench.time_alloc_primitive += 0; // no memory allocation :)
  // preliminary check only on bounds of last state
  if (robot.transform_primitive_last_state_available) {

    if (forward) {
      robot.transform_primitive_last_state(
          *lazy_traj.offset, lazy_traj.motion->traj.states,
          lazy_traj.motion->traj.actions, aux_last_state);

    } else {
      robot.transform_primitive_last_state_backward(
          *lazy_traj.offset, lazy_traj.motion->traj.states,
          lazy_traj.motion->traj.actions, aux_last_state);
    }

    if (!robot.is_state_valid(aux_last_state))
      return false;
  }

  Stopwatch wacht_tp;
  // forward or backward?

  lazy_traj.compute(tmp_traj, forward, check_state, num_valid_states);

  time_bench.time_transform_primitive += wacht_tp.elapsed_ms();


  Stopwatch watch_check_motion;

  if (num_valid_states && *num_valid_states < 1 ) {
    return false;
  }

  if (check_state) {
    // bounds are check when doing the rollout
    if (forward)
      assert(num_valid_states);
    if (*num_valid_states < lazy_traj.motion->traj.states.size()) {
      return false;
    }

  } else {
    // checking backwards is usually faster
    for (size_t i = 0; i < tmp_traj.get_size(); i++) {
      if (!robot.is_state_valid(
              tmp_traj.get_state(tmp_traj.get_size() - 1 - i))) {
        return false;
      }
    }
  }

  time_bench.check_bounds += watch_check_motion.elapsed_ms();

  bool motion_valid;
  auto &motion = lazy_traj.motion;
  time_bench.time_collisions += timed_fun_void([&] {
    if (robot.invariance_reuse_col_shape) {
      Eigen::VectorXd offset = *lazy_traj.offset;
      assert(offset.size() == 2 || offset.size() == 3);
      Eigen::Vector3d __offset;
      if (offset.size() == 2) {
        __offset = Eigen::Vector3d(offset(0), offset(1), 0);
      } else {
        __offset = offset.head<3>();
      }
      assert(motion);
      assert(motion->collision_manager);
      assert(robot.env.get());
      std::vector<fcl::CollisionObject<double> *> objs;
      motion->collision_manager->getObjects(objs);
      motion->collision_manager->shift(__offset);
      fcl::DefaultCollisionData<double> collision_data;
      motion->collision_manager->collide(robot.env.get(), &collision_data,
                                         fcl::DefaultCollisionFunction<double>);
      motion->collision_manager->shift(-__offset);
      motion_valid = !collision_data.result.isCollision();
    } else {
      motion_valid = dynobench::is_motion_collision_free(tmp_traj, robot);
    }
  });
  time_bench.num_col_motions++;
  // check with constraints
  // std::cout << "Printing the tmp traj: " << std::endl;
  // for (auto tr : tmp_traj.get_states()){
  //     std::cout << tr.format(dynobench::FMT) << std::endl;
  // }
  // std::cout << "Finishing printing the tmp traj" << std::endl;
  bool reachesGoal;
  if (!forward){
    reachesGoal = robot.distance(tmp_traj.get_state(tmp_traj.get_size() - 1), goal) <= delta;
  }
  else{
    reachesGoal = robot.distance(tmp_traj.get_state(0), goal) <= delta;
  }
  for (const auto& constraint : constraints){
    // a constraint violation can only occur between t in [current->gScore, tentative_gScore]
    float time_offset = constraint.time - best_node_gScore;
    int time_index = std::lround(time_offset / robot.ref_dt);
    // std::cout << "Time index: " << time_index << std::endl;
    Eigen::VectorXd state_to_check;
    if (reachesGoal && time_index >= (int)tmp_traj.get_size() - 1) {
      state_to_check = tmp_traj.get_state(tmp_traj.get_size() - 1);
    }
    if (time_index >= 0 && time_index < (int)tmp_traj.get_size() - 1) {
      state_to_check = tmp_traj.get_state(time_index);
    }

    if (state_to_check.size() > 0) {
      bool violation = robot.distance(state_to_check, constraint.constrained_state) <= delta;
      if (violation) {
        motion_valid = false;
        // std::cout << "VIOLATION inside lazy traj check" << time_index << " " << tmp_traj.get_size() << std::endl;
        // std::cout << "State to check: " << state_to_check.format(dynobench::FMT) << std::endl;
        // std::cout << "Constraint state: " << constraint.constrained_state.format(dynobench::FMT) << std::endl;
        // throw std::runtime_error("Internal error: constraint violation in check lazy trajectory!");
        break;
      }
    }
  }
  // std::cout << "Motion validity: " << motion_valid << std::endl;
  return motion_valid;
};

void check_goal(dynobench::Model_robot &robot, Eigen::Ref<Eigen::VectorXd> x,
                const Eigen::Ref<const Eigen::VectorXd> &goal,
                dynobench::TrajWrapper &traj_wrapper, double distance_bound,
                size_t num_check_goal, int &chosen_index, bool forward) {
  if(forward){
    x = traj_wrapper.get_state(traj_wrapper.get_size() - 1);
  }
  // for the reverse search
  else {
    x = traj_wrapper.get_state(0);
  }
  Eigen::VectorXd intermediate_sol(robot.nx);
  for (size_t nn = 0; nn < num_check_goal; nn++) {
    size_t index_to_check =
        float(nn + 1) / (num_check_goal + 1) * traj_wrapper.get_size();
    if (double d = robot.distance(traj_wrapper.get_state(index_to_check), goal);
        d <= distance_bound) {
      x = traj_wrapper.get_state(index_to_check);
      chosen_index = index_to_check;
      std::cout << "Found a solution with "
                   "goal checks (not intermediate)! "
                   "\n"
                << "x:" << x.format(FMT) << " d:" << d << std::endl;
      break;
    }
  }
};

// for standalone_tdbastar
void export_constraints(const std::vector<Constraint>& constrained_states, std::string robot_type,
                        size_t robot_id, std::ofstream *out){
  *out << "robot_type:" << std::endl;
  *out << "        ";
  *out << robot_type << std::endl;
  *out << "robot_id:" << std::endl;
  *out << "        ";
  *out << robot_id << std::endl;
  *out << "constraints:" << std::endl;
  for (size_t i = 0; i < constrained_states.size(); ++i){ 
      *out << "  - states:" << std::endl;
      *out << "        ";
      *out << constrained_states[i].constrained_state.format(dynobench::FMT)<< std::endl;
      *out << "    time:" << std::endl;
      *out << "        ";
      *out << constrained_states[i].time << std::endl;
  } 
};

void tdbastar(dynobench::Problem &problem, Options_tdbastar options_tdbastar, 
              Trajectory &traj_out, const std::vector<Constraint>& constraints,
              Out_info_tdb &out_info_tdb, size_t &robot_id, 
              bool reverse_search,
              std::vector<dynobench::Trajectory> &expanded_trajs,
              ompl::NearestNeighbors<AStarNode *>* heuristic_nn,
              ompl::NearestNeighbors<AStarNode *>** heuristic_result){

  // #ifdef DBG_PRINTS
  std::cout << "Running tdbA* for robot " << robot_id << std::endl;
  for (const auto& constraint : constraints){
    std::cout << "constraint at time: " << constraint.time << " ";
    std::cout << constraint.constrained_state.format(dynobench::FMT) << std::endl;
  }
  // #endif
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotTypes[robot_id] + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);
  load_env(*robot, problem);
  const int nx = robot->nx;
  // clean
  traj_out.states.clear();
  traj_out.actions.clear();
  traj_out.cost = 0;
  CHECK(options_tdbastar.motions_ptr,
        "motions should be loaded before calling dbastar");
  std::vector<Motion> &motions = *options_tdbastar.motions_ptr;
// for the reverse search debug
  // std::ofstream out2("../dynoplan/expanded_trajs_rev.yaml");
  // out2 << "trajs:" << std::endl;

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
  ompl::NearestNeighbors<AStarNode *> *T_n = nullptr;

  if (options_tdbastar.use_nigh_nn) {
    T_n = nigh_factory2<AStarNode *>(problem.robotTypes[robot_id], robot);
  } else {
    NOT_IMPLEMENTED;
  }
  // // for the initial heuristics
  if (heuristic_result) {
    // *heuristic_result = T_n;
    *heuristic_result = nigh_factory2<AStarNode *>(problem.robotTypes[robot_id], robot);
  }
  if (options_tdbastar.use_nigh_nn) {
    // if (reverse_search){
    //    T_m = nigh_factory2<Motion *>(problem.robotTypes[robot_id], robot,
    //    [](Motion& m) { return m->getLastStateEig(); });
    // }
    // else {
    //    T_m = nigh_factory2<Motion *>(problem.robotTypes[robot_id], robot);
    // }
    T_m = nigh_factory_t<Motion *>(problem.robotTypes[robot_id], robot, reverse_search);
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
    NOT_IMPLEMENTED; // HERE i could compute delta based on desired branching
                     // factor!
  }

  std::shared_ptr<Heu_fun> h_fun = nullptr;
  std::vector<Heuristic_node> heu_map;
  // h_fun = std::make_shared<Heu_euclidean>(robot, problem.goals[robot_id]);
  
  if(reverse_search){
    h_fun = std::make_shared<Heu_blind>();
  }
  else{
    h_fun = std::make_shared<Heu_roadmap_bwd<ompl::NearestNeighbors<AStarNode *>*, AStarNode>>(robot, heuristic_nn, problem.goals[robot_id]);
  }
  // all_nodes manages the memory.
  // c-pointer don't have onwership.
  // std::vector<std::unique_ptr<AStarNode>> all_nodes;
  // all_nodes.push_back(std::make_unique<AStarNode>());
  std::vector<std::shared_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_shared<AStarNode>());

  AStarNode *start_node = all_nodes.at(0).get();
  start_node->gScore = 0;
  start_node->state_eig = problem.starts[robot_id];
  start_node->hScore = h_fun->h(problem.starts[robot_id]); // robot->lower_bound_time()
  start_node->fScore = start_node->gScore + start_node->hScore; // TODO: BUG
  start_node->is_in_open = true;
  start_node->reaches_goal = (robot->distance(problem.starts[robot_id], problem.goals[robot_id]) <= options_tdbastar.delta);
  start_node->arrivals.push_back({.gScore = 0, .came_from = nullptr, .used_motion = (size_t)-1, .arrival_idx = (size_t)-1});

  DYNO_DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
  DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");

  // auto goal_node = std::make_unique<AStarNode>();
  auto goal_node = std::make_shared<AStarNode>();
  goal_node->state_eig = problem.goals[robot_id];
  open_t open;
  start_node->handle = open.push(start_node);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.traj.states.push_back(Eigen::VectorXd::Zero(robot->nx));

  AStarNode tmp_node;
  tmp_node.state_eig = Eigen::VectorXd::Zero(robot->nx);

  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goals[robot_id]);

  std::mt19937 g = std::mt19937{std::random_device()()};

  double cost_bound =
      options_tdbastar.maxCost; 

  if (options_tdbastar.fix_seed) {
    expander.seed(0);
    g = std::mt19937{0};
    srand(0);
  } else {
    srand(time(0));
  }

  time_bench.time_nearestNode_add +=
      timed_fun_void([&] { T_n->add(start_node); });
  if (reverse_search){
    (*heuristic_result)->add(start_node);
  }
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
                << "EMPTY_QUEUE" << std::endl;
      return true;
    }

    return false;
  };

  AStarNode *best_node = nullptr;
  std::vector<AStarNode *> closed_list;
  std::vector<AStarNode *> neighbors_n;

  const bool debug = false; 
  
  const bool check_intermediate_goal = true;
  const size_t num_check_goal = 0;
    //  4; // Eg, for n = 4 I check: 1/5 , 2/5 , 3/5 , 4/5

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
  while (!stop_search()) {

    // POP best node in queue
    time_bench.time_queue += timed_fun_void([&] {
      best_node = open.top();
      open.pop();
    });
    last_f_score = best_node->fScore;
    closed_list.push_back(best_node);
    best_node->is_in_open = false;
    // bool is_at_goal = best_node->reaches_goal;
  
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
        for (const auto& constraint : constraints) {
          if (constraint.time >= best_node->gScore - 1e-6) {
            bool violation = robot->distance(best_node->state_eig, constraint.constrained_state) <= options_tdbastar.delta;
            if (violation) {
              is_at_goal_no_constraints = false;
              break;
            }
          }
        }
      }
    if (is_at_goal_no_constraints){
      std::cout << "FOUND SOLUTION" << std::endl;
      std::cout << "COST: " << best_node->gScore + best_node->hScore // why f-score ?
                << std::endl;
      std::cout << "x: " << best_node->state_eig.format(FMT) << std::endl;
      std::cout << "d: " << distance_to_goal << std::endl;
      status = Terminate_status::SOLVED;
      break;
    }
    // EXPAND the best node
    size_t num_expansion_best_node = 0;
    std::vector<LazyTraj> lazy_trajs;
    time_bench.time_lazy_expand += timed_fun_void(
        [&] { expander.expand_lazy(best_node->state_eig, lazy_trajs); });
    // lazy_trajs = neighbors_m within R
    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];

      int num_valid_states = -1;
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      
      bool motion_valid =
          check_lazy_trajectory(lazy_traj, *robot, problem.goals[robot_id], time_bench, traj_wrapper, 
                                constraints, best_node->gScore, options_tdbastar.delta,
                                aux_last_state, &ff, &num_valid_states, !reverse_search);
      if (!motion_valid) {
        continue;
      }
      
      // Additional CHECK: if a intermediate state is close to goal. It really
      // helps!
      int chosen_index = -1;
      check_goal(*robot, tmp_node.state_eig, problem.goals[robot_id], traj_wrapper,
                 options_tdbastar.delta_factor_goal * options_tdbastar.delta,
                 num_check_goal, chosen_index, !reverse_search);

      // for the Node, if it reaches after the motion being transferred
      bool reachesGoal = robot->distance(tmp_node.state_eig, problem.goals[robot_id]) <= options_tdbastar.delta;
      // Tentative hScore, gScore
      double hScore;
      time_bench.time_hfun +=
          timed_fun_void([&] { hScore = h_fun->h(tmp_node.state_eig); });
      // assert(hScore >= 0);
      double cost_motion = chosen_index != -1
                               ? chosen_index * robot->ref_dt
                               : (traj_wrapper.get_size() - 1) * robot->ref_dt;

      assert(cost_motion >= 0);

      double gScore = best_node->gScore + cost_motion +
                      options_tdbastar.cost_delta_factor *
                          robot->lower_bound_time(best_node->state_eig,
                                                  traj_wrapper.get_state(0));

      auto tmp_traj = dynobench::trajWrapper_2_Trajectory(traj_wrapper);
      tmp_traj.cost = best_node->gScore; // or gScore + hScore ?
      expanded_trajs.push_back(tmp_traj);
      // for debugging
      // out2 << "  - " << std::endl;
      // tmp_traj.to_yaml_format_short(out2, "    ");
      // CHECK if new State is NOVEL
      time_bench.time_nearestNode_search += timed_fun_void([&] {
        T_n->nearestR(&tmp_node,
                      (1. - options_tdbastar.alpha) * options_tdbastar.delta,
                      neighbors_n);
      });
      if (!neighbors_n.size() || chosen_index != -1) {
        // STATE is NOVEL, we add the node
        num_expansion_best_node++;
        // all_nodes.push_back(std::make_unique<AStarNode>());
        all_nodes.push_back(std::make_shared<AStarNode>());
        AStarNode *__node = all_nodes.back().get();
        __node->state_eig = tmp_node.state_eig;
        __node->gScore = gScore;
        __node->hScore = hScore;
        __node->fScore = gScore + hScore;
        if (chosen_index != -1)
          __node->intermediate_state = chosen_index;
        __node->is_in_open = true;
        __node->reaches_goal = reachesGoal; 
        __node->arrivals.push_back({.gScore = gScore, .came_from = best_node, 
                                    .used_motion = lazy_traj.motion->idx, .arrival_idx = best_node->current_arrival_idx});
        __node->current_arrival_idx = 0;
        time_bench.time_queue +=
            timed_fun_void([&] { __node->handle = open.push(__node); });
        time_bench.time_nearestNode_add +=
            timed_fun_void([&] { T_n->add(__node); });
        if (reverse_search){
          (*heuristic_result)->add(__node);
        }

      } 
      else {
        if(options_tdbastar.rewire){
          for (auto &n : neighbors_n) {
            // STATE is not novel, we udpate
            if (float tentative_g = gScore + 
                    options_tdbastar.cost_delta_factor *
                    robot->lower_bound_time(tmp_node.state_eig, n->state_eig);
                tentative_g < n->gScore) {
              bool update_valid = true;
              if (n->reaches_goal){
                for (const auto& constraint : constraints) {
                  if (constraint.time >= best_node->gScore - 1e-6) {
                    bool violation = robot->distance(n->state_eig, constraint.constrained_state) <= options_tdbastar.delta;
                    if (violation) {
                      update_valid = false;
                      break;
                    }
                  }
                }
              }
              if (update_valid){
                n->gScore = tentative_g;
                n->fScore = tentative_g + n->hScore;
                n->intermediate_state = -1; // reset intermediate state.
                n->arrivals.push_back({.gScore = tentative_g, .came_from = best_node, 
                                          .used_motion = lazy_traj.motion->idx, .arrival_idx = best_node->current_arrival_idx});
                ++n->current_arrival_idx;
                if (n->is_in_open) {
                  time_bench.time_queue +=
                      timed_fun_void([&] { open.increase(n->handle); });
                } else {
                  n->is_in_open = true;
                  time_bench.time_queue +=
                      timed_fun_void([&] { n->handle = open.push(n); });
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
  } // out of while loop
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

  AStarNode *solution = nullptr;

  if (status == Terminate_status::SOLVED) {
    solution = best_node;
    out_info_tdb.solved = true;
  } else {
      if (!reverse_search){
        auto nearest = T_n->nearest(goal_node.get());
        std::cout << "Close distance T_n to goal: "
                  << robot->distance(goal_node->getStateEig(),
                                    nearest->getStateEig())
                  << std::endl;
        solution = nearest;
      }
    out_info_tdb.solved = false;
  }

  if (status == Terminate_status::SOLVED) {
    from_solution_to_yaml_and_traj(*robot, motions, solution, problem, traj_out);
    traj_out.start = problem.starts[robot_id];
    traj_out.goal = problem.goals[robot_id];
    traj_out.check(robot, false);
    traj_out.cost = traj_out.actions.size() * robot->ref_dt;
    dynobench::Feasibility_thresholds thresholds;
    thresholds.col_tol =
        5 * 1e-2; // NOTE: for the systems with 0.01 s integration step,
    // I check collisions only at 0.05s . Thus, an intermediate state
    // could be slightly in collision.
    thresholds.goal_tol = options_tdbastar.delta;
    thresholds.traj_tol = options_tdbastar.delta;
    traj_out.update_feasibility(thresholds, false);
    // CHECK(traj_out.feasible, ""); // should I keep it ? 

    // Sanity check here, that verifies that we obey all constraints
    std::cout << "checking constraints for the final solution " << std::endl;
    for (const auto& constraint : constraints) {
        // std::cout << "constraint t = " << constraint.time << std::endl;
        // std::cout << constraint.constrained_state.format(dynobench::FMT) << std::endl;
        int time_index = std::lround(constraint.time / robot->ref_dt);
        assert(time_index >= 0);
        if (time_index > (int)traj_out.states.size()-1){
          continue;
        }
        // time_index = std::min<int>(time_index, (int)traj_out.states.size()-1);
        assert(time_index < (int)traj_out.states.size());
        // std::cout << robot->distance(traj_out.states.at(time_index), constraint.constrained_state) << std::endl;
        // std::cout << traj_out.states.at(time_index).format(dynobench::FMT) << std::endl;
        float dist = robot->distance(traj_out.states.at(time_index), constraint.constrained_state);
        if (dist <= options_tdbastar.delta){
          std::cout << "VIOLATION in solution  " << dist << " " << "time index in solution: " << time_index << " " << std::endl;
          std::cout << traj_out.states.at(time_index).format(dynobench::FMT) << std::endl;
          // throw std::runtime_error("Internal error: constraint violation in solution!");
          break;
        }
        assert(dist > options_tdbastar.delta);
    }
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

void write_heu_map(const std::vector<Heuristic_node> &heu_map, const char *file,
                   const char *header) {
  std::ofstream out(file);

  if (header) {
    out << header << std::endl;
  }
  const char *four_space = "    ";
  out << "heu_map:" << std::endl;
  for (auto &v : heu_map) {
    out << "  -" << std::endl;
    out << four_space << "x: " << v.x.format(FMT) << std::endl;
    out << four_space << "d: " << v.d << std::endl;
    out << four_space << "p: " << v.p << std::endl;
  }
}

void load_heu_map(const char *file, std::vector<Heuristic_node> &heu_map) {
  std::cout << "loading heu map -- file: " << file << std::endl;
  std::ifstream in(file);
  CHECK(in.is_open(), AT);
  YAML::Node node = YAML::LoadFile(file);

  if (node["heu_map"]) {

    for (const auto &state : node["heu_map"]) {
      std::vector<double> x = state["x"].as<std::vector<double>>();
      Eigen::VectorXd xe = Eigen::VectorXd::Map(x.data(), x.size());
      double d = state["d"].as<double>();
      int p = state["p"].as<double>();
      heu_map.push_back({xe, d, p});
    }
  } else {
    ERROR_WITH_INFO("missing map key");
  }
}

} // namespace dynoplan
