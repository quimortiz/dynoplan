#include "dynoplan/dbastar/dbastar.hpp"

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

void plot_search_tree(std::vector<AStarNode *> nodes,
                      std::vector<Motion> &motions,
                      dynobench::Model_robot &robot, const char *filename) {

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

void from_solution_to_yaml_and_traj(dynobench::Model_robot &robot,
                                    const std::vector<Motion> &motions,
                                    AStarNode *solution,
                                    const dynobench::Problem &problem,
                                    dynobench::Trajectory &traj_out,
                                    std::ofstream *out) {
  std::vector<const AStarNode *> result;

  CHECK(solution, AT);

  const AStarNode *n = solution;
  while (n != nullptr) {
    result.push_back(n);
    // std::cout << n->used_motion << std::endl;
    // si->printState(n->state);
    n = n->came_from;
  }
  std::reverse(result.begin(), result.end());

  std::cout << "result size " << result.size() << std::endl;

  if (out) {
    *out << "  - states:" << std::endl;
  }

  auto space6 = std::string(6, ' ');

  Eigen::VectorXd __tmp(robot.nx);
  Eigen::VectorXd __offset(robot.get_offset_dim());
  for (size_t i = 0; i < result.size() - 1; ++i) {
    const auto node_state = result.at(i)->state_eig;
    const auto &motion = motions.at(result.at(i + 1)->used_motion);
    int take_until = result.at(i + 1)->intermediate_state;
    if (take_until != -1) {
      if (out) {
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
    //
    //
    //
    //
    // transform the motion to match the state

    // get the motion
    robot.offset(node_state, __offset);
    if (out) {
      *out << space6 + "# (tmp) " << __tmp.format(FMT) << std::endl;
      *out << space6 + "# (offset) " << __offset.format(FMT) << std::endl;
    };

    auto &traj = motion.traj;
    std::vector<Eigen::VectorXd> xs = traj.states;
    std::vector<Eigen::VectorXd> us = traj.actions;
    robot.transform_primitive(__offset, traj.states, traj.actions, xs, us);
    // TODO: missing additional offset, if any

    double jump = robot.lower_bound_time(node_state, xs.front());
    CSTR_V(node_state);
    CSTR_V(xs.front());
    std::cout << "jump " << jump << std::endl;

    if (*out) {
      *out << space6 + "# (traj.states.front) "
           << traj.states.front().format(FMT) << std::endl;
      *out << space6 + "# (xs.front) " << xs.front().format(FMT) << std::endl;
    }

    size_t take_num_states = xs.size();
    if (take_until != -1)
      take_num_states = take_until + 1;

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
        traj_out.states.push_back(xs.at(k));
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
    const auto &motion = motions.at(result.at(i + 1)->used_motion);
    int take_until = result.at(i + 1)->intermediate_state;
    if (take_until != -1) {
      if (out) {
        *out << space6 + "# (note: we have stop at intermediate state) "
             << std::endl;
      }
    }

    if (*out) {
      *out << space6 + "# motion " << motion.idx << " with cost " << motion.cost
           << std::endl;
    }

    size_t take_num_actions = motion.actions.size();

    if (take_until != -1) {
      take_num_actions = take_until;
    }
    DYNO_CHECK_LEQ(take_num_actions, motion.actions.size(), AT);
    if (*out) {
      *out << space6 + "# "
           << "take_num_actions " << take_num_actions << std::endl;
    }

    for (size_t k = 0; k < take_num_actions; ++k) {
      const auto &action = motion.traj.actions.at(k);
      if (*out) {
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
};

double automatic_delta(double delta_in, double alpha, RobotOmpl &robot,
                       ompl::NearestNeighbors<Motion *> &T_m) {
  Motion fakeMotion;
  fakeMotion.idx = -1;
  auto si = robot.getSpaceInformation();
  fakeMotion.states.push_back(si->allocState());
  std::vector<Motion *> neighbors_m;
  size_t num_desired_neighbors = (size_t)-delta_in;
  size_t num_samples = std::min<size_t>(1000, T_m.size());

  auto state_sampler = si->allocStateSampler();
  double sum_delta = 0.0;
  for (size_t k = 0; k < num_samples; ++k) {
    do {
      state_sampler->sampleUniform(fakeMotion.states[0]);
    } while (!si->isValid(fakeMotion.states[0]));
    robot.setPosition(fakeMotion.states[0], fcl::Vector3d(0, 0, 0));

    T_m.nearestK(&fakeMotion, num_desired_neighbors + 1, neighbors_m);

    double max_delta =
        si->distance(fakeMotion.states[0], neighbors_m.back()->states.front());
    sum_delta += max_delta;
  }
  double adjusted_delta = (sum_delta / num_samples) / alpha;
  std::cout << "Automatically adjusting delta to: " << adjusted_delta
            << std::endl;

  si->freeState(fakeMotion.states.back());

  return adjusted_delta;
}

void filter_duplicates(std::vector<Motion> &motions, double delta, double alpha,
                       RobotOmpl &robot, ompl::NearestNeighbors<Motion *> &T_m,
                       double factor) {

  auto si = robot.getSpaceInformation();
  size_t num_duplicates = 0;
  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.states.push_back(si->allocState());
  std::vector<Motion *> neighbors_m;
  for (const auto &m : motions) {
    if (m.disabled) {
      continue;
    }

    si->copyState(fakeMotion.states[0], m.states[0]);
    T_m.nearestR(&fakeMotion, factor * delta * alpha, neighbors_m);

    for (Motion *nm : neighbors_m) {
      if (nm == &m || nm->disabled) {
        continue;
      }
      double goal_delta = si->distance(m.states.back(), nm->states.back());
      if (goal_delta < factor * delta * (1 - alpha)) {
        nm->disabled = true;
        ++num_duplicates;
      }
    }
  }
  std::cout << "There are " << num_duplicates << " duplicate motions!"
            << std::endl;
}

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

bool check_lazy_trajectory(LazyTraj &lazy_traj, dynobench::Model_robot &robot,
                           Time_benchmark &time_bench,
                           dynobench::Trajectory &tmp_traj) {
  Stopwatch wacht_mem;
  tmp_traj.states = lazy_traj.motion->traj.states;
  tmp_traj.actions = lazy_traj.motion->traj.actions;
  time_bench.time_alloc_primitive += wacht_mem.elapsed_ms();

  Stopwatch wacht_tp;
  lazy_traj.compute(tmp_traj);
  time_bench.time_transform_primitive += wacht_tp.elapsed_ms();

  Stopwatch wacht_check_motion;
  for (auto &state : tmp_traj.states) {
    if (!robot.is_state_valid(state)) {
      return false;
    }
  }
  time_bench.check_bounds += wacht_check_motion.elapsed_ms();

  bool motion_valid;
  time_bench.time_collisions += timed_fun_void([&] {
    motion_valid = dynobench::is_motion_collision_free(tmp_traj, robot);
  });

  time_bench.num_col_motions++;

  return motion_valid;
};

void dbastar(const dynobench::Problem &problem, Options_dbastar options_dbastar,
             Trajectory &traj_out, Out_info_db &out_info_db) {

  std::cout << "*** options_dbastar ***" << std::endl;
  options_dbastar.print(std::cout);
  std::cout << "***" << std::endl;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);
  load_env(*robot, problem);
  const int nx = robot->nx;

  CHECK(options_dbastar.motions_ptr,
        "motions should be loaded before calling dbastar");
  std::vector<Motion> &motions = *options_dbastar.motions_ptr;

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

  if (options_dbastar.use_nigh_nn) {
    T_m = nigh_factory2<Motion *>(problem.robotType, robot);
  } else {
    NOT_IMPLEMENTED;
  }

  time_bench.time_nearestMotion += timed_fun_void([&] {
    for (size_t i = 0;
         i < std::min(motions.size(), options_dbastar.max_motions); ++i) {
      T_m->add(&motions.at(i));
    }
  });

  if (options_dbastar.use_nigh_nn) {
    T_n = nigh_factory2<AStarNode *>(problem.robotType, robot);
  } else {
    NOT_IMPLEMENTED;
  }

  Expander expander(robot.get(), T_m,
                    options_dbastar.alpha * options_dbastar.delta);

  if (options_dbastar.alpha <= 0 || options_dbastar.alpha >= 1) {
    ERROR_WITH_INFO("Alpha needs to be between 0 and 1!");
  }

  if (options_dbastar.delta < 0) {
    NOT_IMPLEMENTED; // HERE i could compute delta based on desired branching
                     // factor!
  }

  std::shared_ptr<Heu_fun> h_fun = nullptr;
  std::vector<Heuristic_node> heu_map;

  switch (options_dbastar.heuristic) {
  case 0: {
    h_fun = std::make_shared<Heu_euclidean>(robot, problem.goal);
  } break;
  case 1: {
    if (options_dbastar.heu_map_ptr) {
      std::cout << "Heuristic map is already loaded" << std::endl;
    } else {
      if (options_dbastar.heu_map_file.size()) {
        std::cout << "loading map from file " << std::endl;
        load_heu_map(options_dbastar.heu_map_file.c_str(), heu_map);
      } else {
        std::cout << "not heu map provided. Computing one .... " << std::endl;
        time_bench.build_heuristic += timed_fun_void([&] {
          generate_heuristic_map(problem, robot, options_dbastar, heu_map);
        });
        auto filename = "/tmp/dynoplan/tmp_heu_map.yaml";
        create_dir_if_necessary(filename);
        write_heu_map(heu_map, filename);
      }
      options_dbastar.heu_map_ptr = &heu_map;
    }

    auto hh = std::make_shared<Heu_roadmap>(robot, *options_dbastar.heu_map_ptr,
                                            problem.goal, problem.robotType);
    hh->connect_radius_h = options_dbastar.connect_radius_h;
    h_fun = hh;
  } break;
  case -1: {
    h_fun = std::make_shared<Heu_blind>();
  } break;
  default: {
    ERROR_WITH_INFO("not implemented");
  }
  }

  // all_nodes manages the memory.
  // c-pointer don't have onwership.
  std::vector<std::unique_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_unique<AStarNode>());

  AStarNode *start_node = all_nodes.at(0).get();
  start_node->gScore = 0;
  start_node->state_eig = problem.start;
  start_node->hScore = h_fun->h(problem.start);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->came_from = nullptr;
  start_node->is_in_open = true;

  DYNO_DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
  DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");

  auto goal_node = std::make_unique<AStarNode>();
  goal_node->state_eig = problem.goal;

  open_t open;
  start_node->handle = open.push(start_node);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.traj.states.push_back(Eigen::VectorXd::Zero(robot->nx));

  AStarNode tmp_node;
  tmp_node.state_eig = Eigen::VectorXd::Zero(robot->nx);

  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goal);

  std::mt19937 g = std::mt19937{std::random_device()()};

  double cost_bound =
      options_dbastar.maxCost; //  std::numeric_limits<double>::infinity();

  if (options_dbastar.fix_seed) {
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
    std::cout << "expands: " << time_bench.expands
              << " best distance: " << best_distance_to_goal
              << " fscore: " << last_f_score << std::endl;
  };

  Stopwatch watch;
  Terminate_status status = Terminate_status::UNKNOWN;

  auto stop_search = [&] {
    if (static_cast<size_t>(time_bench.expands) >=
        options_dbastar.max_expands) {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }

    if (watch.elapsed_ms() > options_dbastar.search_timelimit) {
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
  std::vector<Trajectory> expanded_trajs; // for debugging

  const bool debug = false;

  // TODO: option to inflat the heuristic
  //
  const bool check_intermediate_goal = true;
  const size_t num_check_goal =
      4; // Eg, for n = 4 I check: 1/5 , 2/5 , 3/5 , 4/5

  while (!stop_search()) {

    // POP best node in queue
    time_bench.time_queue += timed_fun_void([&] {
      best_node = open.top();
      open.pop();
    });
    last_f_score = best_node->fScore;
    closed_list.push_back(best_node);
    best_node->is_in_open = false;

    if (time_bench.expands % print_every == 0) {
      print_search_status();
    }

    time_bench.expands++;

    // CHECK if best node is close ENOUGH to goal
    double distance_to_goal =
        robot->distance(best_node->state_eig, problem.goal);

    if (distance_to_goal < best_distance_to_goal) {
      best_distance_to_goal = distance_to_goal;
    }

    if (distance_to_goal <
        options_dbastar.delta_factor_goal * options_dbastar.delta) {
      std::cout << "FOUND SOLUTION" << std::endl;
      std::cout << "COST: " << best_node->gScore + best_node->hScore
                << std::endl;
      status = Terminate_status::SOLVED;
      break;
    }

    // EXPAND the best node
    size_t num_expansion_best_node = 0;
    std::vector<LazyTraj> lazy_trajs;
    time_bench.time_lazy_expand += timed_fun_void(
        [&] { expander.expand_lazy(best_node->state_eig, lazy_trajs); });

    dynobench::Trajectory tmp_traj;
    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];
      bool motion_valid =
          check_lazy_trajectory(lazy_traj, *robot, time_bench, tmp_traj);

      if (!motion_valid) {
        continue;
      }

      // Additional CHECK: if a intermediate state is close to goal. It really
      // helps!
      tmp_node.state_eig = tmp_traj.states.back();
      int chosen_index = -1;
      Eigen::VectorXd intermediate_sol(robot->nx);
      for (size_t nn = 0; nn < num_check_goal; nn++) {
        size_t index_to_check =
            float(nn + 1) / (num_check_goal + 1) * tmp_traj.states.size();
        if (double d = robot->distance(tmp_traj.states.at(index_to_check),
                                       problem.goal);
            d <= options_dbastar.delta_factor_goal * options_dbastar.delta) {
          std::cout << "Found a solution with intermetidate checks!"
                    << std::endl;
          tmp_node.state_eig = tmp_traj.states.at(index_to_check);
          chosen_index = index_to_check;
          break;
        }
      }

      // Tentative hScore, gScore
      double hScore;
      time_bench.time_hfun +=
          timed_fun_void([&] { hScore = h_fun->h(tmp_node.state_eig); });
      assert(hScore >= 0);

      double cost_motion = chosen_index != -1
                               ? chosen_index * robot->ref_dt
                               : tmp_traj.states.size() * robot->ref_dt;

      assert(cost_motion >= 0);

      double gScore = best_node->gScore + cost_motion +
                      options_dbastar.cost_delta_factor *
                          robot->lower_bound_time(best_node->state_eig,
                                                  tmp_traj.states.front());

      // CHECK if new State is NOVEL
      time_bench.time_nearestNode_search += timed_fun_void([&] {
        T_n->nearestR(&tmp_node,
                      (1. - options_dbastar.alpha) * options_dbastar.delta,
                      neighbors_n);
      });

      if (!neighbors_n.size() || chosen_index != -1) {
        // STATE is NOVEL, we add the node
        num_expansion_best_node++;
        all_nodes.push_back(std::make_unique<AStarNode>());
        AStarNode *__node = all_nodes.back().get();
        __node->state_eig = tmp_node.state_eig;
        __node->gScore = gScore;
        __node->hScore = hScore;
        __node->fScore = gScore + hScore;
        __node->came_from = best_node;
        __node->used_motion = lazy_traj.motion->idx;
        if (chosen_index != -1)
          __node->intermediate_state = chosen_index;
        __node->is_in_open = true;

        time_bench.time_queue +=
            timed_fun_void([&] { __node->handle = open.push(__node); });
        time_bench.time_nearestNode_add +=
            timed_fun_void([&] { T_n->add(__node); });

        if (debug) {
          expanded_trajs.push_back(tmp_traj);
        }

      } else {
        for (auto &n : neighbors_n) {
          // STATE is not novel, we udpate the similar nodes
          if (double tentative_g =
                  gScore +
                  robot->lower_bound_time(tmp_node.state_eig, n->state_eig);
              tentative_g < n->gScore) {
            n->gScore = tentative_g;
            n->fScore = tentative_g + n->hScore;
            n->came_from = best_node;
            n->used_motion = lazy_traj.motion->idx;
            if (n->is_in_open) {
              time_bench.time_queue +=
                  timed_fun_void([&] { open.increase(n->handle); });
            } else {
              time_bench.time_queue +=
                  timed_fun_void([&] { n->handle = open.push(n); });
            }
          }
        }
      }

      if (num_expansion_best_node >= options_dbastar.limit_branching_factor) {
        break;
      }
    }
  }

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
  assert(time_bench.extra_time / time_bench.time_search * 100 <
         20.); // sanity check -- could this fail?

  std::cout << "extra time " << time_bench.extra_time << " "
            << time_bench.extra_time / time_bench.time_search * 100 << "%"
            << std::endl;

  if (debug) {
    std::ofstream out("/tmp/dynoplan/nodes_list.yaml");
    out << "close_list:" << std::endl;
    for (auto &c : closed_list) {
      out << "- " << c->state_eig.format(dynobench::FMT) << std::endl;
    }
    out << "all_nodes:" << std::endl;
    for (auto &c : all_nodes) {
      out << "- " << c->state_eig.format(dynobench::FMT) << std::endl;
    }
    std::ofstream out2("/tmp/dynoplan/expanded_trajs.yaml");

    out2 << "trajs:" << std::endl;
    for (auto &traj : expanded_trajs) {
      out2 << "  - " << std::endl;
      traj.to_yaml_format(out2, "    ");
    }
  }

  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  AStarNode *solution = nullptr;

  if (status == Terminate_status::SOLVED) {
    solution = best_node;
    out_info_db.solved = true;
  } else {
    auto nearest = T_n->nearest(goal_node.get());
    std::cout << "Close distance T_n to goal: "
              << robot->distance(goal_node->getStateEig(),
                                 nearest->getStateEig())
              << std::endl;
    solution = nearest;
    out_info_db.solved = false;
  }

  auto filename = "/tmp/dynoplan/dbastar_out.yaml";
  create_dir_if_necessary(filename);
  std::ofstream out(filename);

  out << "solved: " << (status == Terminate_status::SOLVED) << std::endl;
  out << "problem: " << problem.name << std::endl;
  out << "robot: " << problem.robotType << std::endl;
  out << "status: " << static_cast<int>(status) << std::endl;
  out << "status_str: " << terminate_status_str[static_cast<int>(status)]
      << std::endl;
  out << "all_nodes: " << all_nodes.size() << std::endl;
  out << "closed_list: " << closed_list.size() << std::endl;
  out << "gscore_sol: " << solution->gScore << std::endl;
  out << "distance_to_goal: "
      << robot->distance(solution->getStateEig(), goal_node->getStateEig())
      << std::endl;
  time_bench.write(out);

  out << "result:" << std::endl;
  from_solution_to_yaml_and_traj(*robot, motions, solution, problem, traj_out,
                                 &out);
  traj_out.start = problem.start;
  traj_out.goal = problem.goal;
  traj_out.check(robot, true);
  traj_out.cost = traj_out.actions.size() * robot->ref_dt;

  if (status == Terminate_status::SOLVED) {
    dynobench::Feasibility_thresholds thresholds;
    thresholds.goal_tol = options_dbastar.delta;
    thresholds.traj_tol = options_dbastar.delta;
    traj_out.update_feasibility(thresholds, true);
    CHECK(traj_out.feasible, "");
  }

  traj_out.update_feasibility(dynobench::Feasibility_thresholds(), true);

  {
    std::string filename_id =
        "/tmp/dynoplan/traj_db_" + gen_random(6) + ".yaml";
    std::cout << "saving traj to: " << filename_id << std::endl;
    create_dir_if_necessary(filename_id.c_str());
    std::ofstream out(filename_id);
    traj_out.to_yaml_format(out);
  }

  out_info_db.solved = status == Terminate_status::SOLVED;
  out_info_db.cost = traj_out.cost;
  out_info_db.time_search = time_bench.time_search;
  out_info_db.data = time_bench.to_data();
  out_info_db.data.insert(std::make_pair(
      "terminate_status", terminate_status_str[static_cast<int>(status)]));
  out_info_db.data.insert(
      std::make_pair("solved", std::to_string(bool(out_info_db.solved))));
  out_info_db.data.insert(
      std::make_pair("delta", std::to_string(options_dbastar.delta)));
  out_info_db.data.insert(
      std::make_pair("num_primitives", std::to_string(motions.size())));
}

#if 0



      T_m->nearestR(&fakeMotion, options_dbastar.delta * options_dbastar.alpha,
                    neighbors_m);

      neigh = T_n->nearest(n);
      return 0;

      double d = robot->distance(tmp_traj.states.back(), x_target);

      // take node from open
    }

    // __add_state_timed(start_node, T_n, time_bench);

    // std::vector<const Sample_ *> expanded;
    // std::vector<std::vector<double>> found_nn;
    // ompl::NearestNeighbors<AStarNode *> *T_landmarks;
    // std::vector<AStarNode *> ptrs; // TODO: i should clean memory

    // auto start_node = new AStarNode;
    // start_node->gScore = 0;
    // start_node->state = robot->startState;
    //
    // Stopwatch __watch_h_fun;
    // double hstart = h_fun->h(robot->startState);
    // time_bench.time_hfun += __watch_h_fun.elapsed_ms();
    //
    // start_node->fScore = options_dbastar.epsilon * hstart;
    // start_node->hScore = start_node->fScore;
    //
    // std::cout << "start_node heuristic g: " << start_node->gScore
    //           << " h: " << start_node->hScore << " f: " <<
    //           start_node->fScore
    //           << std::endl;
    //
    // start_node->came_from = nullptr;
    // start_node->used_offset = fcl::Vector3d(0, 0, 0);
    // start_node->used_motion = -1;

    // db-A* search

    auto handle = open.push(start_node);
    start_node->handle = handle;
    start_node->is_in_open = true;

    std::vector<std::vector<double>> _states_debug;

    Motion fakeMotion;
    fakeMotion.idx = -1;
    fakeMotion.states.push_back(si->allocState());

    AStarNode *query_n = new AStarNode();

    ob::State *tmpStateq = si->allocState();
    ob::State *tmpState = si->allocState();
    ob::State *tmpState_debug = si->allocState();
    ob::State *tmpState2 = si->allocState();
    std::vector<Motion *> neighbors_m;
    std::vector<AStarNode *> neighbors_n;

    double last_f_score = start_node->fScore;
    // size_t expands = 0;
    // clock start

    if (options_dbastar.use_landmarks) {
      NOT_IMPLEMENTED;
    }

    // int counter_nn = 0;

    AStarNode *solution = nullptr;

    AStarNode start_node;
    start_node->gScore = 0;
    start_node->state_eig = problem.start;
    start_node->hScore =
        robot->lower_bound_time(start_node->state_eig, problem.goal);
    start_node->fScore = start_node->gScore + start_node->hScore;
    start_node->came_from = nullptr;

    auto goal_node = new AStarNode;
    goal_node->gScore = 0;
    goal_node->state_eig = problem.goal;
    goal_node->hScore = 0;
    goal_node->fScore = 0;
    goal_node->came_from = nullptr;

    bool check_average_min_d = true;
    double total_d = 0;
    int counter = 0;
    double average_min_d = 0;
    if (check_average_min_d) {
      for (auto &p : ptrs) {
        std::vector<AStarNode *> neighbors;
        T_n->nearestK(p, 2, neighbors);
        if (neighbors.front() != p) {
          ERROR_WITH_INFO("why?");
        }
        total_d += si->distance(neighbors.at(1)->state, p->state);
        counter++;
      }
      average_min_d = total_d / counter;
      std::cout << "average min distance " << average_min_d << std::endl;
    }

    Terminate_status status = Terminate_status::UNKNOWN;

    std::cout << "cost low bound at beginning is "
              << robot->cost_lower_bound(startState, goalState) << std::endl;

    bool print_queue = false;
    int print_every = 100;

    if (options_dbastar.add_node_if_better) {
      // didn't check if this works togehter
      DYNO_CHECK_EQ(options_dbastar.duplicate_detection_int,
               static_cast<int>(Duplicate_detection::NO), AT);
    }

    double radius = options_dbastar.delta * (1. - options_dbastar.alpha);

    std::vector<std::vector<double>> data_out_query_Tm;
    std::vector<int> num_nn_motions;
    int max_num_nn_motions = 0;
    std::vector<Motion *> nn_of_best;
    ob::State *state_more_nn = si->getStateSpace()->allocState();

    auto nearest_motion_timed = [&](auto &fakeMotion, auto &neighbors_m) {
      std::vector<double> real;

      bool verbose = true;

      if (options_dbastar.debug) {
        si->getStateSpace()->copyToReals(real, fakeMotion.getState());
        data_out_query_Tm.push_back(real);
      }

      auto out = timed_fun([&] {
        T_m->nearestR(&fakeMotion,
                      options_dbastar.delta * options_dbastar.alpha,
                      neighbors_m);

        num_nn_motions.push_back(neighbors_m.size());
        if (neighbors_m.size() > max_num_nn_motions) {
          max_num_nn_motions = neighbors_m.size();
          si->getStateSpace()->copyState(state_more_nn, fakeMotion.getState());
          nn_of_best = neighbors_m;
        }

        if (!neighbors_m.size() && true) {

          std::cout << "no neighours for state " << std::endl;
          si->printState(fakeMotion.getState(), std::cout);

          std::cout << "close state is  " << std::endl;
          auto close_motion = T_m->nearest(&fakeMotion);
          si->printState(close_motion->getState(), std::cout);
          std::cout << std::endl;

          std::cout << "close distance is:  "
                    << si->distance(close_motion->getState(),
                                    fakeMotion.getState())
                    << std::endl;
          std::cout << "R is " << options_dbastar.delta * options_dbastar.alpha
                    << std::endl;
        }

        return 0;
      });
      time_bench.time_nearestMotion += out.second;
      time_bench.num_nn_motions++;
    };

    auto nearestR_state_timed = [&](auto &query_n, auto &neighbors_n) {
      auto _out = timed_fun([&] {
        T_n->nearestR(query_n, radius, neighbors_n);
        return 0;
      });
      time_bench.num_nn_states++;
      time_bench.time_nearestNode += _out.second;
      time_bench.time_nearestNode_search += _out.second;
    };

    auto is_motion_valid_timed = [&](auto &motion, auto &offset,
                                     bool &motionValid, Trajectory &traj) {
      if (options_dbastar.check_cols) {
        if (options_dbastar.use_collision_shape) {
          // robot->diff_model->invariance_reuse_col_shape) {

          auto out = timed_fun([&] {
            motion->collision_manager->shift(offset);
            fcl::DefaultCollisionData<double> collision_data;
            motion->collision_manager->collide(
                robot->diff_model->env.get(), &collision_data,
                fcl::DefaultCollisionFunction<double>);
            bool motionValid = !collision_data.result.isCollision();
            motion->collision_manager->shift(-offset);
            return motionValid;
          });

          motionValid = out.first;
          time_bench.time_collisions += out.second;
          time_bench.num_col_motions++;
        } else {
          // check all the configuration, starting by the middle

          CHECK(traj.states.size(), AT);

          Stopwatch watch;

          size_t index_start = 0;
          size_t index_last = motion->states.size() - 1;

          // check the first and last state

          size_t nx = robot->diff_model->nx;
          Eigen::VectorXd x(nx);
          x = traj.states.front();

          // robot->toEigen(motion->states.front(), x);

          bool start_good = false;
          bool goal_good = false;
          if (robot->diff_model->collision_check(x)) {
            start_good = true;
          }
          // robot->toEigen(motion->states.back(), x);
          x = traj.states.back();
          if (robot->diff_model->collision_check(x)) {
            goal_good = true;
          }

          if (start_good && goal_good) {

            using Segment = std::pair<size_t, size_t>;
            std::queue<Segment> queue;

            queue.push(Segment{index_start, index_last});

            size_t index_resolution = 1;

            if (robot->diff_model->ref_dt < .05) {
              // TODO: which number to put here?
              index_resolution = 5;
            }

            // I could use a spatial resolution also...

            motionValid = true;
            while (!queue.empty()) {

              auto [si, gi] = queue.front();
              queue.pop();

              if (gi - si > index_resolution) {

                // check if they are very close -> HOW exactly?
                // auto &gix = traj.states.at(gi);
                // auto &six = traj.states.at(si);

                size_t ii = int((si + gi) / 2);

                if (ii == si || ii == gi) {
                  continue;
                }
                // robot->toEigen(motion->states.at(ii), x);
                x = traj.states.at(ii);
                if (robot->diff_model->collision_check(x)) {
                  if (ii != si)
                    queue.push(Segment{ii, gi});
                  if (ii != gi)
                    queue.push(Segment{si, ii});
                } else {
                  motionValid = false;
                  break;
                }
              }
            }
          } else {
            motionValid = false;
          }

          time_bench.time_collisions += watch.elapsed_ms();
          time_bench.num_col_motions++;
        }
      } else {
        motionValid = true;
      }
    };

    auto add_state_timed = [&](auto &node) {
      auto out = timed_fun([&] {
        T_n->add(node);
        return 0;
      });
      time_bench.time_nearestNode += out.second;
      time_bench.time_nearestNode_add += out.second;
    };

    if (!options_dbastar.use_landmarks && !options_dbastar.add_after_expand) {
      if (options_dbastar.debug) {
        std::vector<double> _state;
        si->getStateSpace()->copyToReals(_state, start_node->state);
        _states_debug.push_back(_state);
      }
      add_state_timed(start_node);
    }

    if (options_dbastar.add_after_expand) {
      CHECK(!options_dbastar.add_node_if_better, AT);
    }

    // std::cout << "MOTIONS" << std::endl;
    // std::vector<Motion *> mss;
    // T_m->list(mss);
    // for (auto &ms : mss) {
    //   si->printState(ms->states.at(0));
    // }
    // std::cout << "END MOTIONS" << std::endl;

    auto tac = std::chrono::high_resolution_clock::now();
    time_bench.prepare_time =
        std::chrono::duration<double, std::milli>(tac - tic).count();
#if 0
  static_cast<NearestNeighborsGNATNoThreadSafety_public<AStarNode *> *>(T_n)
      ->set_rebuildSize_(1e8);
#endif

    // check how apart are the primivites!

    std::vector<double> min_distance_primitives(T_m->size());

    std::vector<Motion *> __ms;
    T_m->list(__ms);

    Motion __fakeMotion;

    auto __tmp_state = si->getStateSpace()->allocState();

    CSTR_(robot->isTranslationInvariant());

    double max_distance = 0;
    Motion *ptr_motion_with_max_distance = nullptr;
    Motion *ptr_motion_with_max_distance_nn = nullptr;

    // continue here: for some unk reason, it does not expand states!!

    const bool debug_check_distance_primitives = false;
    if (debug_check_distance_primitives) {
      std::transform(
          __ms.begin(), __ms.end(), min_distance_primitives.begin(),
          [&](auto &p) {
            si->getStateSpace()->copyState(__tmp_state, p->states.back());
            __fakeMotion.states = {__tmp_state};
            if (robot->isTranslationInvariant())
              robot->setPosition(__fakeMotion.states[0],
                                 fcl::Vector3d(0, 0, 0));

            std::vector<Motion *> ns;

            // std::cout << "p:" << std::endl;
            // p->print(std::cout, si);
            // std::cout << "fake motion: " << std::endl;
            // __fakeMotion.print(std::cout, si);

            T_m->nearestK(&__fakeMotion, 1, ns);
            // DYNO_CHECK_LEQ(
            //     si->distance(ns.front()->getState(),
            //     p->getState()), 1e-10, AT);

            double d =
                si->distance(ns.at(0)->getState(), __fakeMotion.getState());

            if (d > max_distance) {
              ptr_motion_with_max_distance = p;
              ptr_motion_with_max_distance_nn = ns.at(0);
              max_distance = d;
            }

            if (d < 1e-3) {
              std::cout << "two states are very close " << std::endl;
              si->getStateSpace()->printState(ns.at(0)->getState());
              si->getStateSpace()->printState(__fakeMotion.getState());

              // std::cout << "nn motion: " << std::endl;
              // ns.at(0)->print(std::cout, si);
              //
              // std::cout << "p motion: " << std::endl;
              // p->print(std::cout, si);
            }
            return d;
          });

      std::cout << "Far motion: " << std::endl;
      ptr_motion_with_max_distance->print(std::cout, si);

      std::cout << "Far motion NN: " << std::endl;
      ptr_motion_with_max_distance_nn->print(std::cout, si);

      double __min = *std::min_element(min_distance_primitives.begin(),
                                       min_distance_primitives.end());
      double __max = *std::max_element(min_distance_primitives.begin(),
                                       min_distance_primitives.end());
      double __sum = std::accumulate(min_distance_primitives.begin(),
                                     min_distance_primitives.end(), 0.);

      std::cout << "REPORT NN inside primitives" << std::endl;

      CSTR_(T_m->size());
      CSTR_(__min);
      CSTR_(__max);
      CSTR_(__sum / min_distance_primitives.size());
    }

    // record the average branching factor

    auto start = std::chrono::steady_clock::now();
    auto get_time_stamp_ms = [&] {
      return static_cast<double>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - start)
              .count());
    };

    double min_distance_to_goal = std::numeric_limits<double>::max();
    ob::State *best_state = si->getStateSpace()->allocState();

    // TODO: do we need a close list?
    std::vector<AStarNode *> closed_list;

    Trajectory tmp_traj;

    int final_number_of_primitives = T_m->size();

    Eigen::VectorXd goalState_eig(robot->diff_model->nx);
    Eigen::VectorXd __current_state(robot->diff_model->nx);
    Eigen::VectorXd __canonical_state(robot->diff_model->nx);
    Eigen::VectorXd offsete(robot->diff_model->get_offset_dim());
    Eigen::VectorXd offseteX(robot->diff_model->get_offset_dim());
    Eigen::VectorXd ___current_state(robot->diff_model->nx);

    robot->toEigen(goalState, goalState_eig);

    Stopwatch watch;
    std::random_device rd;
    std::mt19937 g(rd());
    const bool plot_the_search_tree = false;
    while (true) {
      if (plot_the_search_tree && time_bench.expands == 1000) {
        std::cout << "warning: plot the search tree" << std::endl;
        std::vector<AStarNode *> nodes;
        T_n->list(nodes);
        for (auto &n : nodes) {
          CHECK(n->state, AT);
          n->state_eig.resize(robot->diff_model->nx);
          robot->toEigen(n->state, n->state_eig);
        }

        plot_search_tree(nodes, motions, *robot->diff_model,
                         "/tmp/dynoplan/tree.yaml");
      }

      if (static_cast<size_t>(time_bench.expands) >=
          options_dbastar.max_expands) {
        status = Terminate_status::MAX_EXPANDS;
        std::cout << "BREAK search:"
                  << "MAX_EXPANDS" << std::endl;
        break;
      }

      if (open.empty()) {
        status = Terminate_status::EMPTY_QUEUE;
        std::cout << "BREAK search:"
                  << "EMPTY_QUEUE" << std::endl;
        break;
      }

      if (get_time_stamp_ms() > options_dbastar.search_timelimit) {
        status = Terminate_status::MAX_TIME;
        std::cout << "BREAK search:"
                  << "MAX_TIME" << std::endl;
        break;
      }

      if (print_queue) {
        std::cout << "QUEUE " << std::endl;

        for (auto &o : open) {
          std::cout << " *** " << std::endl;
          std::cout << o->fScore << std::endl;
          std::cout << o->gScore << std::endl;
          std::cout << o->hScore << std::endl;
          printState(std::cout, si, o->state);
          std::cout << "\n*** " << std::endl;
        }
        std::cout << "END QUEUE" << std::endl;
      }

      Stopwatch __sw;
      AStarNode *current = open.top();
      open.pop();
      time_bench.time_queue += __sw.elapsed_ms();
      closed_list.push_back(current);

      // std::cout << "current state ";
      // printState(std::cout, si, current->state);
      // std::cout << std::endl;

      expanded.push_back(current->state);
      ++time_bench.expands;

      if (options_dbastar.add_after_expand) {
        // i have to check if state is novel

        std::vector<AStarNode *> neighbors_n;
        nearestR_state_timed(current, neighbors_n);
        if (neighbors_n.size()) {
          continue;
        } else {
          // add the node to the tree
          add_state_timed(current);
        }
      }

      if (time_bench.expands % print_every == 0 || time_bench.expands == 1) {
        std::cout << "expanded: " << time_bench.expands
                  << " open: " << open.size() << " nodes: " << T_n->size()
                  << " f-score " << current->fScore << " h-score "
                  << current->hScore << " g-score " << current->gScore
                  << std::endl;
      }

      if (time_bench.expands % options_dbastar.rebuild_every == 0 &&
          !options_dbastar.use_landmarks) {

        auto out = timed_fun([&] {
#if 0
        auto p_derived = static_cast<
            NearestNeighborsGNATNoThreadSafety_public<AStarNode *> *>(T_n);
        p_derived->rebuildDataStructure();
        p_derived->set_rebuildSize_(1e8);
#endif
          return 0;
        });
        time_bench.time_nearestNode += out.second;
        time_bench.time_nearestNode_add += out.second;
      }

      if (options_dbastar.heuristic == 0) {

        if (current->fScore < last_f_score) {
          std::cout << "expand: " << time_bench.expands << " state: ";
          si->printState(current->state);
          std::cout << "  -- WARNING, assert(current->fScore >= last_f_score) "
                    << current->fScore << " " << last_f_score << std::endl;
          // throw -1;
        }
      }
      last_f_score = current->fScore;
      // si->printState(current->state);
      // si->printState(goalState);
      double distance_to_goal = si->distance(current->state, goalState);
      if (distance_to_goal < min_distance_to_goal) {
        min_distance_to_goal = distance_to_goal;
        si->getStateSpace()->copyState(best_state, current->state);
      }

      if (distance_to_goal <=
          options_dbastar.delta_factor_goal * options_dbastar.delta) {
        solution = current;
        std::cout << "current state is " << std::endl;
        printState(std::cout, si, current->state);
        std::cout << std::endl;
        status = Terminate_status::SOLVED;
        std::cout << "SOLUTION FOUND!!!! cost: " << current->gScore
                  << std::endl;
        break;
      }

      current->is_in_open = false;

      si->copyState(fakeMotion.states[0], current->state);

      // CHANGE THIS
      if (!options_dbastar.new_invariance) {
        if (robot->isTranslationInvariant())
          robot->setPosition(fakeMotion.states[0], fcl::Vector3d(0, 0, 0));
      } else {
        // new
        robot->toEigen(current->state, __current_state);
        robot->diff_model->canonical_state(__current_state, __canonical_state);
        robot->fromEigen(fakeMotion.states[0], __canonical_state);
      }

      nearest_motion_timed(fakeMotion, neighbors_m);

#if 0
    std::cout << "current state state " << std::endl;
    si->printState(fakeMotion.states.front());
    std::cout << "reporting distances" << std::endl;
    std::vector<Motion *> mss;
    T_m->list(mss);
    std::cout << "mss.size() " << mss.size() << std::endl;
    double min_distance = std::numeric_limits<double>::max();

    for (auto &ms : mss) {
      si->printState(ms->states.at(0));
      double distance = si->distance(fakeMotion.states[0], ms->states.at(0));
      std::cout << "dist "
                << si->distance(fakeMotion.states[0], ms->states.at(0))
                << std::endl;
      if (distance < min_distance)
        min_distance = distance;
    }
    std::cout << "min distance is " << min_distance << std::endl;
    std::cout << "num neighbors " << std::endl;
    std::cout << STR_(neighbors_m.size()) << std::endl;
#endif

      bool has_children = false;

      // use a limit on the branching factor!

      std::shuffle(neighbors_m.begin(), neighbors_m.end(), g);

      if (!neighbors_m.size()) {
        std::cout << "state:" << std::endl;
        si->printState(current->state);
        std::cout << "has neighbors=0" << std::endl;
      }

      int expanded_neighs = 0;

      for (const Motion *motion : neighbors_m) {
        // std::cout << "motion from ";
        // si->getStateSpace()->printState(motion->getState());
        // std::cout << " to ";
        // si->getStateSpace()->printState(motion->states.back());
        // std::cout << std::endl;
        if (motion->disabled) {
          continue;
        }

        if (expanded_neighs >= options_dbastar.limit_branching_factor) {
          break;
        }

        fcl::Vector3d offset(0., 0., 0.);
        fcl::Vector3d computed_offset(0., 0., 0.);
        fcl::Vector3d current_pos(0., 0., 0.);

        Eigen::VectorXd tmp_state_e;

        double tentative_gScore = current->gScore + motion->cost;
        si->copyState(tmpState, motion->states.back());
        si->copyState(tmpState2, motion->states.front());

        if (!options_dbastar.new_invariance) {
          if (robot->isTranslationInvariant()) {
            current_pos = robot->getTransform(current->state).translation();
            offset = current_pos + computed_offset;
            const auto relative_pos =
                robot->getTransform(tmpState).translation();
            robot->setPosition(tmpState, offset + relative_pos);
            robot->setPosition(tmpState2, current_pos);
          }
        } else {

          robot->toEigen(current->state, ___current_state);
          robot->diff_model->offset(___current_state, offsete);
          tmp_traj.states = motion->traj.states;
          tmp_traj.actions = motion->traj.actions;

          robot->diff_model->transform_primitive(
              offsete, motion->traj.states, motion->traj.actions,
              tmp_traj.states, tmp_traj.actions);

          robot->fromEigen(tmpState, tmp_traj.states.back());
          robot->fromEigen(tmpState2, tmp_traj.states.front());

          // which is the current offset?
          robot->diff_model->offset(tmp_traj.states.front(), offseteX);
          offset.head(robot->diff_model->translation_invariance) =
              offseteX.head(robot->diff_model->translation_invariance);
        }

        if (!options_dbastar.new_invariance) {
          if (!si->satisfiesBounds(tmpState)) {
            // todo: CHECK all the states!!
            // std::cout << "Not satisfies bounds " << std::endl;
            continue;
          }
        } else {
          // check all the states in the trajectory

          bool invalid = false;
          for (auto &state : tmp_traj.states) {
            if (!robot->diff_model->is_state_valid(state)) {
              invalid = true;
              break;
            }
          }
          if (invalid)
            continue;
        }

        // check all the stte

        // std::cout << "states:" << std::endl;
        // si->printState(tmpState2);
        // si->printState(current->state);

        tentative_gScore += options_dbastar.cost_delta_factor *
                            robot->cost_lower_bound(tmpState2, current->state);

        __watch_h_fun.reset();
        double tentative_hScore = h_fun->h(tmpState);
        time_bench.time_hfun += __watch_h_fun.elapsed_ms();

        double tentative_fScore =
            tentative_gScore + options_dbastar.epsilon * tentative_hScore;

        if (tentative_fScore > options_dbastar.maxCost) {
          continue;
        }

        bool motionValid;

        is_motion_valid_timed(motion, offset, motionValid, tmp_traj);

        if (!motionValid) {
          // std::cout << "Not Valid because of collisions" << std::endl;
          continue;
        }
        has_children = true;

        expanded_neighs++;

        // check if final states is close to the goal:

        const bool check_intermediate_goal = true;
        if (check_intermediate_goal && options_dbastar.new_invariance) {

          size_t num_check_goal = 4; // the last state is not considered

          // n = 4
          // 0 1 2 3
          // 1/4 2/4 3/4 4/4
          //
          // I check:
          // 1/5 , 2/5 , 3/5 , 4/5
          //

          Eigen::VectorXd intermediate_sol(robot->diff_model->nx);
          for (size_t nn = 0; nn < num_check_goal; nn++) {

            size_t index_to_check =
                float(nn + 1) / (num_check_goal + 1) * tmp_traj.states.size();

            double d = robot->diff_model->distance(
                tmp_traj.states.at(index_to_check), goalState_eig);

            if (d <=
                options_dbastar.delta_factor_goal * options_dbastar.delta) {
              std::cout << "Found a solution -- intermetidate checks"
                        << std::endl;
              CSTR_(nn);
              CSTR_(index_to_check);
              std::cout << "Final state is " << std::endl;
              CSTR_V(tmp_traj.states.at(index_to_check));

              intermediate_sol = tmp_traj.states.at(index_to_check);

              CSTR_(motion->idx);
              std::cout << "Full trajectory is " << std::endl;

              tmp_traj.to_yaml_format(std::cout);

              // what to do now?

              // include this node  in the queue

              // create a new ompl mode
              ob::State *middle_state = si->allocState();
              robot->fromEigen(middle_state, intermediate_sol);
              auto node = new AStarNode();
              node->state = si->cloneState(middle_state);

              // TODO: cost only works for time optimization -- CHANGE!
              node->gScore =
                  current->gScore +
                  options_dbastar.cost_delta_factor *
                      robot->cost_lower_bound(tmpState2, current->state) +
                  robot->diff_model->ref_dt * index_to_check;

              node->fScore = node->hScore + node->gScore;

              node->intermediate_state = index_to_check;
              node->came_from = current;
              node->used_motion = motion->idx;
              node->used_offset = computed_offset;
              node->is_in_open = true;

              Stopwatch _sw;
              auto handle = open.push(node);

              time_bench.time_queue += _sw.elapsed_ms();
              node->handle = handle;

              add_state_timed(node);

              si->freeState(middle_state);
            }
          }
        }

        query_n->state = tmpState;

        if (options_dbastar.debug) {

          std::vector<double> _state;
          si->getStateSpace()->copyToReals(_state, query_n->state);
          _states_debug.push_back(_state);

          std::vector<std::vector<double>> motion_v;

          for (size_t k = 0; k < motion->states.size(); ++k) {
            const auto state = motion->states[k];
            si->copyState(tmpState_debug, state);
            if (robot->isTranslationInvariant()) {
              const fcl::Vector3d relative_pos =
                  robot->getTransform(state).translation();
              robot->setPosition(tmpState_debug, current_pos + relative_pos);
            }
            std::vector<double> x;
            si->getStateSpace()->copyToReals(x, tmpState_debug);
            motion_v.push_back(x);
          }
          expansions.push_back(motion_v);
        }

        if (!options_dbastar.add_after_expand) {

          nearestR_state_timed(query_n, neighbors_n);

          if ((neighbors_n.size() == 0 && !options_dbastar.use_landmarks) ||
              options_dbastar.always_add) {

            bool add_node = true;

            CHECK((duplicate_detection == Duplicate_detection::NO), AT);
#if 0
          if (duplicate_detection == Duplicate_detection::HARD ||
              duplicate_detection == Duplicate_detection::SOFT) {

            std::vector<AStarNode *> neighbors_n2;
            T_n->nearestR(query_n,
                          options_dbastar.factor_duplicate_detection *
                              radius,
                          neighbors_n2);

            if (neighbors_n2.size()) {
              if (duplicate_detection == Duplicate_detection::HARD) {
                add_node = false;
              } else if (duplicate_detection == Duplicate_detection::SOFT) {

                bool is_best = true;
                tentative_fScore = tentative_hScore + tentative_gScore;

                for (auto &n : neighbors_n2) {
                  if (n->fScore < tentative_fScore) {
                    is_best = false;
                    break;
                  }
                }

                if (!is_best) {
                  tentative_hScore *=
                      options_dbastar.epsilon_soft_duplicate;
                  tentative_fScore = tentative_hScore + tentative_gScore;
                }
              }
            }
          }
#endif

            if (add_node) {
              auto node = new AStarNode();
              node->state = si->cloneState(tmpState);
              node->gScore = tentative_gScore;
              node->fScore = tentative_fScore;
              node->hScore = tentative_hScore;
              node->came_from = current;
              node->used_motion = motion->idx;
              node->used_offset = computed_offset;
              node->is_in_open = true;

              Stopwatch _sw;
              auto handle = open.push(node);
              time_bench.time_queue += _sw.elapsed_ms();
              node->handle = handle;

              add_state_timed(node);
            }
          } else {
            // T_n->nearestR(query_n, radius, neighbors_n);
            // check if we have a better path now
            bool added = false;
            bool test_equivalence_class = false;
            if (test_equivalence_class) {
              CHECK(false, AT);
            }

            for (AStarNode *entry : neighbors_n) {

              if (options_dbastar.debug) {
                std::vector<double> landmark_;
                si->getStateSpace()->copyToReals(landmark_, entry->state);
                found_nn.push_back(landmark_);
              }

              // AStarNode* entry = nearest;

              DYNO_CHECK_LEQ(si->distance(entry->state, tmpState),
                        options_dbastar.delta, AT);
              double extra_time_to_reach =
                  options_dbastar.cost_delta_factor *
                  robot->cost_lower_bound(entry->state, tmpState);
              double tentative_gScore_ = tentative_gScore + extra_time_to_reach;
              double delta_score = entry->gScore - tentative_gScore_;
              // double old_fscore = entry->fScore;
              // std::cout  << "time to reach " << time_to_reach <<
              // std::endl;

              if (delta_score > 0) {

                if (options_dbastar.add_node_if_better) {
                  if (!entry->valid)
                    continue;
                  if (!added) {

                    // std::cout << "adding state that is close"  <<
                    // std::endl; si->printState(tmpState);

                    auto node = new AStarNode();
                    node->state = si->cloneState(tmpState);
                    node->gScore = tentative_gScore;
                    node->fScore = tentative_fScore;
                    node->hScore = tentative_hScore;
                    node->came_from = current;
                    node->used_motion = motion->idx;
                    node->used_offset = computed_offset;
                    node->is_in_open = true;
                    Stopwatch _sw;
                    auto handle = open.push(node);
                    time_bench.time_queue += _sw.elapsed_ms();
                    node->handle = handle;

                    add_state_timed(node);
                    added = true;
                  }
                  // I invalidate the neighbour
                  if (entry->is_in_open) {
                    entry->fScore = std::numeric_limits<double>::max();
                    entry->gScore = std::numeric_limits<double>::max();
                    entry->hScore = std::numeric_limits<double>::max();
                    entry->valid = false;

                    Stopwatch _sw;
                    open.decrease(entry->handle);
                    time_bench.time_queue += _sw.elapsed_ms();
                  }
                } else {
                  entry->gScore = tentative_gScore_;
                  // entry->fScore = tentative_fScore;

                  __watch_h_fun.reset();
                  entry->hScore = h_fun->h(entry->state);
                  time_bench.time_hfun += __watch_h_fun.elapsed_ms();

                  // or entry state?
                  // entry->hScore = h_fun->h(tmpState);
                  entry->fScore =
                      entry->gScore + options_dbastar.epsilon * entry->hScore;
                  assert(entry->fScore >= 0);
                  entry->came_from = current;
                  entry->used_motion = motion->idx;
                  entry->used_offset = computed_offset;
                  if (entry->is_in_open) {
                    // std::cout << "improve score  -- old: " <<
                    // old_fscore
                    //           << " -- new -- " << entry->fScore <<
                    //           std::endl;

                    Stopwatch _sw;
                    open.increase(entry->handle); // increase? decrease? check
                                                  // the original implementation
                    time_bench.time_queue += _sw.elapsed_ms();
                  } else {
                    Stopwatch _sw;
                    auto handle = open.push(entry);
                    time_bench.time_queue += _sw.elapsed_ms();
                    entry->handle = handle;
                    entry->is_in_open = true;
                  }
                }
              }
            }
          }
        } else {
          // I directly create a node and add to queue
          auto node = new AStarNode();
          node->state = si->cloneState(tmpState);
          node->gScore = tentative_gScore;
          node->fScore = tentative_fScore;
          node->hScore = tentative_hScore;
          node->came_from = current;
          node->used_motion = motion->idx;
          node->used_offset = computed_offset;
          node->is_in_open = true;

          Stopwatch _sw;
          auto handle = open.push(node);
          time_bench.time_queue += _sw.elapsed_ms();
          node->handle = handle;
        }
      }
      if (!has_children) {
        // std::cout << "state:" << std::endl;
        // si->printState(current->state);
        // std::cout << "has_children=0 and ";
        // CSTR_(neighbors_m.size());
      }
    }

    std::cout << "Closer state:" << std::endl;
    si->getStateSpace()->printState(best_state, std::cout);
    CSTR_(min_distance_to_goal);

    time_bench.time_search = watch.elapsed_ms();
    std::cout << "TIME in search:" << time_bench.time_search << std::endl;

    // clock end
    tac = std::chrono::high_resolution_clock::now();
    time_bench.total_time =
        std::chrono::duration<double, std::milli>(tac - tic).count();

    std::cout << "search has ended " << std::endl;

    if (options_dbastar.debug) {
      {
        std::string filename1 = "/tmp/dynoplan/data_out_query_Tm.txt";
        create_dir_if_necessary(filename1);
        std::ofstream out(filename1);
        print_matrix(out, data_out_query_Tm);

        std::string filename2 = "/tmp/dynoplan/data_out_Tm.txt";
        create_dir_if_necessary(filename2);
        std::ofstream out2(filename2);
        std::vector<Motion *> m;
        T_m->list(m);
        std::vector<std::vector<double>> data_out_Tm(m.size());

        std::transform(m.begin(), m.end(), data_out_Tm.begin(), [&](auto &s) {
          std::vector<double> reals;
          si->getStateSpace()->copyToReals(reals, s->getState());
          return reals;
        });

        print_matrix(out2, data_out_Tm);
      }

      {

        std::string filename1 = "/tmp/dynoplan/data_Tn.txt";
        create_dir_if_necessary(filename1);

        std::ofstream out(filename1);

        std::vector<AStarNode *> m;
        T_n->list(m);
        std::vector<std::vector<double>> data_Tn(m.size());

        std::transform(m.begin(), m.end(), data_Tn.begin(), [&](auto &s) {
          std::vector<double> reals;
          si->getStateSpace()->copyToReals(reals, s->getState());
          return reals;
        });

        print_matrix(out, data_Tn);
      }
    }

    int nn_motions_sum =
        std::accumulate(num_nn_motions.begin(), num_nn_motions.end(), 0);

    int nn_motions_min =
        *std::min_element(num_nn_motions.begin(), num_nn_motions.end());
    int nn_motions_max =
        *std::max_element(num_nn_motions.begin(), num_nn_motions.end());

    std::cout << "state with more neighors " << std::endl;
    si->getStateSpace()->printState(state_more_nn);
    std::cout << std::endl;
    std::cout << "neighs=" << nn_of_best.size() << std::endl;
    // for (auto &n : nn_of_best) {
    //   std::cout << "first" << std::endl;
    //   si->getStateSpace()->printState(n->getState());
    //   std::cout << "last" << std::endl;
    //   si->getStateSpace()->printState(n->states.back());
    // }

    double nn_motions_average = double(nn_motions_sum) / num_nn_motions.size();
    CSTR_(nn_motions_average);
    CSTR_(nn_motions_min);
    CSTR_(nn_motions_max);

    CSTR_(time_bench.expands);
    std::cout << "Terminate status: " << static_cast<int>(status) << " "
              << terminate_status_str[static_cast<int>(status)] << std::endl;

    time_bench.motions_tree_size = T_m->size();
    time_bench.states_tree_size = T_n->size();

    create_dir_if_necessary(options_dbastar.outFile.c_str());

    std::cout << "writing output to " << options_dbastar.outFile << std::endl;

    std::ofstream out(options_dbastar.outFile);
    out << "status: " << terminate_status_str[static_cast<int>(status)]
        << std::endl;
    out << "time_search: " << time_bench.time_search << std::endl;
    // out <<
    //
    //
    // CSTR_(nn_motions_sum / num_nn_motions.size());
    // CSTR_(

    out << "nn_motions_min: " << nn_motions_min << std::endl;
    out << "nn_motions_max: " << nn_motions_max << std::endl;
    out << "nn_motions_average: " << nn_motions_average << std::endl;

    out << "start: ";
    printState(out, si, startState);
    out << std::endl;
    out << "goal: ";
    printState(out, si, goalState);
    out << std::endl;
    out << "solved: " << !(!solution) << std::endl;

    options_dbastar.print(out);
    time_bench.write(out);

    std::cout << "time_bench:" << std::endl;
    time_bench.write(std::cout);
    std::cout << std::endl;

    if (options_dbastar.debug) {
      if (options_dbastar.heuristic == 1) {
#if 0
      out << "batch:" << std::endl;
      auto fun_d = static_cast<Heu_roadmap *>(h_fun.get());
      for (auto &x : fun_d->batch_samples) {
        out << "      - ";
        printState(out, si, x);
        out << std::endl;
      }
#endif
      }

      out << "kdtree:" << std::endl;
      std::vector<AStarNode *> nodes_in_tree;
      T_n->list(nodes_in_tree);

      for (auto &x : nodes_in_tree) {
        out << space6 + "- ";
        printState(out, si, x->state);
        out << std::endl;
      }

      out << "expanded:" << std::endl;
      for (auto &x : expanded) {
        out << space6 + "- ";
        printState(out, si, x);
        out << std::endl;
      }

      out << "found_nn:" << std::endl;
      size_t max_nn = 1000;
      size_t it_nn = 0;
      for (auto &x : found_nn) {
        if (it_nn++ > max_nn)
          break;
        out << space6 + "- ";
        printState(out, x);
        out << std::endl;
      }

      out << "expansions:" << std::endl;
      size_t max_expansion = 1000;
      size_t it_expansion = 0;
      for (auto &e : expansions) {
        if (it_expansion++ > max_expansion)
          break;
        out << space6 + "- " << std::endl;
        for (auto &x : e) {
          out << space6 + "  - ";
          printState(out, x);
          out << std::endl;
        }
      }
    }

    double time_jumps = 0;
    double cost_with_jumps = 0;

    if (!solution) {
      // check which is the closest node in the tree (TODO: think if this should
      // be the queue!)
      AStarNode node;
      node.state = goalState;
      auto nn = T_n->nearest(&node);
      std::cout << "No solution" << std::endl;
      std::cout << "closest node in tree is:" << std::endl;
      si->getStateSpace()->printState(nn->state);
      double d = si->distance(nn->state, goalState);
      std::cout << "distance: " << d << std::endl;

      if (d < options_dbastar.delta_factor_goal * options_dbastar.delta) {
        std::cout << "One node in the tree is close the goal! -- We have "
                     "feasible solution "
                  << std::endl;
        solution = nn;
      }
    }

    // TODO: this only works with offset!!
    if (solution) {
      state_to_eigen(traj_out.start, si, startState);
      state_to_eigen(traj_out.goal, si, goalState);

      out << "cost: " << solution->gScore << std::endl;

      double extra_time_weighted =
          options_dbastar.cost_delta_factor *
          robot->cost_lower_bound(solution->state, goalState);
      std::cout << "extra time " << extra_time_weighted << std::endl;

      std::cout << "solution with extra time with cost-to-goal: "
                << solution->gScore + extra_time_weighted << std::endl;

      std::vector<const AStarNode *> result;

      const AStarNode *n = solution;
      while (n != nullptr) {
        result.push_back(n);
        // std::cout << n->used_motion << std::endl;
        // si->printState(n->state);
        n = n->came_from;
      }
      std::reverse(result.begin(), result.end());

      std::cout << "result size " << result.size() << std::endl;

      if (options_dbastar.debug) {
        std::string filename = "/tmp/dynoplan/state_out.txt";
        create_dir_if_necessary(filename);
        std::ofstream out_states(filename);
        print_matrix(out_states, _states_debug);
      }

      out << "result:" << std::endl;
      out << "  - states:" << std::endl;

      si->copyState(tmpStateq, startState);

      if (!options_dbastar.new_invariance) {
        for (size_t i = 0; i < result.size() - 1; ++i) {
          // Compute intermediate states
          const auto node_state = result[i]->state;
          fcl::Vector3d current_pos(0, 0, 0);

          if (robot->isTranslationInvariant())
            current_pos = robot->getTransform(node_state).translation();
          const auto &motion = motions.at(result[i + 1]->used_motion);
          out << space6 + "# ";
          printState(out, si, node_state); // debug
          out << std::endl;
          out << space6 + "# motion " << motion.idx << " with cost "
              << motion.cost << std::endl; // debug
          // skip last state each

          for (size_t k = 0; k < motion.states.size(); ++k) {
            const auto state = motion.states[k];
            si->copyState(tmpState, state);

            if (robot->isTranslationInvariant()) {
              const fcl::Vector3d relative_pos =
                  robot->getTransform(state).translation();
              robot->setPosition(tmpState, current_pos +
                                               result[i + 1]->used_offset +
                                               relative_pos);
            }

            if (k < motion.states.size() - 1) {
              if (k == 0) {
                out << space6 + "# jump from ";
                printState(out, si, tmpStateq); // debug
                out << " to ";
                printState(out, si, tmpState);                         // debug
                out << " delta " << si->distance(tmpStateq, tmpState); // debug
                double min_time = robot->cost_lower_bound(tmpStateq, tmpState);
                time_jumps += min_time;
                out << " min time " << min_time; // debug
                out << std::endl;
              }

              out << space6 + "- ";
              Eigen::VectorXd x;
              state_to_eigen(x, si, tmpState);
              traj_out.states.push_back(x);

            } else {
              out << space6 + "# "; // debug
              si->copyState(tmpStateq, tmpState);
            }

            printState(out, si, tmpState);

            out << std::endl;
          }
          out << std::endl;
        }
        out << space6 + "- ";

        // printing the last state
        Eigen::VectorXd x;
        state_to_eigen(x, si, tmpState);
        traj_out.states.push_back(x);
        printState(out, si, result.back()->state);
        out << std::endl;
        std::cout << " time jumps " << time_jumps << std::endl;
      } else {
#if 1

        auto &mm = robot->diff_model;

        Eigen::VectorXd __tmp(robot->diff_model->nx);
        Eigen::VectorXd __offset(robot->diff_model->get_offset_dim());
        for (size_t i = 0; i < result.size() - 1; ++i) {
          const auto node_state = result[i]->state;
          const auto &motion = motions.at(result[i + 1]->used_motion);
          int take_until = result[i + 1]->intermediate_state;
          if (take_until != -1) {
            out << space6 + "# (note: we have stopped at intermediate state) "
                << std::endl;
          }
          out << space6 + "# (node_state) ";
          printState(out, si, node_state); // debug
          out << std::endl;
          out << space6 + "# motion " << motion.idx << " with cost "
              << motion.cost << std::endl; // debug
          out << space6 + "# motion first state "
              << motion.traj.states.front().format(FMT) << std::endl;
          out << space6 + "# motion last state "
              << motion.traj.states.back().format(FMT) << std::endl;
          //
          //
          //
          //
          // transform the motion to match the state

          // get the motion
          robot->toEigen(node_state, __tmp);
          robot->diff_model->offset(__tmp, __offset);
          out << space6 + "# (tmp) " << __tmp.format(FMT) << std::endl;
          out << space6 + "# (offset) " << __offset.format(FMT) << std::endl;
          ;

          auto &traj = motion.traj;
          std::vector<Eigen::VectorXd> xs = traj.states;
          std::vector<Eigen::VectorXd> us = traj.actions;
          robot->diff_model->transform_primitive(__offset, traj.states,
                                                 traj.actions, xs, us);
          // TODO: missing additional offset, if any

          out << space6 + "# (traj.states.front) "
              << traj.states.front().format(FMT) << std::endl;
          out << space6 + "# (xs.front) " << xs.front().format(FMT)
              << std::endl;

          size_t take_num_states = xs.size();
          if (take_until != -1)
            take_num_states = take_until + 1;

          for (size_t k = 0; k < take_num_states; ++k) {
            if (k < take_num_states - 1) {
              // print the state
              out << space6 << "- ";
              traj_out.states.push_back(xs.at(k));
            } else if (i == result.size() - 2) {
              out << space6 << "- ";
              traj_out.states.push_back(xs.at(k));
            } else {
              out << space6 << "# (last state) ";
            }
            out << xs.at(k).format(FMT) << std::endl;
          }

          // Continue here!!
          // Just get state + motion
          // skip last, then state... and so on!!!
        }
        out << space6 << "# goal state is " << goalState_eig.format(FMT)
            << std::endl;
#endif
      }
      out << "    actions:" << std::endl;

      int action_counter = 0;
      for (size_t i = 0; i < result.size() - 1; ++i) {
        const auto &motion = motions.at(result.at(i + 1)->used_motion);
        int take_until = result.at(i + 1)->intermediate_state;
        if (take_until != -1) {
          out << space6 + "# (note: we have stop at intermediate state) "
              << std::endl;
        }

        out << space6 + "# motion " << motion.idx << " with cost "
            << motion.cost << std::endl; // debug
        //
        //
        //

        size_t take_num_actions = motion.actions.size();

        if (take_until != -1) {
          take_num_actions = take_until;
        }
        DYNO_CHECK_LEQ(take_num_actions, motion.actions.size(), AT);
        out << space6 + "# "
            << "take_num_actions " << take_num_actions << std::endl;

        for (size_t k = 0; k < take_num_actions; ++k) {
          const auto &action = motion.actions[k];
          out << space6 + "- ";
          action_counter += 1;
          printAction(out, si, action);
          Eigen::VectorXd x;
          control_to_eigen(x, si, action);
          traj_out.actions.push_back(x);
          out << std::endl;
        }
        out << std::endl;
      }
      // dts

#if 0
    // TODO: update this to use the new invariance!
    out << "result2:" << std::endl;
    out << "  - states:" << std::endl;

    // I want a solution that accounts for deltas
    // also, it should include the default

    // continue here
    double dt = robot->dt();

    si->copyState(tmpStateq, startState);

    //  First State
    si->copyState(tmpState, startState);

    std::vector<const Sample_ *> states2;
    std::vector<double> times;
    std::vector<oc::Control *> actions2;

    oc::Control *uzero = si->allocControl();

    copyFromRealsControl(
        si, uzero,
        std::vector<double>(robot->diff_model->u_0.data(),
                            robot->diff_model->u_0.data() +
                                robot->diff_model->u_0.size()));

    std::cout << "action uzero" << std::endl;
    printAction(std::cout, si, uzero);
    std::cout << std::endl;

    for (size_t i = 0; i < result.size(); ++i) {

      const auto node_state = result[i]->state;
      if (i > 0) {
        // I also have to compute a distance
        double delta_time = robot->cost_lower_bound(node_state, tmpState);
        cost_with_jumps += delta_time;
        out << space6 + "# delta time " << delta_time << std::endl;
        double delta_i = si->distance(node_state, tmpState);
        out << space6 + "# delta " << delta_i << std::endl;
        si->printState(node_state, std::cout);
        std::cout << std::endl;
        si->printState(tmpState, std::cout);
        std::cout << std::endl;
        std::cout << delta_i << " <= " << options_dbastar.delta
                  << std::endl;
        assert(delta_i <= options_dbastar.delta);
      }

      out << space6 + "# this is node " << std::endl;
      out << space6 + "# time " << cost_with_jumps << std::endl;
      out << space6 + "- ";
      printState(out, si, node_state);
      out << std::endl;
      times.push_back(cost_with_jumps);

      auto uu = si->allocControl();
      si->copyControl(uu, uzero);
      actions2.push_back(uu);

      out << space6 + "# action ";
      printAction(out, si, uu);
      out << std::endl;

      auto x_ = si->allocState();
      si->copyState(x_, node_state);
      states2.push_back(x_);

      si->copyState(tmpState, node_state);

      if (i < result.size() - 1) {
        fcl::Vector3d current_pos(0, 0, 0);
        if (robot->isTranslationInvariant())
          current_pos = robot->getTransform(node_state).translation();
        const auto &motion = motions.at(result.at(i + 1)->used_motion);
        out << space6 + "# motion " << motion.idx << " with cost "
            << motion.cost << std::endl;

        for (size_t k = 0; k < motion.states.size(); ++k) {
          ompl::control::Control *u;

          if (k < motion.states.size() - 1) {
            u = motion.actions.at(k);
          } else {
            u = uzero;
          }

          const auto state = motion.states.at(k);
          si->copyState(tmpState, state);

          if (robot->isTranslationInvariant()) {
            const fcl::Vector3d relative_pos =
                robot->getTransform(state).translation();
            robot->setPosition(tmpState, current_pos +
                                             result[i + 1]->used_offset +
                                             relative_pos);
          }

          if (k == 0) {
            double delta_time = robot->cost_lower_bound(node_state, tmpState);
            cost_with_jumps += delta_time;
            out << space6 + "# delta time " << delta_time << std::endl;
            double delta_i = si->distance(node_state, tmpState);
            out << space6 + "# delta " << delta_i << std::endl;
            assert(delta_i <= options_dbastar.delta);
          } else {
            cost_with_jumps += dt;
          }
          out << space6 + "# time " << cost_with_jumps << std::endl;
          out << space6 + "- ";
          printState(out, si, tmpState);
          out << std::endl;

          auto x_ = si->allocState();
          si->copyState(x_, tmpState);
          states2.push_back(x_);

          times.push_back(cost_with_jumps);
          auto uu = si->allocControl();
          si->copyControl(uu, u);
          actions2.push_back(uu);

          out << space6 + "# action ";
          printAction(out, si, uu);
          out << std::endl;
        }
      }
    }
    if (true) {
      double delta_time = robot->cost_lower_bound(tmpState, goalState);
      cost_with_jumps += delta_time;
      out << space6 + "# delta time " << delta_time << std::endl;
      double delta_i = si->distance(goalState, tmpState);
      std::cout << "delta_i " << delta_i << std::endl;
      assert(delta_i < options_dbastar.delta);
      out << space6 + "# delta " << delta_i << std::endl;
      out << space6 + "# this is goal " << std::endl;
      out << space6 + "# time " << cost_with_jumps << std::endl;
      out << space6 + "- ";
      printState(out, si, goalState);
      out << std::endl;

      auto x_ = si->allocState();
      si->copyState(x_, tmpState);
      states2.push_back(x_);
      times.push_back(cost_with_jumps);

      // auto uu = si->allocControl();
      // si->copyControl(uu, uzero);
      // actions2.push_back(uu);

    } else {
      out << space6 + "# last state is already the goal" << std::endl;
    }

    out << space4 + "actions:" << std::endl;
    for (auto &a : actions2) {
      out << space6 + "- ";
      printAction(out, si, a);
      out << std::endl;
    }

    out << space4 + "times:" << std::endl;
    for (auto &t : times) {
      out << space6 + "- " << t << std::endl;
    }

    std::cout << "times size " << times.size() << std::endl;
    std::cout << "actions size " << actions2.size() << std::endl;
    std::cout << "states size " << states2.size() << std::endl;

    std::cout << "action counter " << action_counter << std::endl;
#endif
      // statistics for the motions used
      std::map<size_t, size_t> motionsCount; // motionId -> usage count
      for (size_t i = 0; i < result.size() - 1; ++i) {
        auto motionId = result[i + 1]->used_motion;
        auto iter = motionsCount.find(motionId);
        if (iter == motionsCount.end()) {
          motionsCount[motionId] = 1;
        } else {
          iter->second += 1;
        }
      }
      out << space4 + "motion_stats:" << std::endl;
      for (const auto &kv : motionsCount) {
        out << space6 + "" << motions[kv.first].idx << ": " << kv.second
            << std::endl;
      }

      // statistics on where the motion splits are
      out << space4 + "splits:" << std::endl;
      for (size_t i = 0; i < result.size() - 1; ++i) {
        const auto &motion = motions.at(result[i + 1]->used_motion);
        out << space6 + "- " << motion.states.size() - 1 << std::endl;
      }

      out << space4 + "graph_nodes:" << std::endl;
      for (auto &r : result) {
        out << space6 + "- ";
        printState(out, si, r->state);
        out << std::endl;
      }
    }
    std::cout << "WARNING: I am using cost with jumps!" << std::endl;
    traj_out.cost = cost_with_jumps;

    out_info_db.solved = solution;

    out_info_db.time_search = time_bench.time_search;
    out_info_db.data = time_bench.to_data();
    out_info_db.data.insert(std::make_pair(
        "terminate_status", terminate_status_str[static_cast<int>(status)]));

    out_info_db.data.insert(
        std::make_pair("solved", std::to_string(bool(solution))));

    out_info_db.data.insert(
        std::make_pair("delta", std::to_string(options_dbastar.delta)));

    out_info_db.data.insert(std::make_pair(
        "num_primitives", std::to_string(final_number_of_primitives)));

    out_info_db.data.insert(std::make_pair(
        "num_primitives_", std::to_string(final_number_of_primitives)));

    // lets check the

    if (traj_out.states.size()) {
      std::cout << "checking output... " << std::endl;
      traj_out.check(robot->diff_model, true);
    }

    // gen
    {
      std::string filename = "/tmp/dynoplan/traj_db.yaml";
      std::string filename_id =
          "/tmp/dynoplan/traj_db_" + gen_random(6) + ".yaml";
      std::cout << "saving traj to: " << filename << std::endl;
      std::cout << "saving traj to: " << filename_id << std::endl;
      create_dir_if_necessary(filename.c_str());
      create_dir_if_necessary(filename_id.c_str());
      std::ofstream out(filename_id);
      traj_out.to_yaml_format(out);
      std::filesystem::copy(filename_id, filename,
                            std::filesystem::copy_options::overwrite_existing);
    }

    if (out_info_db.solved) {
      out_info_db.cost = solution->gScore;
      out_info_db.cost_with_delta_time = cost_with_jumps;
    } else {
      out_info_db.cost = -1;
      out_info_db.cost_with_delta_time = -1;
    }
  }
#endif

// void Out_info_db::print(std::ostream &out) {
//
//   std::string be = "";
//   std::string af = ": ";
//
//   out << be << STR(cost, af) << std::endl;
//   out << be << STR(cost_with_delta_time, af) << std::endl;
//   out << be << STR(solved, af) << std::endl;
// }

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
