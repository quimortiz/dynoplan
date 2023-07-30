#include "dynoplan/dbrrt/dbrrt.hpp"
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
#include "dynoplan/dbastar/dbastar.hpp"
// #include "ocp.hpp"
#include "dynobench/robot_models.hpp"
#include "dynoplan/ompl/robots.h"
#include "dynoplan/optimization/ocp.hpp"
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

using dynobench::FMT;

void from_solution_to_yaml_and_traj_bwd(dynobench::Model_robot &robot,
                                        AStarNode *solution,
                                        const std::vector<Motion> &motions,
                                        dynobench::Trajectory &traj_out,
                                        std::ofstream *out) {

  bool debug = true;

  const AStarNode *node = solution;
  std::vector<const AStarNode *> result;
  while (node) {
    result.push_back(node);
    node = node->came_from;
  }

  if (result.size() == 1) {
    std::cout << "single node in the bwd tree" << std::endl;
    std::cout << "return an empyt traj " << std::endl;
    traj_out = dynobench::Trajectory();
    return;
  }

  if (debug) {
    std::ofstream debub_bwd("/tmp/dynoplan/bwd_nodes.yaml");
    debub_bwd << "nodes:" << std::endl;
    for (auto &node : result) {
      debub_bwd << "  - " << node->state_eig.format(FMT) << std::endl;
    }
  }

  Eigen::VectorXd __offset = Eigen::VectorXd::Zero(robot.get_offset_dim());

  std::vector<dynobench::Trajectory> trajs_original;
  std::vector<dynobench::Trajectory> trajs_reverse;

  for (size_t i = 0; i < result.size() - 1; i++) {
    auto &motion = motions.at(result.at(i)->used_motion);
    auto parent = result.at(i)->came_from;

    std::cout << "####" << std::endl;
    std::cout << "i " << i << std::endl;
    std::cout << "node " << std::endl;
    CSTR_V(result.at(i)->state_eig);
    std::cout << "motion" << std::endl;
    motion.traj.to_yaml_format(std::cout);
    std::cout << "parent" << std::endl;
    CSTR_V(parent->state_eig);

    robot.offset(parent->state_eig, __offset);
    // move the primitive with the parent
    dynobench::Trajectory traj;
    dynobench::TrajWrapper traj_wrapper;

    traj_wrapper.allocate_size(motion.traj.states.size(),
                               motion.traj.states.front().size(),
                               motion.traj.actions.front().size());

    robot.transform_primitive(__offset, motion.traj.states, motion.traj.actions,
                              traj_wrapper);

    traj = dynobench::trajWrapper_2_Trajectory(traj_wrapper);

    std::cout << "after transformation" << std::endl;
    traj.to_yaml_format(std::cout);

    dynobench::Trajectory traj_original;
    traj_original = traj;
    std::reverse(traj_original.states.begin(), traj_original.states.end());
    std::reverse(traj_original.actions.begin(), traj_original.actions.end());

    trajs_original.push_back(traj_original);
    trajs_reverse.push_back(traj);
  }

  if (debug) {
    std::ofstream debub_bwd("/tmp/dynoplan/bwd_traj_original.yaml");
    debub_bwd << "trajs:" << std::endl;
    for (auto &t : trajs_original) {
      debub_bwd << "  -" << std::endl;
      t.to_yaml_format(debub_bwd, "    ");
    }

    debub_bwd << "trajs_reverse:" << std::endl;
    for (auto &t : trajs_reverse) {
      debub_bwd << "  -" << std::endl;
      t.to_yaml_format(debub_bwd, "    ");
    }
  }

  // now i just have to concatenate the trajectories :)
  // just delete the last state of each trajectory, expect for the last one

  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> us;
  for (auto &t : trajs_original) {
    xs.insert(xs.end(), t.states.begin(), t.states.end() - 1);
    us.insert(us.end(), t.actions.begin(), t.actions.end());
  }
  xs.insert(xs.end(), trajs_original.back().states.back());

  traj_out.states = xs;
  traj_out.actions = us;
}

void from_fwd_bwd_solution_to_yaml_and_traj(
    dynobench::Model_robot &robot, const std::vector<Motion> &motions,
    const std::vector<Motion> &motions_rev, AStarNode *solution_fwd,
    AStarNode *solution_bwd, const dynobench::Problem &problem,
    dynobench::Trajectory &traj_out, dynobench::Trajectory &traj_out_fwd,
    dynobench::Trajectory &traj_out_bwd, std::ofstream *out) {

  std::ofstream *out_fwd = nullptr;
  std::ofstream *out_bwd = nullptr;
  if (out) {
    out_fwd = new std::ofstream("/tmp/dynoplan/fwd.yaml");
    out_bwd = new std::ofstream("/tmp/dynoplan/bwd.yaml");
  }
  from_solution_to_yaml_and_traj(robot, motions, solution_fwd, problem,
                                 traj_out_fwd, out_fwd);

  from_solution_to_yaml_and_traj_bwd(robot, solution_bwd, motions_rev,
                                     traj_out_bwd, out_fwd);

  if (traj_out_fwd.states.size() == 0) {
    traj_out = traj_out_bwd;
  } else if (traj_out_bwd.states.size() == 0) {
    traj_out = traj_out_fwd;
  } else {
    traj_out.states = {traj_out_fwd.states.begin(),
                       traj_out_fwd.states.end() - 1};

    traj_out.states.insert(traj_out.states.end(), traj_out_bwd.states.begin(),
                           traj_out_bwd.states.end());

    traj_out.actions = {traj_out_fwd.actions.begin(),
                        traj_out_fwd.actions.end()};

    traj_out.actions.insert(traj_out.actions.end(),
                            traj_out_bwd.actions.begin(),
                            traj_out_bwd.actions.end());
  }
  // = traj_out_fwd.states;

  delete out_fwd;
  delete out_bwd;
}

struct Planner {

  void solve() {}
};

void nearest_state_timed(AStarNode *n, AStarNode *&neigh,
                         ompl::NearestNeighbors<AStarNode *> *T_n,
                         Time_benchmark &time_bench) {

  assert(n);
  assert(T_n);

  auto _out = timed_fun([&] {
    neigh = T_n->nearest(n);
    return 0;
  });
  time_bench.num_nn_states++;
  time_bench.time_nearestNode += _out.second;
  time_bench.time_nearestNode_search += _out.second;
}

void add_state_timed(AStarNode *node, ompl::NearestNeighbors<AStarNode *> *T_n,
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

void reverse_motions(std::vector<Motion> &motions_rev,
                     dynobench::Model_robot &robot,
                     const std::vector<Motion> &motions) {
  DYNO_CHECK_EQ(motions_rev.size(), 0, AT);
  motions_rev.reserve(motions.size());
  std::cout << "warning:"
            << "reverse of motion primitives only working for translation "
               "invariant systems "
            << std::endl;
  Eigen::VectorXd xf_canonical(robot.nx);
  Eigen::VectorXd offset(robot.get_offset_dim());
  for (auto &m : motions) {
    Motion mrev;

    // robot->canonical_state(m.traj.states.back(),
    // xf_canonical);
    robot.offset(m.traj.states.back(), offset);
    dynobench::Trajectory traj_new;

    dynobench::TrajWrapper traj_wrapper;

    traj_wrapper.allocate_size(m.traj.states.size(),
                               m.traj.states.front().size(),
                               m.traj.actions.front().size());

    robot.transform_primitive(-offset, m.traj.states, m.traj.actions,
                              traj_wrapper);

    traj_new = dynobench::trajWrapper_2_Trajectory(traj_wrapper);

    mrev.traj = traj_new;
    mrev.cost = m.cost;
    mrev.idx = m.idx;
    std::reverse(mrev.traj.states.begin(), mrev.traj.states.end());
    std::reverse(mrev.traj.actions.begin(), mrev.traj.actions.end());

    motions_rev.push_back(std::move(mrev));
  }
}

// refactor: take optimization OUT!!
void dbrrtConnect(const dynobench::Problem &problem,
                  const Options_dbrrt &options_dbrrt,
                  const Options_trajopt &options_trajopt,
                  dynobench::Trajectory &traj_out,
                  dynobench::Info_out &info_out) {

  std::cout << "options dbrrt" << std::endl;
  options_dbrrt.print(std::cout);
  std::cout << "***" << std::endl;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      dynobench::robot_type_to_path(problem.robotType).c_str(), problem.p_lb,
      problem.p_ub);
  load_env(*robot, problem);
  const int nx = robot->nx;

  std::vector<Motion> &motions = *options_dbrrt.motions_ptr;

  std::vector<Motion> motions_rev;

  CHECK(options_dbrrt.motions_ptr, AT);

  std::cout << "example motions " << std::endl;
  motions.front().traj.to_yaml_format(std::cout);
  motions_rev.front().traj.to_yaml_format(std::cout);
  std::cout << "DONE " << std::endl;

  reverse_motions(motions_rev, *robot, motions);

  Time_benchmark time_bench;
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  ompl::NearestNeighbors<Motion *> *T_mrev = nullptr;
  if (options_dbrrt.use_nigh_nn) {
    T_m = nigh_factory2<Motion *>(problem.robotType, robot);
    T_mrev = nigh_factory2<Motion *>(problem.robotType, robot);
  } else {
    NOT_IMPLEMENTED;
  }
  assert(T_mrev);
  assert(T_m);

  time_bench.time_nearestMotion += timed_fun_void([&] {
    for (size_t j = 0; j < std::min(options_dbrrt.max_motions, motions.size());
         j++)
      T_m->add(&motions[j]);
  });

  time_bench.time_nearestMotion += timed_fun_void([&] {
    for (size_t j = 0;
         j < std::min(options_dbrrt.max_motions, motions_rev.size()); j++)
      T_mrev->add(&motions_rev[j]);
  });

  ompl::NearestNeighbors<AStarNode *> *T_n = nullptr;
  ompl::NearestNeighbors<AStarNode *> *T_nrev = nullptr;

  std::vector<AStarNode *> nodes_in_Tn;
  std::vector<AStarNode *> nodes_in_Tnrev;

  if (options_dbrrt.use_nigh_nn) {
    T_n = nigh_factory2<AStarNode *>(problem.robotType, robot);
    T_nrev = nigh_factory2<AStarNode *>(problem.robotType, robot);
  } else {
    NOT_IMPLEMENTED;
  }

  Terminate_status status = Terminate_status::UNKNOWN;

  Expander expander(robot.get(), T_m, options_dbrrt.delta);
  Expander expander_rev(robot.get(), T_mrev, options_dbrrt.delta);

  auto start_node = new AStarNode;
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

  Eigen::VectorXd x(nx);

  CSTR_V(robot->x_lb);
  CSTR_V(robot->x_ub);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.traj.states.push_back(Eigen::VectorXd::Zero(robot->nx));

  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goal);

  std::vector<Eigen::VectorXd> rand_nodes;
  std::vector<Eigen::VectorXd> near_nodes;
  std::vector<dynobench::Trajectory> trajs;
  std::vector<dynobench::Trajectory> chosen_trajs;

  std::mt19937 g = std::mt19937{std::random_device()()};

  if (options_dbrrt.fix_seed) {
    expander.seed(0);
    g = std::mt19937{0};
    srand(0);
  } else {
    srand(time(0));
  }

  Eigen::VectorXd x_rand(robot->nx), x_target(robot->nx);
  AStarNode *rand_node = new AStarNode;

  AStarNode *near_node = nullptr;
  AStarNode *tmp = nullptr;
  AStarNode *solution_fwd = nullptr;
  AStarNode *solution_bwd = nullptr;

  std::vector<AStarNode *> discovered_nodes_fwd, discovered_nodes_bwd;

  double cost_bound =
      options_dbrrt.cost_bound; //  std::numeric_limits<double>::infinity();

  double best_cost_opt = std::numeric_limits<double>::infinity();
  dynobench::Trajectory best_traj_opt;

  // SEARCH STARTS HERE
  add_state_timed(start_node, T_n, time_bench);
  nodes_in_Tn.push_back(start_node);
  add_state_timed(goal_node, T_nrev, time_bench);
  nodes_in_Tnrev.push_back(goal_node);

  std::vector<AStarNode *> discovered_nodes;

  bool expand_forward = true;

  std::vector<dynobench::Trajectory> chosen_trajs_fwd, chosen_trajs_bwd;

  const size_t print_every = 1000;

  auto print_search_status = [&] {
    std::cout << "expands: " << time_bench.expands
              << " best distance: " << best_distance_to_goal
              << " cost bound: " << cost_bound << std::endl;
  };

  Stopwatch watch;

  auto stop_search = [&] {
    if (static_cast<size_t>(time_bench.expands) >= options_dbrrt.max_expands) {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;

      return true;
    }

    if (watch.elapsed_ms() > options_dbrrt.search_timelimit) {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      return true;
    }
    return false;
  };

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

  Eigen::VectorXd __expand_start(robot->nx);
  Eigen::VectorXd __expand_end(robot->nx);

  while (!stop_search()) {
    if (time_bench.expands % print_every == 0)
      print_search_status();

    time_bench.expands++;

    expand_forward = static_cast<double>(rand()) / RAND_MAX <
                     options_dbrrt.prob_expand_forward;

    if (static_cast<double>(rand()) / RAND_MAX < options_dbrrt.goal_bias) {
      if (expand_forward) {
        std::uniform_int_distribution<> distr(0, T_nrev->size() - 1);
        int idx = distr(g);
        x_rand = nodes_in_Tnrev.at(idx)->state_eig;
      } else {
        std::uniform_int_distribution<> distr(0, T_n->size() - 1);
        int idx = distr(g);
        x_rand = nodes_in_Tn.at(idx)->state_eig;
      }
    } else {
      robot->sample_uniform(x_rand);
    }

    rand_node->state_eig = x_rand;

    if (expand_forward) {
      nearest_state_timed(rand_node, near_node, T_n, time_bench);
    } else
      nearest_state_timed(rand_node, near_node, T_nrev, time_bench);
    assert(near_node);

    if (options_dbrrt.debug) {
      rand_nodes.push_back(x_rand);
      near_nodes.push_back(near_node->state_eig);
    }

    double distance_to_rand = robot->distance(x_rand, near_node->state_eig);

    if (distance_to_rand > options_dbrrt.max_step_size) {
      robot->interpolate(x_target, near_node->state_eig, x_rand,
                         options_dbrrt.max_step_size / distance_to_rand);
    } else {
      x_target = x_rand;
    }

    std::vector<LazyTraj> lazy_trajs;

    if (expand_forward) {
      expander.expand_lazy(near_node->state_eig, lazy_trajs);
    } else {
      expander_rev.expand_lazy(near_node->state_eig, lazy_trajs);
    }

    double min_distance = std::numeric_limits<double>::max();
    int best_index = -1;
    dynobench::Trajectory chosen_traj_debug;
    LazyTraj chosen_lazy_traj;

    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      bool motion_valid = check_lazy_trajectory(lazy_traj, *robot, time_bench,
                                                traj_wrapper, nullptr, nullptr);

      if (options_dbrrt.debug) {
        trajs.push_back(dynobench::trajWrapper_2_Trajectory(traj_wrapper));
      }

      if (!motion_valid)
        continue;

      double d = robot->distance(
          traj_wrapper.get_state(traj_wrapper.get_size() - 1), x_target);

      if (d < min_distance) {
        min_distance = d;
        best_index = i;
        chosen_lazy_traj = lazy_traj;

        __expand_start = traj_wrapper.get_state(0);
        __expand_end = traj_wrapper.get_state(traj_wrapper.get_size() - 1);

        if (options_dbrrt.debug) {
          chosen_traj_debug = dynobench::trajWrapper_2_Trajectory(traj_wrapper);
        }

        if (options_dbrrt.choose_first_motion_valid)
          break;
      }
    }

    if (best_index != -1) {
      AStarNode *new_node = new AStarNode();
      new_node->state_eig = __expand_end;
      new_node->hScore =
          robot->lower_bound_time(new_node->state_eig, problem.goal);
      new_node->came_from = near_node;
      new_node->used_motion = chosen_lazy_traj.motion->idx;

      new_node->gScore =
          near_node->gScore + chosen_lazy_traj.motion->cost +
          options_dbrrt.cost_jump *
              robot->lower_bound_time(near_node->state_eig, __expand_start);

      new_node->fScore = new_node->gScore + new_node->hScore;

      if (expand_forward)
        nearest_state_timed(new_node, tmp, T_n, time_bench);
      else
        nearest_state_timed(new_node, tmp, T_nrev, time_bench);

      if (robot->distance(tmp->state_eig, new_node->state_eig) <
          options_dbrrt.delta / 2.) {
        std::cout << "warning: node already in the tree" << std::endl;
        delete new_node; // TODO: use unique ptrs
        continue;
      }

      if (expand_forward) {
        add_state_timed(new_node, T_n, time_bench);
        nodes_in_Tn.push_back(new_node);
        discovered_nodes.push_back(new_node);
        nearest_state_timed(new_node, tmp, T_nrev, time_bench);
      } else {
        add_state_timed(new_node, T_nrev, time_bench);
        nodes_in_Tnrev.push_back(new_node);
        discovered_nodes.push_back(new_node);
        nearest_state_timed(new_node, tmp, T_n, time_bench);
      }

      if (options_dbrrt.debug) {
        if (expand_forward)
          chosen_trajs_fwd.push_back(chosen_traj_debug);
        else
          chosen_trajs_bwd.push_back(chosen_traj_debug);
      }

      double di = robot->distance(tmp->state_eig, new_node->state_eig);

      if (di < options_dbrrt.delta) {
        std::cout << "we have connected the trees!" << std::endl;

        if (expand_forward) {
          solution_fwd = new_node;
          solution_bwd = tmp;
        } else {
          solution_fwd = tmp;
          solution_bwd = new_node;
        }

        if (options_dbrrt.debug) {
          std::vector<AStarNode *> active_nodes_fwd;
          T_n->list(active_nodes_fwd);
          plot_search_tree(active_nodes_fwd, motions, *robot,
                           ("/tmp/dynoplan/db_rrt_tree_fwd_" +
                            std::to_string(info_out.trajs_raw.size()) + ".yaml")
                               .c_str());
          std::vector<AStarNode *> active_nodes_bwd;
          T_nrev->list(active_nodes_bwd);
          plot_search_tree(active_nodes_bwd, motions_rev, *robot,
                           ("/tmp/dynoplan/db_rrt_tree_bwd_" +
                            std::to_string(info_out.trajs_raw.size()) + ".yaml")
                               .c_str());
        }

        status = Terminate_status::SOLVED_RAW;

        info_out.solved_raw = true;
        std::cout << "success! GOAL_REACHED" << std::endl;
        std::cout << "node fwd " << solution_fwd->state_eig.format(FMT)
                  << std::endl;
        std::cout << "node bwd " << solution_bwd->state_eig.format(FMT)
                  << std::endl;

        // TODO: dont write to much to file!!
        std::ofstream file_debug("/tmp/dynoplan/db_rrt_debug_" +
                                 std::to_string(info_out.trajs_raw.size()) +
                                 ".yaml");
        dynobench::Trajectory traj_db, traj_out_fwd, traj_out_bwd;

        from_fwd_bwd_solution_to_yaml_and_traj(
            *robot, motions, motions_rev, solution_fwd, solution_bwd, problem,
            traj_db, traj_out_fwd, traj_out_bwd, &file_debug);

        traj_db.to_yaml_format("/tmp/dynoplan/db_rrt_traj_" +
                               std::to_string(info_out.trajs_raw.size()) +
                               ".yaml");

        traj_out_fwd.to_yaml_format("/tmp/dynoplan/db_rrt_traj_fwd_" +
                                    std::to_string(info_out.trajs_raw.size()) +
                                    ".yaml");

        traj_out_bwd.to_yaml_format("/tmp/dynoplan/db_rrt_traj_bwd_" +
                                    std::to_string(info_out.trajs_raw.size()) +
                                    ".yaml");

        info_out.trajs_raw.push_back(traj_db);

        if (options_dbrrt.do_optimization) {
          Stopwatch sw;
          dynobench::Trajectory traj_opt;
          Result_opti result;

          trajectory_optimization(problem, traj_db, options_trajopt, traj_opt,
                                  result);

          traj_opt.to_yaml_format("/tmp/dynoplan/db_rrt_traj_opt_" +
                                  std::to_string(info_out.trajs_opt.size()) +
                                  ".yaml");

          if (result.feasible == 1) {
            std::cout << "success: optimization is "
                         "feasible!"
                      << std::endl;
            info_out.solved = true;

            if (result.cost < best_cost_opt) {
              best_traj_opt = traj_opt;
            }

            info_out.trajs_opt.push_back(traj_opt);

            if (options_dbrrt.extract_primitives ||
                options_dbrrt.add_to_search_tree) {
              NOT_IMPLEMENTED;
            }

          } else {
            std::cout << "warning: optimization failed" << std::endl;
          }
        }

        if (!options_dbrrt.ao_rrt) {
          if ((options_dbrrt.do_optimization && info_out.solved) ||
              !options_dbrrt.do_optimization) {
            break;
          }
        } else {
          NOT_IMPLEMENTED;
        }
      }
    } else {
      std::cout << "Warning: all expansions failed "
                   "in state "
                << near_node->state_eig.format(FMT) << std::endl;
    }
  }
  time_bench.time_search = watch.elapsed_ms();
  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;
  std::cout << "solved_raw: " << (solution_bwd && solution_fwd != nullptr)
            << std::endl;
  std::cout << "solved_opt:" << bool(info_out.trajs_opt.size()) << std::endl;
  std::cout << "TIME in search:" << time_bench.time_search << std::endl;
  std::cout << "sizeTN: " << T_n->size() << std::endl;

  if (solution_bwd && solution_fwd) {
    std::cout << "cost: " << solution_fwd->gScore + solution_bwd->gScore
              << std::endl;
  } else {
    nearest_state_timed(goal_node, tmp, T_n, time_bench);

    std::cout << "Close distance T_n to goal: "
              << robot->distance(goal_node->getStateEig(), tmp->getStateEig())
              << std::endl;

    nearest_state_timed(start_node, tmp, T_nrev, time_bench);

    std::cout << "Close distance T_nrev to start: "
              << robot->distance(start_node->getStateEig(), tmp->getStateEig())
              << std::endl;

    // tree vs tree
    DYNO_CHECK_EQ(nodes_in_Tn.size(), T_n->size(), AT);

    std::vector<double> distances(nodes_in_Tn.size());
    std::vector<AStarNode *> nn(nodes_in_Tn.size());

    double min_dist = std::numeric_limits<double>::max();
    int best_index = -1;
    for (size_t i = 0; i < nodes_in_Tn.size(); ++i) {
      nearest_state_timed(nodes_in_Tn.at(i), tmp, T_nrev, time_bench);
      nn.at(i) = tmp;
      double di =
          robot->distance(nodes_in_Tn.at(i)->getStateEig(), tmp->getStateEig());
      distances[i] = di;
      if (di < min_dist) {
        min_dist = di;
        best_index = i;
      }
    }
    assert(best_index >= 0);
    std::cout << "nearest pair is " << std::endl;

    std::cout << "FWD" << std::endl;
    nodes_in_Tn.at(best_index)->write(std::cout);

    std::cout << "BWD" << std::endl;
    nn.at(best_index)->write(std::cout);

    std::cout << "distance: " << min_dist << std::endl;
  }

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  if (options_dbrrt.debug) {
    std::ofstream debug_file("debug.yaml");
    std::ofstream debug_file2("debug2.yaml");
    debug_file << "rand_nodes:" << std::endl;
    for (auto &q : rand_nodes) {
      debug_file << "  - " << q.format(FMT) << std::endl;
    }

    debug_file << "near_nodes:" << std::endl;
    for (auto &q : near_nodes) {
      debug_file << "  - " << q.format(FMT) << std::endl;
    }

    debug_file << "discovered_nodes:" << std::endl;
    for (auto &q : discovered_nodes) {
      debug_file << "  - " << q->state_eig.format(FMT) << std::endl;
    }

    debug_file << "chosen_trajs:" << std::endl;
    for (auto &traj : chosen_trajs) {
      debug_file << "  - " << std::endl;
      traj.to_yaml_format(debug_file, "    ");
    }

    debug_file2 << "trajs:" << std::endl;
    for (auto &traj : trajs) {
      debug_file2 << "  - " << std::endl;
      traj.to_yaml_format(debug_file2, "    ");
    }
  }

  std::ofstream out("out_dbrrt.yaml");

  out << "solved: " << bool(solution_fwd && solution_bwd) << std::endl;
  out << "status: " << static_cast<int>(status) << std::endl;
  out << "status_str: " << terminate_status_str[static_cast<int>(status)]
      << std::endl;
  out << "sizeTN: " << T_n->size() << std::endl;
  time_bench.write(out);

  if (solution_bwd && solution_fwd) {
    std::cout << "warning! "
              << "write down data " << std::endl;

    // out << "result:" << std::endl;
    //
    // from_solution_to_yaml_and_traj(*robot, motions,
    // solution, problem, traj_out,
    //                                &out);
    //
    // std::vector<Trajectory>
    // trajs_out(all_solutions_raw.size());
    //
    // for (size_t i = 0; i <
    // all_solutions_raw.size(); i++) {
    //
    //   std::string filename =
    //       "/tmp/dynoplan/dbrrt-" +
    //       std::to_string(i) + ".yaml";
    //
    //   create_dir_if_necessary(filename);
    //   std::cout << "writing to " << filename <<
    //   std::endl; std::ofstream out(filename);
    //   from_solution_to_yaml_and_traj(*robot,
    //   motions, all_solutions_raw.at(i),
    //                                  problem,
    //                                  trajs_out.at(i),
    //                                  &out);
    //   std::string filename2 =
    //       "/tmp/dynoplan/dbrrt-" +
    //       std::to_string(i) + ".traj.yaml";
    //   std::cout << "writing to " << filename2 <<
    //   std::endl; std::ofstream out2(filename2);
    //   create_dir_if_necessary(filename2);
    //   trajs_out.at(i).to_yaml_format(out2);
    // }

    // also save the trajectories
  }

  if (info_out.solved) {
    CHECK(info_out.solved_raw, AT);
  }

  std::cout << "warning: update the trajecotries cost" << std::endl;
  std::for_each(
      info_out.trajs_raw.begin(), info_out.trajs_raw.end(),
      [&](auto &traj) { traj.cost = robot->ref_dt * traj.actions.size(); });

  std::cout << "checking the trajectories in raw " << std::endl;

  for (size_t i = 0; i < info_out.trajs_raw.size(); i++) {
    auto &traj = info_out.trajs_raw.at(i);
    std::cout << "checking traj raw: " << i << std::endl;
    traj.start = problem.start;
    traj.goal = problem.goal;
    traj.check(robot, true);
  }

  std::for_each(
      info_out.trajs_opt.begin(), info_out.trajs_opt.end(),
      [&](auto &traj) { traj.cost = robot->ref_dt * traj.actions.size(); });

  if (info_out.solved) {
    info_out.cost =
        std::min_element(
            info_out.trajs_opt.begin(), info_out.trajs_opt.end(),
            [](const auto &a, const auto &b) { return a.cost < b.cost; })
            ->cost;
  }

  if (info_out.solved_raw) {
    info_out.cost_raw =
        std::min_element(
            info_out.trajs_raw.begin(), info_out.trajs_raw.end(),
            [](const auto &a, const auto &b) { return a.cost < b.cost; })
            ->cost;
  }
}

void dbrrt(const dynobench::Problem &problem,
           const Options_dbrrt &options_dbrrt,
           const Options_trajopt &options_trajopt,
           dynobench::Trajectory &traj_out, dynobench::Info_out &info_out) {

  std::cout << "options dbrrt" << std::endl;
  options_dbrrt.print(std::cout);
  std::cout << "***" << std::endl;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path +
       dynobench::robot_type_to_path(problem.robotType))
          .c_str(),
      problem.p_lb, problem.p_ub);
  const int nx = robot->nx;
  load_env(*robot, problem);

  std::vector<Motion> &motions = *options_dbrrt.motions_ptr;
  CHECK(options_dbrrt.motions_ptr, AT);

  Time_benchmark time_bench;
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  if (options_dbrrt.use_nigh_nn) {
    T_m = nigh_factory2<Motion *>(problem.robotType, robot);
  } else {
    NOT_IMPLEMENTED;
  }

  {
    Stopwatch s;
    assert(T_m);
    for (auto &m : motions) {
      T_m->add(&m);
    }
    time_bench.time_nearestMotion += s.elapsed_ms();
  }

  ompl::NearestNeighbors<AStarNode *> *T_n = nullptr;

  if (options_dbrrt.use_nigh_nn) {
    if (options_dbrrt.ao_rrt) {
      T_n = nigh_factory2<AStarNode *>(
          problem.robotType, robot,
          [](AStarNode *m) { return m->getStateEig(); },
          options_dbrrt.cost_weight);
    } else {
      T_n = nigh_factory2<AStarNode *>(problem.robotType, robot);
    }
  } else {
    NOT_IMPLEMENTED;
  }

  Terminate_status status = Terminate_status::UNKNOWN;

  Expander expander(robot.get(), T_m, options_dbrrt.delta);

  auto start_node = new AStarNode;
  start_node->gScore = 0;
  start_node->state_eig = problem.start;
  start_node->hScore =
      robot->lower_bound_time(start_node->state_eig, problem.goal);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->came_from = nullptr;

  Eigen::VectorXd x(nx);

  CSTR_V(robot->x_lb);
  CSTR_V(robot->x_ub);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.traj.states.push_back(Eigen::VectorXd::Zero(robot->nx));

  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goal);

  std::vector<Eigen::VectorXd> rand_nodes;
  std::vector<Eigen::VectorXd> near_nodes;
  std::vector<dynobench::Trajectory> trajs;
  std::vector<dynobench::Trajectory> chosen_trajs;

  std::mt19937 g = std::mt19937{std::random_device()()};

  if (options_dbrrt.fix_seed) {
    expander.seed(0);
    g = std::mt19937{0};
    srand(0);
  } else {
    srand(time(0));
  }

  Eigen::VectorXd x_rand(robot->nx), x_target(robot->nx);
  AStarNode *rand_node = new AStarNode;
  AStarNode *near_node = nullptr;
  AStarNode *tmp = nullptr;
  AStarNode *solution = nullptr;
  AStarNode *best_node = start_node;

  std::vector<AStarNode *> discovered_nodes, all_solutions_raw;

  time_bench.expands = 0;
  double cost_bound =
      options_dbrrt.cost_bound; //  std::numeric_limits<double>::infinity();

  double best_cost_opt = std::numeric_limits<double>::infinity();
  dynobench::Trajectory best_traj_opt;

  // SEARCH STARTS HERE
  Stopwatch watch;
  add_state_timed(start_node, T_n, time_bench);
  discovered_nodes.push_back(start_node);

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

  Eigen::VectorXd __expand_start(robot->nx);
  Eigen::VectorXd __expand_end(robot->nx);

  while (true) {

    if (static_cast<size_t>(time_bench.expands) >= options_dbrrt.max_expands) {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      break;
    }

    if (watch.elapsed_ms() > options_dbrrt.search_timelimit) {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      break;
    }

    if (time_bench.expands % 500 == 0) {
      std::cout << "expands: " << time_bench.expands
                << " best distance: " << best_distance_to_goal
                << " cost bound: " << cost_bound << std::endl;
    }

    time_bench.expands++;

    if (static_cast<double>(rand()) / RAND_MAX < options_dbrrt.goal_bias) {
      x_rand = problem.goal;
    } else {
      robot->sample_uniform(x_rand);
    }

    rand_node->state_eig = x_rand;

    if (options_dbrrt.ao_rrt) {
      rand_node->gScore = static_cast<double>(rand()) / RAND_MAX * cost_bound;
    }

    nearest_state_timed(rand_node, near_node, T_n, time_bench);

    if (options_dbrrt.ao_rrt &&
        near_node->gScore + near_node->hScore >
            options_dbrrt.best_cost_prune_factor * cost_bound) {
      std::cout << "warning! "
                << "cost of near is above bound -- " << near_node->gScore << " "
                << cost_bound << std::endl;
      continue;
    }

    if (options_dbrrt.debug) {
      rand_nodes.push_back(x_rand);
      near_nodes.push_back(near_node->state_eig);
    }

    double distance_to_rand = robot->distance(x_rand, near_node->state_eig);

    if (distance_to_rand > options_dbrrt.max_step_size) {
      robot->interpolate(x_target, near_node->state_eig, x_rand,
                         options_dbrrt.max_step_size / distance_to_rand);
    } else {
      x_target = x_rand;
    }

    std::vector<LazyTraj> lazy_trajs;

    expander.expand_lazy(near_node->state_eig, lazy_trajs);

    double min_distance = std::numeric_limits<double>::max();
    int best_index = -1;
    dynobench::Trajectory chosen_traj_debug;
    LazyTraj chosen_lazy_traj;

    for (size_t i = 0; i < lazy_trajs.size(); i++) {

      auto &lazy_traj = lazy_trajs[i];
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      bool motion_valid = check_lazy_trajectory(lazy_traj, *robot, time_bench,
                                                traj_wrapper, nullptr, nullptr);

      if (!motion_valid)
        continue;

      double d = robot->distance(
          traj_wrapper.get_state(traj_wrapper.get_size() - 1), x_target);

      if (d < min_distance) {
        min_distance = d;
        best_index = i;
        chosen_lazy_traj = lazy_traj;
        __expand_start = traj_wrapper.get_state(0);
        __expand_end = traj_wrapper.get_state(traj_wrapper.get_size() - 1);

        if (options_dbrrt.debug) {
          chosen_traj_debug = dynobench::trajWrapper_2_Trajectory(traj_wrapper);
        }

        if (options_dbrrt.choose_first_motion_valid)
          break;
      }
    }

    if (best_index != -1) {

      AStarNode *new_node = new AStarNode();
      new_node->state_eig = __expand_end;
      new_node->hScore =
          robot->lower_bound_time(new_node->state_eig, problem.goal);
      new_node->came_from = near_node;
      new_node->used_motion = chosen_lazy_traj.motion->idx;

      new_node->gScore =
          near_node->gScore + chosen_lazy_traj.motion->cost +
          options_dbrrt.cost_jump *
              robot->lower_bound_time(near_node->state_eig, __expand_start);

      new_node->fScore = new_node->gScore + new_node->hScore;

      if (options_dbrrt.ao_rrt &&
          new_node->gScore + new_node->hScore >
              options_dbrrt.best_cost_prune_factor * cost_bound) {
        // std::cout << "warning:
        // "
        //           << "cost of
        //           new is above
        //           bound"
        //           <<
        //           std::endl;
        continue;
      }

      nearest_state_timed(new_node, tmp, T_n, time_bench);
      // TODO: this considers
      // also time in the case of
      // AORRT...

      if (robot->distance(tmp->state_eig, new_node->state_eig) <
          options_dbrrt.delta / 2.) {
        std::cout << "warning: node "
                     "already in the "
                     "tree"
                  << std::endl;
        if (options_dbrrt.ao_rrt) {

          if (new_node->gScore >=
              options_dbrrt.best_cost_prune_factor * tmp->gScore - 1e-12) {
            delete new_node;
            continue;
          }
          std::cout << "but adding "
                       "because best "
                       "cost! -- "
                    << new_node->gScore << " " << tmp->gScore << std::endl;
          // TODO: should I
          // rewire the tree?

        } else {
          delete new_node;
          continue;
        }
      }

      add_state_timed(new_node, T_n, time_bench);
      discovered_nodes.push_back(new_node);

      if (options_dbrrt.debug) {
        chosen_trajs.push_back(chosen_traj_debug);
      }

      double di = robot->distance(new_node->state_eig, problem.goal);

      if (di < best_distance_to_goal) {
        best_distance_to_goal = di;
        best_node = new_node;
      }

      if (di < options_dbrrt.goal_region) {

        solution = new_node;

        if (options_dbrrt.debug) {
          std::vector<AStarNode *> active_nodes;
          T_n->list(active_nodes);
          plot_search_tree(active_nodes, motions, *robot,
                           ("/tmp/dynoplan/"
                            "db_rrt_tree_" +
                            std::to_string(info_out.trajs_raw.size()) + ".yaml")
                               .c_str());
        }

        status = Terminate_status::SOLVED_RAW;
        all_solutions_raw.push_back(solution);

        CSTR_V(new_node->state_eig);
        info_out.solved_raw = true;
        std::cout << "success! "
                     "GOAL_REACHED"
                  << std::endl;

        // TODO: dont write to
        // much to file!!
        std::ofstream file_debug("/tmp/dynoplan/"
                                 "db_rrt_debug_" +
                                 std::to_string(info_out.trajs_raw.size()) +
                                 ".yaml");
        dynobench::Trajectory traj_db;
        from_solution_to_yaml_and_traj(*robot, motions, solution, problem,
                                       traj_db, &file_debug);

        traj_db.to_yaml_format("/tmp/dynoplan/"
                               "db_rrt_traj_" +
                               std::to_string(info_out.trajs_raw.size()) +
                               ".yaml");

        info_out.trajs_raw.push_back(traj_db);

        if (options_dbrrt.do_optimization) {
          Stopwatch sw;
          dynobench::Trajectory traj_opt;
          Result_opti result;

          trajectory_optimization(problem, traj_db, options_trajopt, traj_opt,
                                  result);

          traj_opt.to_yaml_format("/tmp/dynoplan/"
                                  "db_rrt_traj_"
                                  "opt_" +
                                  std::to_string(info_out.trajs_opt.size()) +
                                  ".yaml");

          if (result.feasible == 1) {
            std::cout << "success: "
                         "optimization"
                         " is "
                         "feasible!"
                      << std::endl;
            info_out.solved = true;

            if (result.cost < best_cost_opt) {
              best_traj_opt = traj_opt;
            }

            info_out.trajs_opt.push_back(traj_opt);

            if (options_dbrrt.extract_primitives) {
              // ADD motions to
              // the end of the
              // list, and
              // rebuild the
              // tree.
              size_t number_of_cuts = 5;
              dynobench::Trajectories new_trajectories =
                  cut_trajectory(traj_opt, number_of_cuts, robot);
              dynobench::Trajectories trajs_canonical;
              make_trajs_canonical(*robot, new_trajectories.data,
                                   trajs_canonical.data);

              const bool add_noise_first_state = true;
              const double noise = 1e-7;
              for (auto &t : trajs_canonical.data) {
                t.states.front() +=
                    noise * Eigen::VectorXd::Random(t.states.front().size());
                t.states.back() +=
                    noise * Eigen::VectorXd::Random(t.states.back().size());

                if (startsWith(robot->name, "quad3"
                                            "d")) {
                  t.states.front().segment<4>(3).normalize();
                  t.states.back().segment<4>(3).normalize();
                }
              }

              std::vector<Motion> motions_out;
              for (const auto &traj : trajs_canonical.data) {
                Motion motion_out;
                CHECK(robot, AT)
                motion_out.traj = traj;
                motion_out.cost = traj.cost;
                motion_out.idx = motions.size() + motions_out.size();
                std::cout << "cost of "
                             "motion "
                             "is "
                          << motion_out.cost << std::endl;
                motions_out.push_back(std::move(motion_out));
              }

              motions.insert(motions.end(),
                             std::make_move_iterator(motions_out.begin()),
                             std::make_move_iterator(motions_out.end()));

              std::cout << "Afer "
                           "insert "
                        << motions.size() << std::endl;
              std::cout << "Warning: "
                        << "I am "
                           "inserting "
                           "at the end"
                        << std::endl;

              T_m->clear();

              for (auto &m : motions) {
                T_m->add(&m);
              }

              std::cout << "TODO: "
                           "insert "
                           "also the "
                           "nodes in "
                           "the tree"
                        << std::endl;
            }

            if (options_dbrrt.add_to_search_tree) {

              NOT_IMPLEMENTED;
            }

          } else {
            std::cout << "warning: "
                         "optimization"
                         " failed"
                      << std::endl;
          }
        }

        if (!options_dbrrt.ao_rrt) {
          if ((options_dbrrt.do_optimization && info_out.solved) ||
              !options_dbrrt.do_optimization) {
            break;
          }
        } else {
          std::cout << "warning"
                    << "i am pruning "
                       "with cost of "
                       "raw solution"
                    << std::endl;
          DYNO_CHECK_LEQ(new_node->gScore, cost_bound, AT);
          cost_bound = new_node->gScore;
          solution = new_node;
          std::cout << "New solution "
                       "found! Cost "
                    << cost_bound << std::endl;

          if (options_dbrrt.ao_rrt_rebuild_tree) {

            if (options_dbrrt.debug) {
              std::vector<AStarNode *> active_nodes;
              T_n->list(active_nodes);
              plot_search_tree(active_nodes, motions, *robot,
                               ("/tmp/"
                                "dynoplan/"
                                "db_rrt_tree_"
                                "before_"
                                "prune_" +
                                std::to_string(info_out.trajs_raw.size()) +
                                ".yaml")
                                   .c_str());
            }

            T_n->clear();
            std::cout << "Tree size "
                         "before "
                         "prunning "
                      << T_n->size() << std::endl;
            for (auto &n : discovered_nodes) {
              if (n->gScore + n->hScore <=
                  options_dbrrt.best_cost_prune_factor * cost_bound) {

                add_state_timed(n, T_n, time_bench);
              }
            }
            std::cout << "Tree after "
                         "prunning "
                      << T_n->size() << std::endl;

            if (options_dbrrt.debug) {
              std::vector<AStarNode *> active_nodes;
              T_n->list(active_nodes);
              plot_search_tree(active_nodes, motions, *robot,
                               ("/tmp/"
                                "dynoplan/"
                                "db_rrt_tree_"
                                "after_"
                                "prune_" +
                                std::to_string(info_out.trajs_raw.size()) +
                                ".yaml")
                                   .c_str());
            }
          }
        }
      }
    } else {
      std::cout << "Warning: all "
                   "expansions failed "
                   "in state "
                << near_node->state_eig.format(FMT) << std::endl;
    }
  }
  time_bench.time_search = watch.elapsed_ms();
  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;
  std::cout << "solved_raw: " << (solution != nullptr) << std::endl;
  std::cout << "solved_opt:" << bool(info_out.trajs_opt.size()) << std::endl;
  std::cout << "TIME in search:" << time_bench.time_search << std::endl;
  std::cout << "sizeTN: " << T_n->size() << std::endl;

  if (solution) {
    std::cout << "cost: " << solution->gScore << std::endl;
  } else {
    std::cout << "Close distance: " << best_distance_to_goal << std::endl;
  }

  std::cout << "best node: " << std::endl;
  best_node->write(std::cout);

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  if (options_dbrrt.debug) {
    std::ofstream debug_file("debug.yaml");
    std::ofstream debug_file2("debug2.yaml");
    debug_file << "rand_nodes:" << std::endl;
    for (auto &q : rand_nodes) {
      debug_file << "  - " << q.format(FMT) << std::endl;
    }

    debug_file << "near_nodes:" << std::endl;
    for (auto &q : near_nodes) {
      debug_file << "  - " << q.format(FMT) << std::endl;
    }

    debug_file << "discovered_nodes:" << std::endl;
    for (auto &q : discovered_nodes) {
      debug_file << "  - " << q->state_eig.format(FMT) << std::endl;
    }

    debug_file << "chosen_trajs:" << std::endl;
    for (auto &traj : chosen_trajs) {
      debug_file << "  - " << std::endl;
      traj.to_yaml_format(debug_file, "    ");
    }

    debug_file2 << "trajs:" << std::endl;
    for (auto &traj : trajs) {
      debug_file2 << "  - " << std::endl;
      traj.to_yaml_format(debug_file2, "    ");
    }
  }

  std::ofstream out("out_dbrrt.yaml");

  out << "solved: " << bool(solution) << std::endl;
  out << "status: " << static_cast<int>(status) << std::endl;
  out << "status_str: " << terminate_status_str[static_cast<int>(status)]
      << std::endl;
  out << "sizeTN: " << T_n->size() << std::endl;
  time_bench.write(out);

  if (solution) {

    out << "result:" << std::endl;

    from_solution_to_yaml_and_traj(*robot, motions, solution, problem, traj_out,
                                   &out);

    std::vector<dynobench::Trajectory> trajs_out(all_solutions_raw.size());

    for (size_t i = 0; i < all_solutions_raw.size(); i++) {

      std::string filename = "/tmp/dynoplan/"
                             "dbrrt-" +
                             std::to_string(i) + ".yaml";

      create_dir_if_necessary(filename);
      std::cout << "writing to " << filename << std::endl;
      std::ofstream out(filename);
      from_solution_to_yaml_and_traj(*robot, motions, all_solutions_raw.at(i),
                                     problem, trajs_out.at(i), &out);
      std::string filename2 = "/tmp/dynoplan/"
                              "dbrrt-" +
                              std::to_string(i) + ".traj.yaml";
      std::cout << "writing to " << filename2 << std::endl;
      std::ofstream out2(filename2);
      create_dir_if_necessary(filename2);
      trajs_out.at(i).to_yaml_format(out2);
    }

    // also save the trajectories
  }

  if (info_out.solved) {
    CHECK(info_out.solved_raw, AT);
  }

  std::cout << "warning: update the trajecotries cost" << std::endl;
  std::for_each(
      info_out.trajs_raw.begin(), info_out.trajs_raw.end(),
      [&](auto &traj) { traj.cost = robot->ref_dt * traj.actions.size(); });

  std::for_each(
      info_out.trajs_opt.begin(), info_out.trajs_opt.end(),
      [&](auto &traj) { traj.cost = robot->ref_dt * traj.actions.size(); });

  if (info_out.solved) {

    info_out.cost =
        std::min_element(
            info_out.trajs_opt.begin(), info_out.trajs_opt.end(),
            [](const auto &a, const auto &b) { return a.cost < b.cost; })
            ->cost;
  }

  if (info_out.solved_raw) {
    info_out.cost_raw =
        std::min_element(
            info_out.trajs_raw.begin(), info_out.trajs_raw.end(),
            [](const auto &a, const auto &b) { return a.cost < b.cost; })
            ->cost;
  }
}
} // namespace dynoplan
