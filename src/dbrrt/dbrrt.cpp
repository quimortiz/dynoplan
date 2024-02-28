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

#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/dbastar/dbastar.hpp"
// #include "ocp.hpp"
#include "dynoplan/ompl/robots.h"
#include "dynoplan/optimization/ocp.hpp"

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
  CSTR_(result.size());

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

    CSTR_V(__offset);
    traj_wrapper.allocate_size(motion.traj.states.size(),
                               motion.traj.states.front().size(),
                               motion.traj.actions.front().size());

    // CONTINUE HERE!!

    // robot.transform_primitive(__offset, motion.traj.states,
    // motion.traj.actions,
    //                           traj_wrapper);
    robot.transform_primitiveDirectReverse(__offset, motion.traj.states,
                                           motion.traj.actions, traj_wrapper,
                                           nullptr, nullptr);

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
    create_dir_if_necessary("/tmp/dynoplan");
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
  Eigen::VectorXd xf_canonical(robot.nx);
  Eigen::VectorXd offset(robot.get_offset_dim());
  for (auto &m : motions) {
    Motion mrev;

    dynobench::Trajectory traj_new;

    dynobench::TrajWrapper traj_wrapper;
    traj_wrapper.allocate_size(m.traj.states.size(),
                               m.traj.states.front().size(),
                               m.traj.actions.front().size());

    // std::cout << "original trajectory" << std::endl;
    // m.traj.to_yaml_format(std::cout);

    if (startsWith(robot.name, "unicycle") || startsWith(robot.name, "car")) {
      robot.offset(m.traj.states.back(), offset);
    } else if (startsWith(robot.name, "quad2d") &&
               !startsWith(robot.name, "quad2dpole")) {
      DYNO_CHECK_EQ(offset.size(), 4, "should be 4");
      offset.tail<2>() = m.traj.states.back().segment<2>(3);
      offset.head<2>() = m.traj.states.back().head<2>() -
                         m.traj.actions.size() * robot.ref_dt *
                             m.traj.states.back().segment<2>(3);
    } else if (startsWith(robot.name, "quad3d")) {
      DYNO_CHECK_EQ(offset.size(), 6, "should be 6");
      offset.tail<3>() = m.traj.states.back().segment<3>(7);
      offset.head<3>() = m.traj.states.back().head<3>() -
                         m.traj.actions.size() * robot.ref_dt *
                             m.traj.states.back().segment<3>(7);

    } else if (startsWith(robot.name, "quad2dpole")) {
      // 0:x, 1:y , 2:yaw, 3:theta , 4:vx , 5:vy , 6:w
      offset.tail<2>() = m.traj.states.back().segment<2>(4);
      // tail<3>() = m.traj.states.back().segment<3>(7);
      offset.head<2>() = m.traj.states.back().head<2>() -
                         m.traj.actions.size() * robot.ref_dt *
                             m.traj.states.back().segment<2>(4);
    } else if (startsWith(robot.name, "acrobot")) {
      // Don't do anything for the acrobot

    } else {
      std::string msg = "robot name " + robot.name + "not supported";
      ERROR_WITH_INFO(msg);
    }

    robot.transform_primitive(-offset, m.traj.states, m.traj.actions,
                              traj_wrapper);

    traj_new = dynobench::trajWrapper_2_Trajectory(traj_wrapper);

    // CSTR_V(offset);
    // traj_new.to_yaml_format(std::cout);

    std::reverse(traj_new.states.begin(), traj_new.states.end());
    std::reverse(traj_new.actions.begin(), traj_new.actions.end());

    Eigen::VectorXd __offset(robot.get_offset_dim());
    robot.offset(traj_new.states.front(), __offset);

    // CSTR_V(__offset);
    DYNO_CHECK_LEQ(__offset.norm(), 1e-5, "offset should be zero");

    const bool add_noise_first_state = true;
    const double noise = 1e-7;
    if (add_noise_first_state) {
      traj_new.states.front() +=
          noise * Eigen::VectorXd::Random(traj_new.states.front().size());
      traj_new.states.back() +=
          noise * Eigen::VectorXd::Random(traj_new.states.back().size());

      if (startsWith(robot.name, "quad3d")) {
        // ensure quaternion
        for (auto &s : traj_new.states) {
          s.segment<4>(3).normalize();
        }
      }

      // TODO: robot should have "add noise function"
    }

    traj_to_motion(traj_new, robot, mrev, true);

    mrev.idx = m.idx;

    motions_rev.push_back(std::move(mrev));
  }
}

// refactor: take optimization OUT!!
void dbrrtConnect(const dynobench::Problem &problem,
                  std::shared_ptr<dynobench::Model_robot> robot,
                  const Options_dbrrt &options_dbrrt,
                  const Options_trajopt &options_trajopt,
                  dynobench::Trajectory &traj_out,
                  dynobench::Info_out &info_out) {

  const bool debug_extra = false; // set to true for extra debug output

  std::cout << "options dbrrt" << std::endl;
  options_dbrrt.print(std::cout);
  std::cout << "***" << std::endl;

  const int nx = robot->nx;

  std::vector<Motion> &motions = *options_dbrrt.motions_ptr;
  std::vector<Motion> motions_rev;

  CHECK(options_dbrrt.motions_ptr, AT);
  DYNO_CHECK_EQ(motions.at(0).traj.states.front().size(), nx, AT);

  reverse_motions(motions_rev, *robot, motions);

  DYNO_CHECK_EQ(motions.at(0).traj.states.front().size(), nx, AT);

  std::cout << "example motions " << std::endl;
  assert(motions.size());
  assert(motions_rev.size());
  motions.front().traj.to_yaml_format(std::cout);
  motions_rev.front().traj.to_yaml_format(std::cout);
  std::cout << "DONE " << std::endl;

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

  if (options_dbrrt.seed >= 0) {
    expander.seed(options_dbrrt.seed);
    expander_rev.seed(options_dbrrt.seed);
    g = std::mt19937{static_cast<size_t>(options_dbrrt.seed)};
    srand(options_dbrrt.seed);
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

    if (watch.elapsed_ms() > options_dbrrt.timelimit) {
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
  Eigen::VectorXd aux_last_state(robot->nx);

  dynobench::Trajectory traj_out_fwd, traj_out_bwd;
  bool expand_to_rand = true;
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
        expand_to_rand = false;
      } else {
        std::uniform_int_distribution<> distr(0, T_n->size() - 1);
        int idx = distr(g);
        x_rand = nodes_in_Tn.at(idx)->state_eig;
        expand_to_rand = false;
      }
    } else {
      robot->sample_uniform(x_rand);
      expand_to_rand = true;
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

    Eigen::VectorXd offset(robot->get_offset_dim());

    if (!near_node->motions.size()) {
      if (expand_forward)
        time_bench.time_lazy_expand += timed_fun_void(
            [&] { expander.expand_lazy(near_node->state_eig, lazy_trajs); });
      else
        time_bench.time_lazy_expand += timed_fun_void([&] {
          expander_rev.expand_lazy(near_node->state_eig, lazy_trajs);
        });

      near_node->motions.reserve(lazy_trajs.size());

      std::transform(lazy_trajs.begin(), lazy_trajs.end(),
                     std::back_inserter(near_node->motions),
                     [](LazyTraj &lazy_traj) { return lazy_traj.motion->idx; });
    } else {
      lazy_trajs.reserve(near_node->motions.size());
      robot->offset(near_node->state_eig, offset);

      std::vector<Motion> *motions_ptr =
          expand_forward ? &motions : &motions_rev;

      std::transform(near_node->motions.begin(), near_node->motions.end(),
                     std::back_inserter(lazy_trajs), [&](int idx) {
                       return LazyTraj{.offset = &offset,
                                       .robot = robot.get(),
                                       .motion = &(motions_ptr->at(idx))};
                     });
      std::shuffle(lazy_trajs.begin(), lazy_trajs.end(), g);
    }

    double min_distance = std::numeric_limits<double>::max();
    int best_index = -1;
    dynobench::Trajectory chosen_traj_debug;
    LazyTraj chosen_lazy_traj;
    int valid_expansions = 0;
    int max_valid_expansions;

    if (expand_to_rand)
      max_valid_expansions = options_dbrrt.max_valid_expansions_to_rand;
    else
      max_valid_expansions = options_dbrrt.max_valid_expansions_to_goal;

    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      // TODO: check the bounds while expanding!!
      // OR at least, check bounds of last state!
      bool motion_valid = check_lazy_trajectory(
          lazy_traj, *robot, time_bench, traj_wrapper, aux_last_state, nullptr,
          nullptr, expand_forward);

      if (options_dbrrt.debug) {
        trajs.push_back(dynobench::trajWrapper_2_Trajectory(traj_wrapper));
      }

      if (!motion_valid)
        continue;

      valid_expansions++;

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
      }

      if (valid_expansions >= max_valid_expansions)
        break;
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
        // std::cout << "warning: node already in the tree" << std::endl;
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

      if (di < options_dbrrt.goal_region) {
        std::cout << "we have connected the trees!" << std::endl;

        if (expand_forward) {
          solution_fwd = new_node;
          solution_bwd = tmp;
        } else {
          solution_fwd = tmp;
          solution_bwd = new_node;
        }

        status = Terminate_status::SOLVED_RAW;

        info_out.solved_raw = true;
        std::cout << "success! GOAL_REACHED" << std::endl;
        std::cout << "node fwd " << solution_fwd->state_eig.format(FMT)
                  << std::endl;
        std::cout << "node bwd " << solution_bwd->state_eig.format(FMT)
                  << std::endl;
        status = Terminate_status::SOLVED_RAW;
        std::cout << "breaking search" << std::endl;
        break;
        // TODO: dont write to much to file!!
      }
    } else {
      // std::cout << "Warning: all expansions failed "
      //              "in state "
      //           << near_node->state_eig.format(FMT) << std::endl;
    }
  }

  time_bench.time_search = watch.elapsed_ms();
  time_bench.time_nearestMotion +=
      expander.time_in_nn + expander_rev.time_in_nn;
  std::cout << "expander.time_in_nn: " << expander.time_in_nn << std::endl;
  std::cout << "expander_rev.time_in_nn: " << expander_rev.time_in_nn
            << std::endl;

  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;
  std::cout << "solved_raw: " << (solution_bwd && solution_fwd != nullptr)
            << std::endl;
  std::cout << "solved_opt:" << bool(info_out.trajs_opt.size()) << std::endl;
  std::cout << "TIME in search:" << time_bench.time_search << std::endl;
  std::cout << "sizeTN: " << T_n->size() << std::endl;
  std::cout << "sizeTN_rev: " << T_nrev->size() << std::endl;

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  if (solution_bwd && solution_fwd) {
    std::cout << "SOLVED: cost: " << solution_fwd->gScore + solution_bwd->gScore
              << std::endl;

    std::unique_ptr<std::ofstream> file_debug_ptr = nullptr;

    if (options_dbrrt.debug) {
      file_debug_ptr = std::make_unique<std::ofstream>(
          "/tmp/dynoplan/db_rrt_debug_" +
          std::to_string(info_out.trajs_raw.size()) + ".yaml");
    }

    from_fwd_bwd_solution_to_yaml_and_traj(
        *robot, motions, motions_rev, solution_fwd, solution_bwd, problem,
        traj_out, traj_out_fwd, traj_out_bwd,
        file_debug_ptr ? file_debug_ptr.get() : nullptr);

  } else {
    std::cout << "NOT SOLVED" << std::endl;
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

    if (info_out.solved_raw) {
      traj_out.to_yaml_format("/tmp/dynoplan/db_rrt_traj_" +
                              std::to_string(info_out.trajs_raw.size()) +
                              ".yaml");

      traj_out_fwd.to_yaml_format("/tmp/dynoplan/db_rrt_traj_fwd_" +
                                  std::to_string(info_out.trajs_raw.size()) +
                                  ".yaml");

      traj_out_bwd.to_yaml_format("/tmp/dynoplan/db_rrt_traj_bwd_" +
                                  std::to_string(info_out.trajs_raw.size()) +
                                  ".yaml");
    }
  }

  info_out.trajs_raw.push_back(traj_out);

  info_out.data.insert(
      std::make_pair("time_search", std::to_string(time_bench.time_search)));

  if (debug_extra && options_dbrrt.debug) {
    std::ofstream debug_file("/tmp/dynoplan/debug.yaml");
    std::ofstream debug_file2("/tmp/dynoplan/debug2.yaml");
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

  std::string filename_out = "/tmp/dynoplan/out_dbrrt.yaml";
  create_dir_if_necessary(filename_out);
  std::ofstream out(filename_out);

  out << "solved: " << bool(solution_fwd && solution_bwd) << std::endl;
  out << "status: " << static_cast<int>(status) << std::endl;
  out << "status_str: " << terminate_status_str[static_cast<int>(status)]
      << std::endl;
  out << "sizeTN: " << T_n->size() << std::endl;
  time_bench.write(out);

  if (info_out.solved_raw) {
    std::cout << "WARNING: for feasibility check, I use the MAX of goal_region "
                 "and delta:"
              << std::max(options_dbrrt.goal_region, options_dbrrt.delta)
              << std::endl;
    dynobench::Feasibility_thresholds thresholds;
    thresholds.col_tol =
        5 * 1e-2; // NOTE: for the systems with 0.01 s integration step,
    // I check collisions only at 0.05s . Thus, an intermediate state
    // could be slightly in collision.
    thresholds.goal_tol =
        std::max(options_dbrrt.goal_region, options_dbrrt.delta);
    thresholds.traj_tol =
        std::max(options_dbrrt.goal_region, options_dbrrt.delta);
    traj_out.update_feasibility(thresholds, true);
    // Sanity check that trajectory is actually feasible!!
    CHECK(traj_out.feasible, "");
  }

  traj_out.cost = robot->ref_dt * traj_out.actions.size();
  std::cout << "warning: update the trajecotries cost" << std::endl;
  std::for_each(
      info_out.trajs_raw.begin(), info_out.trajs_raw.end(),
      [&](auto &traj) { traj.cost = robot->ref_dt * traj.actions.size(); });
}

// the Original rrt connect implementation
void dbrrtConnectOrig(const dynobench::Problem &problem,
                      std::shared_ptr<dynobench::Model_robot> robot,
                      const Options_dbrrt &options_dbrrt,
                      const Options_trajopt &options_trajopt,
                      dynobench::Trajectory &traj_out,
                      dynobench::Info_out &info_out) {

  const bool debug_extra = false; // set to true for extra debug output

  std::cout << "options dbrrt" << std::endl;
  options_dbrrt.print(std::cout);
  std::cout << "***" << std::endl;

  const int nx = robot->nx;

  std::vector<Motion> &motions = *options_dbrrt.motions_ptr;
  std::vector<Motion> motions_rev;

  CHECK(options_dbrrt.motions_ptr, AT);
  DYNO_CHECK_EQ(motions.at(0).traj.states.front().size(), nx, AT);

  reverse_motions(motions_rev, *robot, motions);

  DYNO_CHECK_EQ(motions.at(0).traj.states.front().size(), nx, AT);

  std::cout << "example motions " << std::endl;
  assert(motions.size());
  assert(motions_rev.size());
  motions.front().traj.to_yaml_format(std::cout);
  motions_rev.front().traj.to_yaml_format(std::cout);
  std::cout << "DONE " << std::endl;

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

  if (options_dbrrt.seed >= 0) {
    expander.seed(options_dbrrt.seed);
    expander_rev.seed(options_dbrrt.seed);
    g = std::mt19937{static_cast<size_t>(options_dbrrt.seed)};
    srand(options_dbrrt.seed);
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

    if (watch.elapsed_ms() > options_dbrrt.timelimit) {
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
  Eigen::VectorXd aux_last_state(robot->nx);

  dynobench::Trajectory traj_out_fwd, traj_out_bwd;

  expand_forward = true;

  int max_num_trials_col_free = 1000;

  enum EXPANSION_MODE { fwd_to_rand, bwd_to_rand, bwd_to_fwd, fwd_to_bwd };

  const char *expansion_mode_str[] = {"fwd_to_rand", "bwd_to_rand",
                                      "bwd_to_fwd", "fwd_to_bwd"};

  EXPANSION_MODE mode = fwd_to_rand;

  while (!stop_search()) {
    if (time_bench.expands % print_every == 0)
      print_search_status();

    // std::cout << "expand_mode: " << expansion_mode_str[mode] << std::endl;

    time_bench.expands++;

    if (mode == fwd_to_rand || mode == fwd_to_bwd) {
      expand_forward = true;
    } else {
      expand_forward = false;
    }

    if (mode == fwd_to_rand || mode == bwd_to_rand) {

      bool is_collision_free = false;
      int num_tries = 0;
      while (!is_collision_free && num_tries < max_num_trials_col_free) {
        robot->sample_uniform(x_rand);
        is_collision_free = robot->collision_check(x_rand);
        num_tries++;
      }
      DYNO_CHECK(is_collision_free, "cannot generate a valid xrand");

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
    } else {
      // x_target and near_node are set at the end of the previous iteration
    }

    std::vector<LazyTraj> lazy_trajs;

    Eigen::VectorXd offset(robot->get_offset_dim());

    if (!near_node->motions.size()) {
      if (expand_forward)
        time_bench.time_lazy_expand += timed_fun_void(
            [&] { expander.expand_lazy(near_node->state_eig, lazy_trajs); });
      else
        time_bench.time_lazy_expand += timed_fun_void([&] {
          expander_rev.expand_lazy(near_node->state_eig, lazy_trajs);
        });

      near_node->motions.reserve(lazy_trajs.size());

      std::transform(lazy_trajs.begin(), lazy_trajs.end(),
                     std::back_inserter(near_node->motions),
                     [](LazyTraj &lazy_traj) { return lazy_traj.motion->idx; });
    } else {
      // std::cout << "reusing nn" << std::endl;
      lazy_trajs.reserve(near_node->motions.size());
      robot->offset(near_node->state_eig, offset);

      std::vector<Motion> *motions_ptr =
          expand_forward ? &motions : &motions_rev;

      std::transform(near_node->motions.begin(), near_node->motions.end(),
                     std::back_inserter(lazy_trajs), [&](int idx) {
                       return LazyTraj{.offset = &offset,
                                       .robot = robot.get(),
                                       .motion = &(motions_ptr->at(idx))};
                     });
      std::shuffle(lazy_trajs.begin(), lazy_trajs.end(), g);
    }

    double min_distance = std::numeric_limits<double>::max();
    int best_index = -1;
    dynobench::Trajectory chosen_traj_debug;
    LazyTraj chosen_lazy_traj;
    int valid_expansions = 0;

    // If I go to random points i only expands once.
    // if i try to reach the other tree, i expand some
    // and pick the one that brings me closer.

    int max_valid_expansions;

    if (mode == fwd_to_rand || mode == bwd_to_rand)
      max_valid_expansions = options_dbrrt.max_valid_expansions_to_rand;
    else
      max_valid_expansions = options_dbrrt.max_valid_expansions_to_goal;

    for (size_t i = 0; i < lazy_trajs.size(); i++) {
      auto &lazy_traj = lazy_trajs[i];
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      // TODO: check the bounds while expanding!!
      // OR at least, check bounds of last state!
      bool motion_valid = check_lazy_trajectory(
          lazy_traj, *robot, time_bench, traj_wrapper, aux_last_state, nullptr,
          nullptr, expand_forward);

      if (options_dbrrt.debug) {
        trajs.push_back(dynobench::trajWrapper_2_Trajectory(traj_wrapper));
      }

      if (!motion_valid)
        continue;

      valid_expansions++;

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
      }

      if (max_valid_expansions)
        break;
    }

    if (best_index != -1) {
      AStarNode *new_node = new AStarNode();
      new_node->state_eig = __expand_end;
      // THIS has no meaning in RRT Connect!
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

      // TODO: move this where I check the if
      if (robot->distance(tmp->state_eig, new_node->state_eig) <
          options_dbrrt.delta / 2.) {
        delete new_node; // TODO: use unique ptrs

        if (mode == fwd_to_rand) {
          mode = bwd_to_rand;
        } else if (mode == bwd_to_rand) {
          mode = fwd_to_rand;
        } else if (mode == fwd_to_bwd) {
          mode = fwd_to_rand;
        } else if (mode == bwd_to_fwd) {
          mode = bwd_to_rand;
        }

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

      if (di < options_dbrrt.goal_region) {
        std::cout << "we have connected the trees!" << std::endl;

        if (expand_forward) {
          solution_fwd = new_node;
          solution_bwd = tmp;
        } else {
          solution_fwd = tmp;
          solution_bwd = new_node;
        }

        status = Terminate_status::SOLVED_RAW;

        info_out.solved_raw = true;
        std::cout << "success! GOAL_REACHED" << std::endl;
        std::cout << "node fwd " << solution_fwd->state_eig.format(FMT)
                  << std::endl;
        std::cout << "node bwd " << solution_bwd->state_eig.format(FMT)
                  << std::endl;
        status = Terminate_status::SOLVED_RAW;
        std::cout << "breaking search" << std::endl;
        break;
        // TODO: dont write to much to file!!
      }

      // if i was expanding toward random, then i will expand the other tree

      // std::cout << "change from mode " << expansion_mode_str[mode] <<
      // std::endl;

      if (mode == fwd_to_rand) {
        mode = bwd_to_fwd;
        x_target = new_node->state_eig;
        near_node = tmp;
      } else if (mode == bwd_to_rand) {
        mode = fwd_to_bwd;
        x_target = new_node->state_eig;
        near_node = tmp;
      } else if (mode == fwd_to_bwd) {
        mode = fwd_to_rand;
      } else if (mode == bwd_to_fwd) {
        mode = bwd_to_rand;
      }

    } else {

      // std::cout << "change from mode " << expansion_mode_str[mode] <<
      // std::endl; expansion has failed,
      if (mode == fwd_to_rand) {
        mode = bwd_to_rand;
      } else if (mode == bwd_to_rand) {
        mode = fwd_to_rand;
      } else if (mode == fwd_to_bwd) {
        mode = fwd_to_rand;
      } else if (mode == bwd_to_fwd) {
        mode = bwd_to_rand;
      }

      // std::cout << "to " << expansion_mode_str[mode] << std::endl;

      // std::cout << "Warning: all expansions failed "
      //              "in state "
      //           << near_node->state_eig.format(FMT) << std::endl;
    }
  }

  time_bench.time_search = watch.elapsed_ms();
  time_bench.time_nearestMotion +=
      expander.time_in_nn + expander_rev.time_in_nn;
  std::cout << "expander.time_in_nn: " << expander.time_in_nn << std::endl;
  std::cout << "expander_rev.time_in_nn: " << expander_rev.time_in_nn
            << std::endl;

  std::cout << "Terminate status: " << static_cast<int>(status) << " "
            << terminate_status_str[static_cast<int>(status)] << std::endl;
  std::cout << "solved_raw: " << (solution_bwd && solution_fwd != nullptr)
            << std::endl;
  std::cout << "solved_opt:" << bool(info_out.trajs_opt.size()) << std::endl;
  std::cout << "TIME in search:" << time_bench.time_search << std::endl;
  std::cout << "sizeTN: " << T_n->size() << std::endl;
  std::cout << "sizeTN_rev: " << T_nrev->size() << std::endl;

  std::cout << "time_bench:" << std::endl;
  time_bench.write(std::cout);

  if (solution_bwd && solution_fwd) {
    std::cout << "SOLVED: cost: " << solution_fwd->gScore + solution_bwd->gScore
              << std::endl;

    std::unique_ptr<std::ofstream> file_debug_ptr = nullptr;

    if (options_dbrrt.debug) {
      file_debug_ptr = std::make_unique<std::ofstream>(
          "/tmp/dynoplan/db_rrt_debug_" +
          std::to_string(info_out.trajs_raw.size()) + ".yaml");
    }

    from_fwd_bwd_solution_to_yaml_and_traj(
        *robot, motions, motions_rev, solution_fwd, solution_bwd, problem,
        traj_out, traj_out_fwd, traj_out_bwd,
        file_debug_ptr ? file_debug_ptr.get() : nullptr);

  } else {
    std::cout << "NOT SOLVED" << std::endl;
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

    if (info_out.solved_raw) {
      traj_out.to_yaml_format("/tmp/dynoplan/db_rrt_traj_" +
                              std::to_string(info_out.trajs_raw.size()) +
                              ".yaml");

      traj_out_fwd.to_yaml_format("/tmp/dynoplan/db_rrt_traj_fwd_" +
                                  std::to_string(info_out.trajs_raw.size()) +
                                  ".yaml");

      traj_out_bwd.to_yaml_format("/tmp/dynoplan/db_rrt_traj_bwd_" +
                                  std::to_string(info_out.trajs_raw.size()) +
                                  ".yaml");
    }
  }

  info_out.trajs_raw.push_back(traj_out);

  info_out.data.insert(
      std::make_pair("time_search", std::to_string(time_bench.time_search)));

  if (debug_extra && options_dbrrt.debug) {
    std::ofstream debug_file("/tmp/dynoplan/debug.yaml");
    std::ofstream debug_file2("/tmp/dynoplan/debug2.yaml");
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

  std::string filename_out = "/tmp/dynoplan/out_dbrrt.yaml";
  create_dir_if_necessary(filename_out);
  std::ofstream out(filename_out);

  out << "solved: " << bool(solution_fwd && solution_bwd) << std::endl;
  out << "status: " << static_cast<int>(status) << std::endl;
  out << "status_str: " << terminate_status_str[static_cast<int>(status)]
      << std::endl;
  out << "sizeTN: " << T_n->size() << std::endl;
  time_bench.write(out);

  if (info_out.solved_raw) {
    std::cout << "WARNING: for feasibility check, I use the MAX of goal_region "
                 "and delta:"
              << std::max(options_dbrrt.goal_region, options_dbrrt.delta)
              << std::endl;
    dynobench::Feasibility_thresholds thresholds;
    thresholds.col_tol =
        5 * 1e-2; // NOTE: for the systems with 0.01 s integration step,
    // I check collisions only at 0.05s . Thus, an intermediate state
    // could be slightly in collision.
    thresholds.goal_tol =
        std::max(options_dbrrt.goal_region, options_dbrrt.delta);
    thresholds.traj_tol =
        std::max(options_dbrrt.goal_region, options_dbrrt.delta);
    traj_out.update_feasibility(thresholds, true);
    // Sanity check that trajectory is actually feasible!!
    CHECK(traj_out.feasible, "");
  }

  std::cout << "warning: update the trajecotries cost" << std::endl;
  traj_out.cost = robot->ref_dt * traj_out.actions.size();
  std::for_each(
      info_out.trajs_raw.begin(), info_out.trajs_raw.end(),
      [&](auto &traj) { traj.cost = robot->ref_dt * traj.actions.size(); });
}

template <typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator &g) {
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  std::advance(start, dis(g));
  return start;
}

void dbrrt(const dynobench::Problem &problem,
           std::shared_ptr<dynobench::Model_robot> robot,
           const Options_dbrrt &options_dbrrt,
           const Options_trajopt &options_trajopt,
           dynobench::Trajectory &traj_out, dynobench::Info_out &info_out) {

  std::cout << "options dbrrt" << std::endl;
  options_dbrrt.print(std::cout);
  std::cout << "***" << std::endl;

  std::vector<Motion> &motions = *options_dbrrt.motions_ptr;
  CHECK(options_dbrrt.motions_ptr, AT);

  Time_benchmark time_bench;
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  if (options_dbrrt.use_nigh_nn) {
    T_m = nigh_factory2<Motion *>(problem.robotType, robot);
  } else {
    NOT_IMPLEMENTED;
  }

  assert(T_m);
  time_bench.time_nearestMotion += timed_fun_void([&] {
    for (auto &m : motions)
      T_m->add(&m);
  });

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

  auto start_node = std::make_unique<AStarNode>();
  start_node->gScore = 0;
  start_node->state_eig = problem.start;
  start_node->hScore =
      robot->lower_bound_time(start_node->state_eig, problem.goal);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->came_from = nullptr;

  auto goal_node = std::make_unique<AStarNode>();
  goal_node->state_eig = problem.goal;

  Eigen::VectorXd x(robot->nx);

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

  std::mt19937 gen = std::mt19937{std::random_device()()};

  if (options_dbrrt.seed >= 0) {
    expander.seed(options_dbrrt.seed);
    gen = std::mt19937{static_cast<size_t>(options_dbrrt.seed)};
    srand(options_dbrrt.seed);
  } else {
    srand(time(0));
  }

  Eigen::VectorXd x_rand(robot->nx), x_target(robot->nx);
  AStarNode *rand_node = new AStarNode;
  AStarNode *near_node = nullptr;
  AStarNode *tmp = nullptr;
  AStarNode *solution = nullptr;
  AStarNode *best_node = start_node.get();

  std::vector<AStarNode *> discovered_nodes, all_solutions_raw;

  time_bench.expands = 0;
  double cost_bound =
      options_dbrrt.cost_bound; //  std::numeric_limits<double>::infinity();

  double best_cost_opt = std::numeric_limits<double>::infinity();
  dynobench::Trajectory best_traj_opt;

  // SEARCH STARTS HERE

  add_state_timed(start_node.get(), T_n, time_bench);
  discovered_nodes.push_back(start_node.get());

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
  Eigen::VectorXd aux_last_state(robot->nx);

  bool use_non_counter_time = true;
  double non_counter_time = 0;

  Eigen::VectorXd aux(robot->nx);
  Stopwatch watch;

  auto stop_search = [&] {
    if (static_cast<size_t>(time_bench.expands) >= options_dbrrt.max_expands) {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;

      return true;
    }

    if (watch.elapsed_ms() > options_dbrrt.timelimit) {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      return true;
    }
    return false;
  };

  std::vector<AStarNode *> neighbors_n;

  while (!stop_search()) {

    if (time_bench.expands % 500 == 0) {
      std::cout << "expands: " << time_bench.expands
                << " best distance: " << best_distance_to_goal
                << " cost bound: " << cost_bound << std::endl;
    }

    time_bench.expands++;

    bool expand_near_goal =
        static_cast<double>(rand()) / RAND_MAX < options_dbrrt.goal_bias;

    if (expand_near_goal) {
      x_rand = problem.goal;
    } else {
      robot->sample_uniform(x_rand);
    }

    rand_node->state_eig = x_rand;

    if (options_dbrrt.ao_rrt) {
      rand_node->gScore = static_cast<double>(rand()) / RAND_MAX * cost_bound;
    }

    const bool expand_near_node_region = true;
    const int top_k_goal_expansion = 100; // what to put here?
    // Another option is to use TOP K

    if (expand_near_node_region && expand_near_goal) {
      time_bench.time_nearestNode_search += timed_fun_void([&] {
        T_n->nearestK(goal_node.get(), top_k_goal_expansion, neighbors_n);
      });

      if (!neighbors_n.size()) {
        // if no neighbors, just expand the closest one
        nearest_state_timed(rand_node, near_node, T_n, time_bench);
      } else {
        // choose one of the closest nodes at random
        auto it = select_randomly(neighbors_n.begin(), neighbors_n.end(), gen);
        near_node = *it;
      }
    } else {
      nearest_state_timed(rand_node, near_node, T_n, time_bench);
    }

    // TODO : I have chosen the goal, sample a node in a RADIUS around the
    // goal, or use K-neighbors.

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
    int chosen_index; // when i reach goal with an intermediate state
    dynobench::Trajectory chosen_traj_debug;
    LazyTraj chosen_lazy_traj;

    int max_valid_expansions;
    if (expand_near_goal)
      max_valid_expansions = options_dbrrt.max_valid_expansions_to_goal;
    else
      max_valid_expansions = options_dbrrt.max_valid_expansions_to_rand;

    int valid_expansions = 0;

    for (size_t i = 0; i < lazy_trajs.size(); i++) {

      auto &lazy_traj = lazy_trajs[i];
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      bool motion_valid =
          check_lazy_trajectory(lazy_traj, *robot, time_bench, traj_wrapper,
                                aux_last_state, nullptr, nullptr);

      if (!motion_valid)
        continue;
      valid_expansions++;

      double d = robot->distance(
          traj_wrapper.get_state(traj_wrapper.get_size() - 1), x_target);

      chosen_index = -1;
#if 1
      check_goal(*robot, aux, problem.goal, traj_wrapper,
                 options_dbrrt.goal_region, 4, chosen_index);
      if (chosen_index != -1) {
        std::cout << "warning: intermediate state in goal region" << std::endl;
      }
#endif

      if (d < min_distance) {
        min_distance = d;
        best_index = i;
        chosen_lazy_traj = lazy_traj;
        __expand_start = traj_wrapper.get_state(0);

        if (chosen_index == -1)
          __expand_end = traj_wrapper.get_state(traj_wrapper.get_size() - 1);
        else {
          // choose the end that is already in the goal region!
          __expand_end = traj_wrapper.get_state(chosen_index);
        }

        if (options_dbrrt.debug) {
          chosen_traj_debug = dynobench::trajWrapper_2_Trajectory(traj_wrapper);
        }

        if (chosen_index != -1)
          break;
      }

      if (valid_expansions >= max_valid_expansions)
        break;
    }

    if (best_index != -1) {

      AStarNode *new_node = new AStarNode();
      new_node->state_eig = __expand_end;
      new_node->hScore =
          robot->lower_bound_time(new_node->state_eig, problem.goal);
      new_node->came_from = near_node;
      new_node->used_motion = chosen_lazy_traj.motion->idx;

      double cost_motion = chosen_index != -1
                               ? chosen_index * robot->ref_dt
                               : (traj_wrapper.get_size() - 1) * robot->ref_dt;

      new_node->gScore =
          near_node->gScore + cost_motion +
          +options_dbrrt.cost_jump *
              robot->lower_bound_time(near_node->state_eig, __expand_start);

      if (chosen_index != -1)
        new_node->intermediate_state = chosen_index;

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
        if (options_dbrrt.debug) {
          std::cout << "warning: node "
                       "already in the "
                       "tree"
                    << std::endl;
        }
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
      // TODO: I could check N points in the motion, as I do in dbastar

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

        traj_db.time_stamp =
            watch.elapsed_ms() - int(use_non_counter_time) * non_counter_time;

        std::string random_id = gen_random(6);

        traj_db.to_yaml_format("/tmp/dynoplan/"
                               "db_rrt_traj_" +
                               std::to_string(info_out.trajs_raw.size()) + "_" +
                               random_id + ".yaml");

        info_out.trajs_raw.push_back(traj_db);

        if (options_dbrrt.do_optimization) {
          Stopwatch sw;
          dynobench::Trajectory traj_opt;
          Result_opti result;

          Stopwatch sw_opti;
          trajectory_optimization(problem, traj_db, options_trajopt, traj_opt,
                                  result);
          double time_ddp_total = std::stof(result.data.at("time_ddp_total"));
          info_out.infos_opt.push_back(result.data);
          non_counter_time += sw_opti.elapsed_ms() - time_ddp_total;

          traj_opt.time_stamp =
              watch.elapsed_ms() - int(use_non_counter_time) * non_counter_time;

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
      if (options_dbrrt.debug) {
        std::cout << "Warning: all "
                     "expansions failed "
                     "in state "
                  << near_node->state_eig.format(FMT) << std::endl;
      }
    }
  }

  time_bench.time_search = watch.elapsed_ms();

  info_out.data.insert(
      std::make_pair("time_search", std::to_string(time_bench.time_search)));

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

  std::map<std::string, std::string> data;
  data = time_bench.to_data();

  data.insert(std::make_pair("terminate_status",
                             terminate_status_str[static_cast<int>(status)]));
  data.insert(std::make_pair("solved", std::to_string(bool(solution))));
  data.insert(std::make_pair("delta", std::to_string(options_dbrrt.delta)));
  data.insert(std::make_pair("num_primitives", std::to_string(motions.size())));

  // info_out.infos_raw.push_back(

  // inline std::map<std::string, std::string> to_data() const {
  //   out_info_db.data);

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
  traj_out.cost = robot->ref_dt * traj_out.actions.size();
  if (info_out.solved_raw) {
    // TODO : use a double check, as i do in other planners
    traj_out.feasible = true;
  }
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

void idbrrt(const dynobench::Problem &problem,
            std::shared_ptr<dynobench::Model_robot> robot,
            const Options_dbrrt &options_dbrrt,
            const Options_trajopt &options_trajopt,
            dynobench::Trajectory &traj_out, dynobench::Info_out &info_out) {
  bool finished = false;
  Options_dbrrt options_dbrrt_local = options_dbrrt;
  options_dbrrt_local.do_optimization = false;

  DYNO_CHECK_EQ(options_dbrrt.ao_rrt, false,
                "ao rrt not supported in this mode");


  // NOTE: in current implementation i don't change the number 
  // of motions because the RRT is always able to find a solution
  double motion_factor = 1.05;

  double delta_factor = .95;
  size_t it = 0;

  Stopwatch watch;
  double accumulated_time_filtered = 0.0;

  int max_it = 1;

  double time_search = 0;
  double time_opt = 0;

  auto tic = std::chrono::high_resolution_clock::now();
  // int max_motions  = options_dbrrt.max_motions;


  while (!finished) {

    if (it > 0) {
      options_dbrrt_local.delta *= delta_factor;
      options_dbrrt_local.goal_region *= delta_factor;
    }

    dynobench::Info_out info_out_local;
    dynobench::Trajectory traj_dbrrt;

    if (options_dbrrt_local.use_connect) {
      dbrrtConnect(problem, robot, options_dbrrt_local, options_trajopt,
                   traj_dbrrt, info_out_local);

    } else if (options_dbrrt_local.use_connect_orig) {
      dbrrtConnectOrig(problem, robot, options_dbrrt_local, options_trajopt,
                       traj_dbrrt, info_out_local);
    } else {
      dbrrt(problem, robot, options_dbrrt_local, options_trajopt, traj_dbrrt,
            info_out_local);
    }

    time_search += std::stof(info_out_local.data.at("time_search"));

    // search_times.push_back(std::stof(info_out_local.data.at("time_search")));

    accumulated_time_filtered +=
        std::stof(info_out_local.data.at("time_search"));

    if (info_out_local.solved_raw) {
      traj_dbrrt.time_stamp = accumulated_time_filtered;

      std::cout << "traj dbrrt feas " << traj_dbrrt.feasible << std::endl;
      // throw -1;
      info_out.trajs_raw.push_back(traj_dbrrt);
      info_out.solved_raw = true;

      if (options_dbrrt.shortcut) {
        std::cout << "doing shortcuting" << std::endl;
        auto tic = std::chrono::high_resolution_clock::now();

        dynobench::Trajectory traj_cut;
        shortcut_v0_iterative(problem, robot, options_dbrrt_local, traj_dbrrt,
                              traj_cut);
        auto toc = std::chrono::high_resolution_clock::now();

        double ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic)
                .count();

        accumulated_time_filtered += ms;

        traj_dbrrt = traj_cut;

        // TODO: decide if i want to include shortcuting into the search time.
      }

      Result_opti result;
      dynobench::Trajectory traj_out_opti;
      trajectory_optimization(problem, traj_dbrrt, options_trajopt,
                              traj_out_opti, result);
      accumulated_time_filtered += std::stof(result.data.at("time_ddp_total"));

      time_opt += std::stof(result.data.at("time_ddp_total"));

      traj_out_opti.time_stamp = accumulated_time_filtered;

      if (result.feasible) {
        traj_out = traj_out_opti;
        info_out.solved = true;
        info_out.trajs_opt.push_back(traj_out_opti);
        info_out.cost = traj_out_opti.cost;
        finished = true;
      }
    }

    it++;

    double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::high_resolution_clock::now() - tic)
                              .count();
    if (elapsed_time / 1000. > options_dbrrt.timelimit) {
      std::cout << "iDb-RRT: breaking because of time limit" << std::endl;
      break;
    }

    if (it >= options_dbrrt.max_idb_it) {
      std::cout << "iDb-RRT: breaking because of outter iterations" << std::endl;
      break;
    }
  }

  info_out.data.insert(
      std::make_pair("time_search", std::to_string(time_search)));
  info_out.data.insert(std::make_pair("time_opt", std::to_string(time_opt)));
}

// shortcuting

struct Shortcut {

  int start;     // where to start
  int motion_id; // which motion to use
  int end;       // end should be replaced
                 //
                 //
                 //
                 //
  friend std::ostream &operator<<(std::ostream &os, const Shortcut &obj) {
    os << "start: " << obj.start;
    os << " motion_id: " << obj.motion_id;
    os << " end: " << obj.end;
    return os;
  }
};

void shortcut_v0(const dynobench::Problem &problem,
                 std::shared_ptr<dynobench::Model_robot> robot,
                 const Options_dbrrt &options_dbrrt,
                 const dynobench::Trajectory &traj_in,
                 dynobench::Trajectory &traj_out) {

  dynobench::Trajectory traj_in_tmp = traj_in;
  traj_in_tmp.check(robot, true);
  double max_jump = traj_in_tmp.max_jump;
  // DYNO_CHECK_LEQ(traj_out.max_jump, delta, "");

  // create a KD tree with states

  // double delta = .5;
  int original_size = traj_in.states.size();
  double rate_early_terminate = .1;

  // If i find a shortcut that improves 20%, i stop early

  double delta = options_dbrrt.delta;

  ompl::NearestNeighbors<AStarNode *> *T_n =
      nigh_factory2<AStarNode *>(problem.robotType, robot);

  std::vector<AStarNode *> states;

  for (size_t i = 0; i < traj_in.states.size(); i++) {
    auto state = new AStarNode;
    state->id = i;
    state->state_eig = traj_in.states.at(i);
    states.push_back(state);
  }

  for (auto &s : states)
    T_n->add(s);

  std::vector<Motion> &motions = *options_dbrrt.motions_ptr;

  ompl::NearestNeighbors<Motion *> *T_m =
      nigh_factory2<Motion *>(problem.robotType, robot);

  for (size_t j = 0; j < motions.size(); j++) {
    T_m->add(&motions.at(j));
  }

  Expander expander(robot.get(), T_m, options_dbrrt.delta);

  Time_benchmark time_bench;
  Eigen::VectorXd aux_last_state(robot->nx);
  int valid_expansions = 0;
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
  int max_shortcut = 0;

  Shortcut best_shortcut;

  Eigen::VectorXd state_to_expand(robot->nx);
  for (size_t k = 0; k < 100; k++) {
    int rand_ind = rand() % traj_in.states.size();

    // std::cout << "trying to cut from k " << k << std::endl;

    std::vector<LazyTraj> lazy_trajs;
    state_to_expand = traj_in.states.at(rand_ind);
    expander.expand_lazy(state_to_expand, lazy_trajs);

    // check how many candidates
    // std::cout << "applicable trajs " << lazy_trajs.size() << std::endl;

    auto tmp_state = new AStarNode;
    // std::cout << "from " << state_to_expand.format(FMT) << std::endl;

    bool early_terminate = false;
    for (size_t i = 0; i < lazy_trajs.size(); i++) {

      auto &lazy_traj = lazy_trajs[i];
      traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
      bool motion_valid =
          check_lazy_trajectory(lazy_traj, *robot, time_bench, traj_wrapper,
                                aux_last_state, nullptr, nullptr);

      if (!motion_valid)
        continue;
      valid_expansions++;

      // check if this motion is close to any state.

      std::vector<AStarNode *> neighs;
      std::vector<AStarNode *> neighsK;

      Eigen::VectorXd state_to_check =
          traj_wrapper.get_state(traj_wrapper.get_size() - 1);
      tmp_state->state_eig = state_to_check;
      T_n->nearestR(tmp_state, delta, neighs);

      // std::cout << "num neighs " << neighs.size() << std::endl;

      T_n->nearestK(tmp_state, 1, neighsK);

      auto nearest = neighsK.at(0);
      double d =
          robot->distance(traj_wrapper.get_state(traj_wrapper.get_size() - 1),
                          nearest->state_eig);

      for (size_t j = 0; j < neighs.size(); j++) {
        int shortcut = neighs.at(j)->id - rand_ind - traj_wrapper.get_size();
        if (shortcut > 0) {
          if (shortcut > max_shortcut) {
            max_shortcut = shortcut;

            best_shortcut.start = rand_ind;
            best_shortcut.motion_id = lazy_trajs.at(i).motion->idx;
            best_shortcut.end = neighs.at(j)->id;
          }

          early_terminate = max_shortcut > rate_early_terminate * original_size;
          if (early_terminate) {
            std::cout << "early terminate " << shortcut << std::endl;
            break;
          }
        }
      }

      if (early_terminate) {
        std::cout << "early terminate " << std::endl;
        break;
      }
    }
    if (early_terminate) {
      std::cout << "early terminate " << std::endl;
      break;
    }
  }
  std::cout << "max shortcut is " << max_shortcut << std::endl;
  std::cout << best_shortcut << std::endl;

  dynobench::Trajectory traj_new;

  for (size_t i = 0; i < best_shortcut.start; i++) {
    traj_new.states.push_back(traj_in.states.at(i));
    traj_new.actions.push_back(traj_in.actions.at(i));
  }

  Eigen::VectorXd _state_to_expand = traj_in.states.at(best_shortcut.start);

  Eigen::VectorXd __offset(robot->get_offset_dim());

  robot->offset(_state_to_expand, __offset);
  dynobench::Trajectory __traj = motions.at(best_shortcut.motion_id).traj;
  dynobench::TrajWrapper traj_wrap =
      dynobench::Trajectory_2_trajWrapper(__traj);
  robot->transform_primitive(__offset, __traj.states, __traj.actions,
                             traj_wrap);

  // double check the difference

  double d = robot->distance(traj_wrap.get_state(traj_wrap.get_size() - 1),
                             traj_in.states.at(best_shortcut.end));

  DYNO_CHECK_LEQ(d, delta, "why?");

  // add the new motion
  for (size_t i = 0; i < traj_wrap.get_size() - 1; i++) {
    traj_new.states.push_back(traj_wrap.get_state(i));
    traj_new.actions.push_back(traj_wrap.get_action(i));
  }

  // add the last part
  for (size_t i = best_shortcut.end; i < traj_in.states.size(); i++) {
    traj_new.states.push_back(traj_in.states.at(i));
    if (i < traj_in.actions.size())
      traj_new.actions.push_back(traj_in.actions.at(i));
  }

  DYNO_CHECK_EQ(traj_new.states.size(), traj_new.actions.size() + 1, "");

  traj_out = traj_new;

  traj_out.check(robot, true);
  DYNO_CHECK_LEQ(traj_out.max_jump, std::max(max_jump, delta), "");
}

void shortcut_v0_iterative(const dynobench::Problem &problem,
                           std::shared_ptr<dynobench::Model_robot> robot,
                           const Options_dbrrt &options_dbrrt,
                           const dynobench::Trajectory &traj_in,
                           dynobench::Trajectory &traj_out) {

  // get max jump

  DYNO_CHECK(options_dbrrt.motions_ptr, "motions_ptr is null");
  DYNO_CHECK(traj_in.states.size(), "traj_in is too short");

  double rate_stop_shortcut = .9;
  int max_num_shortcuts = 10;

  auto tic = std::chrono::high_resolution_clock::now();
  bool stop_shortcut = false;
  dynobench::Trajectory shortcut_in = traj_in;
  dynobench::Trajectory shortcut_out;
  for (int i = 0; i < max_num_shortcuts; i++) {
    shortcut_v0(problem, robot, options_dbrrt, shortcut_in, shortcut_out);
    DYNO_CHECK_LEQ(shortcut_out.states.size(), shortcut_in.states.size(),
                   ""); // shortcut should not increase

    std::cout << "shortcut iteration " << i << std::endl;
    std::cout << shortcut_out.states.size() << std::endl;
    std::cout << shortcut_in.states.size() << std::endl;
    stop_shortcut = shortcut_out.states.size() >
                    rate_stop_shortcut * shortcut_in.states.size();

    shortcut_in = shortcut_out;

    if (stop_shortcut) {
      std::cout << "breaking because progress is slower than "
                << rate_stop_shortcut << std::endl;
      break;
    }
  }

  auto toc = std::chrono::high_resolution_clock::now();

  std::cout << "shortcut summary" << std::endl;
  std::cout << "From " << traj_in.states.size() << " to "
            << shortcut_out.states.size() << std::endl;
  std::cout << "shortcut time "
            << std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic)
                   .count()
            << " ms" << std::endl;

  traj_out = shortcut_out;
}
} // namespace dynoplan
// namespace dynoplan
//
//
//
//
