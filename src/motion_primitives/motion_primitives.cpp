#include "dynoplan/motion_primitives/motion_primitives.hpp"
#include "crocoddyl/core/utils/timer.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynoplan/optimization/ocp.hpp"
#include <thread>

namespace dynoplan {

void sort_motion_primitives_rand_config(
    const dynobench::Trajectories &__trajs,

    dynobench::Trajectories &trajs_out,
    std::shared_ptr<dynobench::Model_robot> robot, int top_k) {

  dynobench::Trajectories trajs = __trajs;

  for (auto &t : trajs.data) {
    if (!t.start.size()) {
      CHECK(t.states.size(), AT);
      t.start = t.states.front();
    } else {
      CHECK_LEQ((t.start - t.states.front()).norm(), 1e-7, AT);
    }
    if (!t.goal.size()) {
      CHECK(t.states.size(), AT);
      t.goal = t.states.back();
    } else {
      CHECK_LEQ((t.goal - t.states.back()).norm(), 1e-7, AT);
    }
  }

  if (top_k == -1 || top_k > static_cast<int>(trajs.data.size())) {
    top_k = trajs.data.size();
  }

  for (const auto &traj : trajs.data) {
    CHECK_LEQ((traj.states.front() - traj.start).norm(), 1e-8, AT);
    CHECK_LEQ((traj.states.back() - traj.goal).norm(), 1e-8, AT);
  }

  std::vector<int> used_motions;
  std::set<int> unused_motions;

  for (size_t i = 0; i < trajs.data.size(); i++) {
    unused_motions.insert(i);
  }

  enum class Mode { START, GOAL, BOTH };

  Mode mode = Mode::BOTH;
  Eigen::VectorXd x(robot->nx), _x(robot->nx), goal(robot->nx);

  size_t num_translation = robot->get_translation_invariance();
  if (num_translation) {
    Eigen::VectorXd p_lb(num_translation);
    Eigen::VectorXd p_ub(num_translation);
    p_lb.setOnes();
    p_lb *= -1;
    p_ub.setOnes();
    robot->setPositionBounds(p_lb, p_ub);
  }

  for (size_t i = 0; i < top_k; i++) {

    if (i % 1000 == 0) {
      CSTR_(i);
    }

    int best_index = -1;

    switch (mode) {

    case Mode::START: {
      robot->sample_uniform(_x);
      robot->canonical_state(_x, x);

      double min_d = std::sqrt(std::numeric_limits<double>::max());
      for (auto &u : unused_motions) {
        auto &traj = trajs.data.at(u);
        double d = robot->distance(x, traj.states.front());
        if (d < min_d) {
          min_d = d;
          best_index = u;
        }
      }

    } break;

      // sample one configuration
    case Mode::GOAL: {
      robot->sample_uniform(goal);
      double min_d = std::sqrt(std::numeric_limits<double>::max());
      for (auto &u : unused_motions) {
        auto &traj = trajs.data.at(u);
        double d = robot->distance(goal, traj.states.back());
        if (d < min_d) {
          min_d = d;
          best_index = u;
        }
      }

    } break;

    case Mode::BOTH: {

      robot->sample_uniform(_x);
      robot->canonical_state(_x, x);

      robot->sample_uniform(goal);

      double min_d = std::sqrt(std::numeric_limits<double>::max());
      for (auto &u : unused_motions) {
        auto &traj = trajs.data.at(u);
        double d = robot->distance(goal, traj.states.back()) +
                   robot->distance(x, traj.states.front());

        if (d < min_d) {
          min_d = d;
          best_index = u;
        }
      }

      NOT_IMPLEMENTED;

    } break;
    default:
      NOT_IMPLEMENTED;
    }
    CHECK_GEQ(best_index, 0, AT);
    CHECK_LEQ(best_index, trajs.data.size() - 1, AT);
    // assert(best_index < traj.data.size());

    used_motions.push_back(best_index);
    unused_motions.erase(best_index);
  }

  for (auto &i : used_motions) {
    CSTR_(i);
    trajs_out.data.push_back(trajs.data.at(i));
  }
}

void sort_motion_primitives(
    const dynobench::Trajectories &__trajs, dynobench::Trajectories &trajs_out,
    std::function<double(const Eigen::VectorXd &, const Eigen::VectorXd &)>
        distance_fun,
    int top_k, bool naive) {

  dynobench::Trajectories trajs = __trajs;

  for (auto &t : trajs.data) {
    if (!t.start.size()) {
      CHECK(t.states.size(), AT);
      t.start = t.states.front();
    } else {
      CHECK_LEQ((t.start - t.states.front()).norm(), 1e-7, AT);
    }
    if (!t.goal.size()) {
      CHECK(t.states.size(), AT);
      t.goal = t.states.back();
    } else {
      CHECK_LEQ((t.goal - t.states.back()).norm(), 1e-7, AT);
    }
  }

  if (top_k == -1 || top_k > static_cast<int>(trajs.data.size())) {
    top_k = trajs.data.size();
  }

  for (const auto &traj : trajs.data) {
    CHECK_LEQ((traj.states.front() - traj.start).norm(), 1e-8, AT);
    CHECK_LEQ((traj.states.back() - traj.goal).norm(), 1e-8, AT);
  }

  auto goal_dist = [&](const dynobench::Trajectory &a,
                       const dynobench::Trajectory &b) {
    return distance_fun(a.goal, b.goal);
  };
  auto start_dist = [&](const dynobench::Trajectory &a,
                        const dynobench::Trajectory &b) {
    return distance_fun(a.start, b.start);
  };

  // use as first/seed motion the one that moves furthest
  size_t next_best_motion = 0;
  double largest_d = 0;
  for (size_t i = 0; i < trajs.data.size(); i++) {
    auto &traj = trajs.data.at(i);
    double d = distance_fun(traj.start, traj.goal);
    if (d > largest_d) {
      largest_d = d;
      next_best_motion = i;
    }
  }

  std::vector<size_t> used_motions;
  std::set<size_t> unused_motions;

  for (size_t i = 0; i < trajs.data.size(); i++) {
    unused_motions.insert(i);
  }

  used_motions.push_back(next_best_motion);
  unused_motions.erase(next_best_motion);

  if (naive) {

    bool finished = false;
    while (!finished) {

      // chose the best (MAX) from not not_chosen
      double best = std::numeric_limits<double>::lowest();
      int next_best = -1;

      for (auto &j : unused_motions) {

        // compute score

        double s1 = std::numeric_limits<double>::max();
        double s2 = std::numeric_limits<double>::max();

        for (auto &k : used_motions) {
          double _s1 = start_dist(trajs.data.at(j), trajs.data.at(k));
          double _s2 = goal_dist(trajs.data.at(j), trajs.data.at(k));
          s1 = std::min(s1, _s1);
          s2 = std::min(s2, _s2);
        }
        double __best = s1 + s2;

        if (__best > best) {
          best = __best;
          next_best = j;
        }
      }

      used_motions.push_back(next_best);
      unused_motions.erase(next_best);

      finished = used_motions.size() == top_k;
    }

  } else {

    std::vector<std::pair<double, double>> distance_map(trajs.data.size());

    for (auto &mi : unused_motions) {
      auto &m = trajs.data.at(mi);
      CHECK(used_motions.size(), AT);
      distance_map.at(mi).first =
          start_dist(m, trajs.data.at(used_motions.at(0)));
      distance_map.at(mi).second =
          goal_dist(m, trajs.data.at(used_motions.at(0)));
    }

    // TODO: evaluate if I should use a joint space!
    //

    CSTR_(top_k);
    for (size_t k = 1; k < top_k; ++k) {
      if (k % 1000 == 0) {
        CSTR_(k);
      }
      auto it = std::max_element(
          unused_motions.begin(), unused_motions.end(), [&](auto &a, auto &b) {
            return distance_map.at(a).first + distance_map.at(a).second <
                   distance_map.at(b).first + distance_map.at(b).second;
          });

      next_best_motion = *it;
      used_motions.push_back(*it);
      unused_motions.erase(*it);

      // update
      std::for_each(
          unused_motions.begin(), unused_motions.end(), [&](auto &mi) {
            distance_map.at(mi).first = std::min(
                distance_map.at(mi).first,
                start_dist(trajs.data.at(mi), trajs.data.at(next_best_motion)));

            distance_map.at(mi).second = std::min(
                distance_map.at(mi).second,
                goal_dist(trajs.data.at(mi), trajs.data.at(next_best_motion)));
          });
    }
  }

  for (auto &i : used_motions) {
    trajs_out.data.push_back(trajs.data.at(i));
  }
}

void split_motion_primitives(const dynobench::Trajectories &in,
                             const std::string &dynamics,
                             dynobench::Trajectories &out,
                             const Options_primitives &options_primitives) {

  auto robot =
      dynobench::robot_factory((options_primitives.models_base_path +
                                dynobench::robot_type_to_path(dynamics))
                                   .c_str());

  std::random_device rd;  // a seed for the random number engine
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

  for (size_t i = 0; i < in.data.size(); i++) {
    const auto &traj = in.data.at(i);
    const size_t T = traj.actions.size();
    std::uniform_int_distribution<> distrib_start(
        0, T - options_primitives.min_length_cut);
    std::uniform_int_distribution<> distrib_length(
        options_primitives.min_length_cut,
        std::min(options_primitives.max_length_cut, T));

    size_t num_splits =
        std::min(std::max(int(T / options_primitives.min_length_cut), 0),
                 static_cast<int>(options_primitives.max_splits));

    for (size_t j = 0; j < num_splits; j++) {
      size_t start = distrib_start(gen);
      size_t length = distrib_length(gen);

      if (start + length > T) {
        start = T - length;
      }

      if (j == 0) {
        // I always generate one primitive with the original start!
        start = 0;
      }

      else if (j == 1) {
        // the second primitive contains the goal
        start = T - length;
      }

      dynobench::Trajectory new_traj;
      std::cout << "i: " << i << " "
                << "start: " << start << " length:" << length << std::endl;
      new_traj.states = std::vector<Eigen::VectorXd>{
          traj.states.begin() + start,
          traj.states.begin() + start + length + 1};
      new_traj.actions = std::vector<Eigen::VectorXd>{
          traj.actions.begin() + start, traj.actions.begin() + start + length};
      //
      Eigen::VectorXd first_state = new_traj.states.front();

      for (auto &state : new_traj.states) {
        state.head(robot->translation_invariance) -=
            first_state.head(robot->translation_invariance);
      }

      new_traj.start = new_traj.states.front();
      new_traj.goal = new_traj.states.back();
      new_traj.cost = robot->traj_cost(new_traj.states, new_traj.actions);
      new_traj.info = "i:" + std::to_string(i) + "-s:" + std::to_string(start) +
                      "-l:" + std::to_string(length);
      out.data.push_back(new_traj);
    }
  }
}

void improve_motion_primitives(const Options_trajopt &options_trajopt,
                               const dynobench::Trajectories &__trajs_in,
                               const std::string &dynamics,
                               dynobench::Trajectories &trajs_out,
                               const Options_primitives &options_primitives) {

  auto robot_model =
      dynobench::robot_factory((options_primitives.models_base_path +
                                dynobench::robot_type_to_path(dynamics))
                                   .c_str());

  dynobench::Trajectories trajs_in = __trajs_in;
  std::atomic_int num_improves = 0;

  trajs_out.data.resize(trajs_in.data.size());

  int NUM_THREADS = options_primitives.num_threads;

  std::vector<std::thread> threads;

  auto improve = [](auto &trajs_in, auto &i, auto &robot_model, auto &dynamics,
                    auto &options_trajopt, auto &trajs_out,
                    auto &num_improves) {
    auto &traj = trajs_in.data.at(i);

    auto __model = std::shared_ptr<dynobench::Model_robot>(
        robot_model.get(), [](dynobench::Model_robot *) {});

    traj.check(__model, true);
    traj.update_feasibility();
    if (!traj.feasible) {
      std::cout << "warning, trajectory is not feasible! " << std::endl;
    }

    // make sure that trajectories have a cost...

    if (traj.cost > 1e3) {
      traj.cost = robot_model->traj_cost(traj.states, traj.actions);
    }

    dynobench::Problem problem;
    dynobench::Trajectory traj_out;
    CHECK(traj.states.size(), AT);
    CHECK(traj.actions.size(), AT);
    traj.to_yaml_format(std::cout);
    problem.goal = traj.states.back();
    problem.start = traj.states.front();
    problem.robotType = dynamics;

    Result_opti opti_out;

    try {
      trajectory_optimization(problem, traj, options_trajopt, traj_out,
                              opti_out);

      CHECK_EQ(opti_out.feasible, traj_out.feasible, AT);
      if (traj_out.feasible) {
        CHECK_LEQ(
            std::abs(traj_out.cost -
                     robot_model->traj_cost(traj_out.states, traj_out.actions)),
            1e-10, AT);
      }
      CSTR_(traj_out.feasible);
      std::cout << "Previous cost: " << traj.cost << std::endl;
      std::cout << "New cost: " << traj_out.cost << std::endl;
      if (traj_out.feasible && traj_out.cost + 1e-5 < traj.cost) {
        std::cout << "we have a better trajectory!" << std::endl;
        CSTR_(traj_out.cost);
        CSTR_(traj.cost);
        trajs_out.data.at(i) = traj_out;

        num_improves++;

      } else {
        trajs_out.data.at(i) = traj;
      }

    } catch (const std::exception &e) {
      trajs_out.data.at(i) = traj;
    }
  };

  for (size_t j = 0; j < NUM_THREADS; j++) {
    threads.push_back(std::thread([&, NUM_THREADS, j] {
      for (size_t i = j; i < trajs_in.data.size(); i += NUM_THREADS) {
        improve(trajs_in, i, robot_model, dynamics, options_trajopt, trajs_out,
                num_improves);
      }
    }));
  }

  for (auto &th : threads) {
    th.join();
  }

  std::cout << "input trajectories: " << trajs_in.data.size() << std::endl;
  std::cout << "num improves: " << num_improves << std::endl;
}

void make_canonical(const dynobench::Trajectories &trajectories,
                    dynobench::Trajectories &trajectories_out,
                    const std::string &dynamics) {

  auto robot_model =
      dynobench::robot_factory(dynobench::robot_type_to_path(dynamics).c_str());

  //
  Eigen::VectorXd offset(robot_model->get_offset_dim());
  Eigen::VectorXd x0(robot_model->nx);

  trajectories_out.data.resize(trajectories.data.size());

  for (size_t i = 0; i < trajectories.data.size(); i++) {
    auto &traj = trajectories.data.at(i);
    // robot_model->offset(traj.states.at(i), offset);
    robot_model->canonical_state(traj.states.front(), x0);
    std::vector<Eigen::VectorXd> xx = traj.states;
    robot_model->rollout(x0, traj.actions, xx);

    dynobench::Trajectory &traj_out = trajectories_out.data.at(i);
    traj_out.actions = traj.actions;
    traj_out.states = xx;
    traj_out.goal = traj_out.states.back();
    traj_out.start = traj_out.states.front();
  }
}

void generate_primitives_random(const Options_primitives &options_primitives,
                                dynobench::Trajectories &trajectories) {

  auto robot_model = dynobench::robot_factory(
      dynobench::robot_type_to_path(options_primitives.dynamics).c_str());

  size_t num_translation = robot_model->get_translation_invariance();
  if (num_translation) {
    Eigen::VectorXd p_lb(num_translation);
    Eigen::VectorXd p_ub(num_translation);
    p_lb.setOnes();
    p_lb *= -1;
    p_ub.setOnes();
    robot_model->setPositionBounds(p_lb, p_ub);
  }

  auto time_start = std::chrono::steady_clock::now();
  size_t attempts = 0;
  bool finished = false;
  while (!finished) {

    Eigen::VectorXd start(robot_model->nx);

    robot_model->sample_uniform(start);

    if (num_translation) {
      start.head(num_translation).setZero();
    }

    std::cout << "Trying to Generate a path from " << std::endl;

    CSTR_V(start);

    int ref = options_primitives.ref_time_steps;

    // random u's

    size_t nu = robot_model->nu;
    std::vector<Eigen::VectorXd> us;
    for (size_t i = 0; i < ref; i++) {
      Eigen::VectorXd u =
          robot_model->get_u_lb() +
          .5 * (Eigen::VectorXd::Random(nu) + Eigen::VectorXd::Ones(nu))
                   .cwiseProduct(
                       (robot_model->get_u_ub() - robot_model->get_u_lb()));
      us.push_back(u);
    }

    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robot_model->nx));
    robot_model->rollout(start, us, xs);

    dynobench::Trajectory traj;
    traj.start = start;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();
    trajectories.data.push_back(traj);

    attempts++;

    if (attempts >= options_primitives.max_attempts) {
      finished = true;
    }

    if (trajectories.data.size() >= options_primitives.max_num_primitives) {
      finished = true;
    }

    if (get_time_stamp_ms(time_start) / 1000. >=
        options_primitives.time_limit) {
      finished = true;
    }
  }

  CSTR_(attempts);
  CSTR_(trajectories.data.size());
  double success_rate = double(trajectories.data.size()) / attempts;
  CSTR_(success_rate);
}

void generate_primitives(const Options_trajopt &options_trajopt,
                         const Options_primitives &options_primitives,
                         dynobench::Trajectories &trajectories) {

  // generate an empty problem
  //

  bool finished = false;

  int num_primitives = 0;

  auto robot_model =
      dynobench::robot_factory((options_primitives.models_base_path +
                                options_primitives.dynamics + ".yaml")
                                   .c_str());

  size_t num_translation = robot_model->get_translation_invariance();
  if (num_translation) {
    Eigen::VectorXd p_lb(num_translation);
    Eigen::VectorXd p_ub(num_translation);
    p_lb.setOnes();
    p_lb *= -1;
    p_ub.setOnes();
    robot_model->setPositionBounds(p_lb, p_ub);
  }

  auto time_start = std::chrono::steady_clock::now();
  size_t attempts = 0;
  while (!finished) {

    Eigen::VectorXd start(robot_model->nx);
    Eigen::VectorXd goal(robot_model->nx);

    robot_model->sample_uniform(start);

    if (!options_primitives.use_random_displacemenet) {
      robot_model->sample_uniform(goal);
    } else {
      // sample a random displacemenet
      Eigen::VectorXd d_ub(start.size());
      Eigen::VectorXd d_lb(start.size());

      d_ub.setOnes();
      d_lb.setConstant(-1.);

      Eigen::VectorXd d = d_lb + (d_ub - d_lb) * .5 *
                                     (Eigen::VectorXd::Random(start.size()) +
                                      Eigen::VectorXd::Ones(start.size()));

      std::cout << "desisred displacement " << std::endl;
      CSTR_V(d);

      goal = start + d;
      Eigen::VectorXd tmp_goal;
      robot_model->ensure(goal, tmp_goal);
      goal = tmp_goal;

      // check that it is in the limits!
      if (robot_model->is_state_valid(goal)) {
        std::cout << "Warning: goal state is not valid!! " << std::endl;
        continue;
      }
    }
    if (num_translation) {
      goal.head(num_translation) -= start.head(num_translation);
      start.head(num_translation).setZero();
    }

    std::cout << "Trying to Generate a path betweeen " << std::endl;

    // start.setZero();
    // start(6) = 1.;

    CSTR_V(start);
    CSTR_V(goal);

    dynobench::Problem problem;
    problem.models_base_path = options_primitives.models_base_path;
    problem.goal = goal;
    problem.start = start;
    problem.robotType = options_primitives.dynamics;

    // double try

    std::vector<double> try_rates{.5, 1., 2.};

    bool is_first = true;
    bool solved = false;
    dynobench::Trajectory traj_first;
    for (const auto &try_rate : try_rates) {
      dynobench::Trajectory init_guess;
      init_guess.num_time_steps =
          int(try_rate * options_primitives.ref_time_steps);

      dynobench::Trajectory traj;
      Result_opti opti_out;

      trajectory_optimization(problem, init_guess, options_trajopt, traj,
                              opti_out);

      if (is_first) {
        traj_first = traj;
        is_first = false;
      }

      if (opti_out.feasible) {
        solved = true;
        CHECK(traj.states.size(), AT);
        traj.start = traj.states.front();
        traj.goal = traj.states.back();
        trajectories.data.push_back(traj);
        break;
      } else {
        // if (options_primitives.adapt_infeas_primitives) {
        //   ERROR_WITH_INFO("not implemented");
        // }
      }
    }

    if (!solved && options_primitives.adapt_infeas_primitives) {
      std::cout << "adapting a motion primitive" << std::endl;
      CHECK(traj_first.states.size(), AT);
      CHECK(traj_first.actions.size(), AT);
      traj_first.start = traj_first.states.front();
      traj_first.goal = traj_first.states.back();
      trajectories.data.push_back(traj_first);
    }

    attempts++;

    if (attempts >= options_primitives.max_attempts) {
      finished = true;
    }

    if (trajectories.data.size() >= options_primitives.max_num_primitives) {
      finished = true;
    }

    if (get_time_stamp_ms(time_start) / 1000. >=
        options_primitives.time_limit) {
      finished = true;
    }
  }

  CSTR_(attempts);
  CSTR_(trajectories.data.size());
  double success_rate = double(trajectories.data.size()) / attempts;
  CSTR_(success_rate);
}

} // namespace dynoplan
