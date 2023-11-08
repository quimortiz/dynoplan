
#include "dynoplan/ompl/rrt_to.hpp"
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/optimization/ocp.hpp"

namespace og = ompl::geometric;

namespace dynoplan {

struct RRTstar_public_interface : public og::RRTstar {

  RRTstar_public_interface(const ob::SpaceInformationPtr &si)
      : og::RRTstar(si) {}

  ~RRTstar_public_interface() = default;

  void setNearestNeighbors(const std::string &name,
                           const std::shared_ptr<RobotOmpl> &robot) {
    auto t = nigh_factory<og::RRTstar::Motion *>(
        name, robot, [](og::RRTstar::Motion *m) { return m->state; });

    nn_.reset(t);
  }
};

void solve_ompl_geometric(const dynobench::Problem &problem,
                          const Options_geo &options_geo,
                          const Options_trajopt &options_trajopt,
                          dynobench::Trajectory &traj_out,
                          dynobench::Info_out &info_out_omplgeo) {

  std::shared_ptr<RobotOmpl> robot = robot_factory_ompl(problem);

  // create and set a start state
  auto startState = robot->startState;
  auto goalState = robot->goalState;
  auto si = robot->getSpaceInformation();

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  pdef->addStartState(startState);
  pdef->setGoalState(goalState);
  // ;options_geo.goalregion);
  // si->freeState(startState); @Wolfgang WHY??
  // si->freeState(goalState); @Wolfgang WHY??

  // create a planner for the defined space
  std::shared_ptr<ob::Planner> planner;
  if (options_geo.planner == "rrt*") {
    auto pp = new RRTstar_public_interface(si);

    if (options_geo.geo_use_nigh) {
      pp->setNearestNeighbors(problem.robotType, robot);
    }

    if (options_geo.range > 0) {
      pp->setRange(options_geo.range);
    }
    if (options_geo.goalBias > 0) {
      pp->setGoalBias(options_geo.goalBias);
    }
    planner.reset(pp);
  } else {
    NOT_IMPLEMENTED;
  }

  // rrt->setGoalBias(params["goalBias"].as<float>());
  // auto planner(rrt);

  auto start = std::chrono::steady_clock::now();
  bool has_solution = false;
  std::chrono::steady_clock::time_point previous_solution;

  // std::vector<Trajectory> trajectories;

  const bool traj_opti = true;

  auto get_time_stamp_ms = [&] {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start)
            .count());
  };

  std::string id = gen_random(6);
  size_t num_founds_geo_trajs = 0;
  const bool use_non_counter_time = true;

  double non_counter_time = 0;
  pdef->setIntermediateSolutionCallback(
      [&](const ob::Planner *, const std::vector<const ob::State *> &states,
          const ob::Cost cost) {
        dynobench::Trajectory traj;
        dynobench::Trajectory traj_geo;
        traj_geo.time_stamp =
            get_time_stamp_ms() - int(use_non_counter_time) * non_counter_time;

        state_to_eigen(traj_geo.start, si, startState);
        state_to_eigen(traj_geo.goal, si, goalState);
        traj_geo.states.push_back(traj_geo.start);

        auto __states = states;
        std::reverse(__states.begin(), __states.end());

        for (auto &s : __states) {
          Eigen::VectorXd x;
          state_to_eigen(x, si, s);
          robot->diff_model->set_0_velocity(x);
          robot->diff_model->ensure(x);
          traj_geo.states.push_back(x);
        }
        traj_geo.states.push_back(traj_geo.goal);
        traj_geo.feasible = false;
        double time = 0;

        traj_geo.times.resize(traj_geo.states.size());
        traj_geo.times(0) = 0.;

        // compute the times.
        for (size_t i = 0; i < traj_geo.states.size() - 1; i++) {
          auto &x = traj_geo.states.at(i);
          auto &y = traj_geo.states.at(i + 1);
          time += robot->diff_model->lower_bound_time(x, y);
          traj_geo.times(i + 1) = time;
        }

        traj_geo.cost = traj_geo.times.tail(1)(0);

        // add the control

        for (size_t i = 0; i < traj_geo.states.size() - 1; i++) {
          traj_geo.actions.push_back(robot->diff_model->u_0);
        }

        // Path path;
        // path.time_stamp = t;

        // double t =
        // std::chrono::duration_cast<std::chrono::milliseconds>(now
        // - start).count(); double dt =
        // std::chrono::duration_cast<std::chrono::milliseconds>(now -
        // previous_solution).count();
        std::cout << "Intermediate geometric solution! " << cost.value()
                  << std::endl;
        has_solution = true;
        // last_solution_in_sec = dt / 1000.0f;
        std::cout << "printing a new path" << std::endl;
        for (auto &state : states)
          si->printState(state, std::cout);
        std::cout << "printing a new path -- DONE" << std::endl;

        info_out_omplgeo.trajs_raw.push_back(traj_geo);

        std::cout << "traj geo" << std::endl;
        traj_geo.to_yaml_format(std::cout);

        std::string filename = "/tmp/dynoplan/traj_geo_rrt_" +
                               std::to_string(num_founds_geo_trajs) + "_" + id +
                               ".yaml";
        traj_geo.to_yaml_format(filename.c_str());

        if (traj_opti) {

          // Options_trajopt opti;
          // opti.solver_id = 0;
          // opti.control_bounds = 1;
          // opti.use_warmstart = 1;
          // opti.weight_goal = 100;
          // opti.max_iter = 100;
          // opti.noise_level = 1e-4;

          Result_opti result;

          // convert path to eigen
          // file_inout.xs = traj.states;
          // file_inout.us = traj.actions;
          // file_inout.ts = traj.times;

          // compute approximate time.

          std::cout << "*** Sart optimization ***" << std::endl;

          Stopwatch watch;
          trajectory_optimization(problem, traj_geo, options_trajopt, traj,
                                  result);
          double raw_time = watch.elapsed_ms();

          non_counter_time +=
              raw_time - std::stof(result.data.at("time_ddp_total"));

          std::cout << "*** Optimization done ***" << std::endl;

          std::cout << "traj opt" << std::endl;
          traj.to_yaml_format(std::cout);

          // save the trajectory
          std::string filename = "/tmp/dynoplan/traj_geo_opt_" +
                                 std::to_string(num_founds_geo_trajs) + "_" +
                                 id + ".yaml";
          traj.to_yaml_format(filename.c_str());

          if (traj.feasible) {
            info_out_omplgeo.solved = true;

            if (traj.cost < info_out_omplgeo.cost) {
              info_out_omplgeo.cost = traj.cost;
              traj_out = traj;
            }
          }

          traj.time_stamp = get_time_stamp_ms() -
                            int(use_non_counter_time) * non_counter_time;
          info_out_omplgeo.trajs_opt.push_back(traj);
        }

        previous_solution = std::chrono::steady_clock::now();
        num_founds_geo_trajs++;
      });

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps f  r the planner
  // (this will set the optimization objective)
  planner->setup();

  // set a really high cost threshold, so that planner stops after first
  // solution was found
  // pdef->getOptimizationObjective()->setCostThreshold(ob::Cost(1e6));

  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within timelimit
  ob::PlannerStatus solved;

  // solved = planner->ob::Planner::solve(timelimit);
  // terminate if no better solution is found within the timelimit
  // WHY?

  solved = planner->solve(ob::PlannerTerminationCondition([&] {
    if (get_time_stamp_ms() > options_geo.timelimit * 1000) {
      WARN_WITH_INFO("time limit reached");
      return true;
    }

    if (!has_solution) {
      return false;
    }
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - previous_solution)
                    .count() /
                1000.0f;
    return dt > 1.0;
  }));
  std::cout << solved << std::endl;

  // lets print all the paths

  // info_out_omplgeo.solved = solved;

  if (solved) {
    std::cout << "Found solution:" << std::endl;
  } else {
    std::cout << "No solution found" << std::endl;
  }
}

} // namespace dynoplan
