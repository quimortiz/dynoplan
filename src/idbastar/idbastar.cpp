#include "dynoplan/idbastar/idbastar.hpp"

namespace dynoplan {

/*
 * @bried: Iterative Discontinuity Bounded A* (iDb-A*)
 * See the paper for more details:
 * https://arxiv.org/abs/2311.03553
 *
 * @param problem: problem definition
 * @param options_idbas: options for the iterative discontinuity bounded A*
 * @param options_dbastar: options for the dbastar
 * @param options_trajopt: options for the trajectory optimization
 * @param traj_out: output trajectory
 * @param info_out_idbastar: output info
 *
 */
void idbA(const dynobench::Problem &problem,
          const Options_idbAStar &options_idbas,
          const Options_dbastar &options_dbastar,
          const Options_trajopt &options_trajopt,
          dynobench::Trajectory &traj_out,
          Info_out_idbastar &info_out_idbastar) {

  // additional option (TODO quim -> move into options idbastar)
  const bool use_non_counter_time = true;

  // Options for db_astar will change in each iteration.
  // Thus, we do a copy of the input, and only change our local copy
  Options_dbastar options_dbastar_local = options_dbastar;

  std::vector<Motion> motions;

  // Create a robot (necessary for loading the motion primitives)
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  CHECK(robot, AT);

  // Load motion primitives
  load_motion_primitives_new(options_dbastar_local.motionsFile, *robot, motions,
                             options_idbas.max_motions_primitives,
                             options_dbastar_local.cut_actions, false,
                             options_dbastar_local.check_cols);
  options_dbastar_local.motions_ptr = &motions;
  std::cout << "Loading motion primitives -- DONE " << std::endl;

  if (false) {
    // Work in progres::
    // I detected that some primitives of the car_with_trailers are
    // not valid ... and we should fix this.
    // TODO: QUIM think of good way to quickly check that primitives are good --
    // should be robust against bounds on invariances
    std::cout << "checking motion primitives" << std::endl;

    auto good_motion = [&](auto &motion) {
      for (size_t j = 0; j < motion.traj.states.size(); j++) {
        if (!robot->check_state(motion.traj.states.at(j))) {
          return false;
        }
      }
      return true;
    };

    if (robot->name == "car_with_trailers") {
      // TODO: double check that this works for all systems!
      Eigen::VectorXd x_lb = robot->x_lb;
      Eigen::VectorXd x_ub = robot->x_ub;

      robot->x_lb.head(robot->get_translation_invariance()).array() =
          -std::numeric_limits<double>::max();
      robot->x_ub.head(robot->get_translation_invariance()).array() =
          std::numeric_limits<double>::max();

      std::cout << "motions before check" << motions.size() << std::endl;
      motions.erase(std::remove_if(motions.begin(), motions.end(), good_motion),
                    motions.end());
      std::cout << "motions after check" << motions.size() << std::endl;

      for (size_t i = 0; i < motions.size(); i++) {
        motions[i].idx = i;
      }

      robot->x_lb = x_lb;
      robot->x_ub = x_ub;
    }
  }

  // If heuristic is roadmap, load or create the heuristic map
  std::vector<Heuristic_node> heu_map;
  if (options_dbastar.heuristic == 1) {

    if (options_dbastar.heu_map_file.size()) {

      load_heu_map(options_dbastar_local.heu_map_file.c_str(), heu_map);

    } else {
      std::cout << "not heu map provided. Computing one .... " << std::endl;
      // there is not
      generate_heuristic_map(problem, robot, options_dbastar_local, heu_map);
      std::cout << "writing heu map " << std::endl;
      write_heu_map(heu_map, "tmp_heu_map.yaml");
    }
    options_dbastar_local.heu_map_ptr = &heu_map;
  }

  // Variables that will get updated during idbA* search
  info_out_idbastar.cost = 1e8;
  bool finished = false;
  size_t it = 0;
  size_t num_solutions = 0;
  double non_counter_time = 0;
  bool solved_db = false;

  // helper function to get time stamp since beginning of search
  auto start = std::chrono::steady_clock::now();
  auto get_time_stamp_ms = [&] {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start)
            .count());
  };

  while (!finished) {

    // Choose the value of delta and number of primitives for the search
    if (options_idbas.new_schedule) {
      if (it == 0) {
        options_dbastar_local.delta = options_idbas.delta_0;
        options_dbastar_local.max_motions = options_idbas.num_primitives_0;
      } else {
        if (solved_db) {
          options_dbastar_local.delta *= options_idbas.delta_rate;
        } else {
          // basically do not reduce delta if I have not solved with db first
          // (because this often means that we just need more primitives)
          options_dbastar_local.delta *= .9999;
        }
        // We always add more primitives
        options_dbastar_local.max_motions *= options_idbas.num_primitives_rate;
      }
    } else {
      // Schedule from the IROS paper is not supported anymore
      NOT_IMPLEMENTED;
    }

    // Set cost bound for dbastar a little bit higher than best cost so far...
    // Just to give opportunity to find a good solution that can bet optimized
    // latter.
    double delta_cost = 1.2;
    CSTR_(delta_cost);
    options_dbastar_local.maxCost = info_out_idbastar.cost * delta_cost;

    // Run dbastar
    std::cout << "*** Running DB-astar ***" << std::endl;
    dynobench::Trajectory traj_db, traj;
    Out_info_db out_info_db;
    std::string id_db = gen_random(6);
    options_dbastar_local.outFile = "/tmp/dynoplan/i_db_" + id_db + ".yaml";

    Stopwatch sw;
    dbastar(problem, options_dbastar_local, traj_db, out_info_db);
    non_counter_time += sw.elapsed_ms() - out_info_db.time_search;
    solved_db = out_info_db.solved;

    if (solved_db) {
      info_out_idbastar.solved_raw = true;
    }

    std::cout << "warning: using as time only the search!" << std::endl;
    traj_db.time_stamp =
        get_time_stamp_ms() - int(use_non_counter_time) * non_counter_time;
    info_out_idbastar.trajs_raw.push_back(traj_db);
    info_out_idbastar.infos_raw.push_back(out_info_db.data);

    if (out_info_db.solved) {
      {
        // write trajectory to file for debugging
        std::string filename = "/tmp/dynoplan/i_traj_db.yaml";
        std::string filename_id = "/tmp/dynoplan/i_traj_db_" + id_db + ".yaml";
        std::cout << "saving traj to: " << filename << std::endl;
        std::cout << "saving traj to: " << filename_id << std::endl;
        create_dir_if_necessary(filename.c_str());
        create_dir_if_necessary(filename_id.c_str());
        std::ofstream out(filename_id);
        traj_db.to_yaml_format(out);
        std::filesystem::copy(
            filename_id, filename.c_str(),
            std::filesystem::copy_options::overwrite_existing);
      }

      // Start Trajectory optimization
      std::cout << "***Trajectory Optimization -- START ***" << std::endl;
      Result_opti result;
      Stopwatch stopwatch;
      trajectory_optimization(problem, traj_db, options_trajopt, traj, result);
      non_counter_time +=
          stopwatch.elapsed_ms() - std::stof(result.data.at("time_ddp_total"));
      std::cout << "***Trajectory Optimization -- DONE ***" << std::endl;

      {
        // write trajectory to file for debugging
        std::string filename = "/tmp/dynoplan/i_traj_opt.yaml";
        std::string filename_id =
            "/tmp/dynoplan/i_traj_opt_" + gen_random(6) + ".yaml";
        std::cout << "saving traj to: " << filename << std::endl;
        std::cout << "saving traj to: " << filename_id << std::endl;
        create_dir_if_necessary(filename.c_str());
        create_dir_if_necessary(filename_id.c_str());
        std::ofstream out(filename_id);
        traj.to_yaml_format(out);
        std::filesystem::copy(
            filename_id, filename,
            std::filesystem::copy_options::overwrite_existing);
      }

      traj.time_stamp =
          get_time_stamp_ms() - int(use_non_counter_time) * non_counter_time;
      info_out_idbastar.trajs_opt.push_back(traj);
      info_out_idbastar.infos_opt.push_back(result.data);

      if (traj.feasible) {
        num_solutions++;
        info_out_idbastar.solved = true;

        std::cout << "we have a feasible solution! Cost: " << traj.cost
                  << std::endl;
        if (traj.cost < info_out_idbastar.cost) {
          info_out_idbastar.cost = traj.cost;
          traj_out = traj;
        }

        if (options_idbas.add_primitives_opt) {

          // add primtives of the solution after optimization
          // back into our set of primitives for planning.
          size_t number_of_cuts = 5;

          dynobench::Trajectories new_trajectories =
              cut_trajectory(traj, number_of_cuts, robot);

          dynobench::Trajectories trajs_canonical;

          make_trajs_canonical(*robot, new_trajectories.data,
                               trajs_canonical.data);

          {
            std::string filename =
                "/tmp/dynoplan/trajs_cuts_canonical_" + gen_random(6) + ".yaml";
            trajs_canonical.save_file_yaml(filename.c_str());
          }

          // Add a little bit of noise, because NIGH sometimes
          // might crash if some components match exactly
          const bool add_noise_first_state = true;
          const double noise = 1e-7;
          for (auto &t : trajs_canonical.data) {
            t.states.front() +=
                noise * Eigen::VectorXd::Random(t.states.front().size());
            t.states.back() +=
                noise * Eigen::VectorXd::Random(t.states.back().size());

            robot->ensure(t.states.front());
            robot->ensure(t.states.back());
          }

          std::vector<Motion> motions_out;
          for (const auto &traj : trajs_canonical.data) {
            Motion motion_out;
            CHECK(robot, AT)
            traj_to_motion(traj, *robot, motion_out, true);
            motions_out.push_back(std::move(motion_out));
          }

          // Insert the new motions at the beginning of the list,
          // to make sure that they are available for the next iteration
          motions.insert(motions.begin(),
                         std::make_move_iterator(motions_out.begin()),
                         std::make_move_iterator(motions_out.end()));

          std::cout << "Afer insert " << motions.size() << std::endl;
          std::cout << "Warning: "
                    << "I am inserting at the beginning" << std::endl;

          std::cout << "i update the idx of the motions " << std::endl;
          // Very important to update the idx of the motions
          for (size_t i = 0; i < motions.size(); i++) {
            motions[i].idx = i;
          }
        }
      } else {
        std::cout << "Trajectory optimization has failed!" << std::endl;
      }
    }

    it++;

    // Check exit criteria
    if (it >= options_idbas.max_it) {
      finished = true;
      info_out_idbastar.exit_criteria = EXIT_CRITERIA::max_it;
    }

    if (num_solutions >= options_idbas.max_num_sol) {
      finished = true;
      info_out_idbastar.exit_criteria = EXIT_CRITERIA::max_solution;
    }

    if (get_time_stamp_ms() / 1000. > options_idbas.timelimit) {
      finished = true;
      info_out_idbastar.exit_criteria = EXIT_CRITERIA::time_limit;
    }
  }
  // we have finised the search!

  {
    // write solution to file
    std::string filename = "/tmp/dynoplan/i_traj_out.yaml";
    std::string filename_id =
        "/tmp/dynoplan/i_traj_out_" + gen_random(6) + ".yaml";
    std::cout << "saving traj to: " << filename << std::endl;
    std::cout << "saving traj to: " << filename_id << std::endl;
    create_dir_if_necessary(filename.c_str());
    create_dir_if_necessary(filename_id.c_str());
    std::ofstream out(filename_id);
    traj_out.to_yaml_format(out);
    std::filesystem::copy(filename_id, filename,
                          std::filesystem::copy_options::overwrite_existing);
  }

  std::cout << "exit criteria: "
            << static_cast<int>(info_out_idbastar.exit_criteria) << std::endl;
}

void write_results_idbastar(const char *results_file,
                            const dynobench::Problem &problem,
                            const Options_idbAStar &options_idbastar,
                            const Options_dbastar &options_dbastar,
                            const Options_trajopt &options_trajopt,
                            const Info_out_idbastar &info_out_idbastar) {

  std::ofstream results(results_file);

  results << "alg: idbastar" << std::endl;
  results << "problem_file: " << problem.file << std::endl;
  results << "robot_type: " << problem.robotType << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;

  results << "options_idbastar:" << std::endl;
  options_idbastar.print(results, "  ");

  results << "options_dbastar:" << std::endl;
  options_dbastar.print(results, "  ");

  results << "options trajopt:" << std::endl;
  options_trajopt.print(results, "  ");

  info_out_idbastar.to_yaml(results);
}

} // namespace dynoplan
