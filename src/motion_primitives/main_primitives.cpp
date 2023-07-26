
#include "dynoplan/motion_primitives/motion_primitives.hpp"

enum class PRIMITIVE_MODE {
  generate = 0,
  improve = 1,
  split = 2,
  sort = 3,
  merge = 4,
  check = 5,
  stats = 6,
  cut = 7,
  shuffle = 8,
  generate_rand = 9,
  make_canonical = 10,
  bintoyaml = 11,
  yamltobin = 12,
  sort_with_rand_config = 13,
  reduce_set = 14,
};

using namespace dynobench;
using namespace dynoplan;

int main(int argc, const char *argv[]) {

  std::cout << "seed with time " << std::endl;
  srand((unsigned int)time(0));

  CSTR_(__FILE__);
  CSTR_(argc);

  std::cout << "argv: " << std::endl;
  for (int i = 0; i < argc; i++) {
    std::cout << argv[i] << std::endl;
  }

  std::string in_file;
  std::string out_file = "auto";
  int mode_gen_id = 0;
  std::string cfg_file = "";
  Options_trajopt options_trajopt;
  Options_primitives options_primitives;

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message");
  set_from_boostop(desc, VAR_WITH_NAME(in_file));
  set_from_boostop(desc, VAR_WITH_NAME(out_file));
  set_from_boostop(desc, VAR_WITH_NAME(mode_gen_id));
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
  options_primitives.add_options(desc);
  options_trajopt.add_options(desc);

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  if (cfg_file != "") {
    options_trajopt.read_from_yaml(cfg_file.c_str());
  }

  std::cout << " *** options_primitives *** " << std::endl;
  options_primitives.print(std::cout);
  std::cout << " *** " << std::endl;
  std::cout << " *** options_trajopt *** " << std::endl;
  options_trajopt.print(std::cout);
  std::cout << " *** " << std::endl;

  PRIMITIVE_MODE mode = static_cast<PRIMITIVE_MODE>(mode_gen_id);

  std::shared_ptr<dynobench::Model_robot> robot_model =
      dynobench::robot_factory((options_primitives.models_base_path +
                                options_primitives.dynamics + ".yaml")
                                   .c_str());

  if (mode == PRIMITIVE_MODE::make_canonical) {
    dynobench::Trajectories trajectories, trajectories_out;

    trajectories.load_file_boost(in_file.c_str());

    if (options_primitives.max_num_primitives > 0 &&
        static_cast<size_t>(options_primitives.max_num_primitives) <
            trajectories.data.size()) {
      trajectories.data.resize(options_primitives.max_num_primitives);
    }

    if (out_file == "auto") {
      out_file = in_file + ".ca.bin";
    }

    make_canonical(trajectories, trajectories_out,
                   options_primitives.models_base_path +
                       options_primitives.dynamics);

    trajectories_out.save_file_boost(out_file.c_str());
    trajectories_out.save_file_yaml((out_file + ".yaml").c_str(), 1000);

    trajectories_out.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::generate_rand) {
    dynobench::Trajectories trajectories;

    generate_primitives_random(options_primitives, trajectories);

    trajectories.save_file_boost(out_file.c_str());
    trajectories.save_file_yaml((out_file + ".1000.yaml").c_str(), 1000);
    trajectories.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::bintoyaml) {
    dynobench::Trajectories trajectories;
    trajectories.load_file_boost(in_file.c_str());

    // Should I partition?

    size_t max_primitives_per_file = 20000;
    size_t num_files = trajectories.data.size() / max_primitives_per_file;
    if (trajectories.data.size() % max_primitives_per_file != 0) {
      num_files += 1;
    }

    for (size_t i = 0; i < num_files; i++) {

      dynobench::Trajectories tt;
      size_t start_index = i * max_primitives_per_file;
      size_t end_index =
          std::min((i + 1) * max_primitives_per_file, trajectories.data.size());
      tt.data = {trajectories.data.begin() + start_index,
                 trajectories.data.begin() + end_index};
      tt.save_file_yaml((in_file + ".p" + std::to_string(i) + ".yaml").c_str());
    }
  }

  if (mode == PRIMITIVE_MODE::generate) {
    dynobench::Trajectories trajectories;

    generate_primitives(options_trajopt, options_primitives, trajectories);

    trajectories.save_file_boost(out_file.c_str());
    trajectories.save_file_yaml((out_file + ".yaml").c_str());
    trajectories.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::improve) {
    dynobench::Trajectories trajectories, trajectories_out;

    trajectories.load_file_boost(in_file.c_str());

    if (startsWith(options_primitives.dynamics, "quad3d")) {

      for (auto &t : trajectories.data) {

        for (auto &s : t.states) {
          s.segment<4>(3).normalize();
        }
      }
    }

    if (options_primitives.max_num_primitives > 0 &&
        static_cast<size_t>(options_primitives.max_num_primitives) <
            trajectories.data.size()) {
      trajectories.data.resize(options_primitives.max_num_primitives);
    }

    if (out_file == "auto") {
      out_file = in_file + ".im.bin";
    }

    // Options_trajopt options_trajopt;
    // options_trajopt.solver_id =
    //     static_cast<int>(SOLVER::traj_opt_free_time_linear);

    improve_motion_primitives(options_trajopt, trajectories,
                              options_primitives.dynamics, trajectories_out,
                              options_primitives);

    trajectories_out.save_file_boost(out_file.c_str());
    trajectories_out.save_file_yaml((out_file + ".yaml").c_str(), 1000);

    trajectories_out.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::split) {
    dynobench::Trajectories trajectories, trajectories_out;
    trajectories.load_file_boost(in_file.c_str());

    if (startsWith(options_primitives.dynamics, "quad3d")) {

      for (auto &t : trajectories.data) {

        for (auto &s : t.states) {
          s.segment<4>(3).normalize();
        }
      }
    }

    split_motion_primitives(trajectories, options_primitives.dynamics,
                            trajectories_out, options_primitives);

    // std::vector<bool> valid(trajectories_out.data.size(), true);
    dynobench::Trajectories __trajectories_out;
    for (size_t i = 0; i < trajectories_out.data.size(); i++) {
      auto &traj = trajectories_out.data.at(i);
      // traj.check(robot_model, true);
      traj.check(robot_model);
      traj.update_feasibility();
      if (traj.feasible) {
        __trajectories_out.data.push_back(traj);
      }
    }

    trajectories_out = __trajectories_out;

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(trajectories_out.data.begin(), trajectories_out.data.end(), g);
    if (out_file == "auto") {
      out_file = in_file + ".sp.bin";
    }

    trajectories_out.save_file_boost(out_file.c_str());
    trajectories_out.save_file_yaml((out_file + ".yaml").c_str(), 1000);
    trajectories_out.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::sort) {
    dynobench::Trajectories trajectories, trajectories_out;
    trajectories.load_file_boost(in_file.c_str());
    CSTR_(trajectories.data.size());

    if (options_primitives.max_num_primitives > 0 &&
        static_cast<size_t>(options_primitives.max_num_primitives) <
            trajectories.data.size()) {
      trajectories.data.resize(options_primitives.max_num_primitives);
    }

    sort_motion_primitives(
        trajectories, trajectories_out,
        [&](const auto &x, const auto &y) {
          return robot_model->distance(x, y);
        },
        options_primitives.max_num_primitives);

    const bool debug = false;
    if (debug) {
      dynobench::Trajectories trajectories_out_debug;
      sort_motion_primitives(
          trajectories, trajectories_out_debug,
          [&](const auto &x, const auto &y) {
            return robot_model->distance(x, y);
          },
          options_primitives.max_num_primitives, true);

      // check that the trajectories are the same
      for (size_t t = 0; t < trajectories_out_debug.data.size(); t++) {

        auto &traj = trajectories_out_debug.data.at(t);
        auto &traj2 = trajectories_out.data.at(t);

        CHECK_LEQ((traj.start - traj2.start).norm(), 1e-6, AT);
        CHECK_LEQ((traj.goal - traj2.goal).norm(), 1e-6, AT);
      }
    }

    // check that they are valid...

    for (auto &traj : trajectories_out.data) {
      traj.check(robot_model, true);
      traj.update_feasibility();

      if (!traj.feasible) {
        WARN_WITH_INFO("Trajectory is not feasible -- could happen when I "
                       "transform to cannonical form");
      }
    }

    if (out_file == "auto") {
      out_file = in_file + ".so.bin";
    }

    CSTR_(trajectories_out.data.size());
    trajectories_out.save_file_boost(out_file.c_str());
    trajectories_out.save_file_yaml((out_file + ".yaml").c_str(), 1000);

    trajectories_out.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::sort_with_rand_config) {
    dynobench::Trajectories trajectories, trajectories_out;
    trajectories.load_file_boost(in_file.c_str());
    CSTR_(trajectories.data.size());

    // if (options_primitives.max_num_primitives > 0 &&
    //     static_cast<size_t>(options_primitives.max_num_primitives) <
    //         trajectories.data.size()) {
    //   trajectories.data.resize(options_primitives.max_num_primitives);
    // }

    std::shared_ptr<dynobench::Model_robot> robot_model =
        dynobench::robot_factory(
            dynobench::robot_type_to_path(options_primitives.dynamics).c_str());

    sort_motion_primitives_rand_config(trajectories, trajectories_out,
                                       robot_model,
                                       options_primitives.max_num_primitives);

    // check that they are valid...

    for (auto &traj : trajectories_out.data) {
      traj.check(robot_model, true);
      traj.update_feasibility();

      if (!traj.feasible) {
        WARN_WITH_INFO("Trajectory is not feasible -- could happen when I "
                       "transform to cannonical form");
      }
    }

    if (out_file == "auto") {
      out_file = in_file + ".so2.bin";
    }

    CSTR_(trajectories_out.data.size());
    trajectories_out.save_file_boost(out_file.c_str());
    trajectories_out.save_file_yaml((out_file + ".yaml").c_str(), 1000);

    trajectories_out.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::check) {
    dynobench::Trajectories trajectories;
    trajectories.load_file_boost(in_file.c_str());
    CSTR_(trajectories.data.size());

    for (auto &traj : trajectories.data) {
      traj.check(robot_model);
      traj.update_feasibility();
      CHECK(traj.feasible, AT);
    }

    trajectories.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::cut) {
    dynobench::Trajectories trajectories;
    trajectories.load_file_boost(in_file.c_str());
    CSTR_(trajectories.data.size());

    trajectories.data.resize(options_primitives.max_num_primitives);

    if (out_file == "auto") {
      out_file = in_file + ".c" +
                 std::to_string(options_primitives.max_num_primitives) + ".bin";
    }

    trajectories.save_file_boost(out_file.c_str());
    trajectories.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::stats) {
    dynobench::Trajectories trajectories;
    trajectories.load_file_boost(in_file.c_str());
    if (options_primitives.max_num_primitives > 0 &&
        options_primitives.max_num_primitives < trajectories.data.size())
      trajectories.data.resize(options_primitives.max_num_primitives);
    if (out_file == "auto") {
      out_file = in_file + ".stats.yaml";
    }
    CSTR_(trajectories.data.size());
    trajectories.compute_stats(out_file.c_str());
    std::cout << "writing stats to "
              << "/tmp/tmp_stats.yaml" << std::endl;
    std::filesystem::copy(out_file, "/tmp/tmp_stats.yaml",
                          std::filesystem::copy_options::overwrite_existing);
  }

  if (mode == PRIMITIVE_MODE::shuffle) {
    dynobench::Trajectories trajectories;
    trajectories.load_file_boost(in_file.c_str());
    if (options_primitives.max_num_primitives > 0 &&
        options_primitives.max_num_primitives < trajectories.data.size())
      trajectories.data.resize(options_primitives.max_num_primitives);
    CSTR_(trajectories.data.size());

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(trajectories.data.begin(), trajectories.data.end(), g);
    if (out_file == "auto") {
      out_file = in_file + ".sh.bin";
    }

    trajectories.save_file_boost(out_file.c_str());
    trajectories.compute_stats("/tmp/tmp_stats.yaml");
  }

  if (mode == PRIMITIVE_MODE::reduce_set) {

    dynobench::Trajectories trajectories;
    trajectories.load_file_boost(in_file.c_str());

    if (options_primitives.max_num_primitives > 0 &&
        options_primitives.max_num_primitives < trajectories.data.size())
      trajectories.data.resize(options_primitives.max_num_primitives);

    if (out_file == "auto") {
      out_file = in_file + ".less.bin";
    }

    trajectories.save_file_boost(out_file.c_str());
    trajectories.compute_stats("/tmp/tmp_stats.yaml");
  }

  std::ofstream log(out_file + ".log");
  log << "argv: " << std::endl;
  for (int i = 0; i < argc; i++) {
    log << " " << argv[i] << std::endl;
  }
  log << "out: " << out_file << std::endl;
  log << "time: " << get_time_stamp() << std::endl;
  log << "options_primitives:" << std::endl;
  options_primitives.print(log, "  ");

  return 0;
}
