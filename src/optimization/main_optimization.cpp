#include "crocoddyl/core/utils/timer.hpp"
#include "idbastar/optimization/ocp.hpp"

int main(int argc, const char *argv[]) {

  srand(time(0));
  // srand(0); // for reproducibility

  Options_trajopt options_trajopt;
  std::string env_file = "";
  std::string init_file = "";
  std::string cfg_file = "";
  std::string models_base_path = "";
  std::string results_file = "";

  po::options_description desc("Allowed options");
  options_trajopt.add_options(desc);
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(init_file));
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
  set_from_boostop(desc, VAR_WITH_NAME(results_file));
  set_from_boostop(desc, VAR_WITH_NAME(models_base_path));

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
  // print options

  if (cfg_file != "") {
    options_trajopt.read_from_yaml(cfg_file.c_str());
  }

  std::cout << "*** options_trajopt ***" << std::endl;
  options_trajopt.print(std::cout);
  std::cout << "***" << std::endl;

  Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;
  Trajectory traj_out;
  Trajectory traj_db;
  traj_db.read_from_yaml(init_file.c_str());

  Result_opti result;
  // load the initial guess

  trajectory_optimization(problem, traj_db, options_trajopt, traj_out, result);

  CSTR_(results_file);
  std::ofstream results(results_file);

  results << "alg: idbastar" << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;
  results << "env_file: " << env_file << std::endl;
  results << "init_file: " << init_file << std::endl;
  results << "cfg_file: " << cfg_file << std::endl;
  results << "results_file: " << results_file << std::endl;
  results << "options trajopt:" << std::endl;

  options_trajopt.print(results, "  ");
  result.write_yaml(results);
  results << "options trajopt:" << std::endl;

  results << "trajs_opt:" << std::endl;
  results << "  -" << std::endl;
  traj_out.to_yaml_format(results, "    ");

  // saving only the trajectory
  std::string file = results_file + ".trajopt.yaml";
  std::ofstream out(file);
  traj_out.to_yaml_format(out);

#if 0

  std::string env_file = "";
  std::string init_guess = "";
  std::string config_file = "";
  std::string out = "out.yaml";
  std::string out_bench = "out_bench.yaml";
  std::string yaml_solver_file = "";
  std::string yaml_problem_file = "";

  std::string cfg_file = "";



  Options_trajopt options_trajopt;
  Problem problem;
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message");
  options_trajopt.add_options(desc);

  set_from_boostop(desc, VAR_WITH_NAME(out));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(init_guess));
  set_from_boostop(desc, VAR_WITH_NAME(out_bench));
  set_from_boostop(desc, VAR_WITH_NAME(config_file));
  set_from_boostop(desc, VAR_WITH_NAME(yaml_solver_file));
  set_from_boostop(desc, VAR_WITH_NAME(yaml_problem_file));

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 1;
    }

    if (config_file != "") {
      std::ifstream ifs{config_file};
      if (ifs)
        store(parse_config_file(ifs, desc), vm);
      else {
        std::cout << "cannont open config file: " << config_file << std::endl;
      }
      notify(vm);
    }

    if (yaml_solver_file != "") {
      options_trajopt.read_from_yaml(yaml_solver_file.c_str());
    }
    if (yaml_problem_file != "") {
      YAML::Node node = YAML::LoadFile(yaml_problem_file);
      set_from_yaml(node, VAR_WITH_NAME(env_file));
      set_from_yaml(node, VAR_WITH_NAME(init_guess));
    }

    std::cout << "***" << std::endl;
    std::cout << "VARIABLE MAP" << std::endl;
    PrintVariableMap(vm, std::cout);
    std::cout << "***" << std::endl;

    std::cout << "***" << std::endl;
    std::cout << "OPTI PARAMS" << std::endl;
    options_trajopt.print(std::cout);
    std::cout << "***" << std::endl;

  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  problem.read_from_yaml(env_file.c_str());
  Trajectory traj_init, traj_out;
  if (init_guess.size()) {
    traj_init.read_from_yaml(init_guess.c_str());
  }

  Result_opti result;
  crocoddyl::Timer timer;
  trajectory_optimization(problem, traj_init, options_trajopt, traj_out,
                          result);
  CHECK_EQ(traj_out.feasible, result.feasible, AT);
  double d = timer.get_duration();

  {
    std::cout << AT<< " " <<"INFO"<< traj_out.info << std::endl;
    std::ofstream fileout(out);
    std::cout << "writing trajectory to:  " << out << std::endl;
    traj_out.to_yaml_format(fileout, "");
  }

  {

    std::cout << "Result summary " << std::endl;
    std::cout << STR_(result.feasible) << std::endl;
    std::cout << STR_(result.cost) << std::endl;
    std::cout << STR_(result.name) << std::endl;
    std::cout << STR_(result.name) << std::endl;
    std::cout << STR_(result.xs_out.size()) << std::endl;
    std::cout << STR_(result.us_out.size()) << std::endl;

    std::string results_old = "results_old.yaml";
    std::cout << "writing results to:" << results_old << std::endl;
    std::ofstream file_out(results_old);
    result.write_yaml_db(file_out);
  }

  {
    std::cout << "writing results extended to:" << out_bench << std::endl;
    std::ofstream ff(out_bench);
    std::cout << "writing to " << out_bench << std::endl;
    ff << "solver_name: " << options_trajopt.solver_name << std::endl;
    ff << "problem_name: " << problem.name << std::endl;
    ff << "feasible: " << result.feasible << std::endl;
    ff << "cost: " << result.cost << std::endl;
    ff << "time_total: " << d << std::endl;
    ff << "time_stamp: " << get_time_stamp() << std::endl;
    ff << "solver_file: " << yaml_solver_file << std::endl;
    ff << "problem_file: " << yaml_problem_file << std::endl;
  }

#endif

  if (result.feasible) {
    std::cout << "croco success -- returning " << std::endl;
    return EXIT_SUCCESS;
  } else {
    std::cout << "croco failed -- returning " << std::endl;
    return EXIT_FAILURE;
  }
}
