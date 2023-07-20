#include "idbastar/ompl/rrt_to.hpp"
#include "idbastar/optimization/ocp.hpp"

int main(int argc, char *argv[]) {

  srand(time(0));
  // srand(0);

  po::options_description desc("Allowed options");
  Options_geo options_geo;
  Options_trajopt options_trajopt;
  options_geo.add_options(desc);
  options_trajopt.add_options(desc);

  std::string env_file, results_file, cfg_file, models_base_path;
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(results_file));
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
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

  if (cfg_file != "") {
    // TODO: allow for hierarchial yaml
    options_geo.read_from_yaml(cfg_file.c_str());
    options_trajopt.read_from_yaml(cfg_file.c_str());
  }

  std::cout << "*** options_geo ***" << std::endl;
  options_geo.print(std::cout);
  std::cout << "***" << std::endl;

  std::cout << "*** options_traj_opt ***" << std::endl;
  options_trajopt.print(std::cout);
  std::cout << "***" << std::endl;

  Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;

  Info_out info_out_omplgeo;
  Trajectory traj_out;

  solve_ompl_geometric(problem, options_geo, options_trajopt, traj_out,
                       info_out_omplgeo);

  std::cout << "*** info_out_omplgeo   *** " << std::endl;
  info_out_omplgeo.print(std::cout);
  std::cout << "***" << std::endl;

  std::ofstream results(results_file);

  results << "alg: ompl_geo" << std::endl;
  results << "time_stamp: " << get_time_stamp() << std::endl;

  results << "options_geo:" << std::endl;
  options_geo.print(results, "  ");

  results << "options_trajopt:" << std::endl;
  options_trajopt.print(results, "  ");

  info_out_omplgeo.to_yaml(results);
  info_out_omplgeo.print_trajs(results_file.c_str());

  if (traj_out.states.size() && traj_out.actions.size()) {
    std::string file = results_file + ".traj-sol.yaml";
    std::ofstream out(file);
    traj_out.to_yaml_format(out);
  }

  if (info_out_omplgeo.solved) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}
