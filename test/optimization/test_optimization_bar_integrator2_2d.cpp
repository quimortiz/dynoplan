#include <boost/test/unit_test.hpp>
#include "dynoplan/optimization/ocp.hpp"


using namespace dynoplan;
using namespace dynobench;

// #define dynobench_base "../../dynobench/"
#define dynobench_base "../dynoplan/dynobench/"


BOOST_AUTO_TEST_CASE(t_Bar_integrator2_2d_optimization) {


  Options_trajopt options;
  options.solver_id = 1;
  options.max_iter = 200;
  options.weight_goal = 300.;
  options.collision_weight = 50.0;

  // Problem problem(dynobench_base "example/2dint_rod_window.yaml");
  Problem problem("/home/khaledwahba94/imrc/db-CBS/example/bar_integrator2_2d_window.yaml");

  // problem.models_base_path = dynobench_base "models/";
  problem.models_base_path = "/home/khaledwahba94/imrc/db-CBS/dynoplan/dynobench/models/";
  
  Trajectory init_guess("/home/khaledwahba94/imrc/db-CBS/init_guess.yaml");
  // Trajectory init_guess("../init_guess.yaml");

  Trajectory sol;
  Result_opti result;

  trajectory_optimization(problem, init_guess, options, sol, result);

  sol.to_yaml_format("/home/khaledwahba94/imrc/db-CBS/bar_integrator2_2d_opt.yaml");
}