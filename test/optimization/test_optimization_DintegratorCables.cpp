#include <boost/test/unit_test.hpp>
#include "dynoplan/optimization/ocp.hpp"


using namespace dynoplan;
using namespace dynobench;

// #define dynobench_base "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_DintegratorCables_optimization) {


  Options_trajopt options;
  options.solver_id = 0;
  options.max_iter = 200;
  options.weight_goal = 300;
  options.use_finite_diff = false;
  options.collision_weight = 200.0;

  Problem problem("/home/khaledwahba94/imrc/db-CBS/example/cables_integrator2_2d_window.yaml");

  problem.models_base_path = "/home/khaledwahba94/imrc/db-CBS/dynoplan/dynobench/models/";
  Trajectory init_guess("/home/khaledwahba94/imrc/db-CBS/init_guess_cables.yaml");

  Trajectory sol;
  Result_opti result;

  trajectory_optimization(problem, init_guess, options, sol, result);

  sol.to_yaml_format("/home/khaledwahba94/imrc/db-CBS/cables_integrator2_2d_window_opt.yaml");
  // BOOST_TEST(result.feasible);

}