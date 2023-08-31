#include <boost/test/unit_test.hpp>

#include "idbastar/optimization/ocp.hpp"

using namespace dynoplan;
using namespace dynobench;

#define dynobench_base "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_payload_hello) {

  std::cout << ":)" << std::endl;
  BOOST_TEST(true);
}

BOOST_AUTO_TEST_CASE(t_payload_optimization_easy) {

  Options_trajopt options;
  // KHALED: TODO -> modify the start and goal in problem1 and problem2
  Problem problem1(dynobench_base "envs/quad3d_payload/empty_0.yaml");
  Problem problem2(dynobench_base "envs/quad3d_payload/empty_1.yaml");

  problem1.models_base_path = dynobench_base "models/";

  problem2.models_base_path = dynobench_base "models/";

  Trajectory init_guess1;
  Trajectory init_guess2;
  init_guess1.num_time_steps = 100;
  init_guess2.num_time_steps = 100;

  Trajectory sol1, sol2;
  Result_opti result1, result2;

  trajectory_optimization(problem1, init_guess1, options, sol1, result1);
  BOOST_TEST(result1.feasible);

  trajectory_optimization(problem2, init_guess2, options, sol2, result2);

  sol1.to_yaml_format("/tmp/dynoplan_sol1.yaml");
  sol2.to_yaml_format("/tmp/dynoplan_sol2.yaml");

  BOOST_TEST(result2.feasible);

  Problem problem3(
      dynobench_base
      "envs/quad3d_payload/quad3d_payload_one_obs/quad3d_payload_one_obs.yaml");

  problem3.models_base_path = dynobench_base "models/";

  Trajectory init_guess3( dynobench_base  "envs/quad3d_payload/trajectories/"
                         "quad3d_payload_one_obs_init_guess.yaml");

  Result_opti result3;
  Trajectory sol3;

  trajectory_optimization(problem3, init_guess3, options, sol3, result3);
  BOOST_TEST(result3.feasible);

  sol3.to_yaml_format("/tmp/dynoplan_sol3.yaml");

}

BOOST_AUTO_TEST_CASE(t_two_uav) {


  Problem problem( dynobench_base "envs/quad3d_payload/quad3d_payload_one_obs/" "quad3d_payload_one_obs_0_2_pm_hard.yaml");


  problem.models_base_path = dynobench_base "models/";
  Trajectory init_guess ( dynobench_base "envs/quad3d_payload/trajectories/quad3d_payload_2_pm_hard_init_guess.yaml");


  Result_opti result;
  Trajectory sol;

  Options_trajopt options;
  options.weight_goal = 200;
  options.max_iter = 200;

  trajectory_optimization(problem, init_guess, options, sol, result);
  BOOST_TEST(result.feasible);


  sol.to_yaml_format("/tmp/dynoplan_two_uav.yaml");
}

BOOST_AUTO_TEST_CASE(t_two_uav_easy) {
// dynobench/envs/

  Problem problem( dynobench_base "envs/quad3d_payload/empty1_2_pm_easy.yaml");
                  // quad3d_payload_one_obs/" "quad3d_payload_one_obs_0_2_pm_easy.yaml");


  problem.models_base_path = dynobench_base "models/";

  Trajectory init_guess ( dynobench_base "envs/quad3d_payload/trajectories/quad3d_payload_2_pm_hover_initial_guess.yaml");
  // Trajectory init_guess;
  // init_guess.num_time_steps = 1000;
  // Trajectory init_guess ;

  Result_opti result;
  Trajectory sol;

  Options_trajopt options;
  // options.soft_control_bounds = true;
  options.weight_goal = 200;
  options.max_iter = 200;
  options.smooth_traj = false;
  // options.control_bounds = false;
  // options.u_bound_scale = 2.0;

  trajectory_optimization(problem, init_guess, options, sol, result);
  BOOST_TEST(result.feasible);


  sol.to_yaml_format("/tmp/dynoplan_two_uav.yaml");
}




