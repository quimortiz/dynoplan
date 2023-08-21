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

  Trajectory sol;
  Result_opti result1, result2;

  trajectory_optimization(problem1, init_guess1, options, sol, result1);
  BOOST_TEST(result1.feasible);
  trajectory_optimization(problem2, init_guess2, options, sol, result2);
  BOOST_TEST(result2.feasible);

}
