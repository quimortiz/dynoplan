
#include "dynoplan/dbastar/dbastar.hpp"

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include "Eigen/Core"
#include "dynotree/KDTree.h"
#include <boost/program_options.hpp>

// #include "collision_checker.hpp"

// save data without the cluster stuff

#include <filesystem>
#include <random>
#include <regex>
#include <type_traits>

#include <filesystem>
#include <regex>

#include "dynobench/motions.hpp"
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/ompl/robots.h"
#include "dynotree/KDTree.h"
#include <Eigen/Dense>
#include <iostream>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"
#define BASE_PATH_MOTIONS "../../dynomotions/"

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(check_nn) {

  // compare nn of dynobench vs dynotree
  const char *system = "quad2d_v0";

  const char *motions_file =
      "../../dynomotions/quad2d_v0_all_im.bin.sp.bin.ca.bin.small5000.msgpack";

  Eigen::VectorXd p_lb(2);
  Eigen::VectorXd p_ub(2);

  const char *models_base_path = DYNOBENCH_BASE "models/";
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (models_base_path + std::string(system) + ".yaml").c_str(), p_lb, p_ub);

  ompl::NearestNeighbors<AStarNode *> *T_n = nullptr;
  T_n = nigh_factory2<AStarNode *>(system, robot);
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  T_m = nigh_factory2<Motion *>(system, robot);

  std::vector<Motion> motions;

  load_motion_primitives_new(motions_file, *robot, motions, 1e6, false, false,
                             false);

  for (size_t i = 0; i < motions.size(); ++i)
    T_m->add(&motions.at(i));

  Eigen::VectorXd x(6);

  double delta = .4;
  std::vector<Motion *> neighbors_m;

  T_m->nearestR(&motions.at(0), delta, neighbors_m);

  std::cout << "using nigh " << std::endl;
  for (auto &m : neighbors_m) {
    std::cout << "id " << m->idx << "nn " << m->getStateEig().transpose()
              << std::endl;
  }

  {

    size_t total_neighs = 0;
    auto tic = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < motions.size(); ++i) {
      neighbors_m.clear();
      T_m->nearestR(&motions.at(i), delta, neighbors_m);
      total_neighs += neighbors_m.size();
      // T_m->nearestK(&motions.at(i), 10, neighbors_m);
      // (&motions.at(i), delta, neighbors_m);
    }
    auto toc = std::chrono::high_resolution_clock::now();
    std::cout << "nigh time "
              << std::chrono::duration_cast<std::chrono::milliseconds>(toc -
                                                                       tic)
                     .count()
              << " ms" << std::endl;
    std::cout << "total_neighs " << total_neighs << std::endl;
  }

  std::cout << "done " << std::endl;
  using tree_t = dynotree::KDTree<int, 2>;
  using point_t = Eigen::Vector2d;

  using TreeX =
      dynotree::KDTree<int, -1, 32, double, dynotree::Combined<double>>;
  using Space = dynotree::Combined<double>::Space;

  std::vector<Space> spaces;
  Space space;
  spaces.push_back(dynotree::Rn<double, -1>());
  spaces.push_back(dynotree::SO2<double>());
  spaces.push_back(dynotree::Rn<double, -1>());

  dynotree::Combined<double> combi_space(spaces, {2, 1, 3});
  Eigen::VectorXd weights(6);
  weights << 1, 1, .5, .2, .2, .2;
  combi_space.set_weights(weights);

  combi_space.print(std::cout);

  TreeX tree;
  tree.init_tree(6, combi_space);

  for (size_t i = 0; i < motions.size(); ++i) {
    auto &m = motions.at(i);
    tree.addPoint(m.getStateEig(), i);
  }

  auto out = tree.search(motions.at(0).getStateEig());

  std::cout << "nearest is " << out.id << std::endl;
  std::cout << "distance is " << out.distance << std::endl;

  auto out2 = tree.searchBall(motions.at(0).getStateEig(), delta);

  for (size_t i = 0; i < out2.size(); ++i) {
    std::cout << "id " << out2[i].id;
    std::cout << "id " << out2[i].id << "nn "
              << motions.at(out2[i].id).getStateEig().transpose() << std::endl;
  }

  {
    int total_neighs = 0;
    auto tic = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < motions.size(); ++i) {
      auto out = tree.searchBall(motions.at(i).getStateEig(), delta);
      total_neighs += out.size();
      // tree.searchKnn(motions.at(i).getStateEig(), 10);
      // (motions.at(i).getStateEig(), delta);
    }
    std::cout << "total_neighs " << total_neighs << std::endl;
    auto toc = std::chrono::high_resolution_clock::now();
    std::cout << "dynotree time "
              << std::chrono::duration_cast<std::chrono::milliseconds>(toc -
                                                                       tic)
                     .count()
              << " ms" << std::endl;
  }

  // repeat the same but using the euclidean space

  {
    using TreeX = dynotree::KDTree<int, 6, 32, double, dynotree::Rn<double, 6>>;

    TreeX tree;
    tree.init_tree();
    Eigen::VectorXd weights(6);
    weights << 1, 1, .5, .2, .2, .2;
    tree.getStateSpace().set_weights(weights);

    for (size_t i = 0; i < motions.size(); ++i) {
      auto &m = motions.at(i);
      tree.addPoint(m.getStateEig(), i);
    }

    auto out = tree.search(motions.at(0).getStateEig());

    std::cout << "nearest is " << out.id << std::endl;
    std::cout << "distance is " << out.distance << std::endl;

    auto out2 = tree.searchBall(motions.at(0).getStateEig(), delta);

    for (size_t i = 0; i < out2.size(); ++i) {
      std::cout << "id " << out2[i].id;
      std::cout << "id " << out2[i].id << "nn "
                << motions.at(out2[i].id).getStateEig().transpose()
                << std::endl;
    }

    {
      auto tic = std::chrono::high_resolution_clock::now();
      int total_neighs = 0;
      for (size_t i = 0; i < motions.size(); ++i) {
        auto out = tree.searchBall(motions.at(i).getStateEig(), delta);
        // tree.searchKnn(motions.at(i).getStateEig(), 10);
        // (motions.at(i).getStateEig(), delta);
      
            total_neighs += out.size();
            }

            std::cout << "total_neighs " << total_neighs << std::endl;
      auto toc = std::chrono::high_resolution_clock::now();
      std::cout << "dynotree time "
                << std::chrono::duration_cast<std::chrono::milliseconds>(toc -
                                                                         tic)
                       .count()
                << " ms" << std::endl;
    }
  }
}

BOOST_AUTO_TEST_CASE(extra_time) {

  Problem problem(DYNOBENCH_BASE "envs/quad2d_v0/quad_bugtrap.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Options_dbastar options_dbastar;
  options_dbastar.delta = .55;
  options_dbastar.limit_branching_factor = 20;
  options_dbastar.max_motions = 350;
  options_dbastar.search_timelimit = 30000;

  options_dbastar.motionsFile =
      BASE_PATH_MOTIONS "quad2d_v0_all_im.bin.sp.bin.ca.bin."
                        "small5000.msgpack";

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  std::vector<Motion> motions;
  load_motion_primitives_new(
      options_dbastar.motionsFile, *robot, motions, options_dbastar.max_motions,
      options_dbastar.cut_actions, false, options_dbastar.check_cols);

  options_dbastar.motions_ptr = &motions;
  Trajectory traj_out;
  Out_info_db out_info_db;
  dbastar(problem, options_dbastar, traj_out, out_info_db);
}

BOOST_AUTO_TEST_CASE(test_heu_map) {

  Problem problem(DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_env(*robot, problem);

  Options_dbastar options;
  options.max_size_heu_map = 2000;
  options.num_sample_trials = 4000;

  std::vector<Heuristic_node> heu_map;
  generate_heuristic_map(problem, robot, options, heu_map);

  auto filename = "/tmp/dynoplan/tmp_heu_map.yaml";
  create_dir_if_necessary(filename);
  write_heu_map(heu_map, filename);

  // YOU CAN VISUALIZE WITH:
  // python3 ../utils/show_heuristic_map.py --file
  // /tmp/dynoplan/tmp_heu_map.yaml --max 30

  // check a couple of points

  auto hh = std::make_shared<Heu_roadmap>(robot, heu_map, problem.goal,
                                          problem.robotType);

  hh->connect_radius_h = options.connect_radius_h;

  double h1 = hh->h(problem.goal);
  double h2 = hh->h(problem.start);
  double h3 = hh->h(Eigen::Vector3d(5, 5, 0));
  double h4 = hh->h(Eigen::Vector3d(4, 5.5, 1.57));
  double h5 = hh->h(Eigen::Vector3d(1, 5, 0));

  std::vector<double> order_h = {h1, h3, h4, h5, h2};
  std::vector<double> order_expected = {h1, h3, h4, h5, h2};

  std::sort(order_h.begin(), order_h.end());

  std::stringstream ss;
  print_vec(order_h.data(), order_h.size(), ss);

  std::stringstream ss2;
  print_vec(order_expected.data(), order_expected.size(), ss2);

  BOOST_TEST(order_h == order_expected,
             "odrder_h " + ss.str() + " " + "order_expected " + ss2.str());
}

BOOST_AUTO_TEST_CASE(test_bugtrap_heu) {
  // TODO: PLOT the HEU MAP

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle1_v0/bugtrap_0.yaml"));
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Options_dbastar options_dbastar;
  options_dbastar.search_timelimit = 1e5; // in ms
  options_dbastar.max_motions = 100;
  options_dbastar.motionsFile =
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin";

  // DEBUG THIS!! -- add tools for debugging :)

  Out_info_db out_info_db;
  Trajectory traj_out;

  std::vector<Motion> motions;

  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  load_motion_primitives_new(
      options_dbastar.motionsFile, *robot, motions, options_dbastar.max_motions,
      options_dbastar.cut_actions, false, options_dbastar.check_cols);

  options_dbastar.motions_ptr = &motions;

  std::vector<int> heus;
  // heus = {0, 1, -1};
  heus = {0};
  // heus = {1};
  for (auto &heu : heus) {
    options_dbastar.heuristic = heu;
    Trajectory traj_out;
    Out_info_db info_out;
    BOOST_REQUIRE_NO_THROW(
        dbastar(problem, options_dbastar, traj_out, out_info_db));
    std::string message = std::string("heu: ") + std::to_string(heu);
    BOOST_TEST(out_info_db.solved, heu);
    BOOST_TEST(out_info_db.cost < 100., heu);
  }
}
// ADD the test that i use for  the paper heuristic evaluation!

BOOST_AUTO_TEST_CASE(test_eval_multiple) {

  // "tmp_motions_unicycle1_v1.bin.sp.bin.small5000.msgpack"
  // "tmp_motions_unicycle1_v2.bin.sp.bin.small5000.msgpack"
  // "unicycle2_v0__ispso__2023_04_03__15_36_01.bin.im.bin.im.bin.small5000."
  // "msgpack"
  // "car1_v0_all.bin.sp.bin.small5000.msgpack"
  // "quad2d_v0_all_im.bin.sp.bin.ca.bin.small5000.msgpack"
  // "quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack"
  // "acrobot_v0_all2.bin.sp.bin.small5000.msgpack"
  // "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack"
  // "quad3dompl_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack"

  std::vector<Problem> problems{
      DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml",
      DYNOBENCH_BASE "envs/unicycle2_v0/bugtrap_0.yaml",
      DYNOBENCH_BASE "envs/car1_v0/bugtrap_0.yaml",
      DYNOBENCH_BASE "envs/quad2d_v0/quad_bugtrap.yaml",
      DYNOBENCH_BASE "envs/quadrotor_v0/quad_one_obs.yaml",
      DYNOBENCH_BASE "envs/quadrotor_v0/window.yaml",
      DYNOBENCH_BASE "envs/quad2dpole_v0/window.yaml"};

  for (auto &p : problems) {
    p.models_base_path = DYNOBENCH_BASE "models/";
  }

  Options_dbastar o_uni1, o_uni2, o_car, o_quad2d, o_quad3d, o_quad2dpole;

  // TODO: use less primitive so that the test runs faster!!

  o_uni1.max_motions = 300;
  o_uni1.delta = .3;
  o_uni1.motionsFile =
      BASE_PATH_MOTIONS "unicycle1_v0__ispso__2023_04_03__14_56_"
                        "57.bin.im.bin.im.bin.small5000."
                        "msgpack";

  o_uni2.max_motions = 400;
  o_uni2.delta = .4;
  o_uni2.motionsFile =
      BASE_PATH_MOTIONS "unicycle2_v0__ispso__2023_04_03__15_36_01."
                        "bin.im.bin.im.bin.small5000.msgpack";

  o_car.max_motions = 400;
  o_car.delta = .3;
  o_car.motionsFile =
      BASE_PATH_MOTIONS "car1_v0_all.bin.sp.bin.small5000.msgpack";

  o_quad2d.max_motions = 400;
  o_quad2d.delta = .5;
  o_quad2d.motionsFile = BASE_PATH_MOTIONS "quad2d_v0_all_im.bin.sp.bin.ca.bin."
                                           "small5000.msgpack";
  o_quad2d.new_invariance = true;

  o_quad3d.delta = .8;
  o_quad3d.max_motions = 3000;
  o_quad3d.motionsFile =
      BASE_PATH_MOTIONS "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin."
                        "small5000.msgpack";
  o_quad3d.new_invariance = true;

  o_quad2dpole.delta = .8;
  o_quad2dpole.max_motions = 4000;
  o_quad2dpole.motionsFile = BASE_PATH_MOTIONS
      "quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack";
  o_quad2dpole.new_invariance = true;

  std::vector<Options_dbastar> options{
      o_uni1, o_uni2, o_car, o_quad2d, o_quad3d, o_quad3d, o_quad2dpole};

  // std::vector<Options_dbastar> options{o_quad3d, o_quad3d};

  for (auto &o : options) {
    o.search_timelimit = 40 * 10e3;
  }

  // you can choose the heuristic here!!

  DYNO_CHECK_EQ(options.size(), problems.size(), AT);

  for (size_t j = 0; j < options.size(); j++) {

    auto &problem = problems[j];
    auto &option = options[j];

    std::string msg = problem.name;

    Trajectory traj_out;
    Out_info_db info_out;

    // load motions

    std::vector<Motion> motions;

    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotType + ".yaml").c_str(),
        problem.p_lb, problem.p_ub);

    load_motion_primitives_new(option.motionsFile, *robot, motions,
                               option.max_motions, option.cut_actions, false,
                               option.check_cols);

    option.motions_ptr = &motions;

    BOOST_REQUIRE_NO_THROW(dbastar(problem, option, traj_out, info_out));

    BOOST_TEST(info_out.solved, msg);
  }

  // #-
  // #- quadrotor_v0 / quad_one_obs
  // #- quadrotor_v0 / window

  // continue here!!
}
