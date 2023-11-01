#include "dynoplan/dbastar/heuristics.hpp"
#include "dynoplan/nigh_custom_spaces.hpp"

namespace dynoplan {

void generate_heuristic_map(const dynobench::Problem &problem,
                            std::shared_ptr<dynobench::Model_robot> robot,
                            const Options_dbastar &options_dbastar,
                            std::vector<Heuristic_node> &heu_map) {

  std::vector<Eigen::VectorXd> samples;
  Eigen::VectorXd v(robot->nx);

  size_t nx_pr = robot->nx_pr;
  size_t nx = robot->nx;

  Eigen::VectorXd __start = problem.start;
  __start.segment(nx_pr, nx - nx_pr).setZero();

  samples.push_back(__start);
  for (size_t i = 0; i < options_dbastar.num_sample_trials; i++) {
    // generate one sample
    //
    robot->sample_uniform(v);

    // set the velocity components to zero
    v.segment(robot->nx_pr, robot->nx - robot->nx_pr).setZero();

    if (robot->collision_check(v)) {
      samples.push_back(v);
    }

    if (samples.size() == options_dbastar.max_size_heu_map) {
      break;
    }
  }
  Eigen::VectorXd __goal = problem.goal;
  __goal.segment(nx_pr, nx - nx_pr).setZero();
  samples.push_back(__goal); // important! goal should be the last one!!

  build_heuristic_distance_new(samples, robot, heu_map,
                               options_dbastar.heu_connection_radius,
                               options_dbastar.heu_resolution);
}

Heu_roadmap::Heu_roadmap(std::shared_ptr<dynobench::Model_robot> robot,
                         const std::vector<Heuristic_node> &t_heu_map,
                         const Eigen::VectorXd &goal,
                         const std::string &robot_type)
    : robot(robot), goal(goal), heu_map(t_heu_map) {

  std::vector<double> xx(robot->nx, 0.);

  const bool use_nigh = true;

  auto tt = nigh_factory2<Heuristic_node *>(robot_type, robot);
  T_heu.reset(tt);

  for (size_t i = 0; i < heu_map.size(); i++) {
    T_heu->add(&heu_map.at(i));
  }
}

#if 0
Heu_roadmap::Heu_roadmap(std::shared_ptr<RobotOmpl> robot,
                         const std::vector<Heuristic_node> &heu_map,
                         ob::State *goal, const std::string &robot_type)
    : robot(robot), goal(goal) {

  // __x_zero_vel = robot->getSpaceInformation()->allocState();
  std::vector<double> xx(robot->nx, 0.);
  __x_zero_vel = _allocAndFillState(robot->getSpaceInformation(), xx);

  const bool use_nigh = true;

  if (use_nigh) {
    CHECK(robot_type.size(), AT);
    auto tt = nigh_factory<HeuNode *>(robot_type, robot);
    T_heu.reset(tt);
  } else {
    if (robot->getSpaceInformation()->getStateSpace()->isMetricSpace()) {
      T_heu = std::make_shared<
          ompl::NearestNeighborsGNATNoThreadSafety<HeuNode *>>();
    } else {
      T_heu = std::make_shared<ompl::NearestNeighborsSqrtApprox<HeuNode *>>();
    }

    T_heu->setDistanceFunction([this](const HeuNode *a, const HeuNode *b) {
      return this->robot->getSpaceInformation()->distance(a->state, b->state);
    });
  }

  for (size_t i = 0; i < heu_map.size(); i++) {
    //

    ob::State *tmp;
    tmp = robot->getSpaceInformation()->allocState();
    robot->fromEigen(tmp, heu_map.at(i).x);

    HeuNode *ptr = new HeuNode; // memory leak, stop bad code
    ptr->state = tmp;
    ptr->dist = heu_map[i].d;

    T_heu->add(ptr);
  }
}
#endif

void get_distance_all_vertices(const EdgeList &edge_list,
                               const DistanceList &distance_list,
                               double *dist_out, int *parents_out, int n,
                               int goal) {

  using graph_t = boost::adjacency_list<
      boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
      boost::property<boost::edge_weight_t, double>>; // int or double?

  graph_t g(edge_list.data(), edge_list.data() + edge_list.size(),
            distance_list.data(), n);

  typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;

  std::vector<vertex_descriptor> p(num_vertices(g));
  vertex_descriptor s = vertex(goal, g);

  bool verbose = false;
  if (verbose)
    boost::write_graphviz(std::cout, g);

  dijkstra_shortest_paths(
      g, s,
      boost::predecessor_map(boost::make_iterator_property_map(
                                 p.begin(), get(boost::vertex_index, g)))
          .distance_map(boost::make_iterator_property_map(
              dist_out, get(boost::vertex_index, g))));

  boost::graph_traits<graph_t>::vertex_iterator vi, vend;
  for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
    parents_out[*vi] = p[*vi];
  }

  if (verbose) {
    std::cout << "distances and parents:" << std::endl;
    boost::graph_traits<graph_t>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {

      std::cout << "distance(" << *vi << ") = " << dist_out[*vi] << ", ";
      std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;
    }
    std::cout << std::endl;
  }
}

void compute_heuristic_map_new(
    const EdgeList &edge_list, const DistanceList &distance_list,
    const std::vector<Eigen::VectorXd> &batch_samples,
    std::vector<Heuristic_node> &heuristic_map) {
  std::vector<double> distances(batch_samples.size());
  std::vector<int> parents(batch_samples.size());

  auto out3 = timed_fun([&] {
    get_distance_all_vertices(edge_list, distance_list, distances.data(),
                              parents.data(), batch_samples.size(),
                              batch_samples.size() - 1);
    return 0;
  });

  std::cout << "time boost " << out3.second << std::endl;

  heuristic_map.clear();
  heuristic_map.resize(batch_samples.size());
  for (size_t i = 0; i < batch_samples.size(); i++) {
    heuristic_map.at(i) = {batch_samples.at(i), distances.at(i), parents.at(i)};
  }
}

void build_heuristic_distance_new(
    const std::vector<Eigen::VectorXd> &batch_samples,
    std::shared_ptr<dynobench::Model_robot> &robot,
    std::vector<Heuristic_node> &heuristic_map, double distance_threshold,
    double resolution) {

  EdgeList edge_list;
  DistanceList distance_list;

  auto tic = std::chrono::high_resolution_clock::now();

  auto out = timed_fun([&] {
    for (size_t i = 0; i < batch_samples.size(); i++) {
      std::cout << "i " << i << std::endl;
      for (size_t j = i + 1; j < batch_samples.size(); j++) {
        auto p1 = batch_samples.at(i);
        auto p2 = batch_samples.at(j);
        double d = robot->distance(p1, p2);
        if (d < distance_threshold &&
            dynobench::check_edge_at_resolution(p1, p2, robot, resolution)) {
          edge_list.push_back({i, j});
          distance_list.push_back(robot->lower_bound_time(p1, p2));
        }
      }
    }
    return 0;
  });

  std::cout << "time building distance matrix " << out.second << std::endl;

  compute_heuristic_map_new(edge_list, distance_list, batch_samples,
                            heuristic_map);
}

double heuristicCollisionsTree(ompl::NearestNeighbors<Heuristic_node *> *T_heu,
                               const Eigen::VectorXd &x,
                               std::shared_ptr<dynobench::Model_robot> robot,
                               double connect_radius_h) {
  Heuristic_node node;
  node.x = x;
  std::vector<Heuristic_node *> neighbors;
  double min = 1e8;
  CHECK(T_heu, AT);
  T_heu->nearestR(&node, connect_radius_h, neighbors);
  for (const auto &p : neighbors) {
    if (double d = p->d + robot->lower_bound_time_pr(p->x, x); d < min) {
      min = d;
    }
  }
  return min;
}

} // namespace dynoplan
