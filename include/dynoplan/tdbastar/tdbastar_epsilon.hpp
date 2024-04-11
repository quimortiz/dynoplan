#pragma once
#include <algorithm>
// // #include <boost/graph/graphviz.hpp>
#include "Eigen/Core"
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
//
// #include <flann/flann.hpp>
// #include <msgpack.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
//
// // #include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
// OMPL
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
// #include "fclHelper.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"
#include <fcl/fcl.h>

#include "dynoplan/dbastar/heuristics.hpp"
#include "dynoplan/tdbastar/options.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"

namespace dynoplan {

namespace ob = ompl::base;

using Sample = std::vector<double>;
using Sample_ = ob::State;

// focalSet
struct compareFocalHeuristic {
  bool operator()(const open_t::handle_type &h1,
                  const open_t::handle_type &h2) const;
};

typedef typename boost::heap::d_ary_heap<
    open_t::handle_type, boost::heap::arity<2>,
    boost::heap::compare<compareFocalHeuristic>, boost::heap::mutable_<true>>
    focal_t;

void tdbastar_epsilon(
    dynobench::Problem &problem, Options_tdbastar options_dbastar,
    dynobench::Trajectory &traj_out, const std::vector<Constraint> &constraints,
    Out_info_tdb &out_info_tdb, size_t &robot_id, bool reverse_search,
    std::vector<dynobench::Trajectory> &expanded_trajs,
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    std::vector<std::vector<std::pair<std::shared_ptr<AStarNode>, size_t>>>
        &results,
    // std::vector<std::map<size_t, Motion*>> &result_motions,
    std::map<std::string, std::vector<Motion>> &robot_motions,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs,
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *heuristic_nn = nullptr,
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>> **heuristic_result =
        nullptr,
    float w = 0.0, std::string focal_heuristic_name = "volume_wise");

int highLevelfocalHeuristic(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs);

int lowLevelfocalHeuristic(
    const std::vector<dynobench::Trajectory> &solution,
    const std::shared_ptr<AStarNode> node_to_check, size_t &robot_id,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs);

int lowLevelfocalHeuristicShape(
    std::vector<std::vector<std::pair<std::shared_ptr<AStarNode>, size_t>>>
        &results,
    std::map<std::string, std::vector<Motion>> &robot_motions,
    const dynobench::Problem &problem, LazyTraj &lazy_traj, size_t &robot_id,
    const float current_gscore,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots);

int lowLevelfocalHeuristicStateInInterval(
    std::vector<std::vector<std::pair<std::shared_ptr<AStarNode>, size_t>>>
        &results,
    dynobench::TrajWrapper &tmp_traj, size_t &robot_id, float delta,
    const float motion_start, const float motion_end,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots);

bool lazy_trajectory_to_wrapper(
    LazyTraj &lazy_traj, dynobench::Model_robot &robot,
    Time_benchmark &time_bench, dynobench::TrajWrapper &tmp_traj,
    Eigen::Ref<Eigen::VectorXd> x,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng,
    std::vector<fcl::CollisionObjectd *> &objs,
    Eigen::Ref<Eigen::VectorXd> aux_last_state,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state = nullptr,
    int *num_valid_states = nullptr, bool forward = true);

bool check_lazy_trajectory_hybrid(
    LazyTraj &lazy_traj, dynobench::Model_robot &robot,
    const Eigen::Ref<const Eigen::VectorXd> &goal, Time_benchmark &time_bench,
    dynobench::TrajWrapper &tmp_traj, std::shared_ptr<AStarNode> node_to_check,
    const float gscore, const std::vector<Constraint> &constraints, float delta,
    Eigen::Ref<Eigen::VectorXd> aux_last_state,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state = nullptr,
    int *num_valid_states = nullptr, bool forward = true);

} // namespace dynoplan
