#pragma once

// OMPL headers
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <yaml-cpp/node/node.h>

// FCL
#include "fclHelper.hpp"
#include <fcl/fcl.h>

#include "ompl/control/StatePropagator.h"

namespace dynoplan {

// struct Model_robot;

ompl::base::State *
_allocAndFillState(std::shared_ptr<ompl::control::SpaceInformation> si,
                   const std::vector<double> &reals);

ompl::base::State *
allocAndFillState(std::shared_ptr<ompl::control::SpaceInformation> si,
                  const YAML::Node &node);

void copyFromRealsControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                          ompl::control::Control *out,
                          const std::vector<double> &reals);

ompl::control::Control *
allocAndFillControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                    const YAML::Node &node);

void copyToRealsControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                        ompl::control::Control *action,
                        std::vector<double> &reals);

void copyFromRealsControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                          ompl::control::Control *out,
                          const std::vector<double> &reals);

void state_to_stream(std::ostream &out,
                     std::shared_ptr<ompl::control::SpaceInformation> si,
                     const ompl::base::State *state);

void state_to_eigen(Eigen::VectorXd &out,
                    std::shared_ptr<ompl::control::SpaceInformation> si,
                    const ompl::base::State *state);

void control_to_eigen(Eigen::VectorXd &out,
                      std::shared_ptr<ompl::control::SpaceInformation> si,
                      ompl::control::Control *control);

void control_from_eigen(const Eigen::VectorXd &out,
                        std::shared_ptr<ompl::control::SpaceInformation> si,
                        ompl::control::Control *control);

struct RobotOmpl {

  std::shared_ptr<dynobench::Model_robot> diff_model;
  RobotOmpl(std::shared_ptr<dynobench::Model_robot> diff_model);

  virtual ~RobotOmpl() {
    // TODO: erase state and goal!
  }

  // Eigen wrappers...

  size_t nx;    // dim state
  size_t nx_pr; // dim positon-rotation
  size_t nu;    // dim control

  Eigen::VectorXd xx; // data
  Eigen::VectorXd zz; // data
  Eigen::VectorXd yy; // data
  Eigen::VectorXd uu; // data

  virtual void setCustomStateSampling() { NOT_IMPLEMENTED; }

  virtual void geometric_interpolation(const ompl::base::State *from,
                                       const ompl::base::State *to, double t,
                                       ompl::base::State *out);

  virtual void propagate(const ompl::base::State *start,
                         const ompl::control::Control *control,
                         const double duration, ompl::base::State *result);

  virtual double cost_lower_bound(const ompl::base::State *x,
                                  const ompl::base::State *y);

  double cost_lower_bound_pr(const ompl::base::State *x,
                             const ompl::base::State *y);

  double cost_lower_bound_vel(const ompl::base::State *x,
                              const ompl::base::State *y);

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) {
    (void)x_ompl;
    (void)x_eigen;
    ERROR_WITH_INFO("not implemented");
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) {
    (void)control;
    (void)u_eigen;
    ERROR_WITH_INFO("not implemented");
  }

  virtual void fromEigen(ompl::base::State *x_ompl,
                         const Eigen::Ref<const Eigen::VectorXd> &x_eigen) {
    (void)x_ompl;
    (void)x_eigen;
    ERROR_WITH_INFO("not implemented");
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t part = 0) = 0;

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) = 0;

  virtual size_t numParts() { return 1; }

  std::shared_ptr<ompl::control::SpaceInformation> getSpaceInformation() {
    return si_;
  }

  virtual void enforceBounds(ompl::base::State *) const { NOT_IMPLEMENTED; }

  double dt() const;

  bool is2D() const;

  bool isTranslationInvariant() const { return translation_invariant_; }

  std::string getName() const;

public:
  std::shared_ptr<ompl::control::SpaceInformation> si_;
  bool translation_invariant_ = true;
  std::string name_;
  Eigen::VectorXd u_zero;
  Eigen::VectorXd x_lb;
  Eigen::VectorXd x_ub;
  ompl::base::State *startState;
  ompl::base::State *goalState;

  // TODO: fix memory leaks!!!
};

struct RobotStateValidityChecker : public ompl::base::StateValidityChecker {
  std::shared_ptr<RobotOmpl> robot;
  mutable Eigen::VectorXd x_eigen;
  RobotStateValidityChecker(std::shared_ptr<RobotOmpl> robot);

  bool virtual isValid(const ompl::base::State *state) const override;
};

std::shared_ptr<RobotOmpl>
robot_factory_ompl(const dynobench::Problem &problem);

class RobotOmplStatePropagator : public ompl::control::StatePropagator {
public:
  RobotOmplStatePropagator(const ompl::control::SpaceInformationPtr &si,
                           std::shared_ptr<RobotOmpl> robot)
      : ompl::control::StatePropagator(si), robot_(robot) {}

  ~RobotOmplStatePropagator() override = default;

  void propagate(const ompl::base::State *state,
                 const ompl::control::Control *control, double duration,
                 ompl::base::State *result) const override {
    // propagate state
    robot_->propagate(state, control, duration, result);
  }

  bool canPropagateBackward() const override { return false; }

  bool canSteer() const override { return false; }

protected:
  std::shared_ptr<RobotOmpl> robot_;
};

std::ostream &printState(std::ostream &stream, const std::vector<double> &x);

std::ostream &printState(std::ostream &stream,
                         std::shared_ptr<ompl::control::SpaceInformation> si,
                         const ompl::base::State *state,
                         bool add_brackets_comma = true);

// void ompl::base::StateSpace::copyFromReals(
//     State *destination, const std::vector<double> &reals) const;

std::ostream &printAction(std::ostream &stream,
                          std::shared_ptr<ompl::control::SpaceInformation> si,
                          ompl::control::Control *action);

class Motion {

  using Trajectory = dynobench::Trajectory;

public:
  std::vector<ompl::base::State *> states;
  std::vector<ompl::control::Control *> actions;
  Trajectory traj;

  std::shared_ptr<ShiftableDynamicAABBTreeCollisionManager<double>>
      collision_manager;
  std::vector<std::unique_ptr<fcl::CollisionObjectd>> collision_objects;
  // for merged aabb
  std::shared_ptr<ShiftableDynamicAABBTreeCollisionManager<double>>
      collision_manager_merged;
  std::vector<std::unique_ptr<fcl::CollisionObjectd>> collision_objects_merged;

  double cost;
  size_t idx;
  bool disabled = false;
  double get_cost() const { return cost; }

  void print(std::ostream &out,
             std::shared_ptr<ompl::control::SpaceInformation> &si);

  const ompl::base::State *getState() const {
    assert(states.size());
    return states.front();
  }

  const Eigen::VectorXd &getStateEig() const { return traj.states.front(); }
  const Eigen::VectorXd &getLastStateEig() const { return traj.states.back(); }
};

enum class MotionPrimitiveFormat { BOOST, YAML, JSON, MSGPACK, AUTO };

void load_motion_primitives_new(
    const std::string &motionsFile, dynobench::Model_robot &robot,
    std::vector<Motion> &motions, int max_motions, bool cut_actions,
    bool shuffle, bool compute_col = true, bool merged = false,
    MotionPrimitiveFormat format = MotionPrimitiveFormat::AUTO);

void load_motion_primitives(const std::string &motionsFile, RobotOmpl &robot,
                            std::vector<Motion> &motions, int max_motions,
                            bool cut_actions, bool shuffle);

void traj_to_motion(const dynobench::Trajectory &traj,
                    dynobench::Model_robot &robot, Motion &motion_out,
                    bool compute_col, bool merged = false);

void compute_col_shape(Motion &m, dynobench::Model_robot &robot,
                       bool merged = false);

} // namespace dynoplan
