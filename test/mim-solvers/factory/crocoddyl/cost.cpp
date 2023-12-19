///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the cost model unittests factory from the
// Crocoddyl library This modified version is used for testing purposes only
// Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/cost.cpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "cost.hpp"

#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/multibody/residuals/control-gravity.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
// #include "crocoddyl/multibody/residuals/centroidal-momentum.hpp"
#include "crocoddyl/core/activations/quadratic.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/residuals/contact-friction-cone.hpp"
#include "crocoddyl/multibody/residuals/contact-wrench-cone.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/pair-collision.hpp"

namespace mim_solvers {
namespace unittest {

const std::vector<CostModelTypes::Type>
    CostModelTypes::all(CostModelTypes::init_all());
const std::vector<CostModelNoFFTypes::Type>
    CostModelNoFFTypes::all(CostModelNoFFTypes::init_all());
#ifdef PINOCCHIO_WITH_HPP_FCL
const std::vector<CostModelCollisionTypes::Type>
    CostModelCollisionTypes::all(CostModelCollisionTypes::init_all());
#endif // PINOCCHIO_WITH_HPP_FCL

std::ostream &operator<<(std::ostream &os, CostModelTypes::Type type) {
  switch (type) {
  case CostModelTypes::CostModelResidualState:
    os << "CostModelResidualState";
    break;
  case CostModelTypes::CostModelResidualControl:
    os << "CostModelResidualControl";
    break;
  case CostModelTypes::CostModelResidualFramePlacement:
    os << "CostModelResidualFramePlacement";
    break;
  case CostModelTypes::NbCostModelTypes:
    os << "NbCostModelTypes";
    break;
  default:
    break;
  }
  return os;
}

std::ostream &operator<<(std::ostream &os, CostModelNoFFTypes::Type type) {
  switch (type) {
  case CostModelNoFFTypes::CostModelResidualControlGrav:
    os << "CostModelResidualControlGrav";
    break;
  case CostModelNoFFTypes::NbCostModelNoFFTypes:
    os << "NbCostModelNoFFTypes";
    break;
  default:
    break;
  }
  return os;
}

#ifdef PINOCCHIO_WITH_HPP_FCL
std::ostream &operator<<(std::ostream &os, CostModelCollisionTypes::Type type) {
  switch (type) {
  case CostModelCollisionTypes::CostModelResidualPairCollision:
    os << "CostModelResidualPairCollision";
    break;
  case CostModelCollisionTypes::NbCostModelCollisionTypes:
    os << "NbCostModelCollisionTypes";
    break;
  default:
    break;
  }
  return os;
}
#endif // PINOCCHIO_WITH_HPP_FCL

CostModelFactory::CostModelFactory() {}
CostModelFactory::~CostModelFactory() {}

boost::shared_ptr<crocoddyl::CostModelAbstract> CostModelFactory::create(
    CostModelTypes::Type cost_type, StateModelTypes::Type state_type,
    ActivationModelTypes::Type activation_type, std::size_t nu) const {
  StateModelFactory state_factory;
  ActivationModelFactory activation_factory;
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(
          state_factory.create(state_type));

  pinocchio::FrameIndex frame_index = state->get_pinocchio()->frames.size() - 1;
  pinocchio::SE3 frame_SE3 = pinocchio::SE3::Random();

  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }
  switch (cost_type) {
  case CostModelTypes::CostModelResidualState:
    cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_factory.create(activation_type, state->get_ndx()),
        boost::make_shared<crocoddyl::ResidualModelState>(state, state->rand(),
                                                          nu));
    break;
  case CostModelTypes::CostModelResidualControl:
    cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_factory.create(activation_type, nu),
        boost::make_shared<crocoddyl::ResidualModelControl>(
            state, Eigen::VectorXd::Random(nu)));
    break;
  case CostModelTypes::CostModelResidualFramePlacement:
    cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_factory.create(activation_type, 6),
        boost::make_shared<crocoddyl::ResidualModelFramePlacement>(
            state, frame_index, frame_SE3, nu));
    break;
  default:
    throw_pretty(__FILE__ ": Wrong CostModelTypes::Type given");
    break;
  }
  return cost;
}

boost::shared_ptr<crocoddyl::CostModelAbstract>
CostModelFactory::create(CostModelNoFFTypes::Type cost_type,
                         ActivationModelTypes::Type activation_type,
                         std::size_t nu) const {
  StateModelFactory state_factory;
  ActivationModelFactory activation_factory;
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(
          state_factory.create(StateModelTypes::StateMultibody_TalosArm));
  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }

  switch (cost_type) {
  case CostModelNoFFTypes::CostModelResidualControlGrav:
    cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_factory.create(activation_type, state->get_nv()),
        boost::make_shared<crocoddyl::ResidualModelControlGrav>(state, nu));
    break;
  default:
    throw_pretty(__FILE__ ": Wrong CostModelTypes::Type given");
    break;
  }
  return cost;
}

#ifdef PINOCCHIO_WITH_HPP_FCL
boost::shared_ptr<crocoddyl::CostModelAbstract>
CostModelFactory::create(CostModelCollisionTypes::Type cost_type,
                         StateModelTypes::Type state_type,
                         std::size_t nu) const {
  StateModelFactory state_factory;
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(
          state_factory.create(state_type));
  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }
  pinocchio::FrameIndex frame_index = state->get_pinocchio()->frames.size() - 1;
  pinocchio::SE3 frame_SE3 = pinocchio::SE3::Random();
  pinocchio::SE3 frame_SE3_obstacle = pinocchio::SE3::Random();
  double alpha = fabs(Eigen::VectorXd::Random(1)[0]);
  double beta = fabs(Eigen::VectorXd::Random(1)[0]);

  boost::shared_ptr<pinocchio::GeometryModel> geometry =
      boost::make_shared<pinocchio::GeometryModel>(pinocchio::GeometryModel());
  pinocchio::GeomIndex ig_frame =
      geometry->addGeometryObject(pinocchio::GeometryObject(
          "frame", frame_index,
          state->get_pinocchio()->frames[frame_index].parent,
          CollisionGeometryPtr(new hpp::fcl::Capsule(0, alpha)), frame_SE3));
  pinocchio::GeomIndex ig_obs =
      geometry->addGeometryObject(pinocchio::GeometryObject(
          "obs", state->get_pinocchio()->getFrameId("universe"),
          state->get_pinocchio()
              ->frames[state->get_pinocchio()->getFrameId("universe")]
              .parent,
          CollisionGeometryPtr(new hpp::fcl::Capsule(0, beta)),
          frame_SE3_obstacle));
  geometry->addCollisionPair(pinocchio::CollisionPair(ig_frame, ig_obs));

  switch (cost_type) {
  case CostModelCollisionTypes::CostModelResidualPairCollision:
    cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, boost::make_shared<crocoddyl::ActivationModelQuad>(3),
        boost::make_shared<crocoddyl::ResidualModelPairCollision>(
            state, nu, geometry, 0,
            state->get_pinocchio()->frames[frame_index].parent));
    break;
  default:
    throw_pretty(__FILE__ ": Wrong CostModelTypes::Type given");
    break;
  }
  return cost;
}
#endif // PINOCCHIO_WITH_HPP_FCL

boost::shared_ptr<crocoddyl::CostModelAbstract>
create_random_cost(StateModelTypes::Type state_type, std::size_t nu) {
  static bool once = true;
  if (once) {
    srand((unsigned)time(NULL));
    once = false;
  }

  CostModelFactory factory;
  CostModelTypes::Type rand_type = static_cast<CostModelTypes::Type>(
      rand() % CostModelTypes::NbCostModelTypes);
  return factory.create(rand_type, state_type,
                        ActivationModelTypes::ActivationModelQuad, nu);
}

} // namespace unittest
} // namespace mim_solvers
