///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the activation model unittests factory
// from the Crocoddyl library This modified version is used for testing purposes
// only Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/activation.cpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "activation.hpp"

#include "../../random_generator.hpp"
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic.hpp>
#include <crocoddyl/core/utils/exception.hpp>

namespace mim_solvers {
namespace unittest {

const std::vector<ActivationModelTypes::Type>
    ActivationModelTypes::all(ActivationModelTypes::init_all());

std::ostream &operator<<(std::ostream &os, ActivationModelTypes::Type type) {
  switch (type) {
  case ActivationModelTypes::ActivationModelQuad:
    os << "ActivationModelQuad";
    break;
  case ActivationModelTypes::ActivationModelQuadraticBarrier:
    os << "ActivationModelQuadraticBarrier";
    break;
  case ActivationModelTypes::NbActivationModelTypes:
    os << "NbActivationModelTypes";
    break;
  default:
    break;
  }
  return os;
}

ActivationModelFactory::ActivationModelFactory() {}
ActivationModelFactory::~ActivationModelFactory() {}

boost::shared_ptr<crocoddyl::ActivationModelAbstract>
ActivationModelFactory::create(ActivationModelTypes::Type activation_type,
                               std::size_t nr) const {
  boost::shared_ptr<crocoddyl::ActivationModelAbstract> activation;
  Eigen::VectorXd lb = Eigen::VectorXd::Random(nr);
  Eigen::VectorXd ub =
      lb + Eigen::VectorXd::Ones(nr) + Eigen::VectorXd::Random(nr);

  switch (activation_type) {
  case ActivationModelTypes::ActivationModelQuad:
    activation = boost::make_shared<crocoddyl::ActivationModelQuad>(nr);
    break;
  case ActivationModelTypes::ActivationModelQuadraticBarrier:
    activation = boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
        crocoddyl::ActivationBounds(lb, ub));
    break;
  default:
    throw_pretty(__FILE__ ":\n Construct wrong ActivationModelTypes::Type");
    break;
  }
  return activation;
}

} // namespace unittest
} // namespace mim_solvers
