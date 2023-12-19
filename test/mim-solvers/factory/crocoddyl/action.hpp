///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the action model unittests factory from
// the Crocoddyl library This modified version is used for testing purposes only
// Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/action.cpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MIM_SOLVERS_ACTION_FACTORY_HPP_
#define MIM_SOLVERS_ACTION_FACTORY_HPP_

#include <iterator>

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/numdiff/action.hpp"
#include "crocoddyl/multibody/actions/impulse-fwddyn.hpp"
#include "state.hpp"

namespace mim_solvers {
namespace unittest {

struct ActionModelTypes {
  enum Type {
    ActionModelUnicycle,
    ActionModelLQRDriftFree,
    ActionModelLQR,
    ActionModelImpulseFwdDynamics_HyQ,
    ActionModelImpulseFwdDynamics_Talos,
    NbActionModelTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.reserve(NbActionModelTypes);
    for (int i = 0; i < NbActionModelTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

std::ostream &operator<<(std::ostream &os, ActionModelTypes::Type type);

class ActionModelFactory {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ActionModelFactory();
  ~ActionModelFactory();

  boost::shared_ptr<crocoddyl::ActionModelAbstract>
  create(ActionModelTypes::Type type, bool secondInstance = false) const;

  boost::shared_ptr<crocoddyl::ActionModelImpulseFwdDynamics>
  create_impulseFwdDynamics(StateModelTypes::Type state_type) const;
};

} // namespace unittest
} // namespace mim_solvers

#endif // MIM_SOLVERS_ACTION_FACTORY_HPP_
