///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the state model unittests factory from the
// Crocoddyl library This modified version is used for testing purposes only
// Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/state.hpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MIM_SOLVERS_STATE_FACTORY_HPP_
#define MIM_SOLVERS_STATE_FACTORY_HPP_

#include <crocoddyl/core/numdiff/state.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>

#include "pinocchio_model.hpp"

namespace mim_solvers {
namespace unittest {

struct StateModelTypes {
  enum Type {
    StateVector,
    StateMultibody_Hector,
    StateMultibody_TalosArm,
    StateMultibody_HyQ,
    StateMultibody_Talos,
    StateMultibody_RandomHumanoid,
    NbStateModelTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.clear();
    for (int i = 0; i < NbStateModelTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

std::ostream &operator<<(std::ostream &os, StateModelTypes::Type type);

class StateModelFactory {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit StateModelFactory();
  ~StateModelFactory();

  boost::shared_ptr<crocoddyl::StateAbstract>
  create(StateModelTypes::Type state_type) const;
};

} // namespace unittest
} // namespace mim_solvers

#endif // MIM_SOLVERS_STATE_FACTORY_HPP_
