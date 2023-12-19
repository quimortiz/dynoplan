///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the activation model unittests factory
// from the Crocoddyl library This modified version is used for testing purposes
// only Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/activation.hpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MIM_SOLVERS_ACTIVATION_FACTORY_HPP_
#define MIM_SOLVERS_ACTIVATION_FACTORY_HPP_

#include "crocoddyl/core/activation-base.hpp"
#include "crocoddyl/core/numdiff/activation.hpp"

namespace mim_solvers {
namespace unittest {

struct ActivationModelTypes {
  enum Type {
    ActivationModelQuad,
    ActivationModelQuadraticBarrier,
    NbActivationModelTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.reserve(NbActivationModelTypes);
    for (int i = 0; i < NbActivationModelTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

std::ostream &operator<<(std::ostream &os, ActivationModelTypes::Type type);

class ActivationModelFactory {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ActivationModelFactory();
  ~ActivationModelFactory();

  boost::shared_ptr<crocoddyl::ActivationModelAbstract>
  create(ActivationModelTypes::Type activation_type, std::size_t nr = 5) const;
};

} // namespace unittest
} // namespace mim_solvers

#endif // MIM_SOLVERS_ACTIVATION_FACTORY_HPP_
