///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the cost model unittests factory from the
// Crocoddyl library This modified version is used for testing purposes only
// Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/cost.hpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MIM_SOLVERS_COST_FACTORY_HPP_
#define MIM_SOLVERS_COST_FACTORY_HPP_

#include "activation.hpp"
#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/numdiff/cost.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "state.hpp"

namespace mim_solvers {
namespace unittest {

struct CostModelTypes {
  enum Type {
    CostModelResidualState,
    CostModelResidualControl,
    CostModelResidualFramePlacement,
    NbCostModelTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.reserve(NbCostModelTypes);
    for (int i = 0; i < NbCostModelTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

struct CostModelNoFFTypes {
  enum Type { CostModelResidualControlGrav, NbCostModelNoFFTypes };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.clear();
    for (int i = 0; i < NbCostModelNoFFTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

#ifdef PINOCCHIO_WITH_HPP_FCL
struct CostModelCollisionTypes {
  enum Type { CostModelResidualPairCollision, NbCostModelCollisionTypes };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.clear();
    for (int i = 0; i < NbCostModelCollisionTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};
#endif // PINOCCHIO_WITH_HPP_FCL

std::ostream &operator<<(std::ostream &os, CostModelTypes::Type type);
std::ostream &operator<<(std::ostream &os, CostModelNoFFTypes::Type type);
#ifdef PINOCCHIO_WITH_HPP_FCL
std::ostream &operator<<(std::ostream &os, CostModelCollisionTypes::Type type);
#endif // PINOCCHIO_WITH_HPP_FCL

class CostModelFactory {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef crocoddyl::MathBaseTpl<double> MathBase;
  typedef typename MathBase::Vector6s Vector6d;
  typedef pinocchio::GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;

  explicit CostModelFactory();
  ~CostModelFactory();

  boost::shared_ptr<crocoddyl::CostModelAbstract>
  create(CostModelTypes::Type cost_type, StateModelTypes::Type state_type,
         ActivationModelTypes::Type activation_type,
         std::size_t nu = std::numeric_limits<std::size_t>::max()) const;
  boost::shared_ptr<crocoddyl::CostModelAbstract>
  create(CostModelNoFFTypes::Type cost_type,
         ActivationModelTypes::Type activation_type,
         std::size_t nu = std::numeric_limits<std::size_t>::max()) const;

#ifdef PINOCCHIO_WITH_HPP_FCL
  boost::shared_ptr<crocoddyl::CostModelAbstract>
  create(CostModelCollisionTypes::Type cost_type,
         StateModelTypes::Type state_type,
         std::size_t nu = std::numeric_limits<std::size_t>::max()) const;
#endif // PINOCCHIO_WITH_HPP_FCL
};

boost::shared_ptr<crocoddyl::CostModelAbstract>
create_random_cost(StateModelTypes::Type state_type,
                   std::size_t nu = std::numeric_limits<std::size_t>::max());
} // namespace unittest
} // namespace mim_solvers

#endif // MIM_SOLVERS_COST_FACTORY_HPP_
