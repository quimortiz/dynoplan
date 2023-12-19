///////////////////////////////////////////////////////////////////////////////
//
// This file is a modified version of the solvers unittests from the Crocoddyl
// library This modified version is used to test our solvers (mim_solvers
// repository) Original file :
// https://github.com/loco-3d/crocoddyl/blob/devel/unittest/factory/solver.cpp
//
// BSD 3-Clause License
// Copyright (C) 2023, New York University
//
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "solver.hpp"

#include "mim_solvers/csqp.hpp"
#include "mim_solvers/ddp.hpp"
#include "mim_solvers/fddp.hpp"
#include "mim_solvers/kkt.hpp"
#include "mim_solvers/sqp.hpp"
#ifdef MIM_SOLVERS_WITH_PROXQP
#include "mim_solvers/csqp_proxqp.hpp"
#endif
#include <crocoddyl/core/utils/exception.hpp>

namespace mim_solvers {
namespace unittest {

const std::vector<SolverTypes::Type> SolverTypes::all(SolverTypes::init_all());

std::ostream &operator<<(std::ostream &os, SolverTypes::Type type) {
  switch (type) {
  case SolverTypes::SolverKKT:
    os << "SolverKKT";
    break;
  case SolverTypes::SolverDDP:
    os << "SolverDDP";
    break;
  case SolverTypes::SolverFDDP:
    os << "SolverFDDP";
    break;
  case SolverTypes::SolverSQP:
    os << "SolverSQP";
    break;
  case SolverTypes::SolverCSQP:
    os << "SolverCSQP";
    break;
#ifdef MIM_SOLVERS_WITH_PROXQP
  case SolverTypes::SolverPROXQP:
    os << "SolverPROXQP";
    break;
#endif
  case SolverTypes::NbSolverTypes:
    os << "NbSolverTypes";
    break;
  default:
    break;
  }
  return os;
}

SolverFactory::SolverFactory() {}

SolverFactory::~SolverFactory() {}

boost::shared_ptr<crocoddyl::SolverAbstract>
SolverFactory::create(SolverTypes::Type solver_type,
                      ActionModelTypes::Type action_type, size_t T) const {
  boost::shared_ptr<crocoddyl::SolverAbstract> solver;
  boost::shared_ptr<crocoddyl::ActionModelAbstract> model =
      ActionModelFactory().create(action_type);
  boost::shared_ptr<crocoddyl::ActionModelAbstract> model2 =
      ActionModelFactory().create(action_type, true);
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
  const size_t halfway = T / 2;
  for (size_t i = 0; i < halfway; ++i) {
    running_models.push_back(model);
  }
  for (size_t i = 0; i < T - halfway; ++i) {
    running_models.push_back(model2);
  }

  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(model->get_state()->zero(),
                                                     running_models, model);

  switch (solver_type) {
  case SolverTypes::SolverKKT:
    solver = boost::make_shared<mim_solvers::SolverKKT>(problem);
    break;
  case SolverTypes::SolverDDP:
    solver = boost::make_shared<mim_solvers::SolverDDP>(problem);
    break;
  case SolverTypes::SolverFDDP:
    solver = boost::make_shared<mim_solvers::SolverFDDP>(problem);
    break;
  case SolverTypes::SolverSQP:
    solver = boost::make_shared<mim_solvers::SolverSQP>(problem);
    break;
  case SolverTypes::SolverCSQP:
    solver = boost::make_shared<mim_solvers::SolverCSQP>(problem);
    break;
#ifdef MIM_SOLVERS_WITH_PROXQP
  case SolverTypes::SolverPROXQP:
    solver = boost::make_shared<mim_solvers::SolverPROXQP>(problem);
    break;
#endif
  default:
    throw_pretty(__FILE__ ": Wrong SolverTypes::Type given");
    break;
  }
  return solver;
}

} // namespace unittest
} // namespace mim_solvers
