#include <memory>
#include <algorithm>
#include <limits>

#include "nigh/kdtree_batch.hpp"
#include "nigh/kdtree_median.hpp"
#include "nigh/lp_space.hpp"
#include "nigh/so3_space.hpp"
#include <nigh/cartesian_space.hpp>
#include <nigh/scaled_space.hpp>
// OMPL
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/MagicConstants.h>
// dynoplan
#include "dynoplan/ompl/fclHelper.hpp"
#include "dynoplan/ompl/robots.h"
// dynobench
#include "dynobench/dyno_macros.hpp"
#include "dynobench/acrobot.hpp"
#include "dynobench/car.hpp"
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include "dynobench/quadrotor.hpp"
#include "dynobench/unicycle1.hpp"
#include "dynobench/unicycle2.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynoplan {

namespace ob = ompl::base;
namespace oc = ompl::control;

std::ostream &printAction(std::ostream &stream,
                          std::shared_ptr<ompl::control::SpaceInformation> si,
                          oc::Control *action) {
  const size_t dim = si->getControlSpace()->getDimension();
  stream << "[";
  for (size_t d = 0; d < dim; ++d) {
    double *address = si->getControlSpace()->getValueAddressAtIndex(action, d);
    stream << *address;
    if (d < dim - 1) {
      stream << ",";
    }
  }
  stream << "]";
  return stream;
}

std::ostream &printState(std::ostream &stream, const std::vector<double> &x) {
  // continue here
  stream << "[";
  for (size_t d = 0; d < x.size(); ++d) {
    stream << x[d];
    if (d < x.size() - 1) {
      stream << ",";
    }
  }
  stream << "]";
  return stream;
}

std::ostream &printState(std::ostream &stream,
                         std::shared_ptr<ompl::control::SpaceInformation> si,
                         const ob::State *state, bool add_brackets_comma) {
  std::vector<double> reals;
  si->printState(state, std::cout);
  si->getStateSpace()->copyToReals(reals, state);
  if (add_brackets_comma)
    stream << "[";
  for (size_t d = 0; d < reals.size(); ++d) {
    stream << reals[d];
    if (d < reals.size() - 1) {
      if (add_brackets_comma)
        stream << ",";
      else
        stream << " ";
    }
  }
  if (add_brackets_comma)
    stream << "]";
  return stream;
}

double clamp(double val, double min, double max) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

struct ControlSamplerMixer : public oc::ControlSampler {
  std::shared_ptr<dynobench::Model_quad3d> model;
  ControlSamplerMixer(const std::shared_ptr<dynobench::Model_quad3d> &model,
                      const oc::ControlSpace *space, double mean, double stddev)
      : oc::ControlSampler(space), model(model), mean_(mean), stddev_(stddev) {}

  void sample(oc::Control *control) override {
    const unsigned int dim = space_->getDimension();
    const ob::RealVectorBounds &bounds =
        static_cast<const oc::RealVectorControlSpace *>(space_)->getBounds();

    auto *rcontrol =
        static_cast<oc::RealVectorControlSpace::ControlType *>(control);

    double max_T = model->g * model->m * model->u_ub(0);
    double min_T = .5 * max_T;
    double max_M = 1e-5;

    using V4 = Eigen::Vector4d;

    Eigen::Vector4d TM =
        V4(min_T, -max_M, -max_M, -max_M) +
        (V4(max_T, max_M, max_M, max_M) - V4(min_T, -max_M, -max_M, -max_M))
            .cwiseProduct(.5 * (V4::Random(4) + V4::Ones(4)));

    V4 f;
    // CSTR_V(TM);
    model->motorForcesFromThrust(f, TM);
    // CSTR_V(f);
    // CSTR_V((model->B0 * f * model->u_nominal));

    for (unsigned int i = 0; i < 4; ++i) {
      rcontrol->values[i] = clamp(f(i), bounds.low[i], bounds.high[i]);
    }

  }

protected:
  double mean_;
  double stddev_;
};

class ControlSamplerV : public oc::ControlSampler {
public:
  ControlSamplerV(const oc::ControlSpace *space, const Eigen::VectorXd &mean,
                  const Eigen::VectorXd &std)
      : oc::ControlSampler(space), mean(mean), std(std) {}

  void sample(oc::Control *control) override {
    const unsigned int dim = space_->getDimension();

    const ob::RealVectorBounds &bounds =
        static_cast<const oc::RealVectorControlSpace *>(space_)->getBounds();

    auto *rcontrol =
        static_cast<oc::RealVectorControlSpace::ControlType *>(control);
    for (unsigned int i = 0; i < dim; ++i) {
      // rcontrol->values[i] = rng_.uniformReal(bounds.low[i],
      // bounds.high[i]);
      rcontrol->values[i] =
          clamp(rng_.gaussian(mean(i), std(i)), bounds.low[i], bounds.high[i]);
    }
  }

protected:
  Eigen::VectorXd mean;
  Eigen::VectorXd std;
};

class ControlSampler : public oc::ControlSampler {
public:
  ControlSampler(const oc::ControlSpace *space, double mean, double stddev)
      : oc::ControlSampler(space), mean_(mean), stddev_(stddev) {}

  void sample(oc::Control *control) override {
    const unsigned int dim = space_->getDimension();
    const ob::RealVectorBounds &bounds =
        static_cast<const oc::RealVectorControlSpace *>(space_)->getBounds();

    auto *rcontrol =
        static_cast<oc::RealVectorControlSpace::ControlType *>(control);
    for (unsigned int i = 0; i < dim; ++i) {
      // rcontrol->values[i] = rng_.uniformReal(bounds.low[i],
      // bounds.high[i]);
      rcontrol->values[i] =
          clamp(rng_.gaussian(mean_, stddev_), bounds.low[i], bounds.high[i]);
    }
  }

protected:
  double mean_;
  double stddev_;
};

void state_to_stream(std::ostream &out,
                     std::shared_ptr<ompl::control::SpaceInformation> si,
                     const ob::State *state) {

  std::vector<double> reals;
  si->getStateSpace()->copyToReals(reals, state);
  out << "[";
  for (size_t i = 0; i < reals.size(); ++i) {
    out << reals[i];
    if (i < reals.size() - 1) {
      out << ",";
    }
  }
  out << "]" << std::endl;
}

void state_to_eigen(Eigen::VectorXd &out,
                    std::shared_ptr<ompl::control::SpaceInformation> si,
                    const ob::State *state) {

  std::vector<double> reals;
  si->getStateSpace()->copyToReals(reals, state);
  out = Eigen::VectorXd::Map(reals.data(), reals.size());
}

void control_to_eigen(Eigen::VectorXd &out,
                      std::shared_ptr<ompl::control::SpaceInformation> si,
                      ompl::control::Control *control) {

  std::vector<double> reals;
  copyToRealsControl(si, control, reals);
  out = Eigen::VectorXd::Map(reals.data(), reals.size());
}

void control_from_eigen(const Eigen::VectorXd &out,
                        std::shared_ptr<ompl::control::SpaceInformation> si,
                        ompl::control::Control *control) {

  std::vector<double> reals(out.data(), out.data() + out.size());
  copyFromRealsControl(si, control, reals);
}

ob::State *
_allocAndFillState(std::shared_ptr<ompl::control::SpaceInformation> si,
                   const std::vector<double> &reals) {
  ob::State *state = si->allocState();
  si->getStateSpace()->copyFromReals(state, reals);
  return state;
}

ob::State *
allocAndFillState(std::shared_ptr<ompl::control::SpaceInformation> si,
                  const YAML::Node &node) {
  ob::State *state = si->allocState();
  std::vector<double> reals;
  for (const auto &value : node) {
    reals.push_back(value.as<double>());
  }
  si->getStateSpace()->copyFromReals(state, reals);
  return state;
}

// void ompl::base::StateSpace::copyFromReals(
//     State *destination, const std::vector<double> &reals) const {
//   const auto &locations = getValueLocations();
//   assert(reals.size() == locations.size());
//   for (std::size_t i = 0; i < reals.size(); ++i)
//     *getValueAddressAtLocation(destination, locations[i]) = reals[i];
// }

void copyFromRealsControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                          oc::Control *out, const std::vector<double> &reals) {
  for (size_t idx = 0; idx < reals.size(); ++idx) {
    double *address = si->getControlSpace()->getValueAddressAtIndex(out, idx);
    if (address) {
      *address = reals[idx];
    }
  }
}

void copyToRealsControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                        oc::Control *action, std::vector<double> &reals) {

  const size_t dim = si->getControlSpace()->getDimension();
  reals.resize(dim);
  for (size_t d = 0; d < dim; ++d) {
    reals[d] = *si->getControlSpace()->getValueAddressAtIndex(action, d);
  }
}

oc::Control *
allocAndFillControl(std::shared_ptr<ompl::control::SpaceInformation> si,
                    const YAML::Node &node) {
  oc::Control *control = si->allocControl();
  for (size_t idx = 0; idx < node.size(); ++idx) {
    double *address =
        si->getControlSpace()->getValueAddressAtIndex(control, idx);
    if (address) {
      *address = node[idx].as<double>();
    }
  }
  return control;
}

double distance_angle(double a, double b) {
  double result = b - a;
  if (result > M_PI)
    result -= 2 * M_PI;
  else if (result < -M_PI)
    result += 2 * M_PI;
  return result;
}

RobotOmpl::RobotOmpl(std::shared_ptr<dynobench::Model_robot> diff_model)
    : diff_model(diff_model), nx(diff_model->nx), nx_pr(diff_model->nx_pr),
      nu(diff_model->nu) {

  xx.resize(nx); // data
  zz.resize(nx); // data
  yy.resize(nx); // data
  uu.resize(nu); // data

  xx.setZero(); // data
  zz.setZero(); // data
  yy.setZero(); // data
  uu.setZero(); // data
}

std::string RobotOmpl::getName() const { return diff_model->name; };

double RobotOmpl::dt() const { return diff_model->ref_dt; }

bool RobotOmpl::is2D() const { return diff_model->is_2d; }

void RobotOmpl::geometric_interpolation(const ompl::base::State *from,
                                        const ompl::base::State *to, double t,
                                        ompl::base::State *out) {
  toEigen(from, xx);
  toEigen(to, yy);
  diff_model->interpolate(zz, xx, yy, t);
  fromEigen(out, zz);
}

void RobotOmpl::propagate(const ompl::base::State *start,
                          const ompl::control::Control *control,
                          const double duration, ompl::base::State *result) {

  toEigen(start, xx);
  toEigenU(control, uu);

  // use simple Euler integration
  double remaining_time = duration;

  // std::cout << "propagating" << std::endl;
  while (remaining_time > 0.) {
    double dt = std::min(remaining_time, diff_model->ref_dt);
    diff_model->step(yy, xx, uu, dt);
    // CSTR_V(uu);
    // CSTR_V(xx);
    // CSTR_V(yy);
    // CSTR_(dt)
    xx = yy;
    remaining_time -= dt;
  }
  // std::cout << "out ";
  // CSTR_V(xx);
  fromEigen(result, xx);
  enforceBounds(result);
}

double RobotOmpl::cost_lower_bound(const ompl::base::State *x,
                                   const ompl::base::State *y) {
  toEigen(x, xx);
  toEigen(y, yy);
  return diff_model->lower_bound_time(xx, yy);
}

double RobotOmpl::cost_lower_bound_pr(const ompl::base::State *x,
                                      const ompl::base::State *y) {
  toEigen(x, xx);
  toEigen(y, yy);
  return diff_model->lower_bound_time_pr(xx, yy);
}

double RobotOmpl::cost_lower_bound_vel(const ompl::base::State *x,
                                       const ompl::base::State *y) {
  toEigen(x, xx);
  toEigen(y, yy);
  return diff_model->lower_bound_time_vel(xx, yy);
}

class RobotUnicycleFirstOrder : public RobotOmpl {
  using StateSpace = ob::SE2StateSpace;

public:
  virtual ~RobotUnicycleFirstOrder() {}

  RobotUnicycleFirstOrder(
      std::shared_ptr<dynobench::Model_unicycle1> diff_model)
      : RobotOmpl(diff_model) {

    auto space(std::make_shared<StateSpace>());
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    ob::RealVectorBounds cbounds(2);
    ob::RealVectorBounds position_bounds(2);

    for (size_t i = 0; i < 2; i++) {
      cbounds.setLow(i, diff_model->u_lb(i));
      cbounds.setHigh(i, diff_model->u_ub(i));
    }

    for (size_t i = 0; i < 2; i++) {
      position_bounds.setLow(i, diff_model->x_lb(i));
      position_bounds.setHigh(i, diff_model->x_ub(i));
    }

    space->setBounds(position_bounds);
    cspace->setBounds(cbounds);
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);
  }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {

    DYNO_CHECK_EQ(x_eigen.size(), 3, AT);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getX();
    x_eigen(1) = startTyped->getY();
    x_eigen(2) = startTyped->getYaw();
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {

    assert(u_eigen.size() == 2);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;

    u_eigen(0) = ctrl[0];
    u_eigen(1) = ctrl[1];
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {

    assert(x_eigen.size() == 3);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setX(x_eigen(0));
    x_typed->setY(x_eigen(1));
    x_typed->setYaw(x_eigen(2));
  }

  virtual void enforceBounds(ompl::base::State *x) const override {
    enforce_so2(x);
  }

  void enforce_so2(ompl::base::State *x) const {
    auto x_typed = x->as<StateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(1));
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t /*part*/) override {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(
        fcl::Vector3d(stateTyped->getX(), stateTyped->getY(), 0));
    double yaw = stateTyped->getYaw();
    result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return result;
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    auto stateTyped = state->as<StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

  // virtual void copyVelocity(ompl::base::State *state,
  //                           const ompl::base::State *ref) override {
  //
  //
  //
  // }
  //
  //
  //
  //                          const fcl::Vector3d position) override {
  //   auto stateTyped = state->as<StateSpace::StateType>();
  //   stateTyped->setX(position(0));
  //   stateTyped->setY(position(1));
  // }
};

class RobotUnicycleSecondOrder : public RobotOmpl {

public:
  virtual ~RobotUnicycleSecondOrder() {}

  RobotUnicycleSecondOrder(std::shared_ptr<dynobench::Model_unicycle2> model)
      : RobotOmpl(model) {

    auto space(std::make_shared<StateSpace>(model->distance_weights));
    ob::RealVectorBounds position_bounds(2);

    for (size_t i = 0; i < 2; i++) {
      position_bounds.setLow(i, diff_model->x_lb(i));
      position_bounds.setHigh(i, diff_model->x_ub(i));
    }

    space->setPositionBounds(position_bounds);

    ob::RealVectorBounds vel_bounds(1);
    vel_bounds.setLow(diff_model->x_lb(3));
    vel_bounds.setHigh(diff_model->x_ub(3));
    space->setVelocityBounds(vel_bounds);

    ob::RealVectorBounds w_bounds(1);
    w_bounds.setLow(diff_model->x_lb(4));
    w_bounds.setHigh(diff_model->x_ub(4));
    space->setAngularVelocityBounds(w_bounds);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    ob::RealVectorBounds cbounds(2);

    for (size_t i = 0; i < 2; i++) {
      cbounds.setLow(i, diff_model->u_lb(i));
      cbounds.setHigh(i, diff_model->u_ub(i));
    }

    cspace->setBounds(cbounds);

    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t /*part*/) override {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(
        fcl::Vector3d(stateTyped->getX(), stateTyped->getY(), 0));
    double yaw = stateTyped->getYaw();
    result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return result;
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    auto stateTyped = state->as<StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {

    assert(x_eigen.size() == 5);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getX();
    x_eigen(1) = startTyped->getY();
    x_eigen(2) = startTyped->getYaw();
    x_eigen(3) = startTyped->getVelocity();
    x_eigen(4) = startTyped->getAngularVelocity();
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {

    assert(u_eigen.size() == 2);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;

    u_eigen(0) = ctrl[0];
    u_eigen(1) = ctrl[1];
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {

    assert(x_eigen.size() == 5);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setX(x_eigen(0));
    x_typed->setY(x_eigen(1));
    x_typed->setYaw(x_eigen(2));
    x_typed->setVelocity(x_eigen(3));
    x_typed->setAngularVelocity(x_eigen(4));
  }

  virtual void enforceBounds(ompl::base::State *x) const override {
    enforce_so2(x);
  }

  void enforce_so2(ompl::base::State *x) const {
    auto x_typed = x->as<StateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace {
  public:
    class StateType : public ob::CompoundStateSpace::StateType {
    public:
      StateType() = default;

      double getX() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      double getYaw() const {
        return as<ob::SO2StateSpace::StateType>(1)->value;
      }

      double getVelocity() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
      }

      double getAngularVelocity() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[0];
      }

      void setX(double x) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setYaw(double yaw) {
        as<ob::SO2StateSpace::StateType>(1)->value = yaw;
      }

      void setVelocity(double velocity) {
        as<ob::RealVectorStateSpace::StateType>(2)->values[0] = velocity;
      }

      void setAngularVelocity(double angularVelocity) {
        as<ob::RealVectorStateSpace::StateType>(3)->values[0] = angularVelocity;
      }
    };

    StateSpace(Eigen::VectorXd weights) {
      assert(weights.size() == 4);
      setName("CarSO" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                  weights(0)); // position
      addSubspace(std::make_shared<ob::SO2StateSpace>(),
                  weights(1)); // orientation
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1),
                  weights(2)); // velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1),
                  weights(3)); // angular velocity
      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    void setVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(2)->getBounds();
    }

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(3)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getAngularVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(3)->getBounds();
    }

    ob::State *allocState() const override {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override {
      class DefaultProjection : public ob::ProjectionEvaluator {
      public:
        DefaultProjection(const ob::StateSpace *space)
            : ob::ProjectionEvaluator(space) {}

        unsigned int getDimension() const override { return 2; }

        void defaultCellSizes() override {
          cellSizes_.resize(2);
          bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state,
                     Eigen::Ref<Eigen::VectorXd> projection) const override {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<ob::SE2StateSpace::StateType>()
                  ->as<ob::RealVectorStateSpace::StateType>(0)
                  ->values,
              2);
        }
      };

      registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };
};

class Quad2dPole : public RobotOmpl {

public:
  virtual ~Quad2dPole() {}

  Quad2dPole(std::shared_ptr<dynobench::Model_quad2dpole> t_diff_model)
      : RobotOmpl(t_diff_model) {

    CSTR_V(t_diff_model->distance_weights);
    auto space(std::make_shared<StateSpace>(t_diff_model->distance_weights));
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    ob::RealVectorBounds cbounds(2);
    for (size_t i = 0; i < 2; i++) {
      cbounds.setLow(i, diff_model->u_lb(i));
      cbounds.setHigh(i, diff_model->u_ub(i));
    }
    cspace->setBounds(cbounds);

    ob::RealVectorBounds position_bounds(2);
    for (size_t i = 0; i < 2; i++) {
      position_bounds.setLow(i, diff_model->x_lb(i));
      position_bounds.setHigh(i, diff_model->x_ub(i));
    }
    space->setPositionBounds(position_bounds);

    ob::RealVectorBounds vel_bounds(2);
    vel_bounds.setLow(-t_diff_model->params.max_vel);
    vel_bounds.setHigh(t_diff_model->params.max_vel);
    space->setVelocityBounds(vel_bounds);

    ob::RealVectorBounds w_bounds(1);
    w_bounds.setLow(-t_diff_model->params.max_angular_vel);
    w_bounds.setHigh(t_diff_model->params.max_angular_vel);
    space->setAngularVelocityBounds(w_bounds);
    space->setAngularVelocityBoundsQ(w_bounds);

    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    // cspace->setControlSamplerAllocator([](const oc::ControlSpace *space) {
    //   return std::make_shared<ControlSampler>(
    //       space, 1.0, 0.05); // why not uniform in bounds?
    // });
  }

  virtual void setCustomStateSampling() override {

    si_->getControlSpace()->setControlSamplerAllocator(
        [](const oc::ControlSpace *space) {
          return std::make_shared<ControlSampler>(
              space, 1.1, 0.2); // why not uniform in bounds?
        });
  }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {

    assert(x_eigen.size() == 8);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getX();
    x_eigen(1) = startTyped->getY();
    x_eigen(2) = startTyped->getYaw();
    x_eigen(3) = startTyped->getQ();
    x_eigen(4) = startTyped->getVx();
    x_eigen(5) = startTyped->getVy();
    x_eigen(6) = startTyped->getAngularVelocity();
    x_eigen(7) = startTyped->getvQ();
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {

    assert(u_eigen.size() == 2);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;
    u_eigen(0) = ctrl[0];
    u_eigen(1) = ctrl[1];
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {

    assert(x_eigen.size() == 8);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setX(x_eigen(0));
    x_typed->setY(x_eigen(1));
    x_typed->setYaw(x_eigen(2));
    x_typed->setQ(x_eigen(3));
    x_typed->setVx(x_eigen(4));
    x_typed->setVy(x_eigen(5));
    x_typed->setAngularVelocity(x_eigen(6));
    x_typed->setvQ(x_eigen(7));
  }

  virtual void enforceBounds(ompl::base::State *x) const override {
    auto x_typed = x->as<StateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(1));
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(2));
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t /*part*/) override {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(
        fcl::Vector3d(stateTyped->getX(), stateTyped->getY(), 0));
    double yaw = stateTyped->getYaw();
    result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return result;
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace {
  public:
    class StateType : public ob::CompoundStateSpace::StateType {
    public:
      StateType() = default;

      double getX() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      double getYaw() const {
        return as<ob::SO2StateSpace::StateType>(1)->value;
      }
      double getQ() const { return as<ob::SO2StateSpace::StateType>(2)->value; }

      double getVx() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[0];
      }

      double getVy() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[1];
      }

      double getAngularVelocity() const {
        return as<ob::RealVectorStateSpace::StateType>(4)->values[0];
      }

      double getvQ() const {
        return as<ob::RealVectorStateSpace::StateType>(5)->values[0];
      }

      void setX(double x) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setYaw(double yaw) {
        as<ob::SO2StateSpace::StateType>(1)->value = yaw;
      }

      void setQ(double q) { as<ob::SO2StateSpace::StateType>(2)->value = q; }

      void setVx(double vx) {
        as<ob::RealVectorStateSpace::StateType>(3)->values[0] = vx;
      }

      void setVy(double vy) {
        as<ob::RealVectorStateSpace::StateType>(3)->values[1] = vy;
      }

      void setAngularVelocity(double angularVelocity) {
        as<ob::RealVectorStateSpace::StateType>(4)->values[0] = angularVelocity;
      }

      void setvQ(double vq) {
        as<ob::RealVectorStateSpace::StateType>(5)->values[0] = vq;
      }
    };

    StateSpace(const Eigen::VectorXd &distance_weights) {
      DYNO_CHECK_EQ(distance_weights.size(), 6, AT);
      setName("Quad2d" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                  distance_weights(0)); // position
      addSubspace(std::make_shared<ob::SO2StateSpace>(),
                  distance_weights(1)); // orientation
      addSubspace(std::make_shared<ob::SO2StateSpace>(),
                  distance_weights(2)); // orientation
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                  distance_weights(3)); // velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1),
                  distance_weights(4)); // angular velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1),
                  distance_weights(5)); // angular velocity

      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    void setVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(3)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(3)->getBounds();
    }

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(4)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getAngularVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(4)->getBounds();
    }

    void setAngularVelocityBoundsQ(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(5)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getAngularVelocityBoundsQ() const {
      return as<ob::RealVectorStateSpace>(5)->getBounds();
    }

    ob::State *allocState() const override {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override {
      class DefaultProjection : public ob::ProjectionEvaluator {
      public:
        DefaultProjection(const ob::StateSpace *space)
            : ob::ProjectionEvaluator(space) {}

        unsigned int getDimension() const override { return 2; }

        void defaultCellSizes() override {
          cellSizes_.resize(2);
          bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state,
                     Eigen::Ref<Eigen::VectorXd> projection) const override {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<ob::SE2StateSpace::StateType>()
                  ->as<ob::RealVectorStateSpace::StateType>(0)
                  ->values,
              2);
        }
      };

      registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };
};

class Quad2d : public RobotOmpl {

public:
  virtual ~Quad2d() {}

  Quad2d(std::shared_ptr<dynobench::Model_quad2d> t_diff_model)
      : RobotOmpl(t_diff_model) {

    CSTR_V(t_diff_model->distance_weights);
    auto space(std::make_shared<StateSpace>(t_diff_model->distance_weights));
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    ob::RealVectorBounds cbounds(2);
    for (size_t i = 0; i < 2; i++) {
      cbounds.setLow(i, diff_model->u_lb(i));
      cbounds.setHigh(i, diff_model->u_ub(i));
    }
    cspace->setBounds(cbounds);

    ob::RealVectorBounds position_bounds(2);
    for (size_t i = 0; i < 2; i++) {
      position_bounds.setLow(i, diff_model->x_lb(i));
      position_bounds.setHigh(i, diff_model->x_ub(i));
    }
    space->setPositionBounds(position_bounds);

    ob::RealVectorBounds vel_bounds(2);
    vel_bounds.setLow(-t_diff_model->params.max_vel);
    vel_bounds.setHigh(t_diff_model->params.max_vel);
    space->setVelocityBounds(vel_bounds);

    ob::RealVectorBounds w_bounds(1);
    w_bounds.setLow(-t_diff_model->params.max_angular_vel);
    w_bounds.setHigh(t_diff_model->params.max_angular_vel);
    space->setAngularVelocityBounds(w_bounds);

    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);
  }

  virtual void setCustomStateSampling() override {

    si_->getControlSpace()->setControlSamplerAllocator(
        [](const oc::ControlSpace *space) {
          return std::make_shared<ControlSampler>(
              space, 1.0, 0.2); // why not uniform in bounds?
        });
  }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {
    assert(x_eigen.size() == 6);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getX();
    x_eigen(1) = startTyped->getY();
    x_eigen(2) = startTyped->getYaw();
    x_eigen(3) = startTyped->getVx();
    x_eigen(4) = startTyped->getVy();
    x_eigen(5) = startTyped->getAngularVelocity();
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {
    assert(u_eigen.size() == 2);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;
    u_eigen(0) = ctrl[0];
    u_eigen(1) = ctrl[1];
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {
    assert(x_eigen.size() == 6);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setX(x_eigen(0));
    x_typed->setY(x_eigen(1));
    x_typed->setYaw(x_eigen(2));
    x_typed->setVx(x_eigen(3));
    x_typed->setVy(x_eigen(4));
    x_typed->setAngularVelocity(x_eigen(5));
  }

  virtual void enforceBounds(ompl::base::State *x) const override {
    auto x_typed = x->as<StateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(1));
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t /*part*/) override {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(
        fcl::Vector3d(stateTyped->getX(), stateTyped->getY(), 0));
    double yaw = stateTyped->getYaw();
    result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return result;
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace {
  public:
    class StateType : public ob::CompoundStateSpace::StateType {
    public:
      StateType() = default;

      double getX() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      double getYaw() const {
        return as<ob::SO2StateSpace::StateType>(1)->value;
      }

      double getVx() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
      }

      double getVy() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[1];
      }

      double getAngularVelocity() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[0];
      }

      void setX(double x) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setYaw(double yaw) {
        as<ob::SO2StateSpace::StateType>(1)->value = yaw;
      }

      void setVx(double vx) {
        as<ob::RealVectorStateSpace::StateType>(2)->values[0] = vx;
      }

      void setVy(double vy) {
        as<ob::RealVectorStateSpace::StateType>(2)->values[1] = vy;
      }

      void setAngularVelocity(double angularVelocity) {
        as<ob::RealVectorStateSpace::StateType>(3)->values[0] = angularVelocity;
      }
    };

    StateSpace(const Eigen::VectorXd &distance_weights) {
      assert(distance_weights.size() == 4);
      setName("Quad2d" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                  distance_weights(0)); // position
      addSubspace(std::make_shared<ob::SO2StateSpace>(),
                  distance_weights(1)); // orientation

      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                  distance_weights(2)); // velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1),
                  distance_weights(3)); // angular velocity

      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    void setVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(2)->getBounds();
    }

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(3)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getAngularVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(3)->getBounds();
    }

    ob::State *allocState() const override {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override {
      class DefaultProjection : public ob::ProjectionEvaluator {
      public:
        DefaultProjection(const ob::StateSpace *space)
            : ob::ProjectionEvaluator(space) {}

        unsigned int getDimension() const override { return 2; }

        void defaultCellSizes() override {
          cellSizes_.resize(2);
          bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state,
                     Eigen::Ref<Eigen::VectorXd> projection) const override {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<ob::SE2StateSpace::StateType>()
                  ->as<ob::RealVectorStateSpace::StateType>(0)
                  ->values,
              2);
        }
      };

      registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };
};


class Acrobot : public RobotOmpl {

public:
  virtual ~Acrobot() {}
  Acrobot(std::shared_ptr<dynobench::Model_acrobot> t_diff_model)
      : RobotOmpl(t_diff_model) {

    auto space(std::make_shared<StateSpace>(t_diff_model->distance_weights));
    ob::RealVectorBounds vel_bounds(2);
    vel_bounds.setLow(-t_diff_model->params.max_angular_vel);
    vel_bounds.setHigh(t_diff_model->params.max_angular_vel);
    space->setAngularVelocityBounds(vel_bounds);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 1));
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(0, -t_diff_model->params.max_torque);
    cbounds.setHigh(0, t_diff_model->params.max_torque);
    cspace->setBounds(cbounds);
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);
    translation_invariant_ = false;
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {

    assert(x_eigen.size() == 4);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setQ1(x_eigen(0));
    x_typed->setQ2(x_eigen(1));
    x_typed->setW1(x_eigen(2));
    x_typed->setW2(x_eigen(3));
  }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {

    assert(x_eigen.size() == 4);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getQ1();
    x_eigen(1) = startTyped->getQ2();
    x_eigen(2) = startTyped->getW1();
    x_eigen(3) = startTyped->getW2();
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {

    assert(u_eigen.size() == 1);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;
    u_eigen(0) = ctrl[0];
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t /*part*/) override {
    ERROR_WITH_INFO("not implemented");
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    ERROR_WITH_INFO("not implemented");
  }

  virtual void enforceBounds(ob::State *x) const override {

    auto x_typed = x->as<StateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(0));
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace {
  public:
    class StateType : public ob::CompoundStateSpace::StateType {
    public:
      StateType() = default;

      double getQ1() const {
        return as<ob::SO2StateSpace::StateType>(0)->value;
      }

      double getQ2() const {
        return as<ob::SO2StateSpace::StateType>(1)->value;
      }

      double getW1() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
      }

      double getW2() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[1];
      }

      void setQ1(double yaw) {
        as<ob::SO2StateSpace::StateType>(0)->value = yaw;

        ob::SO2StateSpace SO2;
        SO2.enforceBounds(as<ob::SO2StateSpace::StateType>(0));
      }

      void setQ2(double yaw) {
        as<ob::SO2StateSpace::StateType>(1)->value = yaw;
        ob::SO2StateSpace SO2;
        SO2.enforceBounds(as<ob::SO2StateSpace::StateType>(1));
      }

      void setW1(double angularVelocity) {
        as<ob::RealVectorStateSpace::StateType>(2)->values[0] = angularVelocity;
      }

      void setW2(double angularVelocity) {
        as<ob::RealVectorStateSpace::StateType>(2)->values[1] = angularVelocity;
      }
    };

    StateSpace(Eigen::VectorXd weights) {
      assert(weights.size() == 3);
      setName("Acrobot" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::SO2StateSpace>(), weights(0)); // q1
      addSubspace(std::make_shared<ob::SO2StateSpace>(), weights(1)); // q2
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), weights(2));
      lock();
    }

    ~StateSpace() override = default;

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getAngularVelocityBounds() const {
      return as<ob::RealVectorStateSpace>(2)->getBounds();
    }

    void enforceBounds(ob::State *state) const override {
      auto stateTyped = state->as<StateSpace::StateType>();
      stateTyped->setQ1(stateTyped->getQ1());
      stateTyped->setQ2(stateTyped->getQ2());
    }

    ob::State *allocState() const override {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override {
      // class DefaultProjection : public ob::ProjectionEvaluator {
      // public:
      //   DefaultProjection(const ob::StateSpace *space)
      //       : ob::ProjectionEvaluator(space) {}
      //
      //   unsigned int getDimension() const override {
      //     return 2;
      //   }
      //
      //   void defaultCellSizes() override {
      //     // ERROR_WITH_INFO("not implemented");
      //     cellSizes_.resize(2);
      //     bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
      //     cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
      //                     ompl::magic::PROJECTION_DIMENSION_SPLITS;
      //     cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
      //                     ompl::magic::PROJECTION_DIMENSION_SPLITS;
      //   }
      //
      //   void project(const ob::State *state,
      //                Eigen::Ref<Eigen::VectorXd> projection) const override
      //                {
      //
      //     // ERROR_WITH_INFO("not implemented");
      //     // projection = Eigen::Map<const Eigen::VectorXd>(
      //     //     state->as<ob::SE2StateSpace::StateType>()
      //     //         ->as<ob::RealVectorStateSpace::StateType>(0)
      //     //         ->values,
      //     //     2);
      //   }
      // };
      //
      // registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };
};

////////////////////////////////////////////////////////////////////////////////////////////////

// Missing Robot with Trailers and Quadrotor.

// continue here!!!
class RobotCarFirstOrderWithTrailers : public RobotOmpl {
public:
  virtual ~RobotCarFirstOrderWithTrailers() {}
  Eigen::VectorXd hitch_lengths_;
  size_t num_trailers = 0;
  RobotCarFirstOrderWithTrailers(
      std::shared_ptr<dynobench::Model_car_with_trailers> model)
      : RobotOmpl(model) {

    hitch_lengths_ = model->params.hitch_lengths;
    num_trailers = hitch_lengths_.size();

    ob::RealVectorBounds position_bounds(2);
    for (size_t i = 0; i < 2; i++) {
      position_bounds.setLow(i, diff_model->x_lb(i));
      position_bounds.setHigh(i, diff_model->x_ub(i));
    }

    auto space(std::make_shared<StateSpace>(hitch_lengths_.size()));
    space->setPositionBounds(position_bounds);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    ob::RealVectorBounds cbounds(2);

    for (size_t i = 0; i < 2; i++) {
      cbounds.setLow(i, diff_model->u_lb(i));
      cbounds.setHigh(i, diff_model->u_ub(i));
    }
    cspace->setBounds(cbounds);
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);
  }

  virtual size_t numParts() override { return hitch_lengths_.size() + 1; }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {

    assert(x_eigen.size() == 3 + num_trailers);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getX();
    x_eigen(1) = startTyped->getY();
    x_eigen(2) = startTyped->getTheta(0);
    for (size_t i = 0; i < num_trailers; i++) {
      x_eigen(3 + i) = startTyped->getTheta(i + 1);
    }
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {

    assert(u_eigen.size() == 2);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;
    u_eigen(0) = ctrl[0];
    u_eigen(1) = ctrl[1];
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {

    assert(x_eigen.size() == 3 + num_trailers);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setX(x_eigen(0));
    x_typed->setY(x_eigen(1));
    x_typed->setTheta(0, x_eigen(2));

    for (size_t i = 0; i < num_trailers; i++) {
      x_typed->setTheta(i + 1, x_eigen(3 + i));
    }
  }

  virtual void enforceBounds(ompl::base::State *x) const override {
    enforce_so2(x);
  }

  void enforce_so2(ompl::base::State *x) const {
    auto x_typed = x->as<StateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(1));
    SO2.enforceBounds(x_typed->as<ob::SO2StateSpace::StateType>(2));
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t part) override {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3d result;

    if (part == 0) {
      result = Eigen::Translation<double, 3>(
          fcl::Vector3d(stateTyped->getX(), stateTyped->getY(), 0));
      double yaw = stateTyped->getTheta(0);
      result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    } else if (part == 1) {
      fcl::Vector3d pos0(stateTyped->getX(), stateTyped->getY(), 0);
      double theta1 = stateTyped->getTheta(1);
      fcl::Vector3d delta(cosf(theta1), sinf(theta1), 0);
      result = Eigen::Translation<double, 3>(pos0 - delta * hitch_lengths_[0]);
      result.rotate(Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitZ()));
    } else {
      assert(false);
    }
    return result;
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    auto stateTyped = state->as<StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace {
  public:
    class StateType : public ob::CompoundStateSpace::StateType {
    public:
      StateType() = default;

      double getX() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      // 0 means theta of pulling car
      double getTheta(size_t trailer) const {
        return as<ob::SO2StateSpace::StateType>(1 + trailer)->value;
      }

      void setX(double x) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setTheta(size_t trailer, double yaw) {
        auto s = as<ob::SO2StateSpace::StateType>(1 + trailer);
        s->value = yaw;

        // Normalize orientation
        ob::SO2StateSpace SO2;
        SO2.enforceBounds(s);
      }
    };

    StateSpace(size_t numTrailers) {
      setName("CarWithTrailerSO" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 1;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2),
                  1.0);                                        // position
      addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5); // orientation
      for (size_t i = 0; i < numTrailers; ++i) {
        addSubspace(std::make_shared<ob::SO2StateSpace>(),
                    0.5); // orientation
      }
      lock();
    }

    ~StateSpace() override = default;

    bool satisfiesBounds(const ob::State *state) const override {
      auto stateTyped = state->as<StateSpace::StateType>();
      double th0 = stateTyped->getTheta(0);
      double th1 = stateTyped->getTheta(1);
      double delta = th1 - th0;
      double angular_change = atan2(sin(delta), cos(delta));
      if (fabs(angular_change) > M_PI / 4) {
        return false;
      }
      return ob::CompoundStateSpace::satisfiesBounds(state);
    }

    void enforceBounds(ob::State *state) const override {
      auto stateTyped = state->as<StateSpace::StateType>();
      double th0 = stateTyped->getTheta(0);
      double th1 = stateTyped->getTheta(1);
      double delta = th1 - th0;
      double angular_change = atan2(sin(delta), cos(delta));
      // WHY ??
      if (fabs(angular_change) > M_PI / 4) {
        stateTyped->setTheta(1, th0 + angular_change / fabs(angular_change) *
                                          M_PI / 4);
      }
      ob::CompoundStateSpace::enforceBounds(state);
    }

    void setPositionBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    ob::State *allocState() const override {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override {
      class DefaultProjection : public ob::ProjectionEvaluator {
      public:
        DefaultProjection(const ob::StateSpace *space)
            : ob::ProjectionEvaluator(space) {}

        unsigned int getDimension() const override { return 2; }

        void defaultCellSizes() override {
          cellSizes_.resize(2);
          bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state,
                     Eigen::Ref<Eigen::VectorXd> projection) const override {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<ob::SE2StateSpace::StateType>()
                  ->as<ob::RealVectorStateSpace::StateType>(0)
                  ->values,
              2);
        }
      };

      registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };
};

////////////////////////////////////////////////////////////////////////////////////////////////

// See also
// https://github.com/ompl/omplapp/blob/main/src/omplapp/apps/QuadrotorPlanning.cpp
// In the ompl.app example, the control seems to be force + moments
// rather than raw motor forces
class RobotQuadrotor : public RobotOmpl {

public:
  virtual ~RobotQuadrotor() {}
  RobotQuadrotor(std::shared_ptr<dynobench::Model_quad3d> t_model)
      : RobotOmpl(t_model) {

    auto space(std::make_shared<StateSpace>(t_model->distance_weights));

    ob::RealVectorBounds vbounds(3);
    std::cout << STR_(-t_model->params.max_vel) << std::endl;
    vbounds.setLow(-t_model->params.max_vel);
    vbounds.setHigh(t_model->params.max_vel);
    vbounds.check();
    space->setVelocityBounds(vbounds);

    ob::RealVectorBounds wbounds(3);
    std::cout << STR_(t_model->params.max_angular_vel) << std::endl;
    wbounds.setLow(-t_model->params.max_angular_vel);
    wbounds.setHigh(t_model->params.max_angular_vel);
    wbounds.check();
    space->setAngularVelocityBounds(wbounds);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));

    // cspace->setControlSamplerAllocator([](const oc::ControlSpace *space) {
    //   return std::make_shared<ControlSampler>(
    //       space, 1., 0.01); // why not uniform in bounds?
    // });

    // cspace->setControlSamplerAllocator(
    //     [t_model](const oc::ControlSpace *space) {
    //       return std::make_shared<ControlSamplerMixer>(
    //           t_model, space, 1., 0.01); // why not uniform in bounds?
    //     });

    ob::RealVectorBounds cbounds(4);
    cbounds.setLow(0);
    cbounds.setHigh(t_model->params.max_f);
    cspace->setBounds(cbounds);

    ob::RealVectorBounds position_bounds(3);
    std::cout << STR_V(diff_model->x_lb) << std::endl;
    std::cout << STR_V(diff_model->x_ub) << std::endl;
    for (size_t i = 0; i < 3; i++) {
      position_bounds.setLow(i, diff_model->x_lb(i));
      position_bounds.setHigh(i, diff_model->x_ub(i));
    }
    position_bounds.check();
    space->setPositionBounds(position_bounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);
  }

  virtual void setCustomStateSampling() override {

    // check if it is force control or no.

    auto p_derived =
        std::dynamic_pointer_cast<dynobench::Model_quad3d>(diff_model);

    if (p_derived) {
      if (p_derived->params.motor_control) {
        si_->getControlSpace()->setControlSamplerAllocator(
            [](const oc::ControlSpace *space) {
              return std::make_shared<ControlSampler>(space, 1.0, 0.01);
            });
      } else {
        si_->getControlSpace()->setControlSamplerAllocator(
            [](const oc::ControlSpace *space) {
              using v4 = Eigen::Vector4d;
              return std::make_shared<ControlSamplerV>(
                  space, v4(1.0, 0, 0, 0), v4(.2, 0.05, 0.05, 0.05));
            });
      }
    } else {

      ERROR_WITH_INFO("why not quad3d model?");
    }
  }

  virtual void toEigen(const ompl::base::State *x_ompl,
                       Eigen::Ref<Eigen::VectorXd> x_eigen) override {

    assert(x_eigen.size() == 13);
    auto startTyped = x_ompl->as<StateSpace::StateType>();
    x_eigen(0) = startTyped->getX();
    x_eigen(1) = startTyped->getY();
    x_eigen(2) = startTyped->getZ();
    x_eigen(3) = startTyped->getQx();
    x_eigen(4) = startTyped->getQy();
    x_eigen(5) = startTyped->getQz();
    x_eigen(6) = startTyped->getQw();
    x_eigen(7) = startTyped->getVx();
    x_eigen(8) = startTyped->getVy();
    x_eigen(9) = startTyped->getVz();
    x_eigen(10) = startTyped->getWx();
    x_eigen(11) = startTyped->getWy();
    x_eigen(12) = startTyped->getWz();
  }

  virtual void toEigenU(const ompl::control::Control *control,
                        Eigen::Ref<Eigen::VectorXd> u_eigen) override {

    assert(u_eigen.size() == 4);
    const double *ctrl =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()
            ->values;

    for (size_t i = 0; i < 4; i++) {
      u_eigen(0) = ctrl[0];
      u_eigen(1) = ctrl[1];
      u_eigen(2) = ctrl[2];
      u_eigen(3) = ctrl[3];
    }
  }

  virtual void
  fromEigen(ompl::base::State *x_ompl,
            const Eigen::Ref<const Eigen::VectorXd> &x_eigen) override {

    assert(x_eigen.size() == 13);
    auto x_typed = x_ompl->as<StateSpace::StateType>();

    x_typed->setX(x_eigen(0));
    x_typed->setY(x_eigen(1));
    x_typed->setZ(x_eigen(2));

    x_typed->setQx(x_eigen(3));
    x_typed->setQy(x_eigen(4));
    x_typed->setQz(x_eigen(5));
    x_typed->setQw(x_eigen(6));

    x_typed->velocity()[0] = x_eigen(7);
    x_typed->velocity()[1] = x_eigen(8);
    x_typed->velocity()[2] = x_eigen(9);

    x_typed->angularVelocity()[0] = x_eigen(10);
    x_typed->angularVelocity()[1] = x_eigen(11);
    x_typed->angularVelocity()[2] = x_eigen(12);
  }

  virtual fcl::Transform3d getTransform(const ompl::base::State *state,
                                        size_t /*part*/) override {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3d result;

    result = Eigen::Translation<double, 3>(fcl::Vector3d(
        stateTyped->getX(), stateTyped->getY(), stateTyped->getZ()));
    result.rotate(
        Eigen::Quaterniond(stateTyped->rotation().x, stateTyped->rotation().y,
                           stateTyped->rotation().z, stateTyped->rotation().w));
    return result;
  }

  virtual void setPosition(ompl::base::State *state,
                           const fcl::Vector3d position) override {
    auto stateTyped = state->as<StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
    stateTyped->setZ(position(2));
  }

  virtual void enforceBounds(ompl::base::State *state) const override {
    auto x_typed = state->as<StateSpace::StateType>();
    ob::SO3StateSpace SO3;
    SO3.enforceBounds(x_typed->as<ob::SO3StateSpace::StateType>(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace {
  public:
    class StateType : public ob::CompoundStateSpace::StateType {
    public:
      StateType() = default;

      double getX() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      double getZ() const {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
      }

      double getVx() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
      }

      double getVy() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[1];
      }

      double getVz() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[2];
      }

      double getQx() const { return rotation().x; }
      double getQy() const { return rotation().y; }
      double getQz() const { return rotation().z; }
      double getQw() const { return rotation().w; }

      double getWx() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[0];
      }

      double getWy() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[1];
      }

      double getWz() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[2];
      }

      void setX(double x) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setZ(double z) {
        as<ob::RealVectorStateSpace::StateType>(0)->values[2] = z;
      }

      void setQx(double qx) { rotation().x = qx; }
      void setQy(double qy) { rotation().y = qy; }
      void setQz(double qz) { rotation().z = qz; }
      void setQw(double qw) { rotation().w = qw; }

      const ob::SO3StateSpace::StateType &rotation() const {
        return *as<ob::SO3StateSpace::StateType>(1);
      }

      ob::SO3StateSpace::StateType &rotation() {
        return *as<ob::SO3StateSpace::StateType>(1);
      }

      const double *velocity() const {
        return as<ob::RealVectorStateSpace::StateType>(2)->values;
      }

      double *velocity() {
        return as<ob::RealVectorStateSpace::StateType>(2)->values;
      }

      const double *angularVelocity() const {
        return as<ob::RealVectorStateSpace::StateType>(3)->values;
      }

      double *angularVelocity() {
        return as<ob::RealVectorStateSpace::StateType>(3)->values;
      }
    };

    StateSpace(const Eigen::VectorXd &weights) {
      assert(weights.size() == 4);
      setName("Quadrotor" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 2;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), weights(0));
      addSubspace(std::make_shared<ob::SO3StateSpace>(),
                  weights(1)); // orientation
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(3),
                  weights(2)); // velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(3),
                  weights(3)); // angular velocity
      lock();
    }

    ~StateSpace() override = default;

    // bool satisfiesBounds(const ob::State *state) const override
    // {
    //   bool result = ob::CompoundStateSpace::satisfiesBounds(state);
    //   if (!result) {
    //     return false;
    //   }

    //   auto stateTyped = state->as<StateType>();
    //   Eigen::Vector3f up(0,0,1);
    //   Eigen::Quaternionf q(stateTyped->rotation().w,
    //   stateTyped->rotation().x, stateTyped->rotation().y,
    //   stateTyped->rotation().z); const auto& up_transformed =
    //   q._transformVector(up); float angle = acosf(up.dot(up_transformed));
    //   return fabs(angle) < M_PI / 6;
    // }

    void setPositionBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    void setVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    }

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds) {
      as<ob::RealVectorStateSpace>(3)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    ob::State *allocState() const override {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() {
      class SE3DefaultProjection : public ob::ProjectionEvaluator {
      public:
        SE3DefaultProjection(const StateSpace *space)
            : ob::ProjectionEvaluator(space) {}

        unsigned int getDimension() const override { return 3; }

        void defaultCellSizes() override {
          cellSizes_.resize(3);
          bounds_ = space_->as<StateSpace>()->getPositionBounds();

          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) /
                          ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state,
                     Eigen::Ref<Eigen::VectorXd> projection) const override {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<StateSpace::StateType>()
                  ->as<ob::RealVectorStateSpace::StateType>(0)
                  ->values,
              3);
        }
      };

      registerDefaultProjection(std::make_shared<SE3DefaultProjection>(this));
    }
  };

protected:
};

RobotStateValidityChecker::RobotStateValidityChecker(
    std::shared_ptr<RobotOmpl> robot)
    : ompl::base::StateValidityChecker(robot->getSpaceInformation()),
      robot(robot) {
  x_eigen.resize(robot->nx);
}

bool RobotStateValidityChecker::isValid(const ompl::base::State *state) const {
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  robot->toEigen(state, x_eigen);
  return robot->diff_model->collision_check(x_eigen);
}

std::shared_ptr<RobotOmpl>
robot_factory_ompl(const dynobench::Problem &problem) {

  using namespace dynobench;

  // std::string base_path = "../models/";
  std::string suffix = ".yaml";
  std::string robot_model_file =
      problem.models_base_path + problem.robotType + suffix;

  std::shared_ptr<RobotOmpl> out;

  std::cout << "Robot Factory: loading file: " << robot_model_file << std::endl;

  std::cout << "Robot Factory: loading file: " << robot_model_file << std::endl;

  if (!std::filesystem::exists(robot_model_file)) {
    ERROR_WITH_INFO(
        (std::string("file: ") + robot_model_file + " not found: ").c_str());
  }

  YAML::Node node = YAML::LoadFile(robot_model_file);

  assert(node["dynamics"]);
  std::string dynamics = node["dynamics"].as<std::string>();
  std::cout << STR_(dynamics) << std::endl;

  const char *__robot_model_file = robot_model_file.c_str();

  auto &p_lb = problem.p_lb;
  auto &p_ub = problem.p_ub;
  if (dynamics == "unicycle1") {
    out = std::make_shared<RobotUnicycleFirstOrder>(
        std::make_shared<Model_unicycle1>(__robot_model_file, p_lb, p_ub));
  } else if (dynamics == "unicycle2") {
    out = std::make_shared<RobotUnicycleSecondOrder>(
        std::make_shared<Model_unicycle2>(__robot_model_file, p_lb, p_ub));
  } else if (dynamics == "quad2dpole") {
    out = std::make_shared<Quad2dPole>(
        std::make_shared<Model_quad2dpole>(__robot_model_file, p_lb, p_ub));
  } else if (dynamics == "quad2d") {
    out = std::make_shared<Quad2d>(
        std::make_shared<Model_quad2d>(__robot_model_file, p_lb, p_ub));
  } else if (dynamics == "quad3d") {
    out = std::make_shared<RobotQuadrotor>(
        std::make_shared<Model_quad3d>(__robot_model_file, p_lb, p_ub));
  } else if (dynamics == "acrobot") {
    out = std::make_shared<Acrobot>(
        std::make_shared<Model_acrobot>(__robot_model_file, p_lb, p_ub));
  } else if (dynamics == "car_with_trailers") {
    out = std::make_shared<RobotCarFirstOrderWithTrailers>(
        std::make_shared<Model_car_with_trailers>(__robot_model_file, p_lb,
                                                  p_ub));
  } else {
    ERROR_WITH_INFO("dynamics not implemented");
  }

  load_env(*out->diff_model, problem);

  auto si = out->getSpaceInformation();

  double step_size = 1.;
  double min_control_duration = 1.;
  double max_control_duration = 1.;

  std::cout << "WARNING, HARDCODED" << std::endl;
  CSTR_(step_size);
  CSTR_(min_control_duration);
  CSTR_(max_control_duration);

  si->setPropagationStepSize(step_size);
  si->setMinMaxControlDuration(min_control_duration, max_control_duration);

  si->setStateValidityChecker(std::make_shared<RobotStateValidityChecker>(out));

  std::shared_ptr<oc::StatePropagator> statePropagator(
      new RobotOmplStatePropagator(si, out));
  si->setStatePropagator(statePropagator);
  si->setup();

  // add start and goal

  std::vector<double> start_(problem.start.data(),
                             problem.start.data() + problem.start.size());
  std::vector<double> goal_(problem.goal.data(),
                            problem.goal.data() + problem.goal.size());

  out->startState = _allocAndFillState(si, start_);
  out->goalState = _allocAndFillState(si, goal_);

  out->enforceBounds(out->goalState);
  out->enforceBounds(out->startState);

  return out;
}

void load_motion_primitives_new(const std::string &motionsFile,
                                dynobench::Model_robot &robot,
                                std::vector<Motion> &motions, int max_motions,
                                bool cut_actions, bool shuffle,
                                bool compute_col,
                                std::optional<unsigned int> seed) {

  dynobench::Trajectories trajs;
  MotionPrimitiveFormat format;
  std::filesystem::path filePath = motionsFile;
  if (filePath.extension() == ".yaml") {
    format = MotionPrimitiveFormat::YAML;
  } else if (filePath.extension() == ".json")
    format = MotionPrimitiveFormat::JSON;
  else if (filePath.extension() == ".msgpack")
    format = MotionPrimitiveFormat::MSGPACK;
  else if (filePath.extension() == ".bin")
    format = MotionPrimitiveFormat::BOOST;

  switch (format) {
  case MotionPrimitiveFormat::YAML: {
    trajs.load_file_yaml(motionsFile.c_str());
  } break;

  case MotionPrimitiveFormat::BOOST: {
    trajs.load_file_boost(motionsFile.c_str());
  } break;

  case MotionPrimitiveFormat::JSON: {
    trajs.load_file_json(motionsFile.c_str());
  } break;

  case MotionPrimitiveFormat::MSGPACK: {
    trajs.load_file_msgpack(motionsFile.c_str());
  } break;
  case MotionPrimitiveFormat::AUTO: {
    ERROR_WITH_INFO(
        "Incompatible format for motion primitives: should not be here!");
  }
  }

  if (max_motions < trajs.data.size())
    trajs.data.resize(max_motions);

  std::cout << "trajs " << std::endl;
  std::cout << "first state is " << std::endl;
  CSTR_V(trajs.data.front().states.front());

  motions.resize(trajs.data.size());

  // bool add_noise_first_state = true;
  // Deterministic
  bool add_noise_first_state = false;
  CSTR_(add_noise_first_state);

  if (add_noise_first_state) {
    std::cout << "WARNING:"
              << "adding noise to first and last state" << std::endl;
    const double noise = 1e-7;
    for (auto &t : trajs.data) {
      t.states.front() +=
          noise * Eigen::VectorXd::Random(t.states.front().size());

      t.states.back() +=
          noise * Eigen::VectorXd::Random(t.states.back().size());
    }
  }

  if (startsWith(robot.name, "quad3d")) {
    // ensure quaternion
    for (auto &t : trajs.data) {
      for (auto &s : t.states) {
        s.segment<4>(3).normalize();
      }
    }
  }

  CSTR_(trajs.data.size());
  std::cout << "from boost to motion " << std::endl;

  std::transform(trajs.data.begin(), trajs.data.end(), motions.begin(),
                 [&](const auto &traj) {
                   // traj.to_yaml_format(std::cout, "");
                   Motion m;
                   traj_to_motion(traj, robot, m, compute_col);
                   return m;
                 });

  std::cout << "from boost to motion -- DONE " << std::endl;

  CHECK(motions.size(), AT);
  if (motions.front().cost > 1e5) {
    std::cout << "WARNING: motions have infinite cost." << std::endl;
    std::cout << "-- using default cost: TIME" << std::endl;
    for (auto &m : motions) {
      m.cost = robot.ref_dt * m.actions.size();
    }
  }

  if (cut_actions) {
    NOT_IMPLEMENTED;
  }

  if (shuffle) {
    std::mt19937 rng(seed ? *seed : std::random_device{}());
    std::shuffle(motions.begin(), motions.end(), rng);
  }

  for (size_t idx = 0; idx < motions.size(); ++idx) {
    motions[idx].idx = idx;
  }
}

void traj_to_motion(const dynobench::Trajectory &traj,
                    dynobench::Model_robot &robot, Motion &motion_out,
                    bool compute_col,
                    bool merged) {

  motion_out.states.resize(traj.states.size());
  motion_out.traj = traj;

  motion_out.actions.resize(traj.actions.size());

  if (compute_col){
    if(merged)
      compute_col_shape_merged(motion_out, robot);
    else
      compute_col_shape(motion_out, robot);
  }
  motion_out.cost = traj.cost;
}

// add N motions to motion_out. It starts adding from [current_size,....+N]
void motion_to_motion(std::vector<Motion> &robot_motions,
                      std::vector<Motion> &motion_out,
                      dynobench::Model_robot &robot, size_t desired_size) {

  // std::cout << "before adding " << motion_out.size() << std::endl;
  size_t N = std::min(desired_size, robot_motions.size());
  std::cout << "N inside motion-to-motion: " << N << std::endl;
  for (size_t i = motion_out.size(); i < N; i++) {
    dynoplan::Motion m;
    m.traj = robot_motions.at(i).traj;
    m.states.resize(m.traj.states.size());
    m.actions.resize(m.traj.actions.size());
    m.cost = m.traj.cost;
    m.idx = motion_out.size();
    compute_col_shape(m, robot);
    motion_out.push_back(std::move(m));
  }
  // std::cout << "after adding " << motion_out.size() << std::endl;
}

void compute_col_shape(Motion &m, dynobench::Model_robot &robot) {
  for (auto &x : m.traj.states) {

    auto &ts_data = robot.ts_data;
    auto &col_geo = robot.collision_geometries;
    robot.transformation_collision_geometries(x, ts_data);

    for (size_t i = 0; i < ts_data.size(); i++) {
      auto &transform = ts_data.at(i);
      auto co = std::make_unique<fcl::CollisionObjectd>(col_geo.at(i));
      co->setTranslation(transform.translation());
      co->setRotation(transform.rotation());
      co->computeAABB();
      m.collision_objects.push_back(std::move(co));
    }
  }

  std::vector<fcl::CollisionObjectd *> cols_ptrs(m.collision_objects.size());
  std::transform(m.collision_objects.begin(), m.collision_objects.end(),
                 cols_ptrs.begin(), [](auto &ptr) { return ptr.get(); });

  m.collision_manager.reset(
      new ShiftableDynamicAABBTreeCollisionManager<double>());
  // TODO: double check that fcl doesn't take ownership of the objects
  m.collision_manager->registerObjects(cols_ptrs);
};

  void compute_col_shape_merged(Motion & m, dynobench::Model_robot & robot)
  {
    std::vector<std::unique_ptr<fcl::CollisionObjectd>> collision_objects_tmp;
    for (auto &x : m.traj.states)
    {

      auto &ts_data = robot.ts_data;
      auto &col_geo = robot.collision_geometries;
      robot.transformation_collision_geometries(x, ts_data);

      for (size_t i = 0; i < ts_data.size(); i++)
      {
        auto &transform = ts_data.at(i);
        auto co = std::make_unique<fcl::CollisionObjectd>(col_geo.at(i));
        co->setTranslation(transform.translation());
        co->setRotation(transform.rotation());
        co->computeAABB();
        collision_objects_tmp.push_back(std::move(co));
      }
    }
    // get the merged one
    fcl::AABB<double> aabb_merged;
    for (auto &tmp_co : collision_objects_tmp)
    {
      auto aabb_tmp = tmp_co->getAABB();
      aabb_merged += aabb_tmp;
    }

    fcl::Vector3<double> &max_coords = aabb_merged.max_;
    fcl::Vector3<double> &min_coords = aabb_merged.min_;
    std::shared_ptr<fcl::CollisionGeometryd> tmp_geom = std::make_shared<fcl::Boxd>((max_coords[0] - min_coords[0]),
                                                                                    (max_coords[1] - min_coords[1]),
                                                                                    (max_coords[2] - min_coords[2]));
    auto co = std::make_unique<fcl::CollisionObjectd>(tmp_geom);
    m.collision_objects.push_back(std::move(co));

    std::vector<fcl::CollisionObjectd *> cols_ptrs(m.collision_objects.size());
    std::transform(m.collision_objects.begin(), m.collision_objects.end(),
                   cols_ptrs.begin(), [](auto &ptr)
                   { return ptr.get(); });

    m.collision_manager.reset(
        new ShiftableDynamicAABBTreeCollisionManager<double>());
    m.collision_manager->registerObjects(cols_ptrs);
  };

  void Motion::print(std::ostream & out,
                     std::shared_ptr<ompl::control::SpaceInformation> & si)
  {

    out << "states " << std::endl;
    for (auto &state : states)
    {
      printState(out, si, state);
      out << std::endl;
    }
    out << "actions: " << std::endl;
    for (auto &action : actions)
    {
      printAction(std::cout, si, action);
      out << std::endl;
    }

    STRY(cost, out, "", ": ");
    STRY(idx, out, "", ": ");
    STRY(disabled, out, "", ": ");
  }

} // namespace dynoplan
