#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>

#include "Eigen/Core"
#include <boost/program_options.hpp>

#include "crocoddyl/core/numdiff/action.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/states/euclidean.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/utils/timer.hpp"

#include "dynobench/acrobot.hpp"
#include "dynobench/car.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/general_utils.hpp"
#include "dynobench/math_utils.hpp"
#include "dynobench/quadrotor.hpp"
#include "dynobench/robot_models.hpp"

namespace dynoplan {

enum class Control_Mode {
  default_mode,
  free_time,
  free_time_linear,
  free_time_linear_first,
  contour
};

enum class CostTYPE {
  linear,
  least_squares,
};

struct ReportCost {
  double cost;
  int time;
  std::string name;
  Eigen::VectorXd r;
  CostTYPE type;
};

void modify_x_bound_for_free_time_linear(const Eigen::VectorXd &__x_lb,
                                         const Eigen::VectorXd &__x_ub,
                                         const Eigen::VectorXd &__xb__weight,
                                         Eigen::VectorXd &x_lb,
                                         Eigen::VectorXd &x_ub,
                                         Eigen::VectorXd &xb_weight);

void modify_u_for_free_time_linear(const Eigen::VectorXd &__u_lb,
                                   const Eigen::VectorXd &__u_ub,
                                   const Eigen::VectorXd &__u__weight,
                                   const Eigen::VectorXd &__u__ref,
                                   Eigen::VectorXd &u_lb, Eigen::VectorXd &u_ub,
                                   Eigen::VectorXd &u_weight,
                                   Eigen::VectorXd &u_ref);

void modify_u_bound_for_contour(const Eigen::VectorXd &__u_lb,
                                const Eigen::VectorXd &__u_ub,
                                const Eigen::VectorXd &__u__weight,
                                const Eigen::VectorXd &__u__ref,
                                Eigen::VectorXd &u_lb, Eigen::VectorXd &u_ub,
                                Eigen::VectorXd &u_weight,
                                Eigen::VectorXd &u_ref);

void modify_u_bound_for_free_time(const Eigen::VectorXd &__u_lb,
                                  const Eigen::VectorXd &__u_ub,
                                  const Eigen::VectorXd &__u__weight,
                                  const Eigen::VectorXd &__u__ref,
                                  Eigen::VectorXd &u_lb, Eigen::VectorXd &u_ub,
                                  Eigen::VectorXd &u_weight,
                                  Eigen::VectorXd &u_ref, 
                                  const std::map<std::string, double> &params);

void modify_x_bound_for_contour(const Eigen::VectorXd &__x_lb,
                                const Eigen::VectorXd &__x_ub,
                                const Eigen::VectorXd &__xb__weight,
                                Eigen::VectorXd &x_lb, Eigen::VectorXd &x_ub,
                                Eigen::VectorXd &xb_weight, double max_alpha);

struct ActionDataDyno : public crocoddyl::ActionDataAbstractTpl<double> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef double Scalar;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef ActionDataAbstractTpl<Scalar> Base;
  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xnext;

  template <template <typename Scalar> class Model>
  explicit ActionDataDyno(Model<Scalar> *const model) : Base(model) {}
};

struct StateCrocoDyno : crocoddyl::StateAbstractTpl<double> {

public:
  typedef double Scalar;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  bool use_pin = true;
  std::shared_ptr<dynobench::StateDyno> state;
  explicit StateCrocoDyno(const std::shared_ptr<dynobench::StateDyno> &state)
      : StateAbstractTpl<Scalar>(state->nx, state->ndx), state(state){};

  virtual ~StateCrocoDyno() {}

  virtual VectorXs zero() const override { return state->zero(); }

  virtual VectorXs rand() const override { return state->rand(); }

  // x1 - x0
  virtual void diff(const Eigen::Ref<const VectorXs> &x0,
                    const Eigen::Ref<const VectorXs> &x1,
                    Eigen::Ref<VectorXs> dxout) const override {

    assert(state);
    state->diff(x0, x1, dxout);
  }

  virtual void integrate(const Eigen::Ref<const VectorXs> &x,
                         const Eigen::Ref<const VectorXs> &dx,
                         Eigen::Ref<VectorXs> xout) const override {
    assert(state);
    state->integrate(x, dx, xout);
  }

  virtual void Jdiff(const Eigen::Ref<const VectorXs> &x0,
                     const Eigen::Ref<const VectorXs> &x1,
                     Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
                     const crocoddyl::Jcomponent firstsecond =
                         crocoddyl::both) const override {
    (void)firstsecond;
    assert(state);
    state->Jdiff(x0, x1, Jfirst, Jsecond);
  }

  virtual void Jintegrate(
      const Eigen::Ref<const VectorXs> &x, const Eigen::Ref<const VectorXs> &dx,
      Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
      const crocoddyl::Jcomponent firstsecond = crocoddyl::both,
      const crocoddyl::AssignmentOp op = crocoddyl::setto) const override {

    (void)firstsecond;
    (void)op;
    assert(state);
    state->Jintegrate(x, dx, Jfirst, Jsecond);
  }
  virtual void
  JintegrateTransport(const Eigen::Ref<const VectorXs> &x,
                      const Eigen::Ref<const VectorXs> &dx,
                      Eigen::Ref<MatrixXs> Jin,
                      const crocoddyl::Jcomponent firstsecond) const override {
    (void)firstsecond;
    assert(state);
    state->JintegrateTransport(x, dx, Jin);
  }

protected:
  using crocoddyl::StateAbstractTpl<Scalar>::nx_;
  using crocoddyl::StateAbstractTpl<Scalar>::ndx_;
  using crocoddyl::StateAbstractTpl<Scalar>::nq_;
  using crocoddyl::StateAbstractTpl<Scalar>::nv_;
  using crocoddyl::StateAbstractTpl<Scalar>::lb_;
  using crocoddyl::StateAbstractTpl<Scalar>::ub_;
  using crocoddyl::StateAbstractTpl<Scalar>::has_limits_;
};

struct Dynamics {

  typedef crocoddyl::MathBaseTpl<double> MathBase;
  typedef typename MathBase::VectorXs VectorXs;

  std::shared_ptr<dynobench::Model_robot> robot_model;
  double dt = 0;
  Control_Mode control_mode;
  boost::shared_ptr<StateCrocoDyno> state_croco;
  Eigen::VectorXd __v; // data
  std::map<std::string, double> params;

  Dynamics(std::shared_ptr<dynobench::Model_robot> robot_model = nullptr,
           const Control_Mode &control_mode = Control_Mode::default_mode,
           const std::map<std::string, double> &params = {});

  std::shared_ptr<crocoddyl::StateAbstractTpl<double>> state;

  void virtual print_bounds(std::ostream &out) const {
    out << STR_V(x_ub) << std::endl;
    out << STR_V(x_lb) << std::endl;
    out << STR_V(u_ub) << std::endl;
    out << STR_V(u_lb) << std::endl;
    out << STR_V(u_ref) << std::endl;
    out << STR_V(x_weightb) << std::endl;
    out << STR_V(u_weight) << std::endl;
    out << STR_(nx) << std::endl;
    out << STR_(nu) << std::endl;
  }

  virtual void calc(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                        Eigen::Ref<Eigen::MatrixXd> Fu,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual void update_state_and_control() {

    if (control_mode == Control_Mode::default_mode) {
      ;
    } else if (control_mode == Control_Mode::free_time) {
      nu = robot_model->nu + 1;
      Eigen::VectorXd __u_lb(u_lb), __u_ub(u_ub), __u_weight(u_weight),
          __u_ref(u_ref);
      modify_u_bound_for_free_time(__u_lb, __u_ub, __u_weight, __u_ref, u_lb,
                                   u_ub, u_weight, u_ref, params);
    } else if (control_mode == Control_Mode::free_time_linear) {

      Eigen::VectorXd __u_lb(u_lb), __u_ub(u_ub), __u_weight(u_weight),
          __u_ref(u_ref);
      modify_u_for_free_time_linear(__u_lb, __u_ub, __u_weight, __u_ref, u_lb,
                                    u_ub, u_weight, u_ref);

      Eigen::VectorXd __x_lb(x_lb), __x_ub(x_ub), __x_weightb(x_weightb);

      modify_x_bound_for_free_time_linear(__x_lb, __x_ub, __x_weightb, x_lb,
                                          x_ub, x_weightb);

    }

    else if (control_mode == Control_Mode::contour) {
      nu = robot_model->nu + 1;
      nx = robot_model->nx + 1;
      Eigen::VectorXd __u_lb(u_lb), __u_ub(u_ub), __u_weight(u_weight),
          __u_ref(u_ref);
      Eigen::VectorXd __x_lb(x_lb), __x_ub(x_ub), __x_weightb(x_weightb);

      modify_u_bound_for_contour(__u_lb, __u_ub, __u_weight, __u_ref, u_lb,
                                 u_ub, u_weight, u_ref);
      double max_alpha = 1;
      // TODO: user must change the alpha afterwards
      modify_x_bound_for_contour(__x_lb, __x_ub, __x_weightb, x_lb, x_ub,
                                 x_weightb, max_alpha);

    } else {
      ERROR_WITH_INFO("not implemented");
    }
  }

  size_t nx;
  size_t nu;

  Eigen::VectorXd u_lb;
  Eigen::VectorXd u_ub;
  Eigen::VectorXd u_ref;
  Eigen::VectorXd u_weight;

  Eigen::VectorXd x_lb;
  Eigen::VectorXd x_ub;
  Eigen::VectorXd x_weightb;

  virtual ~Dynamics(){};
};

struct Cost {
  size_t nx;
  size_t nu;
  size_t nr;
  std::string name;
  CostTYPE cost_type = CostTYPE::least_squares;

  Cost(size_t nx, size_t nu, size_t nr) : nx(nx), nu(nu), nr(nr) {}

  void check_input_calc(Eigen::Ref<Eigen::VectorXd> r,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) {
    DYNO_CHECK_EQ(static_cast<size_t>(r.size()), nr, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(u.size()), nu, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  }

  void check_input_calc(Eigen::Ref<Eigen::VectorXd> r,
                        const Eigen::Ref<const Eigen::VectorXd> &x) {
    DYNO_CHECK_EQ(static_cast<size_t>(r.size()), nr, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  }

  virtual void calc(Eigen::Ref<Eigen::VectorXd>,
                    const Eigen::Ref<const Eigen::VectorXd> &,
                    const Eigen::Ref<const Eigen::VectorXd> &) {
    throw std::runtime_error(AT);
  }

  virtual void calc(Eigen::Ref<Eigen::VectorXd>,
                    const Eigen::Ref<const Eigen::VectorXd> &) {

    throw std::runtime_error(AT);
  }

  virtual void check_input_calcDiff(
      Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
      Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
      Eigen::Ref<Eigen::MatrixXd> Lxu,
      const Eigen::Ref<const Eigen::VectorXd> &x,
      const Eigen::Ref<const Eigen::VectorXd> &u) {

    DYNO_CHECK_EQ(static_cast<size_t>(u.size()), nu, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Lx.size()), nx, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Lu.size()), nu, AT);

    DYNO_CHECK_EQ(static_cast<size_t>(Lxx.cols()), nx, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Lxx.rows()), nx, AT);

    DYNO_CHECK_EQ(static_cast<size_t>(Luu.cols()), nu, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Luu.rows()), nu, AT);

    DYNO_CHECK_EQ(static_cast<size_t>(Lxu.rows()), nx, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Lxu.cols()), nu, AT);
  }

  virtual void
  check_input_calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                       Eigen::Ref<Eigen::MatrixXd> Lxx,
                       const Eigen::Ref<const Eigen::VectorXd> &x) {

    DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Lx.size()), nx, AT);

    DYNO_CHECK_EQ(static_cast<size_t>(Lxx.cols()), nx, AT);
    DYNO_CHECK_EQ(static_cast<size_t>(Lxx.rows()), nx, AT);
  }

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) {

    (void)Lx;
    (void)Lu;
    (void)Lxx;
    (void)Luu;
    (void)Lxu;
    (void)x;
    (void)u;

    throw std::runtime_error(AT);
  }

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) {

    (void)Lx;
    (void)Lxx;
    (void)x;

    throw std::runtime_error(AT);
  }
  virtual std::string get_name() const { return name; }

  virtual ~Cost() = default;
};

struct Dynamics_free_time {

  typedef crocoddyl::MathBaseTpl<double> MathBase;
  typedef typename MathBase::VectorXs VectorXs;

  std::shared_ptr<dynobench::Model_robot> robot_model;
  std::vector<Cost> costs;
  double dt = 0;
  Control_Mode control_mode;
  boost::shared_ptr<StateCrocoDyno> state_croco;
  Eigen::VectorXd __v; // data

    std::map<std::string, double> params ;

  Dynamics_free_time(
      std::shared_ptr<dynobench::Model_robot> robot_model = nullptr,
      const Control_Mode &control_mode = Control_Mode::default_mode ,
    const std::map<std::string, double> &params = {});

  std::shared_ptr<crocoddyl::StateAbstractTpl<double>> state;

  void virtual print_bounds(std::ostream &out) const {
    out << STR_V(x_ub) << std::endl;
    out << STR_V(x_lb) << std::endl;
    out << STR_V(u_ub) << std::endl;
    out << STR_V(u_lb) << std::endl;
    out << STR_V(u_ref) << std::endl;
    out << STR_V(x_weightb) << std::endl;
    out << STR_V(u_weight) << std::endl;
    out << STR_(nx) << std::endl;
    out << STR_(nu) << std::endl;
  }

  virtual void calc(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                        Eigen::Ref<Eigen::MatrixXd> Fu,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual void update_state_and_control() {

    if (control_mode == Control_Mode::default_mode) {
      ;
    } else if (control_mode == Control_Mode::free_time) {
      nu = robot_model->nu + 1;
      Eigen::VectorXd __u_lb(u_lb), __u_ub(u_ub), __u_weight(u_weight),
          __u_ref(u_ref);
      modify_u_bound_for_free_time(__u_lb, __u_ub, __u_weight, __u_ref, u_lb,
                                   u_ub, u_weight, u_ref, params);
    } else if (control_mode == Control_Mode::free_time_linear) {

      Eigen::VectorXd __u_lb(u_lb), __u_ub(u_ub), __u_weight(u_weight),
          __u_ref(u_ref);
      modify_u_for_free_time_linear(__u_lb, __u_ub, __u_weight, __u_ref, u_lb,
                                    u_ub, u_weight, u_ref);

      Eigen::VectorXd __x_lb(x_lb), __x_ub(x_ub), __x_weightb(x_weightb);

      modify_x_bound_for_free_time_linear(__x_lb, __x_ub, __x_weightb, x_lb,
                                          x_ub, x_weightb);

    }

    else if (control_mode == Control_Mode::contour) {
      nu = robot_model->nu + 1;
      nx = robot_model->nx + 1;
      Eigen::VectorXd __u_lb(u_lb), __u_ub(u_ub), __u_weight(u_weight),
          __u_ref(u_ref);
      Eigen::VectorXd __x_lb(x_lb), __x_ub(x_ub), __x_weightb(x_weightb);

      modify_u_bound_for_contour(__u_lb, __u_ub, __u_weight, __u_ref, u_lb,
                                 u_ub, u_weight, u_ref);
      double max_alpha = 1;
      // TODO: user must change the alpha afterwards
      modify_x_bound_for_contour(__x_lb, __x_ub, __x_weightb, x_lb, x_ub,
                                 x_weightb, max_alpha);

    } else {
      ERROR_WITH_INFO("not implemented");
    }
  }

  size_t nx;
  size_t nu;

  Eigen::VectorXd u_lb;
  Eigen::VectorXd u_ub;
  Eigen::VectorXd u_ref;
  Eigen::VectorXd u_weight;

  Eigen::VectorXd x_lb;
  Eigen::VectorXd x_ub;
  Eigen::VectorXd x_weightb;

  virtual ~Dynamics_free_time(){};
};

struct Quaternion_cost : Cost {

  double k_quat = 1.;

  Quaternion_cost(size_t nx, size_t nu);

  virtual ~Quaternion_cost() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;
};

struct Diff_angle_cost : Cost {

  double k_diff_angle = 20.;

  Eigen::Matrix<double, 2, 1> f = Eigen::Matrix<double, 1, 2>::Zero();
  Eigen::Matrix<double, 2, 4> Jx = Eigen::Matrix<double, 2, 4>::Zero();
  Eigen::Matrix<double, 2, 2> Ju = Eigen::Matrix<double, 2, 2>::Zero();

  std::shared_ptr<dynobench::Model_car_with_trailers> car;
  Diff_angle_cost(size_t nx, size_t nu,
                  std::shared_ptr<dynobench::Model_car_with_trailers> car);

  virtual ~Diff_angle_cost() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;
};

struct Payload_n_acceleration_cost : Cost {

  // TODO: KHALED, we want this generic
  // You just have to modify the selector vector -- see constructor

  std::shared_ptr<dynobench::Model_robot> model;

  double k_acc = 1;

  Eigen::VectorXd selector;
  Eigen::VectorXd f;
  Eigen::MatrixXd acc_u;
  Eigen::MatrixXd acc_x;

  Eigen::MatrixXd Jv_x;
  Eigen::MatrixXd Jv_u;

  Payload_n_acceleration_cost(
      const std::shared_ptr<dynobench::Model_robot> &model_robot, double k_acc);

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual ~Payload_n_acceleration_cost() = default;
};

struct Quad3d_acceleration_cost : Cost {

  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector13d = Eigen::Matrix<double, 13, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;

  std::shared_ptr<dynobench::Model_robot> model;

  double k_acc = 1;
  Vector12d f;
  Vector6d acc;
  Eigen::Matrix<double, 6, 4> acc_u;
  Eigen::Matrix<double, 6, 13> acc_x;

  Eigen::Matrix<double, 12, 13> Jv_x;
  Eigen::Matrix<double, 12, 4> Jv_u;

  Quad3d_acceleration_cost(
      const std::shared_ptr<dynobench::Model_robot> &model_robot);

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual ~Quad3d_acceleration_cost() = default;
};

struct Acceleration_cost_acrobot : Cost {

  dynobench::Model_acrobot model;

  double k_acc = .1;
  Eigen::Vector4d f;
  Eigen::Vector2d acc;
  Eigen::Matrix<double, 2, 1> acc_u;
  Eigen::Matrix<double, 2, 4> acc_x;

  Eigen::Matrix<double, 4, 4> Jv_x;
  Eigen::Matrix<double, 4, 1> Jv_u;

  Acceleration_cost_acrobot(size_t nx, size_t nu);

  virtual ~Acceleration_cost_acrobot() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;
};

struct Acceleration_cost_quad2d : Cost {

  std::shared_ptr<dynobench::Model_robot> model;

  double k_acc = .01;
  Eigen::Matrix<double, 6, 1> f;
  Eigen::Vector3d acc;
  Eigen::Matrix<double, 3, 2> acc_u;
  Eigen::Matrix<double, 3, 6> acc_x;

  Eigen::Matrix<double, 6, 6> Jv_x;
  Eigen::Matrix<double, 6, 2> Jv_u;

  Acceleration_cost_quad2d(
      const std::shared_ptr<dynobench::Model_robot> &model_robot, size_t nx,
      size_t nu);

  virtual ~Acceleration_cost_quad2d() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;
};

struct Contour_cost_alpha_x : Cost {

  double k = 1.5; // bigger than 0
  Contour_cost_alpha_x(size_t nx, size_t nu);

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual ~Contour_cost_alpha_x() = default;
};

struct Time_linear_reg : Cost {
  // u - x

  double k = 50; // bigger than 0
  Time_linear_reg(size_t nx, size_t nu);

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual ~Time_linear_reg() = default;
};

struct Min_time_linear : Cost {
  // u - x

  double k = 5; // bigger than 0
  Min_time_linear(size_t nx, size_t nu);
  virtual ~Min_time_linear() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;
};

struct Contour_cost_alpha_u : Cost {

  double k = 1.5; // bigger than 0
  Contour_cost_alpha_u(size_t nx, size_t nu);
  virtual ~Contour_cost_alpha_u() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;
};

void finite_diff_cost(ptr<Cost> cost, Eigen::Ref<Eigen::MatrixXd> Jx,
                      Eigen::Ref<Eigen::MatrixXd> Ju, const Eigen::VectorXd &x,
                      const Eigen::VectorXd &u, const int nr);

struct Contour_cost_x : Cost {

  ptr<dynobench::Interpolator> path;
  double weight = 20.;
  bool use_finite_diff = false;

  double last_query;
  Eigen::VectorXd last_out;
  Eigen::VectorXd last_J;

  Eigen::MatrixXd __Jx;
  Eigen::VectorXd __r;

  Contour_cost_x(size_t nx, size_t nu, ptr<dynobench::Interpolator> path);

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;
};

struct Contour_cost : Cost {

  ptr<dynobench::Interpolator> path;
  double ref_alpha = 1.;
  double weight_alpha = 2.;
  double weight_diff = 20.;
  double weight_contour = 1.;
  double weight_virtual_control = 2.;
  bool use_finite_diff = false;

  double last_query;
  Eigen::VectorXd last_out;
  Eigen::VectorXd last_J;

  Contour_cost(size_t nx, size_t nu, ptr<dynobench::Interpolator> path);

  virtual ~Contour_cost() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  // virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
  //                   const Eigen::Ref<const Eigen::VectorXd> &x) override;

  // virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
  //                       Eigen::Ref<Eigen::MatrixXd> Lxx,
  //                       const Eigen::Ref<const Eigen::VectorXd> &x)
  //                       override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;
};

struct Col_cost : Cost {

  std::shared_ptr<dynobench::Model_robot> model;
  double margin = .03;
  double last_raw_d = 0;
  double weight;
  Eigen::VectorXd last_x;
  Eigen::VectorXd last_grad;
  dynobench::CollisionOut cinfo;
  Eigen::MatrixXd Jx;
  Eigen::VectorXd v__; //  data

  // TODO: check that the sec_factor is actually save in a test
  double sec_factor = .1;

  // what about a log barrier function? -- maybe I get better gradients

  double faraway_zero_gradient_bound = 1.1 * margin;
  // returns 0 gradient if distance is > than THIS.
  double epsilon = 1e-3; // finite diff

  Col_cost(size_t nx, size_t nu, size_t nr,
           std::shared_ptr<dynobench::Model_robot> model, double weight = 100.);

  virtual ~Col_cost() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  void set_nx_effective(size_t nx_effective) {
    this->nx_effective = nx_effective;
    v__.resize(nx_effective);
  }

  size_t get_nx_effective() { return nx_effective; }

private:
  size_t nx_effective;

  // virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
  //                       Eigen::Ref<Eigen::MatrixXd> Ju,
  //                       const Eigen::Ref<const Eigen::VectorXd> &x,
  //                       const Eigen::Ref<const Eigen::VectorXd> &u);
  //
  // virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
  //                       const Eigen::Ref<const Eigen::VectorXd> &x);
};

struct Control_cost : Cost {

  Eigen::VectorXd u_weight;
  Eigen::VectorXd u_ref;

  Control_cost(size_t nx, size_t nu, size_t nr, const Eigen::VectorXd &u_weight,
               const Eigen::VectorXd &u_ref);

  virtual ~Control_cost() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u);

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u);

  // virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
  //                       Eigen::Ref<Eigen::MatrixXd> Ju,
  //                       const Eigen::Ref<const Eigen::VectorXd> &x,
  //                       const Eigen::Ref<const Eigen::VectorXd> &u);
  //
  // virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
  //                       const Eigen::Ref<const Eigen::VectorXd> &x);
};

// x - ub  <= 0
struct State_bounds : Cost {
  // weight * ( x - ub ) <= 0
  //
  // NOTE:
  // you can implement lower bounds
  // weight = -w
  // ub = lb
  // DEMO:
  // -w ( x - lb ) <= 0
  // w ( x - lb ) >= 0
  // w x >= w lb

  Eigen::VectorXd ub;
  Eigen::VectorXd weight;

  State_bounds(size_t nx, size_t nu, size_t nr, const Eigen::VectorXd &ub,
               const Eigen::VectorXd &weight);

  virtual ~State_bounds() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;
};

struct Control_bounds : Cost {
  // weight * ( x - ub ) <= 0
  //
  // NOTE:
  // you can implement lower bounds
  // weight = -w
  // ub = lb
  // DEMO:
  // -w ( x - lb ) <= 0
  // w ( x - lb ) >= 0
  // w x >= w lb

  Eigen::VectorXd ub;
  Eigen::VectorXd weight;

  Control_bounds(size_t nx, size_t nu, size_t nr, const Eigen::VectorXd &ub,
                 const Eigen::VectorXd &weight);

  virtual ~Control_bounds() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;
};

struct State_cost : Cost {

  Eigen::VectorXd x_weight;
  Eigen::VectorXd ref;

  State_cost(size_t nx, size_t nu, size_t nr, const Eigen::VectorXd &x_weight,
             const Eigen::VectorXd &ref);

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;
};

// TODO: i need some kind of "n effective"
// Check what happens with contour control
struct State_cost_model : Cost {

  Eigen::VectorXd ref;
  Eigen::VectorXd x_weight;
  Eigen::MatrixXd
      x_weight_mat; // for multiplying each row by a different number

  size_t nx_effective;

  // data
  Eigen::MatrixXd Jx0;
  Eigen::MatrixXd Jx1;
  Eigen::MatrixXd Jx1_w;
  Eigen::VectorXd __r;
  Eigen::VectorXd x_weight_sq;

  std::shared_ptr<dynobench::Model_robot> model_robot;

  State_cost_model(const std::shared_ptr<dynobench::Model_robot> &model_robot,
                   size_t nx, size_t nu, const Eigen::VectorXd &x_weight,
                   const Eigen::VectorXd &ref);

  virtual ~State_cost_model() = default;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
                    const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) override;
};

// struct All_cost : Cost {
//
//   std::vector<boost::shared_ptr<Cost>> costs;
//
//   All_cost(size_t nx, size_t nu, size_t nr,
//            const std::vector<boost::shared_ptr<Cost>> &costs);
//
//   virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
//                     const Eigen::Ref<const Eigen::VectorXd> &x,
//                     const Eigen::Ref<const Eigen::VectorXd> &u);
//
//   virtual void calc(Eigen::Ref<Eigen::VectorXd> r,
//                     const Eigen::Ref<const Eigen::VectorXd> &x);
//
//   virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                         Eigen::Ref<Eigen::MatrixXd> Ju,
//                         const Eigen::Ref<const Eigen::VectorXd> &x,
//                         const Eigen::Ref<const Eigen::VectorXd> &u);
//
//   virtual void calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                         const Eigen::Ref<const Eigen::VectorXd> &x);
// };

size_t
get_total_num_features(const std::vector<boost::shared_ptr<Cost>> &features);

class ActionModelDyno : public crocoddyl::ActionModelAbstractTpl<double> {
public:
  typedef double Scalar;
  typedef crocoddyl::ActionDataAbstractTpl<Scalar> ActionDataAbstract;
  typedef ActionModelAbstractTpl<Scalar> Base;
  typedef ActionDataDyno Data;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::VectorXs VectorXs;

  boost::shared_ptr<Dynamics> dynamics;
  std::vector<boost::shared_ptr<Cost>> features;
  size_t nx;
  size_t nu;
  size_t nr;

  Eigen::MatrixXd Jx;
  Eigen::MatrixXd Ju;

  ActionModelDyno(ptr<Dynamics> dynamics,
                  const std::vector<ptr<Cost>> &features);

  virtual ~ActionModelDyno() = default;

  virtual void calc(const boost::shared_ptr<ActionDataAbstract> &data,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract> &data,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual void calc(const boost::shared_ptr<ActionDataAbstract> &data,
                    const Eigen::Ref<const VectorXs> &x);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract> &data,
                        const Eigen::Ref<const VectorXs> &x);

  virtual boost::shared_ptr<ActionDataAbstract> createData();
  virtual bool checkData(const boost::shared_ptr<ActionDataAbstract> &data);

  virtual void print(std::ostream &os) const;
};

class ActionModelDynov2 : public crocoddyl::ActionModelAbstractTpl<double> {
public:
  typedef double Scalar;
  typedef crocoddyl::ActionDataAbstractTpl<Scalar> ActionDataAbstract;
  typedef ActionModelAbstractTpl<Scalar> Base;
  typedef ActionDataDyno Data;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::VectorXs VectorXs;

  std::shared_ptr<dynobench::Model_robot> model_robot;
  std::vector<boost::shared_ptr<Cost>> features;
  size_t nx;
  size_t nu;
  size_t nr;

  size_t nx_orig;
  size_t nu_orig;

  Eigen::MatrixXd Jx;
  Eigen::MatrixXd Ju;

  Control_Mode control_mode;
  ActionModelDynov2(std::shared_ptr<dynobench::Model_robot> model_robot,
                    const std::vector<ptr<Cost>> &features,
                    Control_Mode default_mode);

  virtual ~ActionModelDynov2() = default;

  virtual void calc(const boost::shared_ptr<ActionDataAbstract> &data,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract> &data,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual void calc(const boost::shared_ptr<ActionDataAbstract> &data,
                    const Eigen::Ref<const VectorXs> &x);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract> &data,
                        const Eigen::Ref<const VectorXs> &x);

  virtual boost::shared_ptr<ActionDataAbstract> createData();
  virtual bool checkData(const boost::shared_ptr<ActionDataAbstract> &data);

  virtual void print(std::ostream &os) const;
};

void print_data(
    boost::shared_ptr<crocoddyl::ActionDataAbstractTpl<double>> data);

void check_dyn(boost::shared_ptr<Dynamics> dyn, double eps,
               Eigen::VectorXd x = Eigen::VectorXd(),
               Eigen::VectorXd u = Eigen::VectorXd(), double margin_rate = 10);

ptr<Dynamics>
create_dynamics(std::shared_ptr<dynobench::Model_robot> model_robot,
                const Control_Mode &control_mode = Control_Mode::default_mode,
                const std::map<std::string, double> &params = {});

std::vector<ReportCost>
get_report(ptr<ActionModelDyno> p,
           std::function<void(ptr<Cost>, Eigen::Ref<Eigen::VectorXd>)> fun);

} // namespace dynoplan
