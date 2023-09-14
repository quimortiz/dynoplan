#include "idbastar/optimization/croco_models.hpp"
#include "dynobench/croco_macros.hpp"
#include "dynobench/math_utils.hpp"
// #include "idbastar/ompl/robots.h"
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>

using vstr = std::vector<std::string>;
using V2d = Eigen::Vector2d;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using Vxd = Eigen::VectorXd;
using V1d = Eigen::Matrix<double, 1, 1>;

namespace dynoplan {

template <typename K, typename V>
V GetWithDef(const std::map<K, V> &m, const K &key, const V &defval) {
  typename std::map<K, V>::const_iterator it = m.find(key);
  if (it == m.end()) {
    return defval;
  } else {
    return it->second;
  }
}

Eigen::VectorXd derivate_wrt_time(dynobench::Model_robot &robot_model,
                                  const Eigen::VectorXd &x,
                                  const Eigen::VectorXd &u, double dt) {
  double epsilon = 1e-5;
  Eigen::VectorXd xnexte(robot_model.nx);
  Eigen::VectorXd xnext(robot_model.nx);
  robot_model.step(xnext, x.head(robot_model.nx), u.head(robot_model.nu),
                   dt * u(robot_model.nu));
  robot_model.step(xnexte, x.head(robot_model.nx), u.head(robot_model.nu),
                   dt * u(robot_model.nu) + epsilon);
  Eigen::VectorXd dd = (xnexte - xnext) / epsilon;
  return dd;
};

ptr<Dynamics>
create_dynamics(std::shared_ptr<dynobench::Model_robot> model_robot,
                const Control_Mode &control_mode,
                const std::map<std::string, double> &params) {
  return mk<Dynamics>(model_robot, control_mode, params);
}

void modify_x_bound_for_free_time_linear(const Vxd &__x_lb, const Vxd &__x_ub,
                                         const Vxd &__xb__weight, Vxd &x_lb,
                                         Vxd &x_ub, Vxd &xb_weight) {

  CHECK_EQ(__x_lb.size(), __x_ub.size(), AT);
  CHECK_EQ(__xb__weight.size(), __x_ub.size(), AT);

  size_t nx = __x_lb.size() + 1;

  xb_weight = Vxd(nx);
  x_lb = Vxd(nx);
  x_ub = Vxd(nx);

  double max_time_rate = 2;
  x_lb << __x_lb, 0.;
  x_ub << __x_ub, max_time_rate;
  xb_weight << __xb__weight, 200.;
}

void modify_x_bound_for_contour(const Vxd &__x_lb, const Vxd &__x_ub,
                                const Vxd &__xb__weight, Vxd &x_lb, Vxd &x_ub,
                                Vxd &xb_weight, double max_alpha) {

  CHECK_EQ(__x_lb.size(), __x_ub.size(), AT);
  CHECK_EQ(__xb__weight.size(), __x_ub.size(), AT);

  size_t nx = __x_lb.size() + 1;

  xb_weight = Vxd(nx);
  x_lb = Vxd(nx);
  x_ub = Vxd(nx);

  x_lb << __x_lb, -10.;
  x_ub << __x_ub, max_alpha;
  xb_weight << __xb__weight, 200.;
}

void modify_u_bound_for_contour(const Vxd &__u_lb, const Vxd &__u_ub,
                                const Vxd &__u__weight, const Vxd &__u__ref,
                                Vxd &u_lb, Vxd &u_ub, Vxd &u_weight,
                                Vxd &u_ref) {

  CHECK_EQ(__u_lb.size(), __u_ub.size(), AT);
  CHECK_EQ(__u__weight.size(), __u_ub.size(), AT);
  CHECK_EQ(__u__ref.size(), __u_ub.size(), AT);

  size_t nu = __u_lb.size() + 1;

  u_weight = Vxd(nu);
  u_lb = Vxd(nu);
  u_ub = Vxd(nu);
  u_ref = Vxd(nu);

  u_lb << __u_lb, -10.;
  u_ub << __u_ub, 10.;
  u_ref << __u__ref, 0.;
  u_weight << __u__weight, .1;
}

void modify_u_for_free_time_linear(const Vxd &__u_lb, const Vxd &__u_ub,
                                   const Vxd &__u__weight, const Vxd &__u__ref,
                                   Vxd &u_lb, Vxd &u_ub, Vxd &u_weight,
                                   Vxd &u_ref) {

  CHECK_EQ(__u_lb.size(), __u_ub.size(), AT);
  CHECK_EQ(__u__weight.size(), __u_ub.size(), AT);
  CHECK_EQ(__u__ref.size(), __u_ub.size(), AT);

  size_t nu = __u_lb.size() + 1;

  u_weight = Vxd(nu);
  u_lb = Vxd(nu);
  u_ub = Vxd(nu);
  u_ref = Vxd(nu);

  double max_time_rate = 2;
  u_lb << __u_lb, 0;
  u_ub << __u_ub, max_time_rate;
  u_ref << __u__ref, 0.;
  u_weight << __u__weight, 0;
  // TODO: then I have to add a linear term to the cost!
}

void modify_u_bound_for_free_time(const Vxd &__u_lb, const Vxd &__u_ub,
                                  const Vxd &__u__weight, const Vxd &__u__ref,
                                  Vxd &u_lb, Vxd &u_ub, Vxd &u_weight,
                                  Vxd &u_ref,
                                  const std::map<std::string, double> &params) {

  CHECK_EQ(__u_lb.size(), __u_ub.size(), AT);
  CHECK_EQ(__u__weight.size(), __u_ub.size(), AT);
  CHECK_EQ(__u__ref.size(), __u_ub.size(), AT);

  size_t nu = __u_lb.size() + 1;

  std::cout << STR_(u_weight) << std::endl;
  std::cout << STR_(u_weight.size()) << std::endl;
  u_weight = Vxd(nu);
  u_lb = Vxd(nu);
  u_ub = Vxd(nu);
  u_ref = Vxd(nu);

  const double min_time_rate =
      GetWithDef(params, std::string("min_time_rate"), .4);
  const double max_time_rate =
      GetWithDef(params, std::string("max_time_rate"), 2.);
  const double ref_time_rate =
      GetWithDef(params, std::string("ref_time_rate"), .5);
  const double time_weight = GetWithDef(params, std::string("time_weight"), .7);

  u_lb << __u_lb, min_time_rate;
  u_ub << __u_ub, max_time_rate;
  u_ref << __u__ref, ref_time_rate;
  u_weight << __u__weight, time_weight;
}

void check_input_calc(Eigen::Ref<Eigen::VectorXd> xnext,
                      const Eigen::Ref<const Vxd> &x,
                      const Eigen::Ref<const Vxd> &u, size_t nx, size_t nu) {

  if (static_cast<std::size_t>(x.size()) != nx) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }

  if (static_cast<std::size_t>(u.size()) != nu) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(nu) + ")");
  }

  if (static_cast<std::size_t>(xnext.size()) != nx) {
    throw_pretty("Invalid argument: "
                 << "xnext has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }
};

void check_input_calcdiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                          Eigen::Ref<Eigen::MatrixXd> Fu,
                          const Eigen::Ref<const Vxd> &x,
                          const Eigen::Ref<const Vxd> &u, size_t nx,
                          size_t nu) {

  if (static_cast<std::size_t>(x.size()) != nx) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu) + ")");
  }

  if (static_cast<std::size_t>(Fx.cols()) != nx) {
    throw_pretty("Invalid argument: "
                 << "Fx has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }

  if (static_cast<std::size_t>(Fx.rows()) != nx) {
    throw_pretty("Invalid argument: "
                 << "Fx has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }

  if (static_cast<std::size_t>(Fu.cols()) != nu) {
    throw_pretty("Invalid argument: "
                 << "Fu has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }

  if (static_cast<std::size_t>(Fu.rows()) != nx) {
    throw_pretty("Invalid argument: "
                 << "Fu has wrong dimension (it should be " +
                        std::to_string(nx) + ")");
  }
}

// u - x
Time_linear_reg::Time_linear_reg(size_t nx, size_t nu) : Cost(nx, nu, 1) {

  name = "time_linear_reg";
}

void Time_linear_reg::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  check_input_calc(r, x, u);
  double d = x.tail(1)(0) - u.tail(1)(0);
  r(0) = k * d;
}

void Time_linear_reg::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x) {
  (void)r;
  (void)x;

  ERROR_WITH_INFO("error here! -- we require u and x!");
}

void Time_linear_reg::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::VectorXd> Lu,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               Eigen::Ref<Eigen::MatrixXd> Luu,
                               Eigen::Ref<Eigen::MatrixXd> Lxu,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {

  //  .5 * k^2 ( x - u )^2
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
  double d = x.tail(1)(0) - u.tail(1)(0);
  Lx(nx - 1) += k * k * d;
  Lu(nu - 1) += -k * k * d;
  Lxx(nx - 1, nx - 1) += k * k;
  Luu(nu - 1, nu - 1) += k * k;
  Lxu(nx - 1, nu - 1) += -k * k;
}

void Time_linear_reg::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               const Eigen::Ref<const Eigen::VectorXd> &x) {
  (void)Lx;
  (void)Lxx;
  (void)x;

  ERROR_WITH_INFO("error here! -- we require u and x!");
}

Contour_cost_alpha_x::Contour_cost_alpha_x(size_t nx, size_t nu)
    : Cost(nx, nu, 1) {
  name = "contour-cost-alpha-x";
  cost_type = CostTYPE::linear;
}

void Contour_cost_alpha_x::calc(Eigen::Ref<Vxd> r,
                                const Eigen::Ref<const Vxd> &x,
                                const Eigen::Ref<const Vxd> &u) {
  check_input_calc(r, x, u);
  calc(r, x);
}

void Contour_cost_alpha_x::calc(Eigen::Ref<Vxd> r,
                                const Eigen::Ref<const Vxd> &x) {
  check_input_calc(r, x);
  r(0) = -k * x(nx - 1);
}

void Contour_cost_alpha_x::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::MatrixXd> Lxx,
    const Eigen::Ref<const Eigen::VectorXd> &x) {
  check_input_calcDiff(Lx, Lxx, x);

  Lx(nx - 1) += -k;
}

void Contour_cost_alpha_x::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
    Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
    Eigen::Ref<Eigen::MatrixXd> Lxu, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);

  calcDiff(Lx, Lxx, x);
}

Contour_cost_alpha_u::Contour_cost_alpha_u(size_t nx, size_t nu)
    : Cost(nx, nu, 1) {
  name = "contour-cost-alpha-u";
  cost_type = CostTYPE::linear;
}

void Contour_cost_alpha_u::calc(Eigen::Ref<Vxd> r,
                                const Eigen::Ref<const Vxd> &x,
                                const Eigen::Ref<const Vxd> &u) {
  check_input_calc(r, x, u);
  CHECK_GE(k, 0, AT);

  r(0) = -k * u(nu - 1);
}

void Contour_cost_alpha_u::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
    Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
    Eigen::Ref<Eigen::MatrixXd> Lxu, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);

  Lu(nu - 1) += -k;
}

void finite_diff_cost(ptr<Cost> cost, Eigen::Ref<Eigen::MatrixXd> Jx,
                      Eigen::Ref<Eigen::MatrixXd> Ju, const Vxd &x,
                      const Vxd &u, const int nr) {
  Vxd r_ref(nr);
  cost->calc(r_ref, x, u);
  size_t nu = u.size();
  size_t nx = x.size();

  Ju.setZero();

  double eps = 1e-5;
  for (size_t i = 0; i < nu; i++) {
    Eigen::MatrixXd ue;
    ue = u;
    ue(i) += eps;
    Vxd r_e(nr);
    r_e.setZero();
    cost->calc(r_e, x, ue);
    auto df = (r_e - r_ref) / eps;
    Ju.col(i) = df;
  }

  Jx.setZero();
  for (size_t i = 0; i < nx; i++) {
    Eigen::MatrixXd xe;
    xe = x;
    xe(i) += eps;
    Vxd r_e(nr);
    r_e.setZero();
    cost->calc(r_e, xe, u);
    auto df = (r_e - r_ref) / eps;
    Jx.col(i) = df;
  }
}

Contour_cost_x::Contour_cost_x(size_t nx, size_t nu,
                               ptr<dynobench::Interpolator> path)
    : Cost(nx, nu, nx - 1), path(path),
      last_query(std::numeric_limits<double>::lowest()), last_out(nx - 1),
      last_J(nx - 1), __Jx(nr, nx), __r(nr) {
  name = "contour-cost-x";
  __Jx.setZero();
  __r.setZero();
}
void Contour_cost_x::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                          const Eigen::Ref<const Vxd> &u) {
  // check that r

  check_input_calc(r, x, u);
  calc(r, x);
}

void Contour_cost_x::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x) {
  check_input_calc(r, x);
  double alpha = x(nx - 1);

  last_query = alpha;
  CHECK(path, AT);
  CHECK_GEQ(weight, 0, AT);
  path->interpolate(alpha, last_out, last_J);

  r = weight * (last_out - x.head(nx - 1));
}

void Contour_cost_x::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                              Eigen::Ref<Eigen::VectorXd> Lu,
                              Eigen::Ref<Eigen::MatrixXd> Lxx,
                              Eigen::Ref<Eigen::MatrixXd> Luu,
                              Eigen::Ref<Eigen::MatrixXd> Lxu,
                              const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
};

void Contour_cost_x::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                              Eigen::Ref<Eigen::MatrixXd> Lxx,
                              const Eigen::Ref<const Eigen::VectorXd> &x) {

  CHECK_GE(weight, 0, AT);
  check_input_calcDiff(Lx, Lxx, x);

  double alpha = x(nx - 1);
  if (std::fabs(alpha - last_query) > 1e-8) {
    last_query = alpha;
    assert(path);
    path->interpolate(alpha, last_out, last_J);
  }

  // TODO check if this works correctly for SO(2)?

  __Jx.block(0, 0, nx - 1, nx - 1).diagonal() = -weight * Vxd::Ones(nx - 1);
  __Jx.block(0, nx - 1, nx - 1, 1) = weight * last_J;

  __r = weight * (last_out - x.head(nx - 1));
  // std::cout << "calcDiff __r: " << __r.format(FMT) << std::endl;
  //

  // CSTR_V((__r));
  // CSTR_V((__Jx));
  // CSTR_V((__r.transpose() * __Jx));

  Lx += __r.transpose() * __Jx;

  Lxx += __Jx.transpose() * __Jx;
  // Lxx.diagonal().head(nx - 1) += weight * weight * Vxd::Ones(nx - 1);
  // Lxx(nx - 1, nx - 1) += weight * last_J.transpose() * last_J;
}

Contour_cost::Contour_cost(size_t nx, size_t nu,
                           ptr<dynobench::Interpolator> path)
    : Cost(nx, nu, nx + 1), path(path), last_query(-1.), last_out(nx - 1),
      last_J(nx - 1) {
  name = "contour";
}

void Contour_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                        const Eigen::Ref<const Vxd> &u) {
  // check that r

  check_input_calc(r, x, u);

  double alpha = x(nx - 1);

  last_query = alpha;
  CHECK(path, AT);
  path->interpolate(alpha, last_out, last_J);

  r.head(nx - 1) = weight_contour * weight_diff * (last_out - x.head(nx - 1));
  r(nx - 1) = weight_contour * weight_alpha * (alpha - ref_alpha);
  r(nx) = weight_virtual_control * u(nu - 1);
}

// void Contour_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x)
// {
//   throw -1;
// }

void Contour_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                            Eigen::Ref<Eigen::VectorXd> Lu,
                            Eigen::Ref<Eigen::MatrixXd> Lxx,
                            Eigen::Ref<Eigen::MatrixXd> Luu,
                            Eigen::Ref<Eigen::MatrixXd> Lxu,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);

  Eigen::MatrixXd Jx(nr, nx);
  Eigen::VectorXd r(nr);
  Jx.setZero();
  Eigen::MatrixXd Ju(nr, nu);
  Ju.setZero();

  double alpha = x(nx - 1);
  if (std::fabs(alpha - last_query) > 1e-8) {
    last_query = alpha;
    CHECK(path, AT);
    path->interpolate(alpha, last_out, last_J);
  }

  Jx.block(0, 0, nx - 1, nx - 1).diagonal() =
      -weight_contour * weight_diff * Vxd::Ones(nx - 1);
  Jx.block(0, nx - 1, nx - 1, 1) = weight_contour * weight_diff * last_J;
  Jx(nx - 1, nx - 1) = weight_contour * weight_alpha;
  Ju(nx, nu - 1) = weight_virtual_control;

  r.head(nx - 1) = weight_contour * weight_diff * (last_out - x.head(nx - 1));
  r(nx - 1) = weight_contour * weight_alpha * (alpha - ref_alpha);
  r(nx) = weight_virtual_control * u(nu - 1);

  Lx += r.transpose() * Jx;
  Lu += r.transpose() * Ju;
  Lxx += Jx.transpose() * Jx;
  Luu += Ju.transpose() * Jx;
  Lxu += Jx.transpose() * Ju;
}

Col_cost::Col_cost(size_t nx, size_t nu, size_t nr,
                   std::shared_ptr<dynobench::Model_robot> model, double weight)
    : Cost(nx, nu, nr), model(model), weight(weight) {
  last_x = Vxd::Ones(nx);
  name = "collision";
  nx_effective = nx;

  Jx.resize(1, nx);
  Jx.setZero();

  v__.resize(nx);
  v__.setZero();
}

void Col_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                    const Eigen::Ref<const Vxd> &u) {
  check_input_calc(r, x, u);
  calc(r, x);
}

void Col_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x) {
  check_input_calc(r, x);

  CHECK(model, AT);
  double raw_d;
  bool check_one =
      (x.head(nx_effective) - last_x.head(nx_effective)).norm() < 1e-8;
  bool check_two = (last_raw_d - margin) > 0 &&
                   (x.head(nx_effective) - last_x.head(nx_effective)).norm() <
                       sec_factor * (last_raw_d - margin);

  // std::cout << "checking collisions " << std::endl;

  if (check_one || check_two) {
    raw_d = last_raw_d;
  } else {
    model->collision_distance(x.head(nx_effective), cinfo);
    raw_d = cinfo.distance;
    last_x = x;
    last_raw_d = raw_d;
    // std::cout << " x " << STR_V(x) << " " << raw_d << std::endl;
  }
  double d = weight * (raw_d - margin);
  auto out = Eigen::Matrix<double, 1, 1>(std::min(d, 0.));
  r = out;
  // std::cout << "r col is " << r << " x " << STR_V(x) << std::endl;
}

void Col_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::VectorXd> Lu,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        Eigen::Ref<Eigen::MatrixXd> Luu,
                        Eigen::Ref<Eigen::MatrixXd> Lxu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
  calcDiff(Lx, Lxx, x);
}

void Col_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                        Eigen::Ref<Eigen::MatrixXd> Lxx,
                        const Eigen::Ref<const Eigen::VectorXd> &x) {
  CHECK(model, AT);
  check_input_calcDiff(Lx, Lxx, x);
  Jx.setZero();

  double raw_d, d;
  bool check_one =
      (x - last_x).squaredNorm() < 1e-8 && (last_raw_d - margin) > 0;
  bool check_two = (last_raw_d - margin) > 0 &&
                   (x - last_x).norm() < sec_factor * (last_raw_d - margin);

  if (check_one || check_two) {
    ;
  } else {
    v__.setZero();
    model->collision_distance_diff(v__, raw_d, x);
    last_x = x;
    last_raw_d = raw_d;
    d = weight * (raw_d - margin);
    v__ = v__ * weight;
    if (d <= 0) {
      Jx.block(0, 0, 1, nx_effective) = v__.transpose();
      Lx += d * Jx.transpose();
      Lxx += Jx.transpose() * Jx;
      // std::cout << "contribution from collisions" << std::endl;
      // std::cout << "x " << STR_V(x) << std::endl;
      // std::cout << "Lx " << std::endl;
      // std::cout << Lx << std::endl;
      // std::cout << "Lxx " << std::endl;
      // std::cout << Lxx << std::endl;
    } else {
      ;
    }
  }
};

// void Col_cost::calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                         Eigen::Ref<Eigen::MatrixXd> Ju,
//                         const Eigen::Ref<const Vxd> &x,
//                         const Eigen::Ref<const Vxd> &u) {
//
//   assert(static_cast<std::size_t>(x.size()) == nx);
//   assert(static_cast<std::size_t>(u.size()) == nu);
//
//   std::vector<double> query{x.data(), x.data() + nx_effective};
//   double raw_d, d;
//   Vxd v(nx);
//   bool check_one =
//       (x - last_x).squaredNorm() < 1e-8 && (last_raw_d - margin) > 0;
//   bool check_two = (last_raw_d - margin) > 0 &&
//                    (x - last_x).norm() < sec_factor * (last_raw_d -
//                    margin);
//
//   if (check_one || check_two) {
//     Jx.setZero();
//   } else {
//     auto out = cl->distanceWithFDiffGradient(
//         query, faraway_zero_gradient_bound, epsilon,
//         non_zero_flags.size() ? &non_zero_flags : nullptr);
//     raw_d = std::get<0>(out);
//     last_x = x;
//     last_raw_d = raw_d;
//     d = options_trajopt.collision_weight * (raw_d - margin);
//     auto grad = std::get<1>(out);
//     v = options_trajopt.collision_weight * Vxd::Map(grad.data(),
//     grad.size()); if (d <= 0) {
//       Jx.block(0, 0, 1, nx_effective) = v.transpose();
//     } else {
//       Jx.setZero();
//     }
//   }
//   Ju.setZero();
// };

// void Col_cost::calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                         const Eigen::Ref<const Vxd> &x) {
//
//   auto Ju = Eigen::MatrixXd(1, 1);
//   auto u = Vxd(1);
//   calcDiff(Jx, Ju, x, u);
// }

Control_cost::Control_cost(size_t nx, size_t nu, size_t nr,
                           const Vxd &t_u_weight, const Vxd &t_u_ref)
    : Cost(nx, nu, nr) {
  u_weight = t_u_weight;
  u_ref = t_u_ref;

  CHECK_EQ(static_cast<size_t>(u_weight.size()), nu, AT);
  CHECK_EQ(static_cast<size_t>(u_ref.size()), nu, AT);
  CHECK_EQ(nu, nr, AT);
  name = "control";
}

void Control_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                        const Eigen::Ref<const Vxd> &u) {
  (void)x;
  // check that r
  CHECK_EQ(static_cast<std::size_t>(r.size()), nr, AT);
  CHECK_EQ(static_cast<std::size_t>(x.size()), nx, AT);
  CHECK_EQ(static_cast<std::size_t>(u.size()), nu, AT);
  r = (u - u_ref).cwiseProduct(u_weight);
}

void Control_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                            Eigen::Ref<Eigen::VectorXd> Lu,
                            Eigen::Ref<Eigen::MatrixXd> Lxx,
                            Eigen::Ref<Eigen::MatrixXd> Luu,
                            Eigen::Ref<Eigen::MatrixXd> Lxu,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
  // CSTR_(u_weight);
  // CSTR_V(Lu);
  Lu += (u - u_ref).cwiseProduct(u_weight).cwiseProduct(u_weight);
  // CSTR_V(Lu);
  Luu.diagonal() += u_weight.cwiseProduct(u_weight);
}

// void Control_cost::calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                             Eigen::Ref<Eigen::MatrixXd> Ju,
//                             const Eigen::Ref<const Vxd> &x,
//                             const Eigen::Ref<const Vxd> &u) {
//
//   assert(static_cast<std::size_t>(x.size()) == nx);
//   assert(static_cast<std::size_t>(u.size()) == nu);
//
//   assert(static_cast<std::size_t>(Jx.rows()) == nr);
//   assert(static_cast<std::size_t>(Ju.rows()) == nr);
//   assert(static_cast<std::size_t>(Ju.cols()) == nu);
//   Ju = u_weight.asDiagonal();
//   Jx.setZero();
// }

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

State_bounds::State_bounds(size_t nx, size_t nu, size_t nr, const Vxd &ub,
                           const Vxd &weight)
    : Cost(nx, nu, nr), ub(ub), weight(weight) {
  name = "xbound";
  CHECK_EQ(weight.size(), ub.size(), AT);
  CHECK_EQ(nx, nr, AT);
  CHECK_EQ(static_cast<size_t>(weight.size()), nx, AT);
}

void State_bounds::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                        const Eigen::Ref<const Vxd> &u) {
  // std::cout << "checking bounds " << STR_V(x) << std::endl;
  check_input_calc(r, x, u);
  calc(r, x);
  // std::cout << "checking bounds " << STR_V(x) << std::endl;
}

void State_bounds::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x) {
  check_input_calc(r, x);
  CHECK_EQ(static_cast<std::size_t>(r.size()), nr, AT);
  CHECK_EQ(static_cast<std::size_t>(x.size()), nx, AT);
  r = ((x - ub).cwiseProduct(weight)).cwiseMax(0.);
}

void State_bounds::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                            Eigen::Ref<Eigen::MatrixXd> Lxx,
                            const Eigen::Ref<const Eigen::VectorXd> &x) {
  // Lx += (((x -
  // ub).cwiseProduct(weight)).cwiseMax(0.)).cwiseProduct(weight);

  check_input_calcDiff(Lx, Lxx, x);
  Eigen::Matrix<bool, Eigen::Dynamic, 1> result =
      (x - ub).cwiseProduct(weight).array() >= 0;

  Lx += (x - ub).cwiseProduct(weight).cwiseMax(0.).cwiseProduct(weight);
  Lxx.diagonal() +=
      (result.cast<double>()).cwiseProduct(weight).cwiseProduct(weight);
}

void State_bounds::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                            Eigen::Ref<Eigen::VectorXd> Lu,
                            Eigen::Ref<Eigen::MatrixXd> Lxx,
                            Eigen::Ref<Eigen::MatrixXd> Luu,
                            Eigen::Ref<Eigen::MatrixXd> Lxu,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
  calcDiff(Lx, Lxx, x);
}

Control_bounds::Control_bounds(size_t nx, size_t nu, size_t nr, const Vxd &ub,
                               const Vxd &weight)
    : Cost(nx, nu, nr), ub(ub), weight(weight) {
  name = "xbound";
  CHECK_EQ(weight.size(), ub.size(), AT);
  CHECK_EQ(nu, nr, AT);
  CHECK_EQ(static_cast<size_t>(weight.size()), nu, AT);
}

void Control_bounds::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                          const Eigen::Ref<const Vxd> &u) {
  check_input_calc(r, x, u);
  r = ((u - ub).cwiseProduct(weight)).cwiseMax(0.);
}

void Control_bounds::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x) {
  ERROR_WITH_INFO("should not be called");
}

void Control_bounds::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                              Eigen::Ref<Eigen::MatrixXd> Lxx,
                              const Eigen::Ref<const Eigen::VectorXd> &x) {

  ERROR_WITH_INFO("should not be called");
}

void Control_bounds::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                              Eigen::Ref<Eigen::VectorXd> Lu,
                              Eigen::Ref<Eigen::MatrixXd> Lxx,
                              Eigen::Ref<Eigen::MatrixXd> Luu,
                              Eigen::Ref<Eigen::MatrixXd> Lxu,
                              const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &u) {

  Eigen::Matrix<bool, Eigen::Dynamic, 1> result =
      (u - ub).cwiseProduct(weight).array() >= 0;

  Lu += (u - ub).cwiseProduct(weight).cwiseMax(0.).cwiseProduct(weight);
  Luu.diagonal() +=
      (result.cast<double>()).cwiseProduct(weight).cwiseProduct(weight);
}

// void State_bounds::calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                             const Eigen::Ref<const Vxd> &x) {
//
//   assert(static_cast<std::size_t>(Jx.rows()) == nr);
//   assert(static_cast<std::size_t>(Jx.cols()) == nx);
//
//   Eigen::Matrix<bool, Eigen::Dynamic, 1> result =
//       (x - ub).cwiseProduct(weight).array() >= 0;
//   // std::cout << " x " << x.format(FMT) << std::endl;
//   // std::cout << " ub " << ub.format(FMT) << std::endl;
//   // std::cout << " result " << result.cast<double>().format(FMT)
//   << std::endl; Jx.diagonal() = (result.cast<double>()).cwiseProduct(weight);
// }

State_cost::State_cost(size_t nx, size_t nu, size_t nr, const Vxd &x_weight,
                       const Vxd &ref)
    : Cost(nx, nu, nr), x_weight(x_weight), ref(ref) {
  name = "state";
  CHECK_EQ(static_cast<std::size_t>(x_weight.size()), nx, AT);
  CHECK_EQ(static_cast<std::size_t>(ref.size()), nx, AT);
}

void State_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x,
                      const Eigen::Ref<const Vxd> &u) {
  check_input_calc(r, x, u);
  calc(r, x);
}

void State_cost::calc(Eigen::Ref<Vxd> r, const Eigen::Ref<const Vxd> &x) {
  check_input_calc(r, x);
  r = (x - ref).cwiseProduct(x_weight);
}

void State_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                          Eigen::Ref<Eigen::VectorXd> Lu,
                          Eigen::Ref<Eigen::MatrixXd> Lxx,
                          Eigen::Ref<Eigen::MatrixXd> Luu,
                          Eigen::Ref<Eigen::MatrixXd> Lxu,
                          const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
  calcDiff(Lx, Lxx, x);
}

void State_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                          Eigen::Ref<Eigen::MatrixXd> Lxx,
                          const Eigen::Ref<const Eigen::VectorXd> &x) {
  check_input_calcDiff(Lx, Lxx, x);
  Lx += (x - ref).cwiseProduct(x_weight).cwiseProduct(x_weight);
  Lxx.diagonal() += x_weight.cwiseProduct(x_weight);
}

size_t
get_total_num_features(const std::vector<boost::shared_ptr<Cost>> &features) {
  return std::accumulate(features.begin(), features.end(), 0,
                         [](auto &a, auto &b) { return a + b->nr; });
}

ActionModelDyno::ActionModelDyno(ptr<Dynamics> dynamics,
                                 const std::vector<ptr<Cost>> &features)
    : Base(dynamics->state_croco, dynamics->nu,
           get_total_num_features(features)),
      dynamics(dynamics), features(features), nx(dynamics->nx),
      nu(dynamics->nu), nr(get_total_num_features(features)), Jx(nr, nx),
      Ju(nr, nu) {
  Jx.setZero();
  Ju.setZero();
}

void ActionModelDyno::calc(const boost::shared_ptr<ActionDataAbstract> &data,
                           const Eigen::Ref<const VectorXs> &x,
                           const Eigen::Ref<const VectorXs> &u) {
  Data *d = static_cast<Data *>(data.get());
  d->xnext.setZero();
  dynamics->calc(d->xnext, x, u);

  int index = 0;

  d->cost = 0.;
  d->r.setZero();

  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    size_t &_nr = feat->nr;
    feat->calc(d->r.segment(index, _nr), x, u);

    if (feat->cost_type == CostTYPE::least_squares) {
      d->cost +=
          Scalar(0.5) * d->r.segment(index, _nr).dot(d->r.segment(index, _nr));
    } else if (feat->cost_type == CostTYPE::linear) {
      d->cost += d->r.segment(index, _nr).sum();
    }
    index += _nr;
  }
}

void ActionModelDyno::calcDiff(
    const boost::shared_ptr<ActionDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x, const Eigen::Ref<const VectorXs> &u) {
  Data *d = static_cast<Data *>(data.get());
  d->Fx.setZero();
  d->Fu.setZero();
  dynamics->calcDiff(d->Fx, d->Fu, x, u);

  // create a matrix for the Jacobians
  d->Lx.setZero();
  d->Lu.setZero();
  d->Lxx.setZero();
  d->Luu.setZero();
  d->Lxu.setZero();

  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    feat->calcDiff(d->Lx, d->Lu, d->Lxx, d->Luu, d->Lxu, x, u);
  }
}

void ActionModelDyno::calc(const boost::shared_ptr<ActionDataAbstract> &data,
                           const Eigen::Ref<const VectorXs> &x) {
  Data *d = static_cast<Data *>(data.get());
  d->r.setZero();

  int index = 0;

  d->cost = 0;
  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    size_t &_nr = feat->nr;
    Eigen::Ref<Eigen::VectorXd> r = d->r.segment(index, _nr);
    feat->calc(r, x);
    if (feat->cost_type == CostTYPE::least_squares) {
      d->cost += Scalar(0.5) * r.dot(r);
    } else if (feat->cost_type == CostTYPE::linear) {
      d->cost += r.sum();
    }
    index += _nr;
  }
}

void ActionModelDyno::calcDiff(
    const boost::shared_ptr<ActionDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x) {
  Data *d = static_cast<Data *>(data.get());

  d->Lx.setZero();
  d->Lxx.setZero();

  // size_t index = 0;
  // Jx.setZero();
  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    // size_t &_nr = feat->nr;

    // Eigen::Ref<Vxd> r = d->r.segment(index, _nr);
    // Eigen::Ref<Eigen::MatrixXd> jx = Jx.block(index, 0, _nr, nx);

    feat->calcDiff(d->Lx, d->Lxx, x);

    // if (feat->cost_type == CostTYPE::least_squares) {
    //   data->Lx.noalias() += r.transpose() * jx;
    //   data->Lxx.noalias() += jx.transpose() * jx;
    // } else if (feat->cost_type == CostTYPE::linear) {
    //   data->Lx.noalias() += jx.colwise().sum();
    // }
    // index += _nr;
  }
}

boost::shared_ptr<crocoddyl::ActionDataAbstract> ActionModelDyno::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}
bool ActionModelDyno::checkData(
    const boost::shared_ptr<ActionDataAbstract> &data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (d != NULL) {
    return true;
  } else {
    return false;
  }
}

void ActionModelDyno::print(std::ostream &os) const {
  os << "wrapper" << std::endl;
}

// NEW REFACTOR
//
//

int get_total_num_features_mode(const std::vector<ptr<Cost>> &features,
                                Control_Mode mode) {

  int ff = get_total_num_features(features);
  int additional_features = 0;
  if (mode == Control_Mode::free_time) {
    additional_features += 1; // regularization on dt
  }
  if (mode == Control_Mode::free_time_linear) {
    additional_features += 2; //
  }
  if (mode == Control_Mode::contour) {
    // additional features
  }
  return ff + additional_features;
}

int get_additional_nu(Control_Mode control_mode) {

  switch (control_mode) {

  case Control_Mode::free_time_linear_first:
    NOT_IMPLEMENTED

  case Control_Mode::free_time:
    return 1;

  case Control_Mode::default_mode:
    return 0;

  case Control_Mode::contour:
    return 1;

  case Control_Mode::free_time_linear:
    return 1;
  default:
    NOT_IMPLEMENTED;
  }
}

boost::shared_ptr<StateCrocoDyno>
mk_state_croco(std::shared_ptr<dynobench::Model_robot> model_robot,
               Control_Mode control_mode){};

int get_nx(std::shared_ptr<dynobench::Model_robot> model_robot,
           Control_Mode control_mode){};

ActionModelDynov2::ActionModelDynov2(
    std::shared_ptr<dynobench::Model_robot> model_robot,
    const std::vector<ptr<Cost>> &features, Control_Mode control_mode)
    : Base(mk_state_croco(model_robot, control_mode),
           model_robot->nu + get_additional_nu(control_mode),
           get_total_num_features_mode(features, control_mode)),
      model_robot(model_robot), features(features) {
  nu = get_nu();
  nx = get_state()->get_nq(); //
  std::cout << "nx is " << nx << std::endl;
  // nu(), nr(get_total_num_features(features)),

  nx_orig = model_robot->nx;
  nu_orig = model_robot->nu;

  Jx.resize(nr, nx);
  Ju.resize(nr, nu);

  Jx.setZero();
  Ju.setZero();
}

void __calc_diff(std::shared_ptr<dynobench::Model_robot> robot_model,
                 Control_Mode control_mode, Eigen::Ref<Eigen::MatrixXd> Fx,
                 Eigen::Ref<Eigen::MatrixXd> Fu,
                 const Eigen::Ref<const Eigen::VectorXd> &x,
                 const Eigen::Ref<const Eigen::VectorXd> &u) {

  double dt = robot_model->ref_dt;
  CHECK_GE(dt, 0, AT);
  size_t _nx = robot_model->nx;
  size_t _nu = robot_model->nu;
  size_t nx = x.size();
  size_t nu = u.size();

  Eigen::VectorXd __v(nx); // TODO: remove memory allocation
  __v.setZero();

  Fx.setZero(); // TODO: is this necessary?
  Fu.setZero();

  if (control_mode == Control_Mode::default_mode) {
    robot_model->stepDiff(Fx, Fu, x, u, dt);
  } else if (control_mode == Control_Mode::free_time_linear) {
    CHECK_GE(u(robot_model->nu), 0, AT);
    CHECK_EQ(static_cast<size_t>(__v.size()), _nx, AT);
    if (!startsWith(robot_model->name, "quad3d")) {
      robot_model->stepDiff_with_v(Fx.block(0, 0, _nx, _nx),
                                   Fu.block(0, 0, _nx, _nu), __v, x.head(_nx),
                                   u.head(_nu), dt * u(_nu));
      Fu.block(0, _nu, _nx, 1) = __v * dt;
      Fu(_nx, _nu) = 1.;
      CHECK_EQ(static_cast<size_t>(__v.size()), _nx, AT);
    } else {
      robot_model->stepDiff(Fx.block(0, 0, _nx, _nx), Fu.block(0, 0, _nx, _nu),
                            x.head(_nx), u.head(_nu), dt * u(_nu));
      Eigen::VectorXd dd = derivate_wrt_time(*robot_model, x, u, dt);
      Fu.block(0, _nu, _nx, 1) = robot_model->ref_dt * dd;
      Fu(_nx, _nu) = 1.;
    }
  } else if (control_mode == Control_Mode::free_time) {
    if (!startsWith(robot_model->name, "quad3d")) {
      robot_model->stepDiff_with_v(Fx, Fu.block(0, 0, _nx, _nu), __v, x,
                                   u.head(_nu), dt * u(_nu));
      CHECK_EQ(static_cast<size_t>(__v.size()), _nx, AT);
      Fu.col(_nu) = __v * dt;
    } else {
      robot_model->stepDiff(Fx.block(0, 0, _nx, _nx), Fu.block(0, 0, _nx, _nu),
                            x.head(_nx), u.head(_nu), dt * u(_nu));
      Eigen::VectorXd dd = derivate_wrt_time(*robot_model, x, u, dt);
      Fu.col(_nu) = robot_model->ref_dt * dd;
    }
  } else if (control_mode == Control_Mode::contour) {
    robot_model->stepDiff(Fx.block(0, 0, _nx, _nx), Fu.block(0, 0, _nx, _nu),
                          x.head(_nx), u.head(_nu), dt);

    Fx(nx - 1, nx - 1) = 1.0;
    Fu(nx - 1, nu - 1) = 1.0;
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

void __calc(std::shared_ptr<dynobench::Model_robot> robot_model,
            Control_Mode control_mode, Eigen::Ref<Eigen::VectorXd> xnext,
            const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &u) {
  CHECK_GE(robot_model->ref_dt, 0, AT);
  const size_t &_nx = robot_model->nx;
  const size_t &_nu = robot_model->nu;

  const size_t &nx = x.size();
  const size_t &nu = u.size();

  if (control_mode == Control_Mode::default_mode) {
    robot_model->step(xnext, x, u, robot_model->ref_dt);
  } else if (control_mode == Control_Mode::free_time) {
    // CHECK_GEQ(u(robot_model->nu), 0., AT);
    WARN_GEQ(u(robot_model->nu), 0, "time cannot be negative!");
    robot_model->step(xnext, x, u.head(_nu), robot_model->ref_dt * u(_nu));
  } else if (control_mode == Control_Mode::free_time_linear) {
    WARN_GEQ(u(robot_model->nu), 0, "time cannot be negative!");
    robot_model->step(xnext.head(_nx), x.head(_nx), u.head(_nu),
                      robot_model->ref_dt * u(_nu));
    xnext(_nx) = u(_nu);
  } else if (control_mode == Control_Mode::contour) {
    robot_model->step(xnext.head(_nx), x.head(_nx), u.head(_nu),
                      robot_model->ref_dt);
    xnext(nx - 1) = x(nx - 1) + u(nu - 1);
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

void ActionModelDynov2::calc(const boost::shared_ptr<ActionDataAbstract> &data,
                             const Eigen::Ref<const VectorXs> &x,
                             const Eigen::Ref<const VectorXs> &u) {
  Data *d = static_cast<Data *>(data.get());
  d->xnext.setZero();
  // calculate dynamics
  // dynamics->calc(d->xnext, x, u);

  __calc(model_robot, control_mode, d->xnext, x, u);

  int index = 0;

  d->cost = 0.;
  d->r.setZero();

  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    size_t &_nr = feat->nr;
    feat->calc(d->r.segment(index, _nr), x.head(nx_orig), u.head(nu_orig));

    if (feat->cost_type == CostTYPE::least_squares) {
      d->cost +=
          Scalar(0.5) * d->r.segment(index, _nr).dot(d->r.segment(index, _nr));
    } else if (feat->cost_type == CostTYPE::linear) {
      d->cost += d->r.segment(index, _nr).sum();
    }
    index += _nr;
  }
  // add additional features based on the control mode

  if (control_mode == Control_Mode::free_time) {
    double u_ref = .7;
    double k = 1.;
    d->r(index) = k * (u(nu) - u_ref);
    index++;
  }
  if (control_mode == Control_Mode::free_time_linear) {
    double k_linear = 1.;
    double k_reg = 1.;
    d->r(index) = k_linear * u(nu - 1);
    index++;
    d->r(index) = k_reg * (u(nu - 1) - x(nx - 1));
    index++;
  }
  if (control_mode == Control_Mode::contour) {
    NOT_IMPLEMENTED;
  }

  assert(index == data->r.size());
}

void ActionModelDynov2::calcDiff(
    const boost::shared_ptr<ActionDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x, const Eigen::Ref<const VectorXs> &u) {
  Data *d = static_cast<Data *>(data.get());
  d->Fx.setZero();
  d->Fu.setZero();

  __calc_diff(model_robot, control_mode, d->Fx, d->Fu, x, u);

  // create a matrix for the Jacobians
  d->Lx.setZero();
  d->Lu.setZero();
  d->Lxx.setZero();
  d->Luu.setZero();
  d->Lxu.setZero();

  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    feat->calcDiff(d->Lx, d->Lu, d->Lxx, d->Luu, d->Lxu, x.head(nx_orig),
                   u.head(nu_orig));
  }
  // add additional features based on the control mode

  if (control_mode == Control_Mode::free_time) {
    double k = 1.;
    double u_ref = .7;
    d->Lu(nu - 1) = k * k * (u(nu - 1) - u_ref);
    d->Luu(nu - 1, nu - 1) = k * k;
  } else if (control_mode == Control_Mode::free_time_linear) {
    double k_linear = 1.;
    double k_reg = 1.;
    d->Lu(nx - 1) = k_linear * u(nu - 1);
    d->Lx(nx - 1) = -k_reg * k_reg * (u(nu - 1) - x(nx - 1));
    d->Lx(nx - 1) = k_reg * k_reg * (u(nu - 1) - x(nx - 1));
    d->Lxu(nx - 1, nu - 1) = k_reg * k_reg;
  } else if (control_mode == Control_Mode::contour) {
    NOT_IMPLEMENTED;
  }
}

void ActionModelDynov2::calc(const boost::shared_ptr<ActionDataAbstract> &data,
                             const Eigen::Ref<const VectorXs> &x) {
  Data *d = static_cast<Data *>(data.get());
  d->r.setZero();

  int index = 0;

  d->cost = 0;
  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    size_t &_nr = feat->nr;
    Eigen::Ref<Eigen::VectorXd> r = d->r.segment(index, _nr);
    feat->calc(r, x);
    if (feat->cost_type == CostTYPE::least_squares) {
      d->cost += Scalar(0.5) * r.dot(r);
    } else if (feat->cost_type == CostTYPE::linear) {
      d->cost += r.sum();
    }
    index += _nr;
  }
  // TODO: i should copy here...
}

void ActionModelDynov2::calcDiff(
    const boost::shared_ptr<ActionDataAbstract> &data,
    const Eigen::Ref<const VectorXs> &x) {
  Data *d = static_cast<Data *>(data.get());

  d->Lx.setZero();
  d->Lxx.setZero();

  // size_t index = 0;
  // Jx.setZero();
  for (size_t i = 0; i < features.size(); i++) {
    auto &feat = features.at(i);
    feat->calcDiff(d->Lx, d->Lxx, x);
  }
  // TODO: i should copy here...
}

boost::shared_ptr<crocoddyl::ActionDataAbstract>
ActionModelDynov2::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}
bool ActionModelDynov2::checkData(
    const boost::shared_ptr<ActionDataAbstract> &data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (d != NULL) {
    return true;
  } else {
    return false;
  }
}

void ActionModelDynov2::print(std::ostream &os) const {
  os << "wrapper" << std::endl;
}
//
//

void print_data(
    boost::shared_ptr<crocoddyl::ActionDataAbstractTpl<double>> data) {
  std::cout << "***\n";
  std::cout << "xnext\n" << data->xnext << std::endl;
  std::cout << "Fx:\n" << data->Fx << std::endl;
  std::cout << "Fu:\n" << data->Fu << std::endl;
  std::cout << "r:" << data->r.transpose() << std::endl;
  std::cout << "cost:" << data->cost << std::endl;
  std::cout << "Lx:" << data->Lx.transpose() << std::endl;
  std::cout << "Lu:" << data->Lu.transpose() << std::endl;
  std::cout << "Lxx:\n" << data->Lxx << std::endl;
  std::cout << "Luu:\n" << data->Luu << std::endl;
  std::cout << "Lxu:\n" << data->Lxu << std::endl;
  std::cout << "***\n";
}

void check_dyn(boost::shared_ptr<Dynamics> dyn, double eps, Vxd x, Vxd u,
               double margin_rate) {
  size_t nx = dyn->nx;
  size_t nu = dyn->nu;

  Eigen::MatrixXd Fx(nx, nx);
  Eigen::MatrixXd Fu(nx, nu);
  Fx.setZero();
  Fu.setZero();

  if (!x.size()) {
    x.resize(nx);
    x.setRandom();
  }
  if (!u.size()) {
    u.resize(nu);
    u.setRandom();
  }

  // x.setZero();
  // u.setZero();

  // x.segment(3,4).normalize();
  // x.segment(3, 4) << 0, 0, 0, 1;

  dyn->calcDiff(Fx, Fu, x, u);

  // compute the same using finite diff

  Eigen::MatrixXd FxD(nx, nx);
  FxD.setZero();
  Eigen::MatrixXd FuD(nx, nu);
  FuD.setZero();

  Vxd xnext(nx);
  xnext.setZero();
  dyn->calc(xnext, x, u);
  std::cout << "xnext\n" << std::endl;
  std::cout << xnext.format(dynobench::FMT) << std::endl;
  for (size_t i = 0; i < nx; i++) {
    Eigen::MatrixXd xe;
    xe = x;
    xe(i) += eps;
    Vxd xnexte(nx);
    xnexte.setZero();
    dyn->calc(xnexte, xe, u);
    auto df = (xnexte - xnext) / eps;
    FxD.col(i) = df;
  }

  for (size_t i = 0; i < nu; i++) {
    Eigen::MatrixXd ue;
    ue = u;
    ue(i) += eps;
    Vxd xnexte(nx);
    xnexte.setZero();
    dyn->calc(xnexte, x, ue);
    auto df = (xnexte - xnext) / eps;
    FuD.col(i) = df;
  }
  bool verbose = true;
  if (verbose) {
    std::cout << "Analytical " << std::endl;
    std::cout << "Fx\n" << Fx << std::endl;
    std::cout << "Fu\n" << Fu << std::endl;
    std::cout << "Finite Diff " << std::endl;
    std::cout << "Fx\n" << FxD << std::endl;
    std::cout << "Fu\n" << FuD << std::endl;
  }

  bool check1 = (Fx - FxD).cwiseAbs().maxCoeff() < margin_rate * eps;
  bool check2 = (Fu - FuD).cwiseAbs().maxCoeff() < margin_rate * eps;

  if (!check1) {
    std::cout << "Fx" << std::endl;
    std::cout << Fx << std::endl;
    std::cout << "FxD" << std::endl;
    std::cout << FxD << std::endl;
    std::cout << "Fx - FxD" << std::endl;
    std::cout << Fx - FxD << std::endl;
    CHECK(((Fx - FxD).cwiseAbs().maxCoeff() < margin_rate * eps),
          "max_error is " + std::to_string((Fx - FxD).cwiseAbs().maxCoeff()) +
              ":" AT);
  }

  if (!check2) {
    std::cout << "Fu" << std::endl;
    std::cout << Fu << std::endl;
    std::cout << "FuD" << std::endl;
    std::cout << FuD << std::endl;
    std::cout << "Fu - FuD" << std::endl;
    std::cout << Fu - FuD << std::endl;
    CHECK(((Fu - FuD).cwiseAbs().maxCoeff() < margin_rate * eps), AT);
  }
}

std::vector<ReportCost>
get_report(ptr<ActionModelDyno> p,
           std::function<void(ptr<Cost>, Eigen::Ref<Vxd>)> fun) {
  std::vector<ReportCost> reports;
  for (size_t j = 0; j < p->features.size(); j++) {
    ReportCost report;
    auto &f = p->features.at(j);
    Vxd r(f->nr);
    fun(f, r);
    report.type = CostTYPE::least_squares;
    report.name = f->get_name();
    report.r = r;
    if (f->cost_type == CostTYPE::least_squares) {
      report.cost = .5 * r.dot(r);
    } else if (f->cost_type == CostTYPE::linear) {
      report.cost = r.sum();
    }
    reports.push_back(report);
  }
  return reports;
}

// check gradient

Quaternion_cost::Quaternion_cost(size_t nx, size_t nu) : Cost(nx, nu, 1) {
  name = "quaterion norm";
};

void Quaternion_cost::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x) {
  CHECK_GEQ(k_quat, 0., AT);

  check_input_calc(r, x);

  Eigen::Vector4d q = x.segment<4>(3);
  r(0) = k_quat * (q.squaredNorm() - 1);
}

void Quaternion_cost::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calc(r, x, u);
  calc(r, x);
}

void Quaternion_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               const Eigen::Ref<const Eigen::VectorXd> &x) {
  Eigen::Vector4d q = x.segment<4>(3);
  Lx.segment<4>(3) += k_quat * (q.squaredNorm() - 1.) * 2. * k_quat * q;
  Lxx.block<4, 4>(3, 3) += (2. * 2. * k_quat * k_quat) * q * q.transpose();
}

void Quaternion_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::VectorXd> Lu,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               Eigen::Ref<Eigen::MatrixXd> Luu,
                               Eigen::Ref<Eigen::MatrixXd> Lxu,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {
  (void)Lu;
  (void)Luu;
  (void)Luu;
  (void)Lxu;
  (void)u;

  calcDiff(Lx, Lxx, x);
}

Quad3d_acceleration_cost::Quad3d_acceleration_cost(
    const std::shared_ptr<dynobench::Model_robot> &model_robot)
    : Cost(13, 4, 6), model(model_robot) {
  name = "accel_quad3d";

  acc_u.setZero();
  acc_x.setZero();
  Jv_x.setZero();
  Jv_u.setZero();
  f.setZero();
}

void Quad3d_acceleration_cost::calc(
    Eigen::Ref<Eigen::VectorXd> r, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  model->calcV(f, x, u);
  acc = f.tail<6>();
  r = k_acc * acc;
}

void Quad3d_acceleration_cost::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
    Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
    Eigen::Ref<Eigen::MatrixXd> Lxu, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  model->calcV(f, x, u);
  acc = f.tail<6>();
  model->calcDiffV(Jv_x, Jv_u, x, u);

  // CSTR_V(acc);
  // std::cout << "Jv_x\n" << Jv_x << std::endl;
  // std::cout << "Jv_u\n" << Jv_u <<  std::endl;

  CHECK_EQ(f.size(), 12, AT);
  CHECK_EQ(acc.size(), 6, AT);
  CHECK_EQ(Jv_x.cols(), 13, AT);
  CHECK_EQ(Jv_u.cols(), 4, AT);

  CHECK_EQ(Jv_x.rows(), 12, AT);
  CHECK_EQ(Jv_u.rows(), 12, AT);

  acc_x = Jv_x.block<6, 13>(6, 0);
  acc_u = Jv_u.block<6, 4>(6, 0);

  double k_acc_sq = k_acc * k_acc;
  Lx += k_acc_sq * acc.transpose() * acc_x;
  // CSTR_(Lu);
  Lu += k_acc_sq * acc.transpose() * acc_u;
  // CSTR_(Lu);

  Lxx += k_acc_sq * acc_x.transpose() * acc_x;
  Luu += k_acc_sq * acc_u.transpose() * acc_u;
  Lxu += k_acc_sq * acc_x.transpose() * acc_u;
}

Acceleration_cost_acrobot::Acceleration_cost_acrobot(size_t nx, size_t nu)
    : Cost(nx, nu, 2) {
  name = "acceleration";

  acc_u.setZero();
  acc_x.setZero();
  Jv_x.setZero();
  Jv_u.setZero();
  f.setZero();
}

// 6 Payload
// 6 Cable
// 6
// 7
// 7
// TOTAL:  32
// nx (point):
// 6 payload: (pos, vel)
// 6*num_robots (qc_0, wc_0, qc_1, wc_1, ...,qc_{n-1}, wc_{n-1}), cable vector
// and omega 7* num_robots (q, w): (q_0, w_0, ..., q_{n-1}, w_{n-1}), quat and
// omega
Payload_n_acceleration_cost::Payload_n_acceleration_cost(
    const std::shared_ptr<dynobench::Model_robot> &model_robot, double k_acc)
    : Cost(model_robot->nx, model_robot->nu, model_robot->nx), k_acc(k_acc),
      model(model_robot) {

  name = "acceleration";
  int nx = model_robot->nx;
  int nu = model_robot->nu;
  int num_robots = int((nx - 6) / 13);
  f.resize(nx);
  f.setZero();

  acc_u.resize(nx, nu);
  acc_x.resize(nx, nx);
  Jv_u.resize(nx, nu);
  Jv_x.resize(nx, nx);

  acc_u.setZero();
  acc_x.setZero();
  Jv_u.setZero();
  Jv_x.setZero();

  // TODO@ KHALED -> we need this generic!!
  selector.resize(nx);
  selector.setZero();
  // lets put only the entries that are about acceleration
  // selector for the accelerations: a_payload, wc_dot, w_dot (angular acc cable
  // and uav)
  selector.segment(3, 3).setOnes();
  for (int i = 0; i < num_robots; ++i) {
    selector.segment(6 + 6 * i + 3, 3).setOnes();
    selector.segment(6 + 6 * num_robots + 7 * i + 4, 3).setOnes();
  }

  // selector.segment(6 + 3, 3).setOnes();
  // selector.segment(2 * 6 + 3, 3).setOnes();

  // selector.segment(3 * 6 + 4, 3).setOnes();
  // selector.segment(3 * 6 + 7 + 4, 3).setOnes();
}

void Payload_n_acceleration_cost::calc(
    Eigen::Ref<Eigen::VectorXd> r, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  // CSTR_V(x);
  // CSTR_V(u);
  // CSTR_V(f);
  assert(model);
  model->calcV(f, x.head(model->nx), u.head(model->nu));
  // CSTR_V(f);
  // CSTR_V(selector);
  r = k_acc * f.cwiseProduct(selector);
  // I have to choose some entries...: DONE
}

void Payload_n_acceleration_cost::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
    Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
    Eigen::Ref<Eigen::MatrixXd> Lxu, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(model);
  model->calcV(f, x.head(model->nx), u.head(model->nu));

  model->calcDiffV(Jv_x, Jv_u, x.head(model->nx), u.head(model->nu));

  // set to zeros some of the entries

  acc_x = Jv_x.array().colwise() * selector.array();
  acc_u = Jv_u.array().colwise() * selector.array();

  const double k_acc2 = k_acc * k_acc;
  Lx.head(model->nx) += k_acc2 * f.cwiseProduct(selector).transpose() * acc_x;
  Lu.head(model->nu) += k_acc2 * f.cwiseProduct(selector).transpose() * acc_u;
  Lxx.block(0, 0, model->nx, model->nx) += k_acc2 * acc_x.transpose() * acc_x;
  Luu.block(0, 0, model->nu, model->nu) += k_acc2 * acc_u.transpose() * acc_u;
  Lxu.block(0, 0, model->nx, model->nu) += k_acc2 * acc_x.transpose() * acc_u;
}

void Acceleration_cost_acrobot::calc(
    Eigen::Ref<Eigen::VectorXd> r, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  model.calcV(f, x, u);
  acc = f.tail<2>();
  r = k_acc * acc;
}
void Acceleration_cost_acrobot::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
    Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
    Eigen::Ref<Eigen::MatrixXd> Lxu, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  model.calcV(f, x, u);
  acc = f.tail<2>();

  model.calcDiffV(Jv_x, Jv_u, x, u);

  acc_x = Jv_x.block<2, 4>(2, 0);
  acc_u = Jv_u.block<2, 1>(2, 0);

  const double k_acc2 = k_acc * k_acc;
  Lx += k_acc2 * acc.transpose() * acc_x;
  Lu += k_acc2 * acc.transpose() * acc_u;

  Lxx += k_acc2 * acc_x.transpose() * acc_x;
  Luu += k_acc2 * acc_u.transpose() * acc_u;
  Lxu += k_acc2 * acc_x.transpose() * acc_u;
}

Acceleration_cost_quad2d::Acceleration_cost_quad2d(
    const std::shared_ptr<dynobench::Model_robot> &model_robot, size_t nx,
    size_t nu)
    : Cost(nx, nu, 3), model(model_robot) {
  name = "acceleration";
  acc_u.setZero();
  acc_x.setZero();
  Jv_x.setZero();
  Jv_u.setZero();
  f.setZero();
}

void Acceleration_cost_quad2d::calc(
    Eigen::Ref<Eigen::VectorXd> r, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  model->calcV(f, x, u);
  acc = f.tail<3>();
  r = k_acc * acc;
}

void Acceleration_cost_quad2d::calcDiff(
    Eigen::Ref<Eigen::VectorXd> Lx, Eigen::Ref<Eigen::VectorXd> Lu,
    Eigen::Ref<Eigen::MatrixXd> Lxx, Eigen::Ref<Eigen::MatrixXd> Luu,
    Eigen::Ref<Eigen::MatrixXd> Lxu, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  model->calcV(f, x, u);
  acc = f.tail<3>();

  model->calcDiffV(Jv_x, Jv_u, x, u);

  acc_x = Jv_x.block<3, 6>(3, 0);
  acc_u = Jv_u.block<3, 2>(3, 0);

  const double k_acc2 = k_acc * k_acc;
  Lx += k_acc2 * acc.transpose() * acc_x;
  Lu += k_acc2 * acc.transpose() * acc_u;

  Lxx += k_acc2 * acc_x.transpose() * acc_x;
  Luu += k_acc2 * acc_u.transpose() * acc_u;
  Lxu += k_acc2 * acc_x.transpose() * acc_u;
}

Dynamics::Dynamics(std::shared_ptr<dynobench::Model_robot> robot_model,
                   const Control_Mode &control_mode,
                   const std::map<std::string, double> &params)
    : robot_model(robot_model), control_mode(control_mode), params(params) {
  CHECK(robot_model, AT);
  u_lb = robot_model->get_u_lb();
  u_ub = robot_model->get_u_ub();
  x_lb = robot_model->get_x_lb();
  x_ub = robot_model->get_x_ub();
  u_ref = robot_model->get_u_ref();
  u_weight = robot_model->u_weight;
  x_weightb = robot_model->x_weightb;

  nx = robot_model->nx;
  nu = robot_model->nu;
  dt = robot_model->ref_dt;

  if (control_mode == Control_Mode::default_mode) {
    state_croco = boost::make_shared<StateCrocoDyno>(robot_model->state);
  } else if (control_mode == Control_Mode::free_time) {
    state_croco = boost::make_shared<StateCrocoDyno>(robot_model->state);
    nu++;
    __v.resize(nx);
  } else if (control_mode == Control_Mode::free_time_linear) {

    __v.resize(nx);
    nu++;
    nx++;

    state_croco = boost::make_shared<StateCrocoDyno>(
        std::make_shared<dynobench::CompoundState2>(
            robot_model->state, std::make_shared<dynobench::Rn>(1)));

  }

  else if (control_mode == Control_Mode::contour) {
    nu++;
    nx++;

    state_croco = boost::make_shared<StateCrocoDyno>(
        std::make_shared<dynobench::CompoundState2>(
            robot_model->state, std::make_shared<dynobench::Rn>(1)));
  }

  CHECK_EQ(state_croco->get_nx(), nx, AT);

  update_state_and_control();
}

void Dynamics::calc(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u) {
  CHECK_GE(dt, 0, AT);
  const size_t &_nx = robot_model->nx;
  const size_t &_nu = robot_model->nu;

  const size_t &nx = x.size();
  const size_t &nu = u.size();

  if (control_mode == Control_Mode::default_mode) {
    robot_model->step(xnext, x, u, dt);
  } else if (control_mode == Control_Mode::free_time) {
    // CHECK_GEQ(u(robot_model->nu), 0., AT);
    WARN_GEQ(u(robot_model->nu), 0, "time cannot be negative!");
    robot_model->step(xnext, x, u.head(_nu), dt * u(_nu));
  } else if (control_mode == Control_Mode::free_time_linear) {
    WARN_GEQ(u(robot_model->nu), 0, "time cannot be negative!");
    robot_model->step(xnext.head(_nx), x.head(_nx), u.head(_nu), dt * u(_nu));
    xnext(_nx) = u(_nu);
  } else if (control_mode == Control_Mode::contour) {
    robot_model->step(xnext.head(_nx), x.head(_nx), u.head(_nu), dt);
    xnext(nx - 1) = x(nx - 1) + u(nu - 1);
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

void Dynamics::calcDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                        Eigen::Ref<Eigen::MatrixXd> Fu,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u) {
  CHECK_GE(dt, 0, AT);
  size_t _nx = robot_model->nx;
  size_t _nu = robot_model->nu;
  size_t nx = x.size();
  size_t nu = u.size();

  Fx.setZero(); // TODO: is this necessary?
  Fu.setZero();

  if (control_mode == Control_Mode::default_mode) {
    robot_model->stepDiff(Fx, Fu, x, u, dt);
  } else if (control_mode == Control_Mode::free_time_linear) {
    CHECK_GE(u(robot_model->nu), 0, AT);
    CHECK_EQ(static_cast<size_t>(__v.size()), _nx, AT);
    if (!startsWith(robot_model->name, "quad3d")) {
      robot_model->stepDiff_with_v(Fx.block(0, 0, _nx, _nx),
                                   Fu.block(0, 0, _nx, _nu), __v, x.head(_nx),
                                   u.head(_nu), dt * u(_nu));
      Fu.block(0, _nu, _nx, 1) = __v * dt;
      Fu(_nx, _nu) = 1.;
      CHECK_EQ(static_cast<size_t>(__v.size()), _nx, AT);
    } else {
      robot_model->stepDiff(Fx.block(0, 0, _nx, _nx), Fu.block(0, 0, _nx, _nu),
                            x.head(_nx), u.head(_nu), dt * u(_nu));
      Eigen::VectorXd dd = derivate_wrt_time(*robot_model, x, u, dt);
      Fu.block(0, _nu, _nx, 1) = robot_model->ref_dt * dd;
      Fu(_nx, _nu) = 1.;
    }
  } else if (control_mode == Control_Mode::free_time) {
    if (!startsWith(robot_model->name, "quad3d")) {
      robot_model->stepDiff_with_v(Fx, Fu.block(0, 0, _nx, _nu), __v, x,
                                   u.head(_nu), dt * u(_nu));
      CHECK_EQ(static_cast<size_t>(__v.size()), _nx, AT);
      Fu.col(_nu) = __v * dt;
    } else {
      robot_model->stepDiff(Fx.block(0, 0, _nx, _nx), Fu.block(0, 0, _nx, _nu),
                            x.head(_nx), u.head(_nu), dt * u(_nu));
      Eigen::VectorXd dd = derivate_wrt_time(*robot_model, x, u, dt);
      Fu.col(_nu) = robot_model->ref_dt * dd;
    }
  } else if (control_mode == Control_Mode::contour) {
    robot_model->stepDiff(Fx.block(0, 0, _nx, _nx), Fu.block(0, 0, _nx, _nu),
                          x.head(_nx), u.head(_nu), dt);

    Fx(nx - 1, nx - 1) = 1.0;
    Fu(nx - 1, nu - 1) = 1.0;
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

// void Quaternion_cost::calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                                const Eigen::Ref<const Eigen::VectorXd> &x)
//                                {
//
//   Eigen::Vector4d q = x.segment<4>(3);
//   Jx.row(0).segment<4>(3) = (2. * k_quat) * q;
//
//
//
// }
//
// void Quaternion_cost::calcDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
//                                Eigen::Ref<Eigen::MatrixXd> Ju,
//                                const Eigen::Ref<const Eigen::VectorXd> &x,
//                                const Eigen::Ref<const Eigen::VectorXd> &u)
//                                {
//   calcDiff(Jx, x);
// }

State_cost_model::State_cost_model(
    const std::shared_ptr<dynobench::Model_robot> &model_robot, size_t nx,
    size_t nu, const Eigen::VectorXd &x_weight, const Eigen::VectorXd &ref)
    : Cost(nx, nu, model_robot->state->ndx), ref(ref), x_weight(x_weight),
      nx_effective(model_robot->state->nx), model_robot(model_robot) {
  // x_weight_mat
  CHECK_EQ(ref.size(), x_weight.size(), AT);
  CHECK_EQ(nx_effective, static_cast<size_t>(x_weight.size()), AT);
  x_weight_mat = x_weight.replicate(1, ref.size());

  x_weight_sq = x_weight.cwiseAbs2();

  Jx0.resize(x_weight.size(), x_weight.size());
  Jx1.resize(x_weight.size(), x_weight.size());
  Jx1_w.resize(x_weight.size(), x_weight.size());
  __r.resize(x_weight.size());

  Jx0.setZero();
  Jx1.setZero();
  Jx1_w.setZero();
  Jx1_w.setZero();
  __r.setZero();

  CSTR_(nr);
  CSTR_(nu);
  CSTR_(nx);

  name = "state_cost_model";
}

void State_cost_model::calc(Eigen::Ref<Eigen::VectorXd> r,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {
  (void)u;
  check_input_calc(r, x, u);
  calc(r, x);
}

void State_cost_model::calc(Eigen::Ref<Eigen::VectorXd> r,
                            const Eigen::Ref<const Eigen::VectorXd> &x) {
  check_input_calc(r, x);
  // CSTR_V(r)
  // CSTR_V(ref)
  // CSTR_V(x)
  // CSTR_(nx_effective)
  model_robot->state_diff(r, ref, x.head(nx_effective));
  r.array() *= x_weight.array();
}

void State_cost_model::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                                Eigen::Ref<Eigen::VectorXd> Lu,
                                Eigen::Ref<Eigen::MatrixXd> Lxx,
                                Eigen::Ref<Eigen::MatrixXd> Luu,
                                Eigen::Ref<Eigen::MatrixXd> Lxu,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {
  (void)Lu;
  (void)Luu;
  (void)Lxu;
  (void)u;

  calcDiff(Lx, Lxx, x);
}

void State_cost_model::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                                Eigen::Ref<Eigen::MatrixXd> Lxx,
                                const Eigen::Ref<const Eigen::VectorXd> &x) {
  model_robot->state_diff(__r, ref, x.head(nx_effective));
  model_robot->state_diffDiff(Jx0, Jx1, ref, x.head(nx_effective));

  bool Jx1_is_diagonal = dynobench::is_diagonal(Jx1);

  // CSTR_V(ref);
  // CSTR_V(__r);
  // CSTR_V(x);

  // std::cout << "Jx1 " << std::endl;
  // std::cout << Jx1 << std::endl;

  // CSTR_(Jx1.diagonal().size());
  // CSTR_(__r.size());
  // CSTR_(x_weight_sq.size());

  if (Jx1_is_diagonal) {
    // CHECK_EQ(Lx.head(nx_effective).size(),
    //          __r.cwiseProduct(Jx1.diagonal()).cwiseProduct(x_weight_sq).size(),
    //          AT);
    Lx.head(nx_effective) +=
        __r.cwiseProduct(Jx1.diagonal()).cwiseProduct(x_weight_sq);
    Lxx.diagonal().head(nx_effective) += x_weight_sq;
  } else {
    // CHECK_EQ(Jx1.cols(), x_weight_mat.cols(), AT);
    // CHECK_EQ(Jx1.rows(), x_weight_mat.rows(), AT);
    Jx1_w = Jx1.cwiseProduct(x_weight_mat);
    Lx.block(0, 0, nx_effective, nx_effective).noalias() +=
        __r.cwiseProduct(x_weight) * Jx1_w;
    Lxx.block(0, 0, nx_effective, nx_effective).noalias() +=
        Jx1_w.transpose() * Jx1_w;
  }
}

// u - x

Min_time_linear::Min_time_linear(size_t nx, size_t nu) : Cost(nx, nu, 1) {
  cost_type = CostTYPE::linear;
  name = "min_time_linear";
}

void Min_time_linear::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calc(r, x, u);
  r(0) = k * u.tail(1)(0);
}

void Min_time_linear::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x) {
  check_input_calc(r, x);
  ERROR_WITH_INFO("should not be here");
}

void Min_time_linear::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::VectorXd> Lu,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               Eigen::Ref<Eigen::MatrixXd> Luu,
                               Eigen::Ref<Eigen::MatrixXd> Lxu,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {
  check_input_calcDiff(Lx, Lu, Lxx, Luu, Lxu, x, u);
  Lu(nu - 1) += k;
}

void Min_time_linear::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               const Eigen::Ref<const Eigen::VectorXd> &x)

{
  check_input_calcDiff(Lx, Lxx, x);
  ERROR_WITH_INFO("should not be here");
}

Diff_angle_cost::Diff_angle_cost(
    size_t nx, size_t nu,
    std::shared_ptr<dynobench::Model_car_with_trailers> car)
    : Cost(nx, nu, 2), car(car) {}

void Diff_angle_cost::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x) {
  Eigen::Vector2d __u(0, 0);
  car->constraintsIneq(r, x, __u);
  r = r.cwiseMax(0) * k_diff_angle;
}

void Diff_angle_cost::calc(Eigen::Ref<Eigen::VectorXd> r,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {
  (void)u;
  calc(r, x);
}

void Diff_angle_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               const Eigen::Ref<const Eigen::VectorXd> &x) {
  Eigen::Vector2d __u(0, 0);
  car->constraintsIneq(f, x, __u);

  Eigen::Matrix<bool, Eigen::Dynamic, 1> result = f.array() >= 0;

  if (f.cwiseMax(0).sum() < 1e-12) {
    ;
  } else {
    const double k_diff_angle2 = k_diff_angle * k_diff_angle;
    car->constraintsIneqDiff(Jx, Ju, x, __u);
    Jx = Jx.array().colwise() * result.cast<double>().array();
    Lx += k_diff_angle2 * f.cwiseMax(0.).transpose() * Jx;
    Lxx += k_diff_angle2 * Jx.transpose() * Jx;
  }
}

void Diff_angle_cost::calcDiff(Eigen::Ref<Eigen::VectorXd> Lx,
                               Eigen::Ref<Eigen::VectorXd> Lu,
                               Eigen::Ref<Eigen::MatrixXd> Lxx,
                               Eigen::Ref<Eigen::MatrixXd> Luu,
                               Eigen::Ref<Eigen::MatrixXd> Lxu,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {
  calcDiff(Lx, Lxx, x);
}

} // namespace dynoplan
