#include "dynoplan/optimization/generate_ocp.hpp"
#include "dynobench/joint_robot.hpp"
#include "dynobench/integrator2_2d.hpp"
#include "dynobench/quadrotor_payload_n.hpp"
#include "dynoplan/optimization/croco_models.hpp"

namespace dynoplan {

using dynobench::check_equal;
using dynobench::FMT;
using Vxd = Eigen::VectorXd;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;

void Generate_params::print(std::ostream &out) const {
  auto pre = "";
  auto after = ": ";
  out << pre << STR(collisions, after) << std::endl;
  out << pre << STR(free_time, after) << std::endl;
  out << pre << STR(name, after) << std::endl;
  out << pre << STR(N, after) << std::endl;
  out << pre << STR(contour_control, after) << std::endl;
  out << pre << STR(max_alpha, after) << std::endl;
  out << STR(goal_cost, after) << std::endl;
  STRY(penalty, out, pre, after);

  out << pre << "goal" << after << goal.transpose() << std::endl;
  out << pre << "start" << after << start.transpose() << std::endl;
  out << pre << "states" << std::endl;
  for (const auto &s : states)
    out << "  - " << s.format(FMT) << std::endl;
  out << pre << "states_weights" << std::endl;
  for (const auto &s : states_weights)
    out << "  - " << s.format(FMT) << std::endl;
  out << pre << "actions" << std::endl;
  for (const auto &s : actions)
    out << "  - " << s.format(FMT) << std::endl;
}

ptr<crocoddyl::ShootingProblem>
generate_problem(const Generate_params &gen_args,
                 const Options_trajopt &options_trajopt) {

  std::cout << "**\nGENERATING PROBLEM\n**\nArgs:\n" << std::endl;
  gen_args.print(std::cout);
  std::cout << "**\n" << std::endl;

  std::cout << "**\nOpti Params\n**\n" << std::endl;
  options_trajopt.print(std::cout);
  std::cout << "**\n" << std::endl;

  std::vector<ptr<Cost>> feats_terminal;
  ptr<crocoddyl::ActionModelAbstract> am_terminal;
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> amq_runs;

  if (gen_args.free_time && gen_args.contour_control) {
    CHECK(false, AT);
  }

  std::map<std::string, double> additional_params;
  Control_Mode control_mode;
  if (gen_args.free_time && !gen_args.free_time_linear) {
    control_mode = Control_Mode::free_time;
    additional_params.insert({"time_weight", options_trajopt.time_weight});
    additional_params.insert({"time_ref", options_trajopt.time_ref});
  } else if (gen_args.contour_control) {
    control_mode = Control_Mode::contour;
  } else if (gen_args.free_time_linear && gen_args.free_time) {
    control_mode = Control_Mode::free_time_linear;
  } else {
    control_mode = Control_Mode::default_mode;
  }
  std::cout << "control_mode:" << static_cast<int>(control_mode) << std::endl;

  ptr<Dynamics> dyn =
      create_dynamics(gen_args.model_robot, control_mode, additional_params);

  if (control_mode == Control_Mode::contour) {
    dyn->x_ub.tail<1>()(0) = gen_args.max_alpha;
  }

  CHECK(dyn, AT);

  // dyn->print_bounds(std::cout);

  size_t nu = dyn->nu;
  size_t nx = dyn->nx;

  ptr<Cost> control_feature =
      mk<Control_cost>(nx, nu, nu, dyn->u_weight, dyn->u_ref);

  CSTR_(options_trajopt.control_bounds);
  CSTR_(options_trajopt.soft_control_bounds);

  bool use_hard_bounds = options_trajopt.control_bounds;

  if (options_trajopt.soft_control_bounds) {
    use_hard_bounds = false;
  }


  for (size_t t = 0; t < gen_args.N; t++) {

    std::vector<ptr<Cost>> feats_run;
    // assumes homogeneous robot teams
    if (gen_args.model_robot->name == "joint_robot") {

      auto ptr_derived =
          std::dynamic_pointer_cast<dynobench::Joint_robot>(
              gen_args.model_robot);

      const int num_robots = ptr_derived->nxs.size();

      assert(num_robots > 0);
      assert(!ptr_derived->robot_names.empty());

      const std::string& robot_name = ptr_derived->robot_names.front();

      const int robot_nx = ptr_derived->nxs.front();

      const std::vector<int>& goal_times = ptr_derived->goal_times;

      print_vec(goal_times.data(), goal_times.size());
      CSTR_(gen_args.N);

      if (!goal_times.empty()) {

        Eigen::VectorXd weights = Eigen::VectorXd::Zero(nx);
        for (size_t j = 0; j < goal_times.size(); ++j) {
          if (goal_times[j] <= t + 1) {
            const size_t start_index = j * robot_nx;
            weights.segment(start_index, robot_nx).setOnes();
          }
        }

        if (weights.sum() > 1e-12) {
          std::cout << "warning, adding special goal cost"
                    << std::endl;
          ptr<Cost> state_feature = mk<State_cost_model>(
              gen_args.model_robot,
              nx,
              nu,
              gen_args.penalty *
                  options_trajopt.weight_goal *
                  weights,
              gen_args.goal);

          feats_run.emplace_back(state_feature);
        }
      }
      // Robot specific constraints
      for (int r = 0; r < num_robots; ++r) {

        if (robot_name == "Integrator2_2d") {
          
          const int x_offset = r * robot_nx;
          const int u_offset = r * ptr_derived->nus.front();

          if (options_trajopt.control_bounds) {
            feats_run.push_back(
                mk<ControlSphereBounds>(
                    nx,
                    nu,
                    u_offset + 0,   // ax
                    u_offset + 1,   // ay
                    2.0,            // a_max
                    100));          // weight
          }

          // Velocity magnitude: sqrt(vx^2 + vy^2) <= v_max
          feats_run.push_back(
              mk<VelocitySphereBounds>(
                  nx,
                  nu,
                  x_offset + 2,     // vx
                  x_offset + 3,     // vy
                  5.0,              // v_max
                  100));            // weight
        }
      }
    }

    if (control_mode == Control_Mode::free_time_linear) {
      if (t > 0)
        feats_run.emplace_back(mk<Time_linear_reg>(nx, nu));
      feats_run.emplace_back(mk<Min_time_linear>(nx, nu));
    }

    feats_run.push_back(control_feature);
    // only for double integrator now
    if (options_trajopt.control_bounds && gen_args.model_robot->name == "Integrator2_2d"){
        feats_run.push_back(
          mk<ControlSphereBounds>(
            nx,
            nu,
            0,      // ax index
            1,      // ay index
            /*a_max*/2.0, // 
            /*weight*/100));
      }

    bool time_variant_collision = gen_args.model_robot->time_varying_env.size();

    if (!time_variant_collision && gen_args.collisions &&
        gen_args.model_robot->env) {
      ptr<Cost> cl_feature = mk<Col_cost>(nx, nu, 1, gen_args.model_robot,
                                          options_trajopt.collision_weight);
      feats_run.push_back(cl_feature);

      if (gen_args.contour_control)
        boost::static_pointer_cast<Col_cost>(cl_feature)
            ->set_nx_effective(nx - 1);
    } else if (time_variant_collision && gen_args.collisions) {
      ptr<Cost> cl_feature = mk<Col_cost_moving>(
          t, nx, nu, 1, gen_args.model_robot, options_trajopt.collision_weight,
          /*hard_constrained_collision*/ true);
      feats_run.push_back(cl_feature);

      if (gen_args.contour_control)
        boost::static_pointer_cast<Col_cost>(cl_feature)
            ->set_nx_effective(nx - 1);
    }
    // for soft constrained moving obstacles
    bool time_variant_collision_soft =
        gen_args.model_robot->time_varying_env_soft.size();
    if (time_variant_collision_soft && gen_args.collisions) {
      ptr<Cost> cl_feature = mk<Col_cost_moving>(
          t, nx, nu, 1, gen_args.model_robot, /*collision_weight*/ 20,
          /*hard_constrained_collision*/ false);
      feats_run.push_back(cl_feature);

      if (gen_args.contour_control)
        boost::static_pointer_cast<Col_cost>(cl_feature)
            ->set_nx_effective(nx - 1);
    }
    
    if (startsWith(gen_args.name, "Integrator2_2d")) {
      if (control_mode == Control_Mode::default_mode ||
          control_mode == Control_Mode::free_time) {
        std::cout << "adding regularization on the speed! " << std::endl;

        auto ptr_derived =
            std::dynamic_pointer_cast<dynobench::Integrator2_2d>(
                gen_args.model_robot);

        // Additionally, add regularization!!
        Vxd state_weights(4);
        Vxd state_ref = Vxd::Zero(4);
        state_weights.setOnes();
        state_weights *= 1; //.0001;
        state_weights.segment(0, 2).setZero();

        ptr<Cost> state_feature = mk<State_cost>(
            nx, nu, nx, state_weights, state_ref);
        feats_run.push_back(state_feature);
        // for the spherical constraint for the speed/vel.magnitude
        feats_run.push_back(
              mk<VelocitySphereBounds>(
                  nx,
                  nu,
                  2,          // vx index
                  3,          // vy index
                  /*v_max*/0.5, 
                  /*weight*/100));
      } 
      else {
        NOT_IMPLEMENTED;
      }
    }

    if (gen_args.states_weights.size() && gen_args.states.size()) {

      DYNO_CHECK_EQ(gen_args.states_weights.size(), gen_args.states.size(), AT);
      DYNO_CHECK_EQ(gen_args.states_weights.size(), gen_args.N, AT);

      ptr<Cost> state_feature = mk<State_cost>(
          nx, nu, nx, gen_args.states_weights.at(t), gen_args.states.at(t));
      feats_run.push_back(state_feature);
    }
    const bool add_margin_to_bounds = 1;
    // component-wise constraint for the state
    if (dyn->x_lb.size() && dyn->x_weightb.sum() > 1e-10) {
      Eigen::VectorXd v = dyn->x_lb;
      if (add_margin_to_bounds) {
        v.array() += 0.05;
      }
      feats_run.push_back(mk<State_bounds>(nx, nu, nx, v, -dyn->x_weightb));
    }

    if (dyn->x_ub.size() && dyn->x_weightb.sum() > 1e-10) {
      Eigen::VectorXd v = dyn->x_ub;
      if (add_margin_to_bounds) {
        v.array() -= 0.05;
      }
      feats_run.push_back(mk<State_bounds>(nx, nu, nx, v, dyn->x_weightb));
    }

    if (gen_args.contour_control) {

      CHECK(gen_args.linear_contour, AT);

      ptr<Contour_cost_alpha_u> contour_alpha_u =
          mk<Contour_cost_alpha_u>(nx, nu);
      contour_alpha_u->k = options_trajopt.k_linear;

      feats_run.push_back(contour_alpha_u);
    }

    boost::shared_ptr<crocoddyl::ActionModelAbstract> am_run =
        to_am_base(mk<ActionModelDyno>(dyn, feats_run));

    if (use_hard_bounds) {
      am_run->set_u_lb(options_trajopt.u_bound_scale * dyn->u_lb);
      am_run->set_u_ub(options_trajopt.u_bound_scale * dyn->u_ub);
    }
    amq_runs.push_back(am_run);
  }

  // Terminal
  if (gen_args.contour_control) {
    CHECK(gen_args.linear_contour, AT);
    ptr<Cost> state_bounds =
        mk<State_bounds>(nx, nu, nx, dyn->x_ub, dyn->x_weightb);
    ptr<Contour_cost_x> contour_x =
        mk<Contour_cost_x>(nx, nu, gen_args.interpolator);
    contour_x->weight = options_trajopt.k_contour;

    feats_terminal.push_back(contour_x);
    feats_terminal.push_back(state_bounds);
  }

  if (gen_args.goal_cost) {
    std::cout << "adding goal cost " << std::endl;

    DYNO_CHECK_EQ(static_cast<size_t>(gen_args.goal.size()),
                  gen_args.model_robot->nx, AT);

    Eigen::VectorXd goal_weight = gen_args.model_robot->goal_weight;

    if (!goal_weight.size()) {
      goal_weight.resize(gen_args.model_robot->nx);
      goal_weight.setOnes();
    }

    CSTR_V(goal_weight);

    ptr<Cost> state_feature = mk<State_cost_model>(
        gen_args.model_robot, nx, nu,
        gen_args.penalty * options_trajopt.weight_goal * goal_weight,
        gen_args.goal);

    feats_terminal.push_back(state_feature);
  }
  am_terminal = to_am_base(mk<ActionModelDyno>(dyn, feats_terminal));

  if (options_trajopt.use_finite_diff) {
    std::cout << "using finite diff!" << std::endl;

    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>
        amq_runs_diff(amq_runs.size());

    double disturbance = options_trajopt.disturbance;
    std::transform(
        amq_runs.begin(), amq_runs.end(), amq_runs_diff.begin(),
        [&](const auto &am_run) {
          auto am_rundiff = mk<crocoddyl::ActionModelNumDiff>(am_run, true);
          boost::static_pointer_cast<crocoddyl::ActionModelNumDiff>(am_rundiff)
              ->set_disturbance(disturbance);
          if (options_trajopt.control_bounds) {
            am_rundiff->set_u_lb(am_run->get_u_lb());
            am_rundiff->set_u_ub(am_run->get_u_ub());
          }
          return am_rundiff;
        });

    amq_runs = amq_runs_diff;

    auto am_terminal_diff =
        mk<crocoddyl::ActionModelNumDiff>(am_terminal, true);
    boost::static_pointer_cast<crocoddyl::ActionModelNumDiff>(am_terminal_diff)
        ->set_disturbance(disturbance);
    am_terminal = am_terminal_diff;
  }

  CHECK(am_terminal, AT);

  for (auto &a : amq_runs)
    CHECK(a, AT);

  ptr<crocoddyl::ShootingProblem> problem =
      mk<crocoddyl::ShootingProblem>(gen_args.start, amq_runs, am_terminal);

  return problem;
};

std::vector<ReportCost> report_problem(ptr<crocoddyl::ShootingProblem> problem,
                                       const std::vector<Vxd> &xs,
                                       const std::vector<Vxd> &us,
                                       const char *file_name) {
  std::vector<ReportCost> reports;

  for (size_t i = 0; i < problem->get_runningModels().size(); i++) {
    auto &x = xs.at(i);
    auto &u = us.at(i);
    auto p = boost::static_pointer_cast<ActionModelDyno>(
        problem->get_runningModels().at(i));
    std::vector<ReportCost> reports_i = get_report(
        p, [&](ptr<Cost> f, Eigen::Ref<Vxd> r) { f->calc(r, x, u); });

    for (auto &report_ii : reports_i)
      report_ii.time = i;
    reports.insert(reports.end(), reports_i.begin(), reports_i.end());
  }

  auto p =
      boost::static_pointer_cast<ActionModelDyno>(problem->get_terminalModel());
  std::vector<ReportCost> reports_t = get_report(
      p, [&](ptr<Cost> f, Eigen::Ref<Vxd> r) { f->calc(r, xs.back()); });

  for (auto &report_ti : reports_t)
    report_ti.time = xs.size() - 1;
  ;

  reports.insert(reports.begin(), reports_t.begin(), reports_t.end());

  // write down the reports.
  //

  std::string one_space = " ";
  std::string two_space = "  ";
  std::string four_space = "    ";

  create_dir_if_necessary(file_name);

  std::ofstream reports_file(file_name);
  for (auto &report : reports) {
    reports_file << "-" << one_space << "name: " << report.name << std::endl;
    reports_file << two_space << "time: " << report.time << std::endl;
    reports_file << two_space << "cost: " << report.cost << std::endl;
    reports_file << two_space << "type: " << static_cast<int>(report.type)
                 << std::endl;
    if (report.r.size()) {
      reports_file << two_space << "r: " << report.r.format(FMT) << std::endl;
    }
  }

  return reports;
}

bool check_problem(ptr<crocoddyl::ShootingProblem> problem,
                   ptr<crocoddyl::ShootingProblem> problem2,
                   const std::vector<Vxd> &xs, const std::vector<Vxd> &us) {

  bool equal = true;

  problem->calc(xs, us);
  problem->calcDiff(xs, us);
  auto data_running = problem->get_runningDatas();
  auto data_terminal = problem->get_terminalData();

  // now with finite diff
  problem2->calc(xs, us);
  problem2->calcDiff(xs, us);
  auto data_running_diff = problem2->get_runningDatas();
  auto data_terminal_diff = problem2->get_terminalData();

  double tol = 1e-3;
  bool check;

  check = check_equal(data_terminal_diff->Lx, data_terminal->Lx, tol, tol);
  WARN(check, std::string("LxT:") + AT);
  if (!check)
    equal = false;

  check = check_equal(data_terminal_diff->Lxx, data_terminal->Lxx, tol, tol);
  if (!check)
    equal = false;
  WARN(check, std::string("LxxT:") + AT);

  DYNO_CHECK_EQ(data_running_diff.size(), data_running.size(), AT);
  for (size_t i = 0; i < data_running_diff.size(); i++) {
    auto &d = data_running.at(i);
    auto &d_diff = data_running_diff.at(i);
    CSTR_V(xs.at(i));
    CSTR_V(us.at(i));
    check = check_equal(d_diff->Fx, d->Fx, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Fx:") + AT);
    check = check_equal(d_diff->Fu, d->Fu, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Fu:") + AT);
    check = check_equal(d_diff->Lx, d->Lx, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Lx:") + AT);
    check = check_equal(d_diff->Lu, d->Lu, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Lu:") + AT);
    check = check_equal(d_diff->Fx, d->Fx, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Fx:") + AT);
    check = check_equal(d_diff->Fu, d->Fu, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Fu:") + AT);
    check = check_equal(d_diff->Lxx, d->Lxx, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Lxx:") + AT);
    check = check_equal(d_diff->Lxu, d->Lxu, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Lxu:") + AT);
    check = check_equal(d_diff->Luu, d->Luu, tol, tol);
    if (!check)
      equal = false;
    WARN(check, std::string("Luu:") + AT);
  }
  return equal;
}

} // namespace dynoplan
