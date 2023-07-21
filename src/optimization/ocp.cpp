#include "idbastar/optimization/ocp.hpp"
#include "dynobench/croco_macros.hpp"
#include "dynobench/math_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>

#include "crocoddyl/core/numdiff/action.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/states/euclidean.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/utils/timer.hpp"

#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models.hpp"
#include "idbastar/optimization/croco_models.hpp"

using vstr = std::vector<std::string>;
using V2d = Eigen::Vector2d;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using Vxd = Eigen::VectorXd;
using V1d = Eigen::Matrix<double, 1, 1>;

using dynobench::Trajectory;

namespace dynoplan {

using dynobench::check_equal;
using dynobench::enforce_bounds;
using dynobench::FMT;

class CallbackVerboseQ : public crocoddyl::CallbackAbstract {
public:
  using Traj =
      std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>;
  std::vector<Traj> trajs;

  explicit CallbackVerboseQ() = default;
  ~CallbackVerboseQ() override = default;

  void operator()(crocoddyl::SolverAbstract &solver) override {
    std::cout << "adding trajectory" << std::endl;
    trajs.push_back(std::make_pair(solver.get_xs(), solver.get_us()));
  }

  void store() {
    std::string timestamp = get_time_stamp();
    std::string folder = "iterations/" + timestamp;
    std::filesystem::create_directories(folder);

    for (size_t i = 0; i < trajs.size(); i++) {
      auto &traj = trajs.at(i);
      dynobench::Trajectory tt;
      tt.states = traj.first;
      tt.actions = traj.second;

      std::stringstream ss;
      ss << std::setfill('0') << std::setw(4) << i;
      std::ofstream out(folder + "/it" + ss.str() + ".yaml");
      tt.to_yaml_format(out);
    }
  }
};

void Options_trajopt::add_options(po::options_description &desc) {

  set_from_boostop(desc, VAR_WITH_NAME(soft_control_bounds));
  set_from_boostop(desc, VAR_WITH_NAME(rollout_warmstart));
  set_from_boostop(desc, VAR_WITH_NAME(u_bound_scale));
  set_from_boostop(desc, VAR_WITH_NAME(interp));
  set_from_boostop(desc, VAR_WITH_NAME(ref_x0));
  set_from_boostop(desc, VAR_WITH_NAME(collision_weight));
  set_from_boostop(desc, VAR_WITH_NAME(th_acceptnegstep));
  set_from_boostop(desc, VAR_WITH_NAME(states_reg));
  set_from_boostop(desc, VAR_WITH_NAME(init_reg));
  set_from_boostop(desc, VAR_WITH_NAME(control_bounds));
  set_from_boostop(desc, VAR_WITH_NAME(max_iter));
  set_from_boostop(desc, VAR_WITH_NAME(window_optimize));
  set_from_boostop(desc, VAR_WITH_NAME(window_shift));
  set_from_boostop(desc, VAR_WITH_NAME(solver_id));
  set_from_boostop(desc, VAR_WITH_NAME(use_warmstart));
  set_from_boostop(desc, VAR_WITH_NAME(use_finite_diff));
  set_from_boostop(desc, VAR_WITH_NAME(k_linear));
  set_from_boostop(desc, VAR_WITH_NAME(noise_level));
  set_from_boostop(desc, VAR_WITH_NAME(k_contour));
  set_from_boostop(desc, VAR_WITH_NAME(smooth_traj));
  set_from_boostop(desc, VAR_WITH_NAME(weight_goal));
  set_from_boostop(desc, VAR_WITH_NAME(shift_repeat));
  set_from_boostop(desc, VAR_WITH_NAME(solver_name));
  set_from_boostop(desc, VAR_WITH_NAME(tsearch_max_rate));
  set_from_boostop(desc, VAR_WITH_NAME(tsearch_min_rate));
  set_from_boostop(desc, VAR_WITH_NAME(tsearch_num_check));
  set_from_boostop(desc, VAR_WITH_NAME(welf_format));
  set_from_boostop(desc, VAR_WITH_NAME(linear_search));
}

void Options_trajopt::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Options_trajopt::__read_from_node(const YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(soft_control_bounds));
  set_from_yaml(node, VAR_WITH_NAME(noise_level));
  set_from_yaml(node, VAR_WITH_NAME(welf_format));
  set_from_yaml(node, VAR_WITH_NAME(rollout_warmstart));
  set_from_yaml(node, VAR_WITH_NAME(u_bound_scale));
  set_from_yaml(node, VAR_WITH_NAME(interp));
  set_from_yaml(node, VAR_WITH_NAME(ref_x0));
  set_from_yaml(node, VAR_WITH_NAME(collision_weight));
  set_from_yaml(node, VAR_WITH_NAME(th_acceptnegstep));
  set_from_yaml(node, VAR_WITH_NAME(states_reg));
  set_from_yaml(node, VAR_WITH_NAME(init_reg));
  set_from_yaml(node, VAR_WITH_NAME(solver_name));
  set_from_yaml(node, VAR_WITH_NAME(solver_id));
  set_from_yaml(node, VAR_WITH_NAME(use_warmstart));
  set_from_yaml(node, VAR_WITH_NAME(control_bounds));
  set_from_yaml(node, VAR_WITH_NAME(k_linear));
  set_from_yaml(node, VAR_WITH_NAME(k_contour));
  set_from_yaml(node, VAR_WITH_NAME(max_iter));
  set_from_yaml(node, VAR_WITH_NAME(window_optimize));
  set_from_yaml(node, VAR_WITH_NAME(window_shift));
  set_from_yaml(node, VAR_WITH_NAME(max_mpc_iterations));
  set_from_yaml(node, VAR_WITH_NAME(debug_file_name));
  set_from_yaml(node, VAR_WITH_NAME(weight_goal));
  set_from_yaml(node, VAR_WITH_NAME(collision_weight));
  set_from_yaml(node, VAR_WITH_NAME(smooth_traj));
  set_from_yaml(node, VAR_WITH_NAME(shift_repeat));
  set_from_yaml(node, VAR_WITH_NAME(tsearch_max_rate));
  set_from_yaml(node, VAR_WITH_NAME(tsearch_min_rate));
  set_from_yaml(node, VAR_WITH_NAME(tsearch_num_check));
  set_from_yaml(node, VAR_WITH_NAME(linear_search));
}

void Options_trajopt::read_from_yaml(YAML::Node &node) {

  if (node["options_trajopt"]) {
    __read_from_node(node["options_trajopt"]);
  } else {
    __read_from_node(node);
  }
}

void Options_trajopt::print(std::ostream &out, const std::string &be,
                            const std::string &af) const {

  out << be << STR(shift_repeat, af) << std::endl;
  out << be << STR(soft_control_bounds, af) << std::endl;
  out << be << STR(welf_format, af) << std::endl;
  out << be << STR(u_bound_scale, af) << std::endl;
  out << be << STR(interp, af) << std::endl;
  out << be << STR(ref_x0, af) << std::endl;
  out << be << STR(th_acceptnegstep, af) << std::endl;
  out << be << STR(states_reg, af) << std::endl;
  out << be << STR(solver_name, af) << std::endl;
  out << be << STR(CALLBACKS, af) << std::endl;
  out << be << STR(solver_id, af) << std::endl;
  out << be << STR(use_finite_diff, af) << std::endl;
  out << be << STR(use_warmstart, af) << std::endl;
  out << be << STR(repair_init_guess, af) << std::endl;
  out << be << STR(control_bounds, af) << std::endl;
  out << be << STR(th_stop, af) << std::endl;
  out << be << STR(init_reg, af) << std::endl;
  out << be << STR(th_acceptnegstep, af) << std::endl;
  out << be << STR(noise_level, af) << std::endl;
  out << be << STR(max_iter, af) << std::endl;
  out << be << STR(window_optimize, af) << std::endl;
  out << be << STR(window_shift, af) << std::endl;
  out << be << STR(max_mpc_iterations, af) << std::endl;
  out << be << STR(debug_file_name, af) << std::endl;
  out << be << STR(k_linear, af) << std::endl;
  out << be << STR(k_contour, af) << std::endl;
  out << be << STR(weight_goal, af) << std::endl;
  out << be << STR(collision_weight, af) << std::endl;
  out << be << STR(smooth_traj, af) << std::endl;

  out << be << STR(tsearch_max_rate, af) << std::endl;
  out << be << STR(tsearch_min_rate, af) << std::endl;
  out << be << STR(tsearch_num_check, af) << std::endl;
  out << be << STR(linear_search, af) << std::endl;
}

// const char *SOLVER_txt[] = {"traj_opt",
//                             "traj_opt_free_time",
//                             "traj_opt_smooth_then_free_time",
//                             "mpc",
//                             "mpcc",
//                             "mpcc2",
//                             "traj_opt_mpcc",
//                             "mpc_nobound_mpcc",
//                             "mpcc_linear",
//                             "time_search_traj_opt",
//                             "mpc_adaptative",
//                             "traj_opt_free_time_proxi",
//                             "traj_opt_no_bound_bound",
//                             "traj_opt_free_time_proxi_linear",
//                             "none"};

void PrintVariableMap(const boost::program_options::variables_map &vm,
                      std::ostream &out) {
  for (po::variables_map::const_iterator it = vm.cbegin(); it != vm.cend();
       it++) {
    out << "> " << it->first;
    if (((boost::any)it->second.value()).empty()) {
      out << "(empty)";
    }
    if (vm[it->first].defaulted() || it->second.defaulted()) {
      out << "(default)";
    }
    out << "=";

    bool is_char;
    try {
      boost::any_cast<const char *>(it->second.value());
      is_char = true;
    } catch (const boost::bad_any_cast &) {
      is_char = false;
    }
    bool is_str;
    try {
      boost::any_cast<std::string>(it->second.value());
      is_str = true;
    } catch (const boost::bad_any_cast &) {
      is_str = false;
    }

    auto &type = ((boost::any)it->second.value()).type();

    if (type == typeid(int)) {
      out << vm[it->first].as<int>() << std::endl;
    } else if (type == typeid(size_t)) {
      out << vm[it->first].as<size_t>() << std::endl;
    } else if (type == typeid(bool)) {
      out << vm[it->first].as<bool>() << std::endl;
    } else if (type == typeid(double)) {
      out << vm[it->first].as<double>() << std::endl;
    } else if (is_char) {
      out << vm[it->first].as<const char *>() << std::endl;
    } else if (is_str) {
      std::string temp = vm[it->first].as<std::string>();
      if (temp.size()) {
        out << temp << std::endl;
      } else {
        out << "true" << std::endl;
      }
    } else { // Assumes that the only remainder is vector<string>
      try {
        std::vector<std::string> vect =
            vm[it->first].as<std::vector<std::string>>();
        uint i = 0;
        for (std::vector<std::string>::iterator oit = vect.begin();
             oit != vect.end(); oit++, ++i) {
          out << "\r> " << it->first << "[" << i << "]=" << (*oit) << std::endl;
        }
      } catch (const boost::bad_any_cast &) {
        out << "UnknownType(" << ((boost::any)it->second.value()).type().name()
            << ")" << std::endl;
        assert(false);
      }
    }
  }
};

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
                 const Options_trajopt &options_trajopt, size_t &nx,
                 size_t &nu) {
  std::cout << "**\nGENERATING PROBLEM\n**\n" << std::endl;
  std::cout << "**\nGenArgs\n**\n" << std::endl;
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

  Control_Mode control_mode;
  if (gen_args.free_time && !gen_args.free_time_linear) {
    control_mode = Control_Mode::free_time;
  } else if (gen_args.contour_control) {
    control_mode = Control_Mode::contour;
  } else if (gen_args.free_time_linear && gen_args.free_time) {
    control_mode = Control_Mode::free_time_linear;
  } else {
    control_mode = Control_Mode::default_mode;
  }
  std::cout << "control_mode:" << static_cast<int>(control_mode) << std::endl;

  ptr<Dynamics> dyn = create_dynamics(gen_args.model_robot, control_mode);

  if (control_mode == Control_Mode::contour) {
    dyn->x_ub.tail<1>()(0) = gen_args.max_alpha;
  }

  CHECK(dyn, AT);

  dyn->print_bounds(std::cout);

  nu = dyn->nu;
  nx = dyn->nx;

  ptr<Cost> control_feature =
      mk<Control_cost>(nx, nu, nu, dyn->u_weight, dyn->u_ref);

  CSTR_(options_trajopt.control_bounds);
  CSTR_(options_trajopt.soft_control_bounds);

  bool use_hard_bounds = options_trajopt.control_bounds;

  if (options_trajopt.soft_control_bounds) {
    use_hard_bounds = false;
  }

  // CHECK(
  //     !(options_trajopt.control_bounds &&
  //     options_trajopt.soft_control_bounds), AT);

  for (size_t t = 0; t < gen_args.N; t++) {

    std::vector<ptr<Cost>> feats_run;

    if (control_mode == Control_Mode::free_time_linear) {
      if (t > 0)
        feats_run.emplace_back(mk<Time_linear_reg>(nx, nu));
      feats_run.emplace_back(mk<Min_time_linear>(nx, nu));
    }

    feats_run.push_back(control_feature);

    if (options_trajopt.soft_control_bounds) {
      std::cout << "Experimental" << std::endl;
      Eigen::VectorXd v = Eigen::VectorXd(nu);
      v.setConstant(100);
      feats_run.push_back(mk<Control_bounds>(nx, nu, nu, dyn->u_lb, -v));
      feats_run.push_back(mk<Control_bounds>(nx, nu, nu, dyn->u_ub, v));
    }

    // feats_run.push_back(mk<State_bounds>(nx, nu, nx, v, -v);

    if (gen_args.collisions && gen_args.model_robot->env) {
      ptr<Cost> cl_feature = mk<Col_cost>(nx, nu, 1, gen_args.model_robot,
                                          options_trajopt.collision_weight);
      feats_run.push_back(cl_feature);

      if (gen_args.contour_control)
        boost::static_pointer_cast<Col_cost>(cl_feature)
            ->set_nx_effective(nx - 1);
    }
    //

    if (startsWith(gen_args.name, "car1")) {

      auto ptr_derived =
          std::dynamic_pointer_cast<dynobench::Model_car_with_trailers>(
              gen_args.model_robot);

      CHECK(ptr_derived, AT);
      std::cout << "adding diff angle cost" << std::endl;
      ptr<Cost> state_feature = mk<Diff_angle_cost>(nx, nu, ptr_derived);
      feats_run.push_back(state_feature);
    }

    if (startsWith(gen_args.name, "quad2d") &&
        !startsWith(gen_args.name, "quad2dpole")) {
      std::cout << "adding regularization on w and v" << std::endl;

      Vxd state_weights(nx);
      state_weights.setZero();
      state_weights.segment<3>(3) = .2 * V3d::Ones();
      Vxd state_ref = Vxd::Zero(nx);

      ptr<Cost> state_feature =
          mk<State_cost>(nx, nu, nx, state_weights, state_ref);
      feats_run.push_back(state_feature);

      if (control_mode == Control_Mode::default_mode) {
        ptr<Cost> acc_cost =
            mk<Acceleration_cost_quad2d>(gen_args.model_robot, nx, nu);
        feats_run.push_back(acc_cost);
      }
    }
    if (startsWith(gen_args.name, "quad2dpole")) {
      std::cout << "adding regularization on w and v, and vq" << std::endl;

      Vxd state_weights(nx);
      state_weights.setZero();
      state_weights.segment<4>(4) = .2 * V4d::Ones();
      Vxd state_ref = Vxd::Zero(nx);

      ptr<Cost> state_feature =
          mk<State_cost>(nx, nu, nx, state_weights, state_ref);
      feats_run.push_back(state_feature);
    }

    if (startsWith(gen_args.name, "quad3d")) {
      if (control_mode == Control_Mode::default_mode) {
        std::cout << "adding regularization on w and v, q" << std::endl;
        Vxd state_weights(13);
        state_weights.setOnes();
        state_weights *= 0.01;
        state_weights.segment(0, 3).setZero();
        state_weights.segment(3, 4).setConstant(0.1);

        Vxd state_ref = Vxd::Zero(13);
        state_ref(6) = 1.;

        ptr<Cost> state_feature =
            mk<State_cost>(nx, nu, nx, state_weights, state_ref);
        feats_run.push_back(state_feature);

        std::cout << "adding cost on quaternion norm" << std::endl;
        ptr<Cost> quat_feature = mk<Quaternion_cost>(nx, nu);
        boost::static_pointer_cast<Quaternion_cost>(quat_feature)->k_quat = 1.;
        feats_run.push_back(quat_feature);

        std::cout << "adding regularization on acceleration" << std::endl;
        ptr<Cost> acc_feature =
            mk<Quad3d_acceleration_cost>(gen_args.model_robot);
        boost::static_pointer_cast<Quad3d_acceleration_cost>(acc_feature)
            ->k_acc = .005;

        feats_run.push_back(acc_feature);
      } else if (control_mode == Control_Mode::contour) {
        std::cout << "adding regularization on w and v, q" << std::endl;
        Vxd state_weights(14);
        state_weights.setOnes();
        state_weights *= 0.05;
        state_weights.segment(0, 3).setZero();
        state_weights.segment(3, 4).setConstant(0.1);
        state_weights(13) = 0;

        Vxd state_ref = Vxd::Zero(14);
        state_ref(6) = 1.;

        ptr<Cost> state_feature =
            mk<State_cost>(nx, nu, nx, state_weights, state_ref);
        feats_run.push_back(state_feature);

        std::cout << "adding cost on quaternion norm" << std::endl;
        ptr<Cost> quat_feature = mk<Quaternion_cost>(nx, nu);
        boost::static_pointer_cast<Quaternion_cost>(quat_feature)->k_quat = 1.;
        feats_run.push_back(quat_feature);
      }
    }
    if (startsWith(gen_args.name, "quad3d_v5") && t == gen_args.N / 2 + 1) {
      std::cout << "adding special waypoint" << std::endl;
      Vxd state_weights(13);
      state_weights.setZero();
      state_weights.segment(3, 4).array() = 200;
      Vxd state_ref = Vxd::Zero(13);
      Eigen::Vector4d ref_quat(1, 0, 0, 0);
      state_ref.segment(3, 4) = ref_quat;

      CSTR_V(state_weights);
      CSTR_V(state_ref);
      // std::cout << state_weights << std::endl;
      // std::cout << state_ref << std::endl;
      ptr<Cost> state_feature =
          mk<State_cost>(nx, nu, nx, state_weights, state_ref);
      feats_run.push_back(state_feature);
    }

    if (startsWith(gen_args.name, "quad3d_v6")) {
      std::cout << "adding special waypoint" << std::endl;
      Vxd state_weights(13);
      state_weights.setZero();
      state_weights.segment(0, 3).array() = 200;
      Vxd state_ref = Vxd::Zero(13);

      bool add_waypoint = true;
      // t1 = 20, t2 = 40, t3=60, t4=80
      if (t == 20) {
        state_ref.head(3) = Eigen::Vector3d(1, 1, 1.5);
      } else if (t == 40) {
        state_ref.head(3) = Eigen::Vector3d(-1, 1, 1.5);
      } else if (t == 60) {
        state_ref.head(3) = Eigen::Vector3d(-1, -1, 1.5);
      } else if (t == 80) {
        state_ref.head(3) = Eigen::Vector3d(1, -1, 1.5);
      } else {
        add_waypoint = false;
      }

      if (add_waypoint) {
        CSTR_V(state_weights);
        CSTR_V(state_ref);
        ptr<Cost> state_feature =
            mk<State_cost>(nx, nu, nx, state_weights, state_ref);
        feats_run.push_back(state_feature);
      }
    }

    if (startsWith(gen_args.name, "acrobot")) {
      // TODO: refactor so that the features are local to the robots!!
      if (control_mode == Control_Mode::default_mode) {
        std::cout << "adding regularization on v" << std::endl;
        Vxd state_weights(4);
        Vxd state_ref = Vxd::Zero(4);

        state_weights.setOnes();
        state_weights *= .0001;
        state_weights.segment(0, 2).setZero();

        ptr<Cost> state_feature =
            mk<State_cost>(nx, nu, nx, state_weights, state_ref);
        feats_run.push_back(state_feature);

        ptr<Cost> acc_cost = mk<Acceleration_cost_acrobot>(nx, nu);
        feats_run.push_back(acc_cost);
      }
    }

    if (gen_args.states_weights.size() && gen_args.states.size()) {

      CHECK_EQ(gen_args.states_weights.size(), gen_args.states.size(), AT);
      CHECK_EQ(gen_args.states_weights.size(), gen_args.N, AT);

      ptr<Cost> state_feature = mk<State_cost>(
          nx, nu, nx, gen_args.states_weights.at(t), gen_args.states.at(t));
      feats_run.push_back(state_feature);
    }
    const bool add_margin_to_bounds = 1;
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

      // std::cout << "warning, no contour in non-terminal states" << std::endl;
      // ptr<Contour_cost_x> contour_x =
      //     mk<Contour_cost_x>(nx, nu, gen_args.interpolator);
      // contour_x->weight = options_trajopt.k_contour;

      // std::cout << "warning, no cost on alpha in non-terminal states"
      //           << std::endl;
      // idea: use this only if it is close
      // ptr<Contour_cost_alpha_x> contour_alpha_x =
      //     mk<Contour_cost_alpha_x>(nx, nu);
      // contour_alpha_x->k = .1 * options_trajopt.k_linear;

      // feats_run.push_back(contour_x);
      // feats_run.push_back(contour_alpha_x);
    }

    boost::shared_ptr<crocoddyl::ActionModelAbstract> am_run =
        to_am_base(mk<ActionModelQ>(dyn, feats_run));

    if (use_hard_bounds) {
      am_run->set_u_lb(options_trajopt.u_bound_scale * dyn->u_lb);
      am_run->set_u_ub(options_trajopt.u_bound_scale * dyn->u_ub);
    }
    amq_runs.push_back(am_run);
  }

  // Terminal
  if (gen_args.contour_control) {
    // TODO: add penalty here!
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
    // ptr<Cost> state_feature = mk<State_cost>(
    //     nx, nu, nx, options_trajopt.weight_goal * Vxd::Ones(nx),
    //     gen_args.goal);

    CHECK_EQ(static_cast<size_t>(gen_args.goal.size()),
             gen_args.model_robot->nx, AT);
    ptr<Cost> state_feature =
        mk<State_cost_model>(gen_args.model_robot, nx, nu,
                             gen_args.penalty * options_trajopt.weight_goal *
                                 Vxd::Ones(gen_args.model_robot->nx),
                             gen_args.goal);

    feats_terminal.push_back(state_feature);
  }
  am_terminal = to_am_base(mk<ActionModelQ>(dyn, feats_terminal));

  if (options_trajopt.use_finite_diff) {
    std::cout << "using finite diff!" << std::endl;

    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>
        amq_runs_diff(amq_runs.size());

    // double disturbance = 1e-4; // should be high, becaues I have collisions
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

#if 0
void read_from_file(File_parser_inout &inout) {


  YAML::Node init;
  if (inout.init_guess != "") {
    init = load_yaml_safe(inout.init_guess);
  }
  YAML::Node env = load_yaml_safe(inout.env_file);

  if (!env["robots"]) {
    CHECK(false, AT);
    // ...
  }

  Problem problem;
  problem.read_from_yaml(inout.env_file.c_str());

  inout.name = problem.robotType;

  Vxd uzero;

  // if (__in(vstr{"unicycle_first_order_0", "unicycle_second_order_0",
  //               "car_first_order_with_1_trailers_0"},
  //          inout.name))
  //   dt = .1;
  // else if (__in(vstr{"quad2d", "quadrotor_0", "acrobot"}, inout.name))
  //   dt = .01;
  // else
  //   CHECK(false, AT);

  // load the yaml file

  std::string base_path = "../models/";
  std::string suffix = ".yaml";
  inout.robot_model_file = base_path + inout.name + suffix;
  std::cout << "loading file: " << inout.robot_model_file << std::endl;
  YAML::Node robot_model = load_yaml_safe(inout.robot_model_file);
  CHECK(robot_model["dt"], AT);
  inout.dt = robot_model["dt"].as<double>();
  std::cout << STR_(inout.dt) << std::endl;

  // // load the collision checker
  // if (__in(vstr{"unicycle_first_order_0", "unicycle_second_order_0",
  //               "car_first_order_with_1_trailers_0", "quad2d",
  //               "quadrotor_0"},
  //          inout.name)) {
  //   inout.cl = mk<CollisionChecker>();
  //   inout.cl->load(inout.env_file);
  // } else {
  //   std::cout << "this robot doesn't have collision checking " << std::endl;
  //   inout.cl = nullptr;
  // }

  std::vector<std::vector<double>> states;
  // std::vector<Vxd> xs_init;
  // std::vector<Vxd> us_init;

  size_t N;
  std::vector<std::vector<double>> actions;

  inout.start = problem.start;
  inout.goal = problem.goal;

  if (inout.xs.size() && inout.us.size()) {
    std::cout << "using xs and us as init guess " << std::endl;
  } else if (inout.init_guess != "") {
    if (!inout.new_format) {

      CHECK(init["result"], AT);
      CHECK(init["result"][0], AT);
      get_states_and_actions(init["result"][0], inout.xs, inout.us);

    } else {

      std::vector<Vxd> _xs_init;
      std::vector<Vxd> _us_init;

      CHECK(init["result2"], AT);
      CHECK(init["result2"][0], AT);

      get_states_and_actions(init["result2"][0], _xs_init, _us_init);

      std::cout << "Reading results in the new format " << std::endl;

      std::vector<double> _times;

      for (const auto &time : init["result2"][0]["times"]) {
        _times.push_back(time.as<double>());
      }

      // 0 ... 3.5
      // dt is 1.
      // 0 1 2 3 4

      // we use floor in the time to be more agressive
      std::cout << STR_(inout.dt) << std::endl;

      double total_time = _times.back();

      size_t num_time_steps = std::ceil(total_time / inout.dt);

      Vxd times = Vxd::Map(_times.data(), _times.size());

      std::vector<Vxd> xs_init_new;
      std::vector<Vxd> us_init_new;

      size_t nx = states.at(0).size();
      size_t nu = actions.at(0).size();

      auto ts =
          Vxd::LinSpaced(num_time_steps + 1, 0, num_time_steps * inout.dt);

      std::cout << "taking samples at " << ts.transpose() << std::endl;

      for (size_t ti = 0; ti < num_time_steps + 1; ti++) {
        Vxd xout(nx);
        Vxd Jout(nx);

        if (ts(ti) > times.tail(1)(0))
          xout = _xs_init.back();
        else
          linearInterpolation(times, _xs_init, ts(ti), xout, Jout);
        xs_init_new.push_back(xout);
      }

      auto times_u = times.head(times.size() - 1);
      for (size_t ti = 0; ti < num_time_steps; ti++) {
        Vxd uout(nu);
        Vxd Jout(nu);
        if (ts(ti) > times_u.tail(1)(0))
          uout = _us_init.back();
        else
          linearInterpolation(times_u, _us_init, ts(ti), uout, Jout);
        us_init_new.push_back(uout);
      }

      N = num_time_steps;

      std::cout << "N: " << N << std::endl;
      std::cout << "us:  " << us_init_new.size() << std::endl;
      std::cout << "xs: " << xs_init_new.size() << std::endl;

      inout.xs = xs_init_new;
      inout.us = us_init_new;

      std::ofstream debug_file("debug.txt");

      for (auto &x : inout.xs) {
        debug_file << x.format(FMT) << std::endl;
      }
      debug_file << "---" << std::endl;

      for (auto &u : inout.us) {
        debug_file << u.format(FMT) << std::endl;
      }
    }
  } else {
    std::cout << "Warning: "
              << "no init guess " << std::endl;
    std::cout << "using T: " << inout.T << " start " << inout.start.format(FMT)
              << std::endl;

    Vxd x0 = inout.start;

    if (options_trajopt.ref_x0) {
      std::cout << "Warning: using a ref x0, instead of start" << std::endl;

      if (startsWith(inout.name, "unicycle1")) {
        ;
      } else if (startsWith(inout.name, "unicycle2")) {
        x0.segment(3, 2) = Vxd::Zero(2);
        ;
      } else if (startsWith(inout.name, "quad2d")) {
        x0.segment(2, 4) = Vxd::Zero(4);
      } else if (startsWith(inout.name, "quadrotor_0")) {
        x0.segment(3, 4) << 0, 0, 0, 1;
        x0.segment(7, 6) = Vxd::Zero(6);
      } else if (startsWith(inout.name, "acrobot")) {
        x0.segment(2, 2) = Vxd::Zero(2);
      } else if (startsWith(inout.name, "unicycle_first_order_0")) {
        ;
      } else if (startsWith(inout.name, "quad3d")) {
        x0.segment(3, 3).setZero(); // quaternion imaginary
        x0.segment(7, 6).setZero(); // velocities
      } else {
        ERROR_WITH_INFO("not implemented");
      }
      inout.xs = std::vector<Vxd>(inout.T + 1, x0);
    }
    // lets use slerp.
    else if (options_trajopt.interp) {

      if (inout.name == "quadrotor_0") {

        auto x_s = inout.start.segment(0, 3);
        auto x_g = inout.goal.segment(0, 3);
        // auto  x_g = Eigen::Quaterniond(inout.goal.segment(3,4));

        auto q_s = Eigen::Quaterniond(inout.start.segment<4>(3));
        auto q_g = Eigen::Quaterniond(inout.goal.segment<4>(3));

        inout.xs = std::vector<Vxd>(inout.T + 1, x0);

        for (size_t i = 0; i < inout.xs.size(); i++) {

          double dt = double(i) / (inout.xs.size() - 1);
          auto q_ = q_s.slerp(dt, q_g);
          auto x = x_s + dt * (x_g - x_s);
          inout.xs.at(i).segment(0, 3) = x;
          inout.xs.at(i).segment(3, 4) = q_.coeffs();
        }
      } else {
        CHECK(false, AT);
      }

    } else {
      inout.xs = std::vector<Vxd>(inout.T + 1, x0);
    }

    if (startsWith(inout.name, "unicycle")) {
      uzero = Vxd::Zero(2);
    } else if (startsWith(inout.name, "car")) {
      uzero = Vxd::Zero(2);
    } else if (startsWith(inout.name, "quad2d")) {
      uzero = Vxd::Ones(2);
    } else if (startsWith(inout.name, "quad3d")) {
      uzero = Vxd::Ones(4);
    } else if (inout.name == "acrobot") {
      uzero = Vxd::Zero(1);
    } else {
      CHECK(false, AT);
    }

    inout.us = std::vector<Vxd>(inout.T, uzero);
  }

  bool verbose = false;
  if (verbose) {

    std::cout << "states " << std::endl;
    for (auto &x : inout.xs)
      std::cout << x.format(FMT) << std::endl;

    std::cout << "actions " << std::endl;
    for (auto &u : inout.us)
      std::cout << u.format(FMT) << std::endl;
  }
}
#endif

void convert_traj_with_variable_time(const std::vector<Vxd> &xs,
                                     const std::vector<Vxd> &us,
                                     std::vector<Vxd> &xs_out,
                                     std::vector<Vxd> &us_out, const double &dt,
                                     const dynobench::StateQ &state) {
  CHECK(xs.size(), AT);
  CHECK(us.size(), AT);
  CHECK_EQ(xs.size(), us.size() + 1, AT);

  size_t N = us.size();
  size_t nx = xs.front().size();

  size_t nu = us.front().size();
  double total_time =
      std::accumulate(us.begin(), us.end(), 0., [&dt](auto &a, auto &b) {
        return a + dt * b(b.size() - 1);
      });

  std::cout << "original total time: " << dt * us.size() << std::endl;
  std::cout << "new total_time: " << total_time << std::endl;

  size_t num_time_steps = std::ceil(total_time / dt);
  std::cout << "number of time steps " << num_time_steps << std::endl;
  std::cout << "new total time " << num_time_steps * dt << std::endl;
  double scaling_factor = num_time_steps * dt / total_time;
  std::cout << "scaling factor " << scaling_factor << std::endl;
  CHECK_GEQ(scaling_factor, 1, AT);
  CHECK_GEQ(scaling_factor, 1., AT);

  // now I have to sample at every dt
  // TODO: lOOK for Better solution than the trick with scaling

  auto times = Vxd(N + 1);
  times.setZero();
  for (size_t i = 1; i < static_cast<size_t>(times.size()); i++) {
    times(i) = times(i - 1) + dt * us.at(i - 1)(nu - 1);
  }
  // std::cout << times.transpose() << std::endl;

  // TODO: be careful with SO(2)
  std::vector<Vxd> x_out, u_out;
  for (size_t i = 0; i < num_time_steps + 1; i++) {
    double t = i * dt / scaling_factor;
    Vxd out(nx);
    Vxd Jout(nx);
    linearInterpolation(times, xs, t, state, out, Jout);
    x_out.push_back(out);
  }

  std::vector<Vxd> u_nx_orig(us.size());
  std::transform(us.begin(), us.end(), u_nx_orig.begin(),
                 [&nu](auto &s) { return s.head(nu - 1); });

  for (size_t i = 0; i < num_time_steps; i++) {
    double t = i * dt / scaling_factor;
    Vxd out(nu - 1);
    // std::cout << " i and time and num_time_steps is " << i << " " << t << "
    // "
    //           << num_time_steps << std::endl;
    Vxd J(nu - 1);
    linearInterpolation(times.head(times.size() - 1), u_nx_orig, t, state, out,
                        J);
    u_out.push_back(out);
  }

  std::cout << "u out " << u_out.size() << std::endl;
  std::cout << "x out " << x_out.size() << std::endl;

  xs_out = x_out;
  us_out = u_out;
}

std::vector<ReportCost> report_problem(ptr<crocoddyl::ShootingProblem> problem,
                                       const std::vector<Vxd> &xs,
                                       const std::vector<Vxd> &us,
                                       const char *file_name) {
  std::vector<ReportCost> reports;

  for (size_t i = 0; i < problem->get_runningModels().size(); i++) {
    auto &x = xs.at(i);
    auto &u = us.at(i);
    auto p = boost::static_pointer_cast<ActionModelQ>(
        problem->get_runningModels().at(i));
    std::vector<ReportCost> reports_i = get_report(
        p, [&](ptr<Cost> f, Eigen::Ref<Vxd> r) { f->calc(r, x, u); });

    for (auto &report_ii : reports_i)
      report_ii.time = i;
    reports.insert(reports.end(), reports_i.begin(), reports_i.end());
  }

  auto p =
      boost::static_pointer_cast<ActionModelQ>(problem->get_terminalModel());
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
  // for (auto &x : xs) {
  //   CSTR_V(x);
  //   CSTR_(x.size());
  // }
  // std::cout << "us" << std::endl;
  // for (auto &u : us) {
  //
  //   CSTR_(u.size());
  //   CSTR_V(u);
  // }

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

  CHECK_EQ(data_running_diff.size(), data_running.size(), AT);
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

void write_states_controls(const std::vector<Eigen::VectorXd> &xs,
                           const std::vector<Eigen::VectorXd> &us,
                           std::shared_ptr<dynobench::Model_robot> model_robot,
                           const dynobench::Problem &problem,
                           const char *filename) {

  // store the init guess:
  dynobench::Trajectory __traj;
  __traj.actions = us;
  __traj.states = xs;

  {
    std::ofstream out(filename + std::string(".raw.yaml"));
    out << "states:" << std::endl;
    for (auto &x : xs) {
      out << "- " << x.format(FMT) << std::endl;
    }
    out << "actions:" << std::endl;
    for (auto &u : us) {
      out << "- " << u.format(FMT) << std::endl;
    }
  }

  if (__traj.actions.front().size() > model_robot->nu) {
    for (size_t i = 0; i < __traj.actions.size(); i++) {
      __traj.actions.at(i) =
          Eigen::VectorXd(__traj.actions.at(i).head(model_robot->nu));
    }
  }
  if (__traj.states.front().size() > model_robot->nx) {
    for (size_t i = 0; i < __traj.states.size(); i++) {
      __traj.states.at(i) =
          Eigen::VectorXd(__traj.states.at(i).head(model_robot->nx));
    }
  }

  // }

  __traj.start = problem.start;
  __traj.goal = problem.goal;

  // create directory if necessary
  if (const std::filesystem::path path =
          std::filesystem::path(filename).parent_path();
      !path.empty()) {
    std::filesystem::create_directories(path);
  }

  std::ofstream init_guess(filename);
  CSTR_(filename);

  __traj.check(model_robot, true);
  __traj.to_yaml_format(init_guess);
}

void __trajectory_optimization(
    const dynobench::Problem &problem,
    std::shared_ptr<dynobench::Model_robot> &model_robot,
    const dynobench::Trajectory &init_guess,
    const Options_trajopt &options_trajopt, dynobench::Trajectory &traj,
    Result_opti &opti_out) {

  const bool modify_to_match_goal_start = false;
  const bool store_iterations = false;
  const std::string folder_tmptraj = "/tmp/dbastar/";

  CSTR_(store_iterations);
  CSTR_V(init_guess.states.back());

  std::cout
      << "WARNING: "
      << "Cleaning data in opti_out at beginning of __trajectory_optimization"
      << std::endl;
  opti_out.data.clear();

  auto callback_quim = mk<CallbackVerboseQ>();

  {
    dynobench::Trajectory __init_guess = init_guess;
    __init_guess.start = problem.start;
    __init_guess.goal = problem.goal;
    std::cout << "checking traj input of __trajectory_optimization "
              << std::endl;
    __init_guess.check(model_robot, true);
    std::cout << "checking traj input of __trajectory_optimization -- DONE "
              << std::endl;
  }

  size_t ddp_iterations = 0;
  double ddp_time = 0;
  Options_trajopt options_trajopt_local = options_trajopt;

  bool check_with_finite_diff = true;

  std::string name = problem.robotType;
  size_t _nx = model_robot->nx;
  size_t _nu = model_robot->nu;

  std::vector<SOLVER> solvers{SOLVER::traj_opt,
                              SOLVER::traj_opt_free_time_proxi,
                              SOLVER::mpc,
                              SOLVER::mpcc,
                              SOLVER::mpcc2,
                              SOLVER::mpcc_linear,
                              SOLVER::mpc_adaptative,
                              SOLVER::traj_opt_free_time_proxi_linear};

  CHECK(__in_if(solvers,
                [&](const SOLVER &s) {
                  return s ==
                         static_cast<SOLVER>(options_trajopt_local.solver_id);
                }),
        AT);

  std::cout << STR_(options_trajopt_local.solver_id) << std::endl;

  bool verbose = false;
  auto xs_init = init_guess.states;
  auto us_init = init_guess.actions;
  CHECK_EQ(xs_init.size(), us_init.size() + 1, AT);
  size_t N = init_guess.actions.size();
  auto goal = problem.goal;
  auto start = problem.start;
  double dt = model_robot->ref_dt;

  SOLVER solver = static_cast<SOLVER>(options_trajopt_local.solver_id);

  // TODO: put this inside each model
  // if (options_trajopt.repair_init_guess) {
  //   if (startsWith(name, "unicycle") || startsWith(name, "car")) {
  //
  //     std::cout << "WARNING: reparing init guess, annoying SO2" <<
  //     std::endl; for (size_t i = 1; i < N + 1; i++) {
  //       xs_init.at(i)(2) = xs_init.at(i - 1)(2) +
  //                          diff_angle(xs_init.at(i)(2), xs_init.at(i -
  //                          1)(2));
  //     }
  //     Eigen::VectorXd goal_init = goal;
  //     goal(2) = xs_init.at(N)(2) + diff_angle(goal(2), xs_init.at(N)(2));
  //     if ((goal_init - goal).norm() > 1e-10) {
  //       std::cout << "WARNING: goal has been updated" << std::endl;
  //     }
  //     // std::cout << "goal is now (maybe updated) " << goal.transpose()
  //     //           << std::endl;
  //   } else if (startsWith(name, "car")) {
  //     std::cout << "WARNING: reparing init guess, annoying SO2" <<
  //     std::endl; for (size_t i = 1; i < N + 1; i++) {
  //       xs_init.at(i)(2) = xs_init.at(i - 1)(2) +
  //                          diff_angle(xs_init.at(i)(2), xs_init.at(i -
  //                          1)(2));
  //
  //       xs_init.at(i)(3) = xs_init.at(i - 1)(3) +
  //                          diff_angle(xs_init.at(i)(3), xs_init.at(i -
  //                          1)(3));
  //     }
  //     goal(2) = xs_init.at(N)(2) + diff_angle(goal(2), xs_init.at(N)(2));
  //     goal(3) = xs_init.at(N)(3) + diff_angle(goal(3), xs_init.at(N)(3));
  //     std::cout << "goal is now (maybe updated) " << goal.transpose()
  //               << std::endl;
  //   } else if (startsWith(name, "acrobot")) {
  //
  //     std::cout << "WARNING: reparing init guess, annoying SO2" <<
  //     std::endl; for (size_t i = 1; i < N + 1; i++) {
  //       for (size_t j = 0; j < 2; j++)
  //         xs_init.at(i)(j) = xs_init.at(i - 1)(j) +
  //                            diff_angle(xs_init.at(i)(j), xs_init.at(i -
  //                            1)(j));
  //     }
  //     Eigen::VectorXd goal_init = goal;
  //     for (size_t j = 0; j < 2; j++)
  //       goal(j) = xs_init.at(N)(j) + diff_angle(goal(j), xs_init.at(N)(j));
  //     if ((goal_init - goal).norm() > 1e-10) {
  //       std::cout << "WARNING: goal has been updated" << std::endl;
  //     }
  //     // std::cout << "goal is now (maybe updated) " << goal.transpose()
  //     //           << std::endl;
  //   }
  // }

  if (modify_to_match_goal_start) {
    std::cout << "WARNING: "
              << "i modify last state to match goal" << std::endl;
    xs_init.back() = goal;
    xs_init.front() = start;
  }

  write_states_controls(xs_init, us_init, model_robot, problem,
                        (folder_tmptraj + "init_guess.yaml").c_str());

  size_t num_smooth_iterations =
      dt > .05 ? 3 : 5; // TODO: as option in command line

  // if (startsWith(problem.robotType, "quad3d")) {
  //   for (auto &s : xs_init) {
  //     if (s(6) < 0) {
  //       s.segment<4>(3) *= -1.;
  //     }
  //   }
  // }

  std::vector<Eigen::VectorXd> xs_init__ = xs_init;

  if (startsWith(problem.robotType, "quad3d")) {
    std::cout
        << "WARNING: "
        << "i use quaternion interpolation, but distance is euclidean norm"
        << std::endl;

    // flip the start state if necessary

    double d1 = (xs_init.front().segment<4>(3) - start.segment<4>(3)).norm();
    double d2 = (xs_init.front().segment<4>(3) + start.segment<4>(3)).norm();

    if (d2 < d1) {
      std::cout << "WARNING: "
                << "i flip the start state" << std::endl;
      xs_init.front().segment<4>(3) *= -1.;
    }

    for (size_t j = 0; j < xs_init.size() - 1; j++) {
      Eigen::Quaterniond qa(xs_init.at(j).segment<4>(3)),
          qb(xs_init.at(j + 1).segment<4>(3)), qres;
      double t = 1;
      qres = qa.slerp(t, qb);
      std::cout << "qres " << qres.coeffs().format(FMT) << std::endl;
      xs_init.at(j + 1).segment<4>(3) = qres.coeffs();
    }

    // check the goal state...

    double d1g = (xs_init.back().segment<4>(3) - goal.segment<4>(3)).norm();
    double d2g = (xs_init.back().segment<4>(3) + goal.segment<4>(3)).norm();

    if (d2g < d1g) {

      write_states_controls(
          xs_init, us_init, model_robot, problem,
          (folder_tmptraj + "init_guess_issue_quat.yaml").c_str());

      WARN_WITH_INFO("quad3d -- I flip the quaternion of the goal state");
      goal.segment<4>(3) *= -1.;
    }
  }

  if (options_trajopt_local.smooth_traj) {
    // TODO: not working well for the quadcopter -> smoothing
    // is not nice, (create high rotation between equivalent states!)
    for (size_t i = 0; i < num_smooth_iterations; i++) {
      xs_init = smooth_traj2(xs_init, *model_robot->state);
      // if (startsWith(problem.robotType, "quad3d")) {
      //   for (auto &s : xs_init) {
      //     s.segment<4>(3).normalize();
      //   }
      // }
    }

    for (size_t i = 0; i < num_smooth_iterations; i++) {
      us_init = smooth_traj2(us_init, dynobench::Rn(us_init.front().size()));
    }

    // store the smooth traj
  }

  // TODO: this should be guaranteed now...
  // if (problem.robotType == "quadrotor_0") {
  if (startsWith(problem.robotType, "quad3d")) {
    for (auto &s : xs_init) {
      s.segment<4>(3).normalize();
    }
  }

  write_states_controls(xs_init, us_init, model_robot, problem,
                        (folder_tmptraj + "init_guess_smooth.yaml").c_str());

  if (verbose) {
    std::cout << "states " << std::endl;
    for (auto &x : xs_init)
      std::cout << x.format(FMT) << std::endl;
  }

  bool success = false;
  std::vector<Vxd> xs_out;
  std::vector<Vxd> us_out;

  create_dir_if_necessary(options_trajopt_local.debug_file_name.c_str());
  std::ofstream debug_file_yaml(options_trajopt_local.debug_file_name);
  debug_file_yaml << "robotType: " << problem.robotType << std::endl;
  debug_file_yaml << "N: " << N << std::endl;
  debug_file_yaml << "start: " << start.format(FMT) << std::endl;
  debug_file_yaml << "goal: " << goal.format(FMT) << std::endl;
  debug_file_yaml << "xs0: " << std::endl;
  for (auto &x : xs_init)
    debug_file_yaml << "  - " << x.format(FMT) << std::endl;

  debug_file_yaml << "us0: " << std::endl;
  for (auto &x : us_init)
    debug_file_yaml << "  - " << x.format(FMT) << std::endl;

  if (solver == SOLVER::mpc || solver == SOLVER::mpcc ||
      solver == SOLVER::mpcc_linear || solver == SOLVER::mpc_adaptative) {
    // i could not stop when I reach the goal, only stop when I reach
    // it with the step move. Then, I would do the last at full speed?
    // ( I hope :) ) Anyway, now is just fine

    CHECK_GEQ(options_trajopt_local.window_optimize,
              options_trajopt_local.window_shift, AT);

    bool finished = false;

    std::vector<Vxd> xs_opt;
    std::vector<Vxd> us_opt;

    std::vector<Vxd> xs_init_rewrite = xs_init;
    std::vector<Vxd> us_init_rewrite = us_init;

    std::vector<Vxd> xs_warmstart_mpcc;
    std::vector<Vxd> us_warmstart_mpcc;

    std::vector<Vxd> xs_warmstart_adptative;
    std::vector<Vxd> us_warmstart_adptative;

    xs_opt.push_back(start);
    xs_init_rewrite.at(0) = start;

    debug_file_yaml << "opti:" << std::endl;

    auto times = Vxd::LinSpaced(xs_init.size(), 0, (xs_init.size() - 1) * dt);

    double max_alpha = times(times.size() - 1);

    ptr<dynobench::Interpolator> path =
        mk<dynobench::Interpolator>(times, xs_init, model_robot->state);
    ptr<dynobench::Interpolator> path_u =
        mk<dynobench::Interpolator>(times.head(times.size() - 1), us_init);

    std::vector<Vxd> xs;
    std::vector<Vxd> us;

    Vxd previous_state = start;
    ptr<crocoddyl::ShootingProblem> problem_croco;

    Vxd goal_mpc(_nx);

    bool is_last = false;

    double total_time = 0;
    size_t counter = 0;
    size_t total_iterations = 0;
    size_t window_optimize_i = 0;

    bool close_to_goal = false;

    Vxd goal_with_alpha(_nx + 1);
    goal_with_alpha.head(_nx) = goal;
    goal_with_alpha(_nx) = max_alpha;

    bool last_reaches_ = false;
    size_t index_first_goal = 0;
    while (!finished) {

      if (solver == SOLVER::mpc || solver == SOLVER::mpc_adaptative) {

        auto start_i = previous_state;
        if (solver == SOLVER::mpc) {
          CHECK_GEQ(int(N) - int(counter * options_trajopt_local.window_shift),
                    0, AT);
          size_t remaining_steps =
              N - counter * options_trajopt_local.window_shift;

          window_optimize_i =
              std::min(options_trajopt_local.window_optimize, remaining_steps);

          int subgoal_index =
              counter * options_trajopt_local.window_shift + window_optimize_i;

          is_last = options_trajopt_local.window_optimize > remaining_steps;

          if (is_last)
            goal_mpc = goal;
          else
            goal_mpc = xs_init.at(subgoal_index);
        } else if (solver == SOLVER::mpc_adaptative) {

          std::cout << "previous state is " << previous_state.format(FMT)
                    << std::endl;
          auto it = std::min_element(
              path->x.begin(), path->x.end(),
              [&](const auto &a, const auto &b) {
                return model_robot->distance(a, previous_state) <
                       model_robot->distance(b, previous_state);
              });

          size_t index = std::distance(path->x.begin(), it);
          std::cout << "starting with approx index " << index << std::endl;
          std::cout << "Non adaptative index would be: "
                    << counter * options_trajopt_local.window_shift
                    << std::endl;

          window_optimize_i = options_trajopt_local.window_optimize;
          // next goal:
          size_t goal_index = index + window_optimize_i;
          CSTR_(goal_index);
          if (goal_index > xs_init.size() - 1) {
            std::cout << "trying to reach the goal " << std::endl;
            goal_mpc = goal;
          } else {
            goal_mpc = xs_init.at(goal_index);
          }
        }

        std::cout << "is_last:" << is_last << std::endl;
        std::cout << "counter:" << counter << std::endl;
        std::cout << "goal_i:" << goal_mpc.transpose() << std::endl;
        std::cout << "start_i:" << start_i.transpose() << std::endl;

        // i nd

        Generate_params gen_args{
            .free_time = false,
            .name = name,
            .N = window_optimize_i,
            .goal = goal_mpc,
            .start = start_i,
            .model_robot = model_robot,
            .states = {},
            .actions = {},
            .collisions = options_trajopt_local.collision_weight > 1e-3};

        size_t nx, nu;

        std::cout << "gen problem " << STR_(AT) << std::endl;
        problem_croco =
            generate_problem(gen_args, options_trajopt_local, nx, nu);

        // report problem

        if (options_trajopt_local.use_warmstart) {

          if (solver == SOLVER::mpc) {
            xs = std::vector<Vxd>(
                xs_init_rewrite.begin() +
                    counter * options_trajopt_local.window_shift,
                xs_init_rewrite.begin() +
                    counter * options_trajopt_local.window_shift +
                    window_optimize_i + 1);

            us = std::vector<Vxd>(
                us_init_rewrite.begin() +
                    counter * options_trajopt_local.window_shift,
                us_init_rewrite.begin() +
                    counter * options_trajopt_local.window_shift +
                    window_optimize_i);

          } else {

            if (counter) {
              std::cout << "new warmstart" << std::endl;
              xs = xs_warmstart_adptative;
              us = us_warmstart_adptative;

              size_t missing_steps = window_optimize_i - us.size();

              Vxd u_last = Vxd::Zero(nu);

              u_last.head(model_robot->nu) = model_robot->u_0;

              Vxd x_last = xs.back();

              // TODO: Sample the interpolator to get new init guess.

              if (options_trajopt_local.shift_repeat) {
                for (size_t i = 0; i < missing_steps; i++) {
                  us.push_back(u_last);
                  xs.push_back(x_last);
                }
              } else {

                std::cout << "filling window by sampling the trajectory"
                          << std::endl;
                Vxd last = xs_warmstart_adptative.back().head(_nx);

                auto it =
                    std::min_element(path->x.begin(), path->x.end(),
                                     [&](const auto &a, const auto &b) {
                                       return model_robot->distance(a, last) <
                                              model_robot->distance(b, last);
                                     });

                size_t last_index = std::distance(path->x.begin(), it);
                double alpha_of_last = path->times(last_index);
                std::cout << STR_(last_index) << std::endl;
                // now I

                Vxd out(_nx);
                Vxd J(_nx);

                Vxd out_u(_nu);
                Vxd J_u(_nu);

                for (size_t i = 0; i < missing_steps; i++) {
                  {
                    path->interpolate(
                        std::min(alpha_of_last + (i + 1) * dt, max_alpha), out,
                        J);
                    xs.push_back(out);
                  }

                  {
                    path_u->interpolate(
                        std::min(alpha_of_last + i * dt, max_alpha - dt), out_u,
                        J_u);
                    us.push_back(out_u);
                  }
                }
              }

            } else {
              std::cout << "first iteration -- using first" << std::endl;

              if (window_optimize_i + 1 < xs_init.size()) {
                xs = std::vector<Vxd>(xs_init.begin(),
                                      xs_init.begin() + window_optimize_i + 1);
                us = std::vector<Vxd>(us_init.begin(),
                                      us_init.begin() + window_optimize_i);
              } else {
                std::cout << "Optimizing more steps than required" << std::endl;
                xs = xs_init;
                us = us_init;

                size_t missing_steps = window_optimize_i - us.size();
                Vxd u_last = Vxd::Zero(nu);

                u_last.head(model_robot->nu) = model_robot->u_0;

                Vxd x_last = xs.back();

                // TODO: Sample the interpolator to get new init guess.

                for (size_t i = 0; i < missing_steps; i++) {
                  us.push_back(u_last);
                  xs.push_back(x_last);
                }
              }

              // xs = std::vector<Vxd>(window_optimize_i + 1,
              // gen_args.start); us =
              // std::vector<Vxd>(window_optimize_i, Vxd::Zero(nu));
            }
          }
          CHECK_EQ(xs.size(), window_optimize_i + 1, AT);
          CHECK_EQ(us.size(), window_optimize_i, AT);

          CHECK_EQ(xs.size(), us.size() + 1, AT);
          CHECK_EQ(us.size(), window_optimize_i, AT);

        } else {
          xs = std::vector<Vxd>(window_optimize_i + 1, gen_args.start);
          us = std::vector<Vxd>(window_optimize_i, Vxd::Zero(nu));
        }

        if (!options_trajopt_local.use_finite_diff && check_with_finite_diff) {

          options_trajopt_local.use_finite_diff = true;
          options_trajopt_local.disturbance = 1e-4;
          std::cout << "gen problem " << STR_(AT) << std::endl;
          ptr<crocoddyl::ShootingProblem> problem_fdiff =

              generate_problem(gen_args, options_trajopt_local, nx, nu);

          // std::cout << "xs" << std::endl;
          // for (auto &x : xs)
          //   CSTR_V(x);
          // std::cout << "us" << std::endl;
          // for (auto &u : us)
          //   CSTR_V(u);

          check_problem(problem_croco, problem_fdiff, xs, us);
          options_trajopt_local.use_finite_diff = false;
        }

      } else if (solver == SOLVER::mpcc || solver == SOLVER::mpcc_linear) {

        window_optimize_i =
            std::min(options_trajopt_local.window_optimize, us_init.size());

        std::cout << "previous state " << previous_state.format(FMT)
                  << std::endl;
        // approx alpha of first state
        auto it = std::min_element(
            path->x.begin(), path->x.end(), [&](const auto &a, const auto &b) {
              return model_robot->distance(a, previous_state) <=
                     model_robot->distance(b, previous_state);
            });
        size_t first_index = std::distance(path->x.begin(), it);
        double alpha_of_first = path->times(first_index);

        size_t expected_last_index = first_index + window_optimize_i;

        // std::cout << "starting with approx first_index " <<
        // first_index << std::endl; std::cout << "alpha of first " <<
        // alpha_of_first << std::endl;

        Vxd alpha_refs =
            Vxd::LinSpaced(window_optimize_i + 1, alpha_of_first,
                           alpha_of_first + window_optimize_i * dt);

        double expected_final_alpha =
            std::min(alpha_of_first + window_optimize_i * dt, max_alpha);

        std::cout << STR(first_index, ":") << std::endl;
        std::cout << STR(expected_final_alpha, ":") << std::endl;
        std::cout << STR(expected_last_index, ":") << std::endl;
        std::cout << STR(alpha_of_first, ":") << std::endl;
        std::cout << STR(max_alpha, ":") << std::endl;

        size_t nx, nu;

        Vxd start_ic(_nx + 1);
        start_ic.head(_nx) = previous_state.head(_nx);
        start_ic(_nx) = alpha_of_first;

        bool goal_cost = false;

        if (expected_final_alpha > max_alpha - 1e-3 || close_to_goal) {
          std::cout << "alpha_refs > max_alpha || close to goal" << std::endl;
          goal_cost = true; // new
        }

        std::cout << "goal " << goal_with_alpha.format(FMT) << std::endl;
        std::cout << STR_(goal_cost) << std::endl;

        std::vector<Vxd> state_weights;
        std::vector<Vxd> _states(window_optimize_i);

        int try_faster = 5;
        if (last_reaches_) {
          std::cout << "last_reaches_ adds goal cost special" << std::endl;
          std::cout << "try_faster: " << try_faster << std::endl;

          state_weights.resize(window_optimize_i);
          _states.resize(window_optimize_i);

          for (size_t t = 0; t < window_optimize_i; t++) {
            if (t > index_first_goal - options_trajopt_local.window_shift -
                        try_faster)
              state_weights.at(t) = 1. * Vxd::Ones(_nx + 1);
            else
              state_weights.at(t) = Vxd::Zero(_nx + 1);
          }

          for (size_t t = 0; t < window_optimize_i; t++) {
            _states.at(t) = goal_with_alpha;
          }
        }

        Generate_params gen_args{
            .free_time = false,
            .name = name,
            .N = window_optimize_i,
            .goal = goal,
            .start = start_ic,
            .model_robot = model_robot,
            .states = _states,
            .states_weights = state_weights,
            .actions = {},
            .contour_control = true,
            .interpolator = path,
            .max_alpha = max_alpha,
            .linear_contour = solver == SOLVER::mpcc_linear,
            .goal_cost = goal_cost,
            .collisions = options_trajopt_local.collision_weight > 1e-3

        };

        std::cout << "gen problem " << STR_(AT) << std::endl;
        problem_croco =
            generate_problem(gen_args, options_trajopt_local, nx, nu);

        if (options_trajopt_local.use_warmstart) {

          // TODO: I need a more clever initialization. For example,
          // using the ones missing from last time, and then the
          // default?

          std::cout << "warmstarting " << std::endl;

          std::vector<Vxd> xs_i;
          std::vector<Vxd> us_i;

          std::cout << STR(counter, ":") << std::endl;

          if (counter) {
            std::cout << "reusing solution from last iteration "
                         "(window swift)"
                      << std::endl;
            xs_i = xs_warmstart_mpcc;
            us_i = us_warmstart_mpcc;

            size_t missing_steps = window_optimize_i - us_i.size();

            Vxd u_last = Vxd::Zero(_nu + 1);
            u_last.head(model_robot->nu) = model_robot->u_0;
            Vxd x_last = xs_i.back();

            // TODO: Sample the interpolator to get new init guess.

            if (options_trajopt_local.shift_repeat) {
              std::cout << "filling window with last solution " << std::endl;
              for (size_t i = 0; i < missing_steps; i++) {
                us_i.push_back(u_last);
                xs_i.push_back(x_last);
              }
            } else {
              // get the alpha  of the last one.
              std::cout << "filling window by sampling the trajectory"
                        << std::endl;
              Vxd last = xs_warmstart_mpcc.back().head(_nx);

              auto it =
                  std::min_element(path->x.begin(), path->x.end(),
                                   [&](const auto &a, const auto &b) {
                                     return model_robot->distance(a, last) <=
                                            model_robot->distance(b, last);
                                   });

              size_t last_index = std::distance(path->x.begin(), it);
              double alpha_of_last = path->times(last_index);
              std::cout << STR_(last_index) << std::endl;
              // now I

              Vxd out(_nx);
              Vxd J(_nx);

              Vxd out_u(_nu);
              Vxd J_u(_nu);

              Vxd uu(_nu + 1);
              Vxd xx(_nx + 1);
              for (size_t i = 0; i < missing_steps; i++) {
                {
                  path->interpolate(
                      std::min(alpha_of_last + (i + 1) * dt, max_alpha), out,
                      J);
                  xx.head(_nx) = out;
                  xx(_nx) = alpha_of_last + i * dt;
                  xs_i.push_back(xx);
                }

                {
                  path_u->interpolate(
                      std::min(alpha_of_last + i * dt, max_alpha - dt), out_u,
                      J_u);
                  uu.head(_nu) = out_u;
                  uu(_nu) = dt;
                  us_i.push_back(uu);
                }
              }
            }
          } else {
            std::cout << "first iteration, using initial guess trajectory"
                      << std::endl;

            Vxd x(_nx + 1);
            Vxd u(_nu + 1);
            for (size_t i = 0; i < window_optimize_i + 1; i++) {

              x.head(_nx) = xs_init.at(i);
              x(_nx) = alpha_of_first + dt * i;
              xs_i.push_back(x);

              if (i < window_optimize_i) {
                u.head(_nu) = us_init.at(i);
                u(_nu) = dt;
                us_i.push_back(u);
              }
            }
          }
          xs = xs_i;
          us = us_i;

        } else {
          std::cout << "no warmstart " << std::endl;
          // no warmstart
          Vxd u0c(_nu + 1);
          u0c.head(_nu).setZero();
          u0c(_nu) = dt;
          xs = std::vector<Vxd>(window_optimize_i + 1, gen_args.start);
          us = std::vector<Vxd>(window_optimize_i, u0c);
        }
        CHECK_EQ(xs.size(), window_optimize_i + 1, AT);
        CHECK_EQ(us.size(), window_optimize_i, AT);

        if (!options_trajopt_local.use_finite_diff && check_with_finite_diff) {

          options_trajopt_local.use_finite_diff = true;
          options_trajopt_local.disturbance = 1e-4;
          std::cout << "gen problem " << STR_(AT) << std::endl;
          ptr<crocoddyl::ShootingProblem> problem_fdiff =
              generate_problem(gen_args, options_trajopt_local, nx, nu);

          check_problem(problem_croco, problem_fdiff, xs, us);
          options_trajopt_local.use_finite_diff = false;
        }
      }
      // report problem

      // auto models = problem->get_runningModels();

      crocoddyl::SolverBoxFDDP ddp(problem_croco);
      ddp.set_th_stop(options_trajopt_local.th_stop);
      ddp.set_th_acceptnegstep(options_trajopt_local.th_acceptnegstep);

      if (options_trajopt_local.CALLBACKS) {
        std::vector<ptr<crocoddyl::CallbackAbstract>> cbs;
        cbs.push_back(mk<crocoddyl::CallbackVerbose>());
        if (store_iterations) {
          cbs.push_back(callback_quim);
        }
        ddp.setCallbacks(cbs);
      }

      // ENFORCING BOUNDS

      Vxd x_lb, x_ub, u_lb, u_ub;

      double inf = std::numeric_limits<double>::max();
      if (solver == SOLVER::mpcc || solver == SOLVER::mpcc_linear) {

        x_lb = -inf * Vxd::Ones(_nx + 1);
        x_ub = inf * Vxd::Ones(_nx + 1);

        u_lb = -inf * Vxd::Ones(_nu + 1);
        u_ub = inf * Vxd::Ones(_nu + 1);

        x_lb(x_lb.size() - 1) = 0;
        x_ub(x_ub.size() - 1) = max_alpha;
        u_lb(u_lb.size() - 1) = 0;

      } else {
        std::cout << "_nx" << _nx << std::endl;
        x_lb = -inf * Vxd::Ones(_nx);
        x_ub = inf * Vxd::Ones(_nx);

        u_lb = -inf * Vxd::Ones(_nu);
        u_ub = inf * Vxd::Ones(_nu);
      }

      if (options_trajopt_local.noise_level > 1e-8) {
        for (size_t i = 0; i < xs.size(); i++) {
          std::cout << "i " << i << " " << xs.at(i).size() << std::endl;
          xs.at(i) += options_trajopt_local.noise_level *
                      Vxd::Random(xs.front().size());
          xs.at(i) = enforce_bounds(xs.at(i), x_lb, x_ub);

          if (startsWith(problem.robotType, "quad3d")) {
            for (auto &s : xs) {
              s.segment<4>(3).normalize();
            }
          }
        }

        for (size_t i = 0; i < us.size(); i++) {
          us.at(i) += options_trajopt_local.noise_level *
                      Vxd::Random(us.front().size());
          us.at(i) = enforce_bounds(us.at(i), u_lb, u_ub);
        }
      }

      crocoddyl::Timer timer;

      if (!options_trajopt_local.use_finite_diff)
        report_problem(problem_croco, xs, us, "/tmp/dbastar/report-0.yaml");
      std::cout << "solving with croco " << AT << std::endl;

      std::string random_id = gen_random(6);

      {
        std::string filename =
            folder_tmptraj + "init_guess_" + random_id + ".yaml";
        write_states_controls(xs, us, model_robot, problem, filename.c_str());
      }

      std::cout << "CROCO optimize" << AT << std::endl;
      ddp.solve(xs, us, options_trajopt_local.max_iter, false,
                options_trajopt_local.init_reg);
      std::cout << "CROCO optimize -- DONE" << std::endl;

      if (store_iterations) {
        callback_quim->store();
      }

      {

        for (auto &s : ddp.get_xs()) {
          CSTR_V(s);
        }
        for (auto &u : ddp.get_us()) {
          CSTR_V(u);
        }

        std::string filename = folder_tmptraj + "opt_" + random_id + ".yaml";
        write_states_controls(ddp.get_xs(), ddp.get_us(), model_robot, problem,
                              filename.c_str());

        std::string filename_raw =
            folder_tmptraj + "opt_" + random_id + ".raw.yaml";
        dynobench::Trajectory traj;
        traj.states = ddp.get_xs();
        traj.actions = ddp.get_us();
        traj.to_yaml_format(filename_raw.c_str());
      }

      double time_i = timer.get_duration();
      size_t iterations_i = ddp.get_iter();
      ddp_iterations += ddp.get_iter();
      ddp_time += timer.get_duration();
      if (!options_trajopt_local.use_finite_diff)
        report_problem(problem_croco, ddp.get_xs(), ddp.get_us(),
                       "/tmp/dbastar/report-1.yaml");

      std::cout << "time: " << time_i << std::endl;
      std::cout << "iterations: " << iterations_i << std::endl;
      total_time += time_i;
      total_iterations += iterations_i;
      std::vector<Vxd> xs_i_sol = ddp.get_xs();
      std::vector<Vxd> us_i_sol = ddp.get_us();

      previous_state =
          xs_i_sol.at(options_trajopt_local.window_shift).head(_nx);

      size_t copy_steps = 0;

      auto fun_is_goal = [&](const auto &x) {
        return model_robot->distance(x.head(_nx), goal) < 1e-2;
      };

      if (solver == SOLVER::mpc_adaptative) {
        // check if I reach the goal.

        size_t final_index = window_optimize_i;
        Vxd x_last = ddp.get_xs().at(final_index);
        std::cout << "**\n" << std::endl;
        std::cout << "checking as final index: " << final_index << std::endl;
        std::cout << "last state: " << x_last.format(FMT) << std::endl;
        std::cout << "true last state: " << ddp.get_xs().back().format(FMT)
                  << std::endl;
        std::cout << "distance to goal: "
                  << model_robot->distance(x_last.head(_nx), goal) << std::endl;

        if (fun_is_goal(x_last)) {
          std::cout << " x last " << x_last.format(FMT) << "reaches the goal"
                    << std::endl;

          std::cout << "setting x last to true " << std::endl;
          is_last = true;
        }

      }

      else if (solver == SOLVER::mpcc || solver == SOLVER::mpcc_linear) {

        std::cout << "if final reaches the goal, i stop" << std::endl;
        std::cout << "ideally, I should check if I can check the goal "
                     "faster, "
                     "with a small linear search "
                  << std::endl;
        // Vxd x_last = ddp.get_xs().back();
        size_t final_index = window_optimize_i;
        // size_t final_index = options_trajopt_local.window_shift;
        std::cout << "final index is " << final_index << std::endl;

        double alpha_mpcc = ddp.get_xs().at(final_index)(_nx);
        Vxd x_last = ddp.get_xs().at(final_index);
        last_reaches_ = fun_is_goal(ddp.get_xs().back());

        std::cout << "**\n" << std::endl;
        std::cout << "checking as final index: " << final_index << std::endl;
        std::cout << "alpha_mpcc:" << alpha_mpcc << std::endl;
        std::cout << "last state: " << x_last.format(FMT) << std::endl;
        std::cout << "true last state: " << ddp.get_xs().back().format(FMT)
                  << std::endl;
        std::cout << "distance to goal: "
                  << model_robot->distance(x_last.head(_nx), goal) << std::endl;

        std::cout << "last_reaches_: " << last_reaches_ << std::endl;
        std::cout << "\n**\n";

        if (last_reaches_) {

          auto it = std::find_if(ddp.get_xs().begin(), ddp.get_xs().end(),
                                 [&](const auto &x) { return fun_is_goal(x); });

          bool __flag = it != ddp.get_xs().end();
          CHECK(__flag, AT);

          index_first_goal = std::distance(ddp.get_xs().begin(), it);
          std::cout << "index first goal " << index_first_goal << std::endl;
        }

        if (std::fabs(alpha_mpcc - times(times.size() - 1)) < 1. &&
            fun_is_goal(x_last)) {

          is_last = true;
          // check which is the first state that is close to goal

          std::cout << "checking first state that reaches the goal "
                    << std::endl;

          auto it = std::find_if(ddp.get_xs().begin(), ddp.get_xs().end(),
                                 [&](const auto &x) { return fun_is_goal(x); });

          assert(it != ddp.get_xs().end());

          window_optimize_i = std::distance(ddp.get_xs().begin(), it);
          std::cout << "changing the number of steps to optimize(copy) to "
                    << window_optimize_i << std::endl;
        }

        std::cout << "checking if i am close to the goal " << std::endl;

        // check 1

        for (size_t i = 0; i < ddp.get_xs().size(); i++) {
          auto &x = ddp.get_xs().at(i);
          if (model_robot->distance(x.head(_nx), goal) < 1e-1) {
            std::cout << "one state is close to goal! " << std::endl;
            close_to_goal = true;
          }

          if (std::fabs(x(_nx) - max_alpha) < 1e-1) {
            std::cout << "alpha is close to final " << std::endl;
            close_to_goal = true;
          }
        }

        std::cout << "done" << std::endl;
      }

      if (is_last)
        copy_steps = window_optimize_i;
      else
        copy_steps = options_trajopt_local.window_shift;

      for (size_t i = 0; i < copy_steps; i++)
        xs_opt.push_back(xs_i_sol.at(i + 1).head(_nx));

      for (size_t i = 0; i < copy_steps; i++)
        us_opt.push_back(us_i_sol.at(i).head(_nu));

      if (solver == SOLVER::mpc) {
        for (size_t i = 0; i < window_optimize_i; i++) {
          xs_init_rewrite.at(1 + counter * options_trajopt_local.window_shift +
                             i) = xs_i_sol.at(i + 1).head(_nx);

          us_init_rewrite.at(counter * options_trajopt_local.window_shift + i) =
              us_i_sol.at(i).head(_nu);
        }
      } else if (solver == SOLVER::mpc_adaptative) {

        xs_warmstart_adptative.clear();
        us_warmstart_adptative.clear();

        for (size_t i = copy_steps; i < window_optimize_i; i++) {
          xs_warmstart_adptative.push_back(xs_i_sol.at(i));
          us_warmstart_adptative.push_back(us_i_sol.at(i));
        }
        xs_warmstart_adptative.push_back(xs_i_sol.back());

      }

      else if (solver == SOLVER::mpcc || solver == SOLVER::mpcc_linear) {

        xs_warmstart_mpcc.clear();
        us_warmstart_mpcc.clear();

        for (size_t i = copy_steps; i < window_optimize_i; i++) {
          xs_warmstart_mpcc.push_back(xs_i_sol.at(i));
          us_warmstart_mpcc.push_back(us_i_sol.at(i));
        }
        xs_warmstart_mpcc.push_back(xs_i_sol.back());
      }

      debug_file_yaml << "  - xs0:" << std::endl;
      for (auto &x : xs)
        debug_file_yaml << "    - " << x.format(FMT) << std::endl;

      debug_file_yaml << "    us0:" << std::endl;
      for (auto &u : us)
        debug_file_yaml << "    - " << u.format(FMT) << std::endl;

      debug_file_yaml << "    xsOPT:" << std::endl;
      for (auto &x : xs_i_sol)
        debug_file_yaml << "    - " << x.format(FMT) << std::endl;

      debug_file_yaml << "    usOPT:" << std::endl;
      for (auto &u : us_i_sol)
        debug_file_yaml << "    - " << u.format(FMT) << std::endl;

      debug_file_yaml << "    start: " << xs.front().format(FMT) << std::endl;

      if (solver == SOLVER::mpc || solver == SOLVER::mpc_adaptative) {
        debug_file_yaml << "    goal: " << goal_mpc.format(FMT) << std::endl;
      } else if (solver == SOLVER::mpcc || solver == SOLVER::mpcc_linear) {
        double alpha_mpcc = ddp.get_xs().back()(_nx);
        Vxd out(_nx);
        Vxd Jout(_nx);
        path->interpolate(alpha_mpcc, out, Jout);
        debug_file_yaml << "    alpha: " << alpha_mpcc << std::endl;
        debug_file_yaml << "    state_alpha: " << out.format(FMT) << std::endl;
      }

      CHECK_EQ(us_i_sol.size() + 1, xs_i_sol.size(), AT);

      // copy results

      if (is_last) {
        finished = true;
        std::cout << "finished: "
                  << "is_last" << std::endl;
      }

      counter++;

      if (counter > options_trajopt_local.max_mpc_iterations) {
        finished = true;
        std::cout << "finished: "
                  << "max mpc iterations" << std::endl;
      }
    }
    std::cout << "Total TIME: " << total_time << std::endl;
    std::cout << "Total Iterations: " << total_iterations << std::endl;

    xs_out = xs_opt;
    us_out = us_opt;

    debug_file_yaml << "xsOPT: " << std::endl;
    for (auto &x : xs_out)
      debug_file_yaml << "  - " << x.format(FMT) << std::endl;

    debug_file_yaml << "usOPT: " << std::endl;
    for (auto &u : us_out)
      debug_file_yaml << "  - " << u.format(FMT) << std::endl;

    // checking feasibility

    std::cout << "checking feasibility" << std::endl;
    ptr<Col_cost> feat_col = mk<Col_cost>(_nx, _nu, 1, model_robot);
    boost::static_pointer_cast<Col_cost>(feat_col)->margin = 0.;
    Vxd goal_last = Vxd::Map(goal.data(), goal.size());

    ptr<Dynamics> dyn;

    bool free_time = false;
    // solver == SOLVER::traj_opt_free_time_proxi;

    std::vector<Eigen::VectorXd> __xs_out = xs_out;
    std::vector<Eigen::VectorXd> __us_out = us_out;
    Eigen::VectorXd dt_check(__us_out.size());
    std::vector<Eigen::VectorXd> xs_check(__xs_out.size());
    std::vector<Eigen::VectorXd> us_check(__us_out.size());
    if (free_time) {
      for (size_t i = 0; i < static_cast<size_t>(dt_check.size()); i++)
        dt_check(i) = __us_out.at(i).tail<1>()(0) * model_robot->ref_dt;
    } else {
      dt_check.setConstant(model_robot->ref_dt);
    }

    for (size_t i = 0; i < xs_check.size(); i++)
      xs_check.at(i) = __xs_out.at(i).head(model_robot->nx);

    for (size_t i = 0; i < us_check.size(); i++)
      us_check.at(i) = __us_out.at(i).head(model_robot->nu);

    double goal_tol = 1e-2;
    double col_tol = 1e-2;
    bool feasible_traj =
        check_trajectory(xs_check, us_check, dt_check, model_robot) < 1e-2;
    bool col_free = check_cols(model_robot, xs_check) < col_tol;
    std::cout << "distance to goal "
              << model_robot->distance(xs_check.back(), goal) << std::endl;
    bool reaches_goal = model_robot->distance(xs_check.back(), goal) < goal_tol;

    std::cout << STR_(feasible_traj) << std::endl;
    std::cout << STR_(col_free) << std::endl;
    std::cout << STR_(reaches_goal) << std::endl;

    success = col_free && feasible_traj && reaches_goal;

  } else if (solver == SOLVER::traj_opt ||
             solver == SOLVER::traj_opt_free_time_proxi ||
             solver == SOLVER::traj_opt_free_time_proxi_linear) {

    if (solver == SOLVER::traj_opt_free_time_proxi ||
        solver == SOLVER::traj_opt_free_time_proxi_linear) {
      std::vector<Vxd> us_init_time(us_init.size());
      size_t nu = us_init.front().size();
      for (size_t i = 0; i < N; i++) {
        Vxd u(nu + 1);
        u.head(nu) = us_init.at(i);
        u(nu) = 1.;
        us_init_time.at(i) = u;
      }
      us_init = us_init_time;
    }

    if (solver == SOLVER::traj_opt_free_time_proxi_linear) {
      std::vector<Vxd> xs_init_time(xs_init.size());
      size_t nx = xs_init.front().size();
      for (size_t i = 0; i < xs_init_time.size(); i++) {
        Vxd x(nx + 1);
        x.head(nx) = xs_init.at(i);
        x(nx) = 1.;
        xs_init_time.at(i) = x;
      }
      xs_init = xs_init_time;
      Eigen::VectorXd old_start = start;
      start.resize(nx + 1);
      start << old_start, 1.;
    }

    // if reg

    std::vector<Vxd> regs;
    if (options_trajopt_local.states_reg && solver == SOLVER::traj_opt) {
      double state_reg_weight = 100.;
      regs = std::vector<Vxd>(xs_init.size() - 1,
                              state_reg_weight * Vxd::Ones(_nx));
    }

    Generate_params gen_args{
        .free_time = solver == SOLVER::traj_opt_free_time_proxi ||
                     solver == SOLVER::traj_opt_free_time_proxi_linear,
        .free_time_linear = solver == SOLVER::traj_opt_free_time_proxi_linear,
        .name = name,
        .N = N,
        .goal = goal,
        .start = start,
        .model_robot = model_robot,
        .states = {xs_init.begin(), xs_init.end() - 1},
        .states_weights = regs,
        .actions = us_init,
        .collisions = options_trajopt_local.collision_weight > 1e-3

    };

    size_t nx, nu;

    std::cout << "gen problem " << STR_(AT) << std::endl;

    auto fun = [&](auto &gen_args, auto &options_trajopt_local,
                   const std::vector<Eigen::VectorXd> &xs_init,
                   const std::vector<Eigen::VectorXd> &us_init, size_t &nx,
                   size_t &nu, bool check_with_finite_diff, size_t N,
                   const std::string &name, size_t &ddp_iterations,
                   double &ddp_time, std::vector<Eigen::VectorXd> &xs_out,
                   std::vector<Eigen::VectorXd> &us_out) {
      ptr<crocoddyl::ShootingProblem> problem_croco =
          generate_problem(gen_args, options_trajopt_local, nx, nu);

      // check gradient

      std::vector<Vxd> xs(N + 1, gen_args.start);
      std::vector<Vxd> us(N, Vxd::Zero(nu));

      if (options_trajopt_local.use_warmstart) {
        xs = xs_init;
        us = us_init;
      }

      if (!options_trajopt_local.use_finite_diff && check_with_finite_diff) {

        std::cout << "Checking with finite diff " << std::endl;
        options_trajopt_local.use_finite_diff = true;
        options_trajopt_local.disturbance = 1e-6;
        std::cout << "gen problem " << STR_(AT) << std::endl;
        ptr<crocoddyl::ShootingProblem> problem_fdiff =
            generate_problem(gen_args, options_trajopt_local, nx, nu);

        check_problem(problem_croco, problem_fdiff, xs, us);
        options_trajopt_local.use_finite_diff = false;
      }

      if (options_trajopt_local.noise_level > 0.) {
        for (size_t i = 0; i < xs.size(); i++) {
          CHECK_EQ(static_cast<size_t>(xs.at(i).size()), nx, AT);
          xs.at(i) += options_trajopt_local.noise_level * Vxd::Random(nx);

          if (startsWith(problem.robotType, "quad3d")) {
            for (auto &s : xs) {
              s.segment<4>(3).normalize();
            }
          }
        }

        for (size_t i = 0; i < us.size(); i++) {
          CHECK_EQ(static_cast<size_t>(us.at(i).size()), nu, AT);
          us.at(i) += options_trajopt_local.noise_level * Vxd::Random(nu);
        }
      }

      crocoddyl::SolverBoxFDDP ddp(problem_croco);
      ddp.set_th_stop(options_trajopt_local.th_stop);
      ddp.set_th_acceptnegstep(options_trajopt_local.th_acceptnegstep);

      if (options_trajopt_local.CALLBACKS) {
        std::vector<ptr<crocoddyl::CallbackAbstract>> cbs;
        cbs.push_back(mk<crocoddyl::CallbackVerbose>());
        if (store_iterations) {
          cbs.push_back(callback_quim);
        }
        ddp.setCallbacks(cbs);
      }

      crocoddyl::Timer timer;

      if (!options_trajopt_local.use_finite_diff)
        report_problem(problem_croco, xs, us, "/tmp/dbastar/report-0.yaml");
      std::cout << "solving with croco " << AT << std::endl;

      std::string random_id = gen_random(6);
      {
        std::string filename =
            folder_tmptraj + "init_guess_" + random_id + ".yaml";
        write_states_controls(xs, us, model_robot, problem, filename.c_str());
      }

      std::cout << "CROCO optimize" << AT << std::endl;
      ddp.solve(xs, us, options_trajopt_local.max_iter, false,
                options_trajopt_local.init_reg);

      if (store_iterations) {
        callback_quim->store();
      }

      std::cout << "CROCO optimize -- DONE" << std::endl;
      {
        std::string filename = folder_tmptraj + "opt_" + random_id + ".yaml";
        write_states_controls(ddp.get_xs(), ddp.get_us(), model_robot, problem,
                              filename.c_str());
      }

      std::cout << "time: " << timer.get_duration() << std::endl;

      ddp_iterations += ddp.get_iter();
      ddp_time += timer.get_duration();

      xs_out = ddp.get_xs();
      us_out = ddp.get_us();

      if (!options_trajopt_local.use_finite_diff)
        report_problem(problem_croco, xs_out, us_out,
                       "/tmp/dbastar/report-1.yaml");
    };

    std::vector<Eigen::VectorXd> _xs_out, _us_out, xs_init_p, us_init_p;

    xs_init_p = xs_init;
    us_init_p = us_init;
    size_t penalty_iterations = 1;
    CSTR_(penalty_iterations);
    for (size_t i = 0; i < penalty_iterations; i++) {
      std::cout << "PENALTY iteration " << i << std::endl;
      gen_args.penalty = std::pow(10., double(i) / 2.);

      if (i > 0) {
        options_trajopt_local.noise_level = 0;
      }

      fun(gen_args, options_trajopt_local, xs_init_p, us_init_p, nx, nu,
          check_with_finite_diff, N, name, ddp_iterations, ddp_time, _xs_out,
          _us_out);

      xs_init_p = _xs_out;
      us_init_p = _us_out;
    }

    // check the distance to the goal:
    // ptr<Col_cost> feat_col = mk<Col_cost>(nx, nu, 1, model_robot);
    // boost::static_pointer_cast<Col_cost>(feat_col)->margin = 0.;

    // feasible = check_feas(feat_col, ddp.get_xs(), ddp.get_us(),
    // gen_args.goal);

    bool __free_time = solver == SOLVER::traj_opt_free_time_proxi ||
                       solver == SOLVER::traj_opt_free_time_proxi_linear;

    std::vector<Eigen::VectorXd> __xs_out = _xs_out;
    std::vector<Eigen::VectorXd> __us_out = _us_out;

    Eigen::VectorXd dt_check(__us_out.size());
    std::vector<Eigen::VectorXd> xs_check(__xs_out.size());
    std::vector<Eigen::VectorXd> us_check(__us_out.size());

    if (__free_time) {
      for (size_t i = 0; i < static_cast<size_t>(dt_check.size()); i++)
        dt_check(i) = __us_out.at(i).tail<1>()(0) * model_robot->ref_dt;
    } else {
      dt_check.setConstant(model_robot->ref_dt);
    }

    for (size_t i = 0; i < xs_check.size(); i++)
      xs_check.at(i) = __xs_out.at(i).head(model_robot->nx);

    for (size_t i = 0; i < us_check.size(); i++)
      us_check.at(i) = __us_out.at(i).head(model_robot->nu);

    double goal_tol = 1e-2;
    double col_tol = 1e-2;
    bool feasible_traj =
        check_trajectory(xs_check, us_check, dt_check, model_robot) < 1e-2;
    bool col_free = check_cols(model_robot, xs_check) < col_tol;
    bool reaches_goal = model_robot->distance(xs_check.back(), goal) < goal_tol;
    CSTR_(model_robot->distance(xs_check.back(), goal))

    std::cout << STR_(feasible_traj) << std::endl;
    std::cout << STR_(col_free) << std::endl;
    std::cout << STR_(reaches_goal) << std::endl;

    success = col_free && feasible_traj && reaches_goal;

    CSTR_(success);

    if (__in({SOLVER::traj_opt_free_time_proxi,
              SOLVER::traj_opt_free_time_proxi_linear},
             solver)) {

      std::vector<Eigen::VectorXd> xs(_xs_out.size());
      std::vector<Eigen::VectorXd> us(_us_out.size());

      std::transform(_xs_out.begin(), _xs_out.end(), xs.begin(),
                     [&](auto &o) { return o.head(model_robot->nx); });

      std::transform(_us_out.begin(), _us_out.end(), us.begin(),
                     [&](auto &o) { return o.head(model_robot->nu); });

      Eigen::VectorXd ts(xs.size());
      ts(0) = 0.;

      for (size_t i = 0; i < us.size(); i++) {
        ts(i + 1) = ts(i) + _us_out.at(i).tail(1)(0) * model_robot->ref_dt;
      }

      CSTR_(ts.tail(1)(0))

      std::cout << "before resample " << std::endl;
      std::cout << xs.back().format(FMT) << std::endl;
      Eigen::VectorXd times;
      resample_trajectory(xs_out, us_out, times, xs, us, ts,
                          model_robot->ref_dt, model_robot->state);

      if (startsWith(model_robot->name, "quad3d")) {
        for (auto &s : xs_out) {
          s.segment<4>(3).normalize();
        }
      }

      std::cout << "after resample " << std::endl;
      std::cout << xs_out.back().format(FMT) << std::endl;

      std::cout << "max error after "
                << max_rollout_error(model_robot, xs_out, us_out) << std::endl;

    } else {
      xs_out = _xs_out;
      us_out = _us_out;
    }

    debug_file_yaml << "xsOPT: " << std::endl;
    for (auto &x : xs_out)
      debug_file_yaml << "  - " << x.format(FMT) << std::endl;

    debug_file_yaml << "usOPT: " << std::endl;
    for (auto &u : us_out)
      debug_file_yaml << "  - " << u.format(FMT) << std::endl;
  }

  std::string filename = "/tmp/dbastar/out.txt";
  create_dir_if_necessary(filename);
  std::ofstream results_txt(filename);

  for (auto &x : xs_out)
    results_txt << x.transpose().format(FMT) << std::endl;

  results_txt << "---" << std::endl;
  for (auto &u : us_out)
    results_txt << u.transpose().format(FMT) << std::endl;

  opti_out.success = success;
  // in some s
  // opti_out.feasible = feasible;

  opti_out.xs_out = xs_out;
  opti_out.us_out = us_out;
  opti_out.cost = us_out.size() * dt;

  traj.states = xs_out;
  traj.actions = us_out;

  // consider only the original components!!

  if (traj.actions.front().size() > model_robot->nu) {
    for (size_t i = 0; i < traj.actions.size(); i++) {
      Eigen::VectorXd tmp = traj.actions.at(i).head(model_robot->nu);
      traj.actions.at(i) = tmp;
    }
  }
  if (traj.states.front().size() > model_robot->nx) {
    for (size_t i = 0; i < traj.states.size(); i++) {
      Eigen::VectorXd tmp = traj.states.at(i).head(model_robot->nx);
      traj.states.at(i) = tmp;
    }
  }

  traj.start = problem.start;

  traj.goal = problem.goal;
  traj.cost = traj.actions.size() * model_robot->ref_dt;
  traj.info = "\"ddp_iterations=" + std::to_string(ddp_iterations) +
              ";"
              "ddp_time=" +
              std::to_string(ddp_time) + "\"";

  opti_out.data.insert({"ddp_time", std::to_string(ddp_time)});

  if (opti_out.success) {

    double traj_tol = 1e-2;
    double goal_tol = 1e-2;
    double col_tol = 1e-2;
    double x_bound_tol = 1e-2;
    double u_bound_tol = 1e-2;

    traj.to_yaml_format(std::cout);

    std::cout << "Final CHECK" << std::endl;
    CSTR_(model_robot->name);

    traj.check(model_robot, true);
    std::cout << "Final CHECK -- DONE" << std::endl;

    dynobench::Feasibility_thresholds thresholds{.traj_tol = traj_tol,
                                                 .goal_tol = goal_tol,
                                                 .col_tol = col_tol,
                                                 .x_bound_tol = x_bound_tol,
                                                 .u_bound_tol = u_bound_tol};

    traj.update_feasibility(thresholds);

    opti_out.feasible = traj.feasible;

    if (!traj.feasible) {
      std::cout << "WARNING: "
                << "why first feas and now infeas? (could happen using the "
                   "time proxi) "
                << std::endl;

      if (!__in({SOLVER::traj_opt_free_time_proxi,
                 SOLVER::traj_opt_free_time_proxi_linear},
                solver) &&
          options_trajopt_local.u_bound_scale <= 1 + 1e-8) {
        // ERROR_WITH_INFO("why?");
        std::cout << "WARNING"
                  << "solver says feasible, but check says infeasible!"
                  << std::endl;
        traj.feasible = false;
        opti_out.feasible = false;
        opti_out.success = false;
      }
    }
  } else {
    traj.feasible = false;
    opti_out.feasible = false;
  }
}

void trajectory_optimization(const dynobench::Problem &problem,
                             const Trajectory &init_guess,
                             const Options_trajopt &options_trajopt,
                             Trajectory &traj, Result_opti &opti_out) {

  double time_ddp_total = 0;
  Stopwatch watch;
  Options_trajopt options_trajopt_local = options_trajopt;

  std::shared_ptr<dynobench::Model_robot> model_robot =
      dynobench::robot_factory(
          (problem.models_base_path + problem.robotType + ".yaml").c_str());

  load_env_quim(*model_robot, problem);

  size_t _nx = model_robot->nx;
  size_t _nu = model_robot->nu;

  Trajectory tmp_init_guess = init_guess;
  Trajectory tmp_solution;

  if (options_trajopt_local.welf_format) {
    std::shared_ptr<dynobench::Model_quad3d> robot_derived =
        std::dynamic_pointer_cast<dynobench::Model_quad3d>(model_robot);
    tmp_init_guess = from_welf_to_quim(init_guess, robot_derived->u_nominal);
  }

  if (startsWith(problem.robotType, "quad3d")) {
    for (auto &s : tmp_init_guess.states) {
      s.segment<4>(3).normalize();
    }
  }

  CSTR_(model_robot->ref_dt);

  if (!tmp_init_guess.states.size() && tmp_init_guess.num_time_steps == 0) {
    ERROR_WITH_INFO("define either xs_init or num time steps");
  }

  if (!tmp_init_guess.states.size() && !tmp_init_guess.actions.size()) {

    std::cout << "Warning: no xs_init or us_init has been provided. "
              << std::endl;

    tmp_init_guess.states.resize(init_guess.num_time_steps + 1);

    std::for_each(tmp_init_guess.states.begin(), tmp_init_guess.states.end(),
                  [&](auto &x) {
                    if (options_trajopt_local.ref_x0)
                      x = model_robot->get_x0(problem.start);
                    else
                      x = problem.start;
                  });

    tmp_init_guess.actions.resize(tmp_init_guess.states.size() - 1);
    std::for_each(tmp_init_guess.actions.begin(), tmp_init_guess.actions.end(),
                  [&](auto &x) { x = model_robot->u_0; });

    CSTR_V(tmp_init_guess.states.front());
    CSTR_V(tmp_init_guess.actions.front());
  }

  if (tmp_init_guess.states.size() && !tmp_init_guess.actions.size()) {

    std::cout << "Warning: no us_init has been provided -- using u_0: "
              << model_robot->u_0.format(FMT) << std::endl;

    tmp_init_guess.actions.resize(tmp_init_guess.states.size() - 1);

    std::for_each(tmp_init_guess.actions.begin(), tmp_init_guess.actions.end(),
                  [&](auto &x) { x = model_robot->u_0; });
  }

  if (init_guess.times.size()) {
    std::cout << "i have time stamps, I resample the trajectory" << std::endl;

    {
      std::cout << "check for input" << std::endl;
      Trajectory(init_guess).check(model_robot, true);
      std::cout << "check for input -- DONE" << std::endl;
    }

    resample_trajectory(tmp_init_guess.states, tmp_init_guess.actions,
                        tmp_init_guess.times, init_guess.states,
                        init_guess.actions, init_guess.times,
                        model_robot->ref_dt, model_robot->state);

    if (startsWith(problem.robotType, "quad3d")) {

      for (auto &s : tmp_init_guess.states) {
        s.segment<4>(3).normalize();
      }
    }
  }
  CHECK(tmp_init_guess.actions.size(), AT);
  CHECK(tmp_init_guess.states.size(), AT);
  CHECK_EQ(tmp_init_guess.states.size(), tmp_init_guess.actions.size() + 1, AT);

  // check the init guess trajectory

  std::cout << "Report on the init guess " << std::endl;
  WARN_WITH_INFO("should I copy the first state in the init guess? -- now yes");
  tmp_init_guess.start = problem.start;
  tmp_init_guess.check(model_robot, true);
  std::cout << "Report on the init guess -- DONE " << std::endl;

  switch (static_cast<SOLVER>(options_trajopt.solver_id)) {

  case SOLVER::mpc_nobound_mpcc: {

    size_t num_solve_mpc_no_bounds = 3;
    size_t num_solve_mpcc_with_bounds = 3;

    bool do_mpcc = true;
    std::cout << "MPC" << std::endl;

    for (size_t i = 0; i < num_solve_mpc_no_bounds; i++) {
      std::cout << "iteration " << i << std::endl;

      options_trajopt_local.solver_id = static_cast<int>(SOLVER::mpc);
      options_trajopt_local.control_bounds = 0;
      options_trajopt_local.debug_file_name =
          "/tmp/dbastar/debug_file_mpc_" + std::to_string(i) + ".yaml";

      std::cout << "**\nopti params is " << std::endl;

      options_trajopt_local.print(std::cout);

      if (i > 0) {
        tmp_init_guess = tmp_solution;
      } else {
      }

      __trajectory_optimization(problem, model_robot, tmp_init_guess,
                                options_trajopt_local, tmp_solution, opti_out);
      time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
      CSTR_(time_ddp_total);

      if (!opti_out.success) {
        std::cout << "warning"
                  << " "
                  << "not success" << std::endl;
        do_mpcc = false;
        break;
      }
    }

    std::cout << "MPCC" << std::endl;
    if (do_mpcc) {
      for (size_t i = 0; i < num_solve_mpcc_with_bounds; i++) {

        std::cout << "iteration " << i << std::endl;
        options_trajopt_local.solver_id = static_cast<int>(SOLVER::mpcc);
        options_trajopt_local.control_bounds = 1;
        options_trajopt_local.debug_file_name =
            "/tmp/dbastar/debug_file_mpcc_" + std::to_string(i) + ".yaml";

        tmp_init_guess = tmp_solution;

        __trajectory_optimization(problem, model_robot, tmp_init_guess,
                                  options_trajopt_local, tmp_solution,
                                  opti_out);

        time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
        CSTR_(time_ddp_total);
        if (!opti_out.success) {
          std::cout << "warning"
                    << " "
                    << "not success" << std::endl;
          break;
        }
      }
    }
    traj = tmp_solution;
    CHECK_EQ(traj.feasible, opti_out.feasible, AT);

  } break;

  case SOLVER::traj_opt_smooth_then_free_time: {
    // continue here

    bool do_free_time = true;
    options_trajopt_local.control_bounds = false;
    options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
    options_trajopt_local.control_bounds = 0;
    options_trajopt_local.debug_file_name =
        "/tmp/dbastar/debug_file_trajopt.yaml";
    std::cout << "**\nopti params is " << std::endl;
    options_trajopt_local.print(std::cout);

    __trajectory_optimization(problem, model_robot, tmp_init_guess,
                              options_trajopt_local, tmp_solution, opti_out);

    time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
    CSTR_(time_ddp_total);
    if (!opti_out.success) {
      std::cout << "warning"
                << " "
                << "not success" << std::endl;
      do_free_time = false;
    }

    if (do_free_time) {

      tmp_init_guess.states = tmp_solution.states;
      tmp_init_guess.actions = tmp_solution.actions;

      options_trajopt_local.control_bounds = true;
      options_trajopt_local.solver_id =
          static_cast<int>(SOLVER::traj_opt_free_time);
      options_trajopt_local.control_bounds = 1;
      options_trajopt_local.debug_file_name =
          "/tmp/dbastar/debug_file_trajopt_freetime.yaml";

      __trajectory_optimization(problem, model_robot, tmp_solution,
                                options_trajopt_local, traj, opti_out);
      time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
      CSTR_(time_ddp_total);
    }
    CHECK_EQ(traj.feasible, opti_out.feasible, AT);

  } break;

  case SOLVER::first_fixed_then_free_time: {
    // TODO: test this!!

    bool do_free_time = true;

    options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
    options_trajopt_local.debug_file_name =
        "/tmp/dbastar/debug_file_trajopt_0.yaml";

    std::cout << "**\nopti params is " << std::endl;
    options_trajopt_local.print(std::cout);

    __trajectory_optimization(problem, model_robot, tmp_init_guess,
                              options_trajopt_local, tmp_solution, opti_out);

    time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
    CSTR_(time_ddp_total);
    if (!opti_out.success) {
      std::cout << "warning"
                << " "
                << "not of smoothing" << std::endl;
      do_free_time = false;
    }

    if (do_free_time) {

      bool do_final_repair_step = true;
      options_trajopt_local.control_bounds = true;
      options_trajopt_local.solver_id =
          static_cast<int>(SOLVER::traj_opt_free_time_proxi);
      options_trajopt_local.debug_file_name =
          "/tmp/dbastar/debug_file_trajopt_freetime_proxi.yaml";
      std::cout << "**\nopti params is " << std::endl;
      options_trajopt_local.print(std::cout);

      __trajectory_optimization(problem, model_robot, tmp_solution,
                                options_trajopt_local, tmp_solution, opti_out);
      time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
      CSTR_(time_ddp_total);

      if (!opti_out.success) {
        std::cout << "warning"
                  << " "
                  << "infeasible" << std::endl;
        do_final_repair_step = false;
      }

      if (do_final_repair_step) {

        std::cout << "time proxi was feasible, doing final step " << std::endl;
        options_trajopt_local.control_bounds = true;
        options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
        options_trajopt_local.control_bounds = 1;
        options_trajopt_local.debug_file_name =
            "/tmp/dbastar/debug_file_trajopt_after_freetime_proxi.yaml";

        __trajectory_optimization(problem, model_robot, tmp_solution,
                                  options_trajopt_local, traj, opti_out);
        time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
        CSTR_(time_ddp_total);
      }
      CHECK_EQ(traj.feasible, opti_out.feasible, AT);
    }

  } break;

  case SOLVER::time_search_traj_opt: {

    CHECK(tmp_init_guess.actions.size(), AT);
    CHECK(tmp_init_guess.states.size(), AT);

    auto check_with_rate = [&](double rate, Result_opti &opti_out_local,
                               Trajectory &traj_out) {
      double dt = model_robot->ref_dt;
      CSTR_(dt);

      std::vector<Vxd> us_init = tmp_init_guess.actions;
      std::vector<Vxd> xs_init = tmp_init_guess.states;

      Trajectory traj_rate_i;
      traj_rate_i.goal = tmp_init_guess.goal;
      traj_rate_i.start = tmp_init_guess.start;

      Vxd times = Vxd::LinSpaced(us_init.size() + 1, 0, us_init.size() * dt);

      // resample a trajectory
      size_t original_n = us_init.size();
      Vxd times_2 = rate * times;

      // create an interpolator
      // I need a state to interpolate :)
      // CONTINUE HERE!!
      dynobench::Interpolator interp_x(times_2, xs_init, model_robot->state);
      dynobench::Interpolator interp_u(times_2.head(us_init.size()), us_init);

      int new_n = std::ceil(rate * original_n);

      Vxd new_times = Vxd::LinSpaced(new_n + 1, 0, new_n * dt);
      std::vector<Vxd> new_xs(new_n + 1);
      std::vector<Vxd> new_us(new_n);

      Vxd x(_nx);
      Vxd Jx(_nx);
      Vxd u(_nu);
      Vxd Ju(_nu);

      for (size_t i = 0; i < static_cast<size_t>(new_times.size()); i++) {
        interp_x.interpolate(new_times(i), x, Jx);
        new_xs.at(i) = x;

        if (i < static_cast<size_t>(new_times.size()) - 1) {
          interp_u.interpolate(new_times(i), u, Ju);
          new_us.at(i) = u;
        }
      }

      traj_rate_i.states = new_xs;
      traj_rate_i.actions = new_us;

      options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);

      if (startsWith(problem.robotType, "quad3d")) {
        for (auto &s : traj_rate_i.states) {
          s.segment<4>(3).normalize();
        }
      }

      __trajectory_optimization(problem, model_robot, traj_rate_i,
                                options_trajopt_local, traj_out,
                                opti_out_local);

      CHECK((opti_out_local.data.find("ddp_time") != opti_out_local.data.end()),
            AT);
      time_ddp_total += std::stod(opti_out_local.data.at("ddp_time"));
      CSTR_(time_ddp_total);
    };

    Vxd rates = Vxd::LinSpaced(options_trajopt_local.tsearch_num_check,
                               options_trajopt_local.tsearch_min_rate,
                               options_trajopt_local.tsearch_max_rate);

    Result_opti opti_out_local;
    opti_out_local.name = opti_out.name;
    size_t counter = 0;

    // linear search
    // auto it =
    //     std::find_if(rates.data(), rates.data() + rates.size(), [&](auto
    //     rate) {
    //       std::cout << "checking rate " << rate << std::endl;
    //       options_trajopt.debug_file_name =
    //           "debug_file_trajopt_" + std::to_string(counter++) +
    //           ".yaml";
    //       check_with_rate(file_inout, rate, opti_out_local);
    //       return opti_out_local.feasible;
    //     });

    //  binary search

    Result_opti best;
    Trajectory best_traj;
    best.name = opti_out.name;
    best.cost = std::numeric_limits<double>::max();

    std::cout << "rates are:" << std::endl;
    print_vec(rates.data(), rates.size());

    double *it = nullptr;

    if (!options_trajopt_local.linear_search) {
      it = std::lower_bound(rates.data(), rates.data() + rates.size(), true,
                            [&](auto rate, auto val) {
                              (void)val;
                              std::cout << "checking rate " << rate
                                        << std::endl;
                              options_trajopt_local.debug_file_name =
                                  "/tmp/dbastar/debug_file_trajopt_" +
                                  std::to_string(counter++) + ".yaml";
                              Trajectory traj_out;
                              check_with_rate(rate, opti_out_local, traj_out);
                              if (opti_out_local.feasible) {
                                std::cout << "if feasible -- COST"
                                          << opti_out_local.cost << std::endl;
                                CHECK_GEQ(best.cost, opti_out_local.cost, AT);
                                best_traj = traj_out;
                                best = opti_out_local;
                              }
                              std::cout << "feasibility of rate: " << rate
                                        << " is " << opti_out_local.feasible
                                        << std::endl;
                              return !opti_out_local.feasible;
                            });
    } else {
      it = std::find_if(
          rates.data(), rates.data() + rates.size(), [&](const auto &rate) {
            std::cout << "checking rate " << rate << std::endl;
            options_trajopt_local.debug_file_name =
                "/tmp/dbastar/debug_file_trajopt_" + std::to_string(counter++) +
                ".yaml";
            Trajectory traj_out;
            check_with_rate(rate, opti_out_local, traj_out);
            if (opti_out_local.feasible) {
              std::cout << "if feasible -- COST: " << opti_out_local.cost
                        << std::endl;
              CHECK_GEQ(best.cost, opti_out_local.cost, AT);
              best_traj = traj_out;
              best = opti_out_local;
            }
            std::cout << "feasibility of rate: " << rate << " is "
                      << opti_out_local.feasible << std::endl;
            return opti_out_local.feasible;
          });
    }

    if (it == rates.data() + rates.size()) {
      std::cout << "all rates are infeasible " << std::endl;
      opti_out.feasible = false;
    } else {
      size_t index = std::distance(rates.data(), it);
      std::cout << "first valid is index " << index << " rate " << rates(index)
                << std::endl;
      opti_out = best;
      traj = best_traj;
    }
    CHECK_EQ(traj.feasible, opti_out.feasible, AT);
  } break;

  case SOLVER::traj_opt_free_time_linear: {

    bool do_final_repair_step = true;
    options_trajopt_local.control_bounds = true;
    options_trajopt_local.solver_id =
        static_cast<int>(SOLVER::traj_opt_free_time_proxi_linear);
    options_trajopt_local.control_bounds = 1;
    options_trajopt_local.debug_file_name =
        "/tmp/dbastar/debug_file_trajopt_freetime_proxi.yaml";
    std::cout << "**\nopti params is " << std::endl;
    options_trajopt_local.print(std::cout);

    __trajectory_optimization(problem, model_robot, tmp_init_guess,
                              options_trajopt_local, tmp_solution, opti_out);

    time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
    CSTR_(time_ddp_total);

    if (!opti_out.success) {
      std::cout << "warning"
                << " "
                << "not success" << std::endl;
      do_final_repair_step = false;
    }

    if (do_final_repair_step) {
      std::cout << "time proxi was feasible, doing final step " << std::endl;
      options_trajopt_local.control_bounds = true;
      options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
      options_trajopt_local.control_bounds = 1;
      options_trajopt_local.debug_file_name =
          "/tmp/dbastar/debug_file_trajopt_after_freetime_proxi.yaml";

      __trajectory_optimization(problem, model_robot, tmp_solution,
                                options_trajopt_local, traj, opti_out);
      time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
      CSTR_(time_ddp_total);
    }

    CHECK_EQ(traj.feasible, opti_out.feasible, AT);
  } break;

  case SOLVER::traj_opt_free_time: {

    bool do_final_repair_step = true;
    options_trajopt_local.control_bounds = true;
    options_trajopt_local.solver_id =
        static_cast<int>(SOLVER::traj_opt_free_time_proxi);
    options_trajopt_local.control_bounds = 1;
    options_trajopt_local.debug_file_name =
        "/tmp/dbastar/debug_file_trajopt_freetime_proxi.yaml";
    std::cout << "**\nopti params is " << std::endl;
    options_trajopt_local.print(std::cout);

    __trajectory_optimization(problem, model_robot, tmp_init_guess,
                              options_trajopt_local, tmp_solution, opti_out);
    time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
    CSTR_(time_ddp_total);

    if (!opti_out.success) {
      std::cout << "warning"
                << " "
                << "infeasible" << std::endl;
      do_final_repair_step = false;
    }

    if (do_final_repair_step) {

      std::cout << "time proxi was feasible, doing final step " << std::endl;
      options_trajopt_local.control_bounds = true;
      options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
      options_trajopt_local.control_bounds = 1;
      options_trajopt_local.debug_file_name =
          "/tmp/dbastar/debug_file_trajopt_after_freetime_proxi.yaml";

      __trajectory_optimization(problem, model_robot, tmp_solution,
                                options_trajopt_local, traj, opti_out);
      time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
      CSTR_(time_ddp_total);
    }
    CHECK_EQ(traj.feasible, opti_out.feasible, AT);
  } break;

  case SOLVER::traj_opt_no_bound_bound: {

    bool do_opti_with_real_bounds = true;
    options_trajopt_local.control_bounds = true;
    options_trajopt_local.u_bound_scale = 1.5;
    options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
    options_trajopt_local.debug_file_name =
        "/tmp/dbastar/debug_file_trajopt_bound_scale.yaml";
    std::cout << "**\nopti params is " << std::endl;
    options_trajopt_local.print(std::cout);

    __trajectory_optimization(problem, model_robot, tmp_init_guess,
                              options_trajopt_local, tmp_solution, opti_out);
    time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
    CSTR_(time_ddp_total);

    // solve without bounds  --

    if (!opti_out.success) {
      std::cout << "warning"
                << " "
                << "not success" << std::endl;
      do_opti_with_real_bounds = false;
    }

    if (do_opti_with_real_bounds) {
      std::cout << "bound scale was succesful, optimize with real bounds"
                << std::endl;

      options_trajopt_local.control_bounds = true;
      options_trajopt_local.u_bound_scale = 1.;
      options_trajopt_local.solver_id = static_cast<int>(SOLVER::traj_opt);
      options_trajopt_local.debug_file_name =
          "/tmp/dbastar/debug_file_trajopt_bound.yaml";
      std::cout << "**\nopti params is " << std::endl;
      options_trajopt_local.print(std::cout);

      __trajectory_optimization(problem, model_robot, tmp_solution,
                                options_trajopt_local, traj, opti_out);
      time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
      CSTR_(time_ddp_total);
    }
    CHECK_EQ(traj.feasible, opti_out.feasible, AT);
  } break;

  default: {
    __trajectory_optimization(problem, model_robot, tmp_init_guess,
                              options_trajopt_local, traj, opti_out);
    time_ddp_total += std::stod(opti_out.data.at("ddp_time"));
    CSTR_(time_ddp_total);
    CHECK_EQ(traj.feasible, opti_out.feasible, AT);
  }
  }

  // convert the format if necessary

  if (options_trajopt_local.welf_format) {
    Trajectory traj_welf;
    std::shared_ptr<dynobench::Model_quad3d> robot_derived =
        std::dynamic_pointer_cast<dynobench::Model_quad3d>(model_robot);
    traj_welf = from_quim_to_welf(traj, robot_derived->u_nominal);
    traj = traj_welf;
  }

  double time_raw = watch.elapsed_ms();
  opti_out.data.insert({"time_raw", std::to_string(time_raw)});
  opti_out.data.insert({"time_ddp_total", std::to_string(time_ddp_total)});
}

void Result_opti::write_yaml(std::ostream &out) {
  out << "feasible: " << feasible << std::endl;
  out << "success: " << success << std::endl;
  out << "cost: " << cost << std::endl;
  if (data.size()) {
    out << "info:" << std::endl;
    for (const auto &[k, v] : data) {
      out << "  " << k << ": " << v << std::endl;
    }
  }

  out << "xs_out: " << std::endl;
  for (auto &x : xs_out)
    out << "  - " << x.format(FMT) << std::endl;

  out << "us_out: " << std::endl;
  for (auto &u : us_out)
    out << "  - " << u.format(FMT) << std::endl;
}

void Result_opti::write_yaml_db(std::ostream &out) {
  // CHECK((name != ""), AT);
  out << "feasible: " << feasible << std::endl;
  out << "success: " << success << std::endl;
  out << "cost: " << cost << std::endl;
  out << "result:" << std::endl;
  out << "  - states:" << std::endl;
  for (auto &x : xs_out) {
    // if (__in(vstr{"unicycle_first_order_0", "unicycle_second_order_0",
    //               "car_first_order_with_1_trailers_0", "quad2d"},
    //          name)) {
    //   x(2) = std::remainder(x(2), 2 * M_PI);
    // } else if (name == "acrobot") {
    //   x(0) = std::remainder(x(0), 2 * M_PI);
    //   x(1) = std::remainder(x(1), 2 * M_PI);
    // }
    out << "      - " << x.format(FMT) << std::endl;
  }

  out << "    actions:" << std::endl;
  for (auto &u : us_out) {
    out << "      - " << u.format(FMT) << std::endl;
  }
};

void File_parser_inout::add_options(po::options_description &desc) {
  // desc.add_options()("env",
  // po::value<std::string>(&env_file)->required())(
  //     "waypoints", po::value<std::string>(&init_guess)->required())(
  //     "new_format",
  //     po::value<bool>(&new_format)->default_value(new_format));
  set_from_boostop(desc, VAR_WITH_NAME(init_guess));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(new_format));
  set_from_boostop(desc, VAR_WITH_NAME(problem_name));
  set_from_boostop(desc, VAR_WITH_NAME(T));
}

void File_parser_inout::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  CHECK(std::filesystem::exists(file), AT);
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void File_parser_inout::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(env_file));
  set_from_yaml(node, VAR_WITH_NAME(init_guess));
  set_from_yaml(node, VAR_WITH_NAME(new_format));
  set_from_yaml(node, VAR_WITH_NAME(problem_name));
  set_from_yaml(node, VAR_WITH_NAME(T));
}

void File_parser_inout::print(std::ostream &out) {
  std::string be = "";
  std::string af = ": ";
  out << be << STR(init_guess, af) << std::endl;
  out << be << STR(problem_name, af) << std::endl;
  out << be << STR(env_file, af) << std::endl;
  out << be << STR(new_format, af) << std::endl;
  out << be << STR(name, af) << std::endl;
  out << be << STR(T, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << "size" << af << us.size() << std::endl;
  out << be << "xs " << std::endl;
  for (const auto &s : xs)
    out << "  - " << s.format(FMT) << std::endl;

  out << be << "us" << std::endl;
  for (const auto &s : us)
    out << "  - " << s.format(FMT) << std::endl;

  out << be << "start" << af << start.format(FMT) << std::endl;
  out << be << "goal" << af << goal.format(FMT) << std::endl;
}

// std::vector<Eigen::VectorXd>
// smooth_traj(const std::vector<Eigen::VectorXd> &us_init) {
//   size_t n = us_init.front().size();
//   std::vector<Vxd> us_out(us_init.size());
//   // kernel
//
//   Vxd kernel(5);
//
//   kernel << 1, 2, 3, 2, 1;
//
//   kernel /= kernel.sum();
//
//   for (size_t i = 0; i < static_cast<size_t>(us_init.size()); i++) {
//     Vxd out = Vxd::Zero(n);
//     for (size_t j = 0; j < static_cast<size_t>(kernel.size()); j++) {
//       out += kernel(j) *
//              us_init.at(inside_bounds(int(i - kernel.size() / 2 + j),
//              int(0),
//                                       int(us_init.size() - 1)));
//     }
//     us_out.at(i) = out;
//   }
//   return us_out;
// }

std::vector<Eigen::VectorXd>
smooth_traj2(const std::vector<Eigen::VectorXd> &xs_init,
             const dynobench::StateQ &state) {
  size_t n = xs_init.front().size();
  size_t ndx = state.ndx;
  CHECK_EQ(n, state.nx, AT);
  std::vector<Vxd> xs_out(xs_init.size(), Eigen::VectorXd::Zero(n));

  // compute diff vectors

  Eigen::VectorXd diffA = Eigen::VectorXd::Zero(ndx);
  Eigen::VectorXd diffB = Eigen::VectorXd::Zero(ndx);
  Eigen::VectorXd diffC = Eigen::VectorXd::Zero(ndx);

  xs_out.front() = xs_init.front();
  xs_out.back() = xs_init.back();
  for (size_t i = 1; i < xs_init.size() - 1; i++) {
    state.diff(xs_init.at(i - 1), xs_init.at(i), diffA);
    state.diff(xs_init.at(i - 1), xs_init.at(i + 1), diffB);

    if (i == xs_init.size() - 2) {
      state.integrate(xs_init.at(i - 1), (diffA + diffB / 2.) / 2.,
                      xs_out.at(i));
    } else {
      state.diff(xs_init.at(i - 1), xs_init.at(i + 2), diffC);
      state.integrate(xs_init.at(i - 1), (diffA + diffB / 2. + diffC / 3.) / 3.,
                      xs_out.at(i));
    }
  }
  // smooth the diffs
  return xs_out;
}

std::vector<Eigen::VectorXd>
smooth_traj(const std::vector<Eigen::VectorXd> &xs_init,
            const dynobench::StateQ &state) {
  size_t n = xs_init.front().size();
  size_t ndx = state.ndx;
  CHECK_EQ(n, state.nx, AT);
  std::vector<Vxd> xs_out(xs_init.size(), Eigen::VectorXd::Zero(n));

  Vxd kernel(5);

  kernel << 1, 2, 3, 2, 1;

  kernel /= kernel.sum();

  // compute diff vectors

  std::vector<Vxd> diffs(xs_init.size() - 1, Eigen::VectorXd::Zero(ndx));
  std::vector<Vxd> diffs_smooth(diffs.size(), Eigen::VectorXd::Zero(ndx));

  for (size_t i = 0; i < diffs.size(); i++) {
    state.diff(xs_init.at(i), xs_init.at(i + 1), diffs.at(i));
  }
  // smooth the diffs

  for (size_t i = 0; i < static_cast<size_t>(diffs.size()); i++) {
    for (size_t j = 0; j < static_cast<size_t>(kernel.size()); j++) {
      diffs_smooth.at(i) +=
          kernel(j) *
          diffs.at(dynobench::inside_bounds(int(i - kernel.size() / 2 + j),
                                            int(0), int(diffs.size() - 1)));
    }
  }

  xs_out.at(0) = xs_init.at(0);
  for (size_t i = 1; i < xs_out.size(); i++) {
    state.integrate(xs_init.at(i - 1), diffs_smooth.at(i - 1), xs_out.at(i));
  }

  for (size_t i = 0; i < xs_init.size(); i++) {
    CSTR_V(xs_out.at(i));
    CSTR_V(xs_init.at(i));
  }

  return xs_out;
}
} // namespace dynoplan
