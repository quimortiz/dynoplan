

#include "dynoplan/optimization/options.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/general_utils.hpp"

namespace dynoplan {

void Options_trajopt::add_options(po::options_description &desc) {

  set_from_boostop(desc, VAR_WITH_NAME(time_ref));
  set_from_boostop(desc, VAR_WITH_NAME(time_weight));
  set_from_boostop(desc, VAR_WITH_NAME(check_with_finite_diff));

  set_from_boostop(desc, VAR_WITH_NAME(name));
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

  set_from_yaml(node, VAR_WITH_NAME(time_ref));
  set_from_yaml(node, VAR_WITH_NAME(time_weight));
  set_from_yaml(node, VAR_WITH_NAME(check_with_finite_diff));

  set_from_yaml(node, VAR_WITH_NAME(name));
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

  out << be << STR(name, af) << std::endl;
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

} // namespace dynoplan
