#include "dynoplan/dbastar/options.hpp"
#include "dynobench/general_utils.hpp"
#include <boost/program_options.hpp>

namespace dynoplan {

void Options_dbastar::__load_data(void *source, bool boost, bool write,
                                  const std::string &be) {

  Loader loader;
  loader.use_boost = boost;
  loader.print = write;
  loader.source = source;
  loader.be = be;

  loader.set(VAR_WITH_NAME(fix_seed));
  loader.set(VAR_WITH_NAME(new_invariance));
  loader.set(VAR_WITH_NAME(connect_radius_h));
  loader.set(VAR_WITH_NAME(always_add));
  loader.set(VAR_WITH_NAME(use_collision_shape));
  loader.set(VAR_WITH_NAME(delta));
  loader.set(VAR_WITH_NAME(epsilon));
  loader.set(VAR_WITH_NAME(alpha));
  loader.set(VAR_WITH_NAME(filterDuplicates));
  loader.set(VAR_WITH_NAME(maxCost));
  loader.set(VAR_WITH_NAME(heuristic));
  loader.set(VAR_WITH_NAME(max_motions));
  loader.set(VAR_WITH_NAME(heu_resolution));
  loader.set(VAR_WITH_NAME(delta_factor_goal));
  loader.set(VAR_WITH_NAME(cost_delta_factor));
  loader.set(VAR_WITH_NAME(rebuild_every));
  loader.set(VAR_WITH_NAME(num_sample_trials));
  loader.set(VAR_WITH_NAME(max_expands));
  loader.set(VAR_WITH_NAME(cut_actions));
  loader.set(VAR_WITH_NAME(duplicate_detection_int));
  loader.set(VAR_WITH_NAME(use_landmarks));
  loader.set(VAR_WITH_NAME(factor_duplicate_detection));
  loader.set(VAR_WITH_NAME(epsilon_soft_duplicate));
  loader.set(VAR_WITH_NAME(add_node_if_better));
  loader.set(VAR_WITH_NAME(limit_branching_factor));
  loader.set(VAR_WITH_NAME(debug));
  loader.set(VAR_WITH_NAME(add_after_expand));
  loader.set(VAR_WITH_NAME(motionsFile));
  loader.set(VAR_WITH_NAME(outFile));
  loader.set(VAR_WITH_NAME(search_timelimit));
  loader.set(VAR_WITH_NAME(max_size_heu_map));
  loader.set(VAR_WITH_NAME(heu_map_file));
  loader.set(VAR_WITH_NAME(heu_connection_radius));
  loader.set(VAR_WITH_NAME(use_nigh_nn));
  loader.set(VAR_WITH_NAME(check_cols));
}

void Options_dbastar::add_options(po::options_description &desc) {
  __load_data(&desc, true);
}

void Options_dbastar::print(std::ostream &out, const std::string &be,
                            const std::string &af) const {

  auto ptr = const_cast<Options_dbastar *>(this);
  ptr->__load_data(&out, false, true, be);
}

void Options_dbastar::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Options_dbastar::read_from_yaml(YAML::Node &node) {

  if (node["options_dbastar"]) {
    __read_from_node(node["options_dbastar"]);
  } else {
    __read_from_node(node);
  }
}
void Options_dbastar::__read_from_node(const YAML::Node &node) {
  __load_data(&const_cast<YAML::Node &>(node), false);
}

} // namespace dynoplan
