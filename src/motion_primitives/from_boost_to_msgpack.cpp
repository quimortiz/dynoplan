
#include "dynobench/motions.hpp"
#include <string>
#include <vector>

using namespace dynobench;
int main() {

  std::vector<std::string> files = {
      "../cloud/motionsV2/good/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin.im.bin",
      "../cloud/motionsV2/good/unicycle1_v1/"
      "tmp_motions_unicycle1_v1.bin.sp.bin",
      "../cloud/motionsV2/good/unicycle1_v2/"
      "tmp_motions_unicycle1_v2.bin.sp.bin",
      "../cloud/motionsV2/good/unicycle2_v0/"
      "unicycle2_v0__ispso__2023_04_03__15_36_01.bin.im.bin.im.bin",
      "../cloud/motionsV2/good/car1_v0/car1_v0_all.bin.sp.bin",
      "../cloud/motionsV2/good/quad2d_v0/quad2d_v0_all_im.bin.sp.bin.ca.bin",
      "../cloud/motionsV2/good/quad2dpole_v0/"
      "quad2dpole_all.bin.im.bin.sp1.bin.ca.bin",
      "../cloud/motionsV2/good/acrobot_v0/acrobot_v0_all2.bin.sp.bin",
      "../cloud/motionsV2/good/quad3d_v0/"
      "quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin",
      "../cloud/motionsV2/good/quad3d_omplapp/"
      "quad3dompl_all.bin.im.bin.sp1.bin.ca.bin",
  };

  // std::vector<std::string> files = {
  //     "../data/motion_primitives/unicycle1_v0/"
  //     "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin"};

  std::vector<std::string> files_out;

  for (auto &f : files) {
    Trajectories trajs;
    trajs.load_file_boost(f.c_str());
    trajs.data.resize(5000);
    std::string fileout = f + ".small5000.msgpack";
    files_out.push_back(fileout);
    trajs.save_file_msgpack((fileout).c_str());
  }

  std::cout << "files out " << std::endl;
  for (auto &f : files_out) {
    std::cout << f << std::endl;
  }

#if 0


  // load boost
  Stopwatch sw;
  for (auto &f : files) {
    Trajectories trajs;
    trajs.load_file_boost(f.c_str());
  }
  std::cout << "boost: " << sw.elapsed_ms() << std::endl;

  Stopwatch sw2;
  for (auto &f : files) {
    Trajectories trajs;
    trajs.load_file_msgpack((f + ".msgpack").c_str());
  }
  std::cout << "mgspack: " << sw2.elapsed_ms() << std::endl;

#endif
}
