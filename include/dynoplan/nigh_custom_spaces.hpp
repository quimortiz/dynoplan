#pragma once

#include <cassert>
#include <cstddef> // missing std::size_t include in nigh

#include <nigh/impl/kdtree_median/strategy.hpp>
#include <nigh/kdtree_batch.hpp>
#include <nigh/kdtree_median.hpp>
#include <nigh/se3_space.hpp>
#include <nigh/so2_space.hpp>
#include <nigh/so3_space.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include "dynobench/general_utils.hpp"
#include "dynoplan/ompl/robots.h"

namespace dynoplan {

namespace nigh = unc::robotics::nigh;
namespace ob = ompl::base;

using Space = nigh::CartesianSpace<
    nigh::L2Space<double, 2>,
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>>;

using __Space =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>>;

using __SpaceWithCost =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>>;

using SpaceUni2 = nigh::CartesianSpace<
    nigh::L2Space<double, 2>,
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>,
    nigh::ScaledSpace<nigh::L2Space<double, 1>, std::ratio<1, 4>>,
    nigh::ScaledSpace<nigh::L2Space<double, 1>, std::ratio<1, 4>>>;

using __SpaceUni2 =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>>;

using __SpaceUni2WithCost =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>>;

using SpaceIntegrator2 = nigh::CartesianSpace<
    nigh::L2Space<double, 2>,
    nigh::ScaledSpace<nigh::L2Space<double, 2>, std::ratio<1, 4>>>;

using __SpaceIntegrator2 =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 2>>>;

// x y theta  vx  vw
using SpaceQuad2d = nigh::CartesianSpace<
    nigh::L2Space<double, 2>,
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>,
    nigh::ScaledSpace<nigh::L2Space<double, 2>, std::ratio<1, 5>>,
    nigh::ScaledSpace<nigh::L2Space<double, 1>, std::ratio<1, 10>>>;

using __SpaceQuad2dPole =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>>;

using __SpaceQuad2d =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 1>>>;

using SpaceAcrobot = nigh::CartesianSpace<
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>,
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>,
    nigh::ScaledSpace<nigh::L2Space<double, 2>, std::ratio<1, 5>>>;

using __SpaceAcrobot =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 2>>>;

using SpaceQuad3d = nigh::CartesianSpace<
    nigh::L2Space<double, 3>,
    nigh::ScaledSpace<nigh::SO3Space<double>, std::ratio<1, 2>>,
    nigh::ScaledSpace<nigh::L2Space<double, 3>, std::ratio<1, 10>>,
    nigh::ScaledSpace<nigh::L2Space<double, 3>, std::ratio<1, 20>>>;

using __SpaceQuad3d =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 3>>,
                         nigh::ScaledSpace<nigh::SO3Space<double>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 3>>,
                         nigh::ScaledSpace<nigh::L2Space<double, 3>>>;

using SpaceCar1 = nigh::CartesianSpace<
    nigh::L2Space<double, 2>,
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>,
    nigh::ScaledSpace<nigh::SO2Space<double>, std::ratio<1, 2>>>;

using __SpaceCar1 =
    nigh::CartesianSpace<nigh::ScaledSpace<nigh::L2Space<double, 2>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>,
                         nigh::ScaledSpace<nigh::SO2Space<double>>>;

// _T will be pointer to something, e.g State*
template <typename _T> struct NN_quim {

  virtual void add(const _T &data) = 0;

  virtual void add(const std::vector<_T> &data) = 0;

  virtual void clear() = 0;

  virtual bool remove(const _T &) = 0;

  virtual _T nearest(const _T &data) const = 0;

  virtual void nearestK(const _T &data, std::size_t k,
                        std::vector<std::pair<_T, double>> __nbh) const = 0;

  virtual void nearestR(const _T &data, double radius,
                        std::vector<std::pair<size_t, double>> __nbh) const = 0;

  virtual std::size_t size() const = 0;

  virtual void list(std::vector<_T> &data) const = 0;

  virtual ~NN_quim() = default;
};

template <typename _T, typename Space>
struct NearestNeighborsNigh : public ompl::NearestNeighbors<_T> {

  using Key = typename Space::Type;

  struct Functor {
    std::vector<Key> *keys;
    Functor(std::vector<Key> *keys) : keys(keys) {}
    const Key &operator()(const std::size_t &i) const { return keys->at(i); }
  };

  using Tree = nigh::Nigh<size_t, Space, Functor, nigh::NoThreadSafety,
                          nigh::KDTreeBatch<>>;

  // unc::robotics::nigh::KDTreeBatch<>>;
  std::function<Key(_T const &)> data_to_key;
  std::vector<_T> __data{};
  std::vector<Key> __keys{};
  std::vector<size_t> __idxs{};

  Functor functor;
  Tree tree;

  NearestNeighborsNigh() : functor(&this->__keys), tree(Space{}, functor) {}

  NearestNeighborsNigh(std::function<Key(_T const &)> data_to_key)
      : data_to_key(data_to_key), functor(&this->__keys),
        tree(Space{}, functor) {}

  NearestNeighborsNigh(const Space &space)
      : functor(&this->__keys), tree(space, functor) {}

  NearestNeighborsNigh(const Space &space,
                       std::function<Key(_T const &)> data_to_key)
      : data_to_key(data_to_key), functor(&this->__keys), tree(space, functor) {
  }

  virtual void add(const _T &data) override {
    __data.push_back(data);
    __keys.push_back(data_to_key.operator()(data));
    __idxs.push_back(__idxs.size());
    tree.insert(__idxs.back());
  }

  virtual void add(const std::vector<_T> &data) override {
    for (auto &d : data) {
      add(d);
    }
  }

  virtual bool reportsSortedResults() const override { ERROR_WITH_INFO(AT); };

  virtual void clear() override {
    std::cout << "clearing nigh tree" << std::endl;
    tree.clear();
    __data.clear();
    __keys.clear();
    __idxs.clear();
  };

  bool remove(const _T &) override { ERROR_WITH_INFO(AT); }

  virtual _T nearest(const _T &data) const override {
    Key key = data_to_key.operator()(data);
    std::optional<std::pair<size_t, double>> pt = tree.nearest(key);
    if (pt) {
      return __data.at(pt.value().first);
    } else {
      ERROR_WITH_INFO(AT);
    }
  }

  virtual void nearestK(const _T &data, std::size_t k,
                        std::vector<_T> &nbh) const override {
    std::vector<std::pair<size_t, double>> __nbh;

    Key key = data_to_key.operator()(data);
    tree.nearest(__nbh, key, k);
    nbh.resize(__nbh.size());
    std::transform(__nbh.begin(), __nbh.end(), nbh.begin(),
                   [this](const auto &x) { return __data.at(x.first); });
  }

  virtual void nearestR(const _T &data, double radius,
                        std::vector<_T> &nbh) const override {
    size_t max_k = 1e6; // max number of possible neighbours
    std::vector<std::pair<size_t, double>> __nbh;

    Key key = data_to_key.operator()(data);
    tree.nearest(__nbh, key, max_k, radius);

    nbh.resize(__nbh.size());
    std::transform(__nbh.begin(), __nbh.end(), nbh.begin(),
                   [this](auto &x) { return __data.at(x.first); });
  }

  virtual std::size_t size() const override { return __data.size(); }

  virtual void list(std::vector<_T> &data) const override { data = __data; }
};

template <typename _T>
ompl::NearestNeighbors<_T> *nigh_factory(
    const std::string &name, const std::shared_ptr<RobotOmpl> &robot,
    std::function<const ob::State *(_T)> fun = [](_T m) {
      return m->getState();
    }) {
  ompl::NearestNeighbors<_T> *out = nullptr;

  auto &w = robot->diff_model->distance_weights;
  // CSTR_V(w);

  if (startsWith(name, "unicycle1")) {

    auto data_to_key = [robot, fun](_T const &m) {
      const ob::State *s = fun(m);
      Eigen::Vector3d __x;
      robot->toEigen(s, __x);
      return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2));
    };

    DYNO_CHECK_EQ(w.size(), 2, AT);
    __Space space(double(w(0)), double(w(1)));

    out = new NearestNeighborsNigh<_T, __Space>(space, data_to_key);

  } else if (startsWith(name, "unicycle2")) {

    auto data_to_key = [robot, fun](_T const &m) {
      using Vector5d = Eigen::Matrix<double, 5, 1>;
      using Vector1d = Eigen::Matrix<double, 1, 1>;
      const ob::State *s = fun(m);
      Vector5d __x;
      robot->toEigen(s, __x);
      return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2), Vector1d(__x(3)),
                        Vector1d(__x(4)));
    };

    // out = new NearestNeighborsNigh<_T, SpaceUni2>(data_to_key);

    DYNO_CHECK_EQ(w.size(), 4, AT);
    __SpaceUni2 space(w(0), w(1), w(2), w(3));
    out = new NearestNeighborsNigh<_T, __SpaceUni2>(space, data_to_key);
  }

  else if (startsWith(name, "quad2dpole")) {

    auto data_to_key = [robot, fun](_T const &m) {
      using Vector8d = Eigen::Matrix<double, 8, 1>;
      using Vector1d = Eigen::Matrix<double, 1, 1>;
      const ob::State *s = fun(m);
      Vector8d __x;
      robot->toEigen(s, __x);
      return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2), __x(3),
                        Eigen::Vector2d(__x(4), __x(5)), V1d(__x(6)),
                        V1d(__x(7)));
    };

    using Space = __SpaceQuad2dPole;
    DYNO_CHECK_EQ(w.size(), 6, AT);
    Space space(w(0), w(1), w(2), w(3), w(4), w(5));

    out = new NearestNeighborsNigh<_T, Space>(space, data_to_key);

  }

  else if (startsWith(name, "quad2d")) {

    auto data_to_key = [robot, fun](_T const &m) {
      using Vector6d = Eigen::Matrix<double, 6, 1>;
      const ob::State *s = fun(m);
      Vector6d __x;
      robot->toEigen(s, __x);
      return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2),
                        Eigen::Vector2d(__x(3), __x(4)), V1d(__x(5)));
    };

    DYNO_CHECK_EQ(w.size(), 4, AT);
    __SpaceQuad2d space(w(0), w(1), w(2), w(3));

    out = new NearestNeighborsNigh<_T, __SpaceQuad2d>(space, data_to_key);

    // continue here!!
  } else if (startsWith(name, "acrobot")) {

    auto data_to_key = [robot, fun](_T const &m) {
      const ob::State *s = fun(m);
      Eigen::Vector4d __x;
      robot->toEigen(s, __x);
      return std::tuple(__x(0), __x(1), Eigen::Vector2d(__x(2), __x(3)));
    };

    // out = new NearestNeighborsNigh<_T, SpaceAcrobot>(data_to_key);

    DYNO_CHECK_EQ(w.size(), 3, AT);
    __SpaceAcrobot space(w(0), w(1), w(2));

    out = new NearestNeighborsNigh<_T, __SpaceAcrobot>(space, data_to_key);

  } else if (startsWith(name, "quad3d")) {

    auto data_to_key = [robot, fun](_T const &m) {
      const ob::State *s = fun(m);
      using Vector13d = Eigen::Matrix<double, 13, 1>;
      Vector13d __x;
      robot->toEigen(s, __x);
      return std::tuple(Eigen::Vector3d(__x(0), __x(1), __x(2)),
                        Eigen::Quaterniond(__x(3), __x(4), __x(5), __x(6)),
                        Eigen::Vector3d(__x(7), __x(8), __x(9)),
                        Eigen::Vector3d(__x(10), __x(11), __x(12)));
    };

    DYNO_CHECK_EQ(w.size(), 4, AT);
    __SpaceQuad3d space(w(0), w(1), w(2), w(3));
    // out = new NearestNeighborsNigh<_T, SpaceQuad3d>(data_to_key);
    out = new NearestNeighborsNigh<_T, __SpaceQuad3d>(space, data_to_key);

  } else if (startsWith(name, "car1")) {

    auto data_to_key = [robot, fun](_T const &m) {
      const ob::State *s = fun(m);
      using Vector4d = Eigen::Matrix<double, 4, 1>;
      Vector4d __x;
      robot->toEigen(s, __x);
      return std::tuple(Eigen::Vector2d(__x(0), __x(1)), __x(2), __x(3));
    };
    // out = new NearestNeighborsNigh<_T, SpaceCar1>(data_to_key);

    DYNO_CHECK_EQ(w.size(), 3, AT);
    __SpaceCar1 space(w(0), w(1), w(2));
    // out = new NearestNeighborsNigh<_T, SpaceQuad3d>(data_to_key);
    out = new NearestNeighborsNigh<_T, __SpaceCar1>(space, data_to_key);
  }

  CHECK(out, AT);
  return out;
}

template <typename _T>
ompl::NearestNeighbors<_T> *nigh_factory2(
    const std::string &name,
    const std::shared_ptr<dynobench::Model_robot> &robot,
    std::function<const Eigen::VectorXd(_T)> fun =
        [](_T m) { return m->getStateEig(); },
    double cost_scale = -1) {
  ompl::NearestNeighbors<_T> *out = nullptr;

  auto &w = robot->distance_weights;
  // CSTR_V(w);

  if (startsWith(name, "unicycle1")) {

    if (cost_scale < 0) {
      auto data_to_key = [robot, fun](_T const &m) {
        // DYNO_CHECK_EQ(fun(m).size(), 3, ""); //
        // assert(fun(m).size() == 3);
        Eigen::Vector3d __x = fun(m);
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2));
      };

      DYNO_CHECK_EQ(w.size(), 2, AT);
      __Space space(double(w(0)), double(w(1)));

      out = new NearestNeighborsNigh<_T, __Space>(space, data_to_key);
    } else {

      std::cout << "Warning: State space with cost!" << std::endl;
      auto data_to_key = [robot, fun](_T const &m) {
        Eigen::Vector3d __x = fun(m);
        double c = m->get_cost();
        using Vector1d = Eigen::Matrix<double, 1, 1>;
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2), Vector1d(c));
      };

      DYNO_CHECK_EQ(w.size(), 2, AT);
      __SpaceWithCost space(double(w(0)), double(w(1)), cost_scale);

      out = new NearestNeighborsNigh<_T, __SpaceWithCost>(space, data_to_key);
    }

  } else if (startsWith(name, "unicycle2")) {

    if (cost_scale < 0) {
      auto data_to_key = [robot, fun](_T const &m) {
        using Vector5d = Eigen::Matrix<double, 5, 1>;
        using Vector1d = Eigen::Matrix<double, 1, 1>;
        Vector5d __x = fun(m);
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2),
                          Vector1d(__x(3)), Vector1d(__x(4)));
      };

      // out = new NearestNeighborsNigh<_T, SpaceUni2>(data_to_key);

      DYNO_CHECK_EQ(w.size(), 4, AT);
      __SpaceUni2 space(w(0), w(1), w(2), w(3));
      out = new NearestNeighborsNigh<_T, __SpaceUni2>(space, data_to_key);
    } else {

      auto data_to_key = [robot, fun](_T const &m) {
        using Vector5d = Eigen::Matrix<double, 5, 1>;
        using Vector1d = Eigen::Matrix<double, 1, 1>;
        Vector5d __x = fun(m);
        double c = m->get_cost();
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2),
                          Vector1d(__x(3)), Vector1d(__x(4)), Vector1d(c));
      };

      DYNO_CHECK_EQ(w.size(), 4, AT);
      __SpaceUni2WithCost space(w(0), w(1), w(2), w(3), cost_scale);
      out =
          new NearestNeighborsNigh<_T, __SpaceUni2WithCost>(space, data_to_key);
    }
  } else if (startsWith(name, "integrator2")) {
      auto data_to_key = [robot, fun](_T const &m) {
      using Vector4d = Eigen::Matrix<double, 4, 1>;
      Vector4d __x = fun(m);
      return std::tuple(Eigen::Vector2d(__x.head(2)),Eigen::Vector2d(__x(2), __x(3)));
    };

    DYNO_CHECK_EQ(w.size(), 2, AT);
    __SpaceIntegrator2 space(w(0), w(1));
    out =
        new NearestNeighborsNigh<_T, __SpaceIntegrator2>(space, data_to_key);
  } else if (startsWith(name, "quad2dpole")) {

    auto data_to_key = [robot, fun](_T const &m) {
      using Vector8d = Eigen::Matrix<double, 8, 1>;
      using Vector1d = Eigen::Matrix<double, 1, 1>;
      Vector8d __x = fun(m);
      return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2), __x(3),
                        Eigen::Vector2d(__x(4), __x(5)), V1d(__x(6)),
                        V1d(__x(7)));
    };

    using Space = __SpaceQuad2dPole;
    DYNO_CHECK_EQ(w.size(), 6, AT);
    Space space(w(0), w(1), w(2), w(3), w(4), w(5));

    out = new NearestNeighborsNigh<_T, Space>(space, data_to_key);

  }

  else if (startsWith(name, "quad2d")) {

    auto data_to_key = [robot, fun](_T const &m) {
      using Vector6d = Eigen::Matrix<double, 6, 1>;
      Vector6d __x = fun(m);
      return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2),
                        Eigen::Vector2d(__x(3), __x(4)), V1d(__x(5)));
    };

    DYNO_CHECK_EQ(w.size(), 4, AT);
    __SpaceQuad2d space(w(0), w(1), w(2), w(3));

    out = new NearestNeighborsNigh<_T, __SpaceQuad2d>(space, data_to_key);

    // continue here!!
  } else if (startsWith(name, "acrobot")) {

    auto data_to_key = [robot, fun](_T const &m) {
      Eigen::Vector4d __x = fun(m);
      return std::tuple(__x(0), __x(1), Eigen::Vector2d(__x(2), __x(3)));
    };

    // out = new NearestNeighborsNigh<_T, SpaceAcrobot>(data_to_key);

    DYNO_CHECK_EQ(w.size(), 3, AT);
    __SpaceAcrobot space(w(0), w(1), w(2));

    out = new NearestNeighborsNigh<_T, __SpaceAcrobot>(space, data_to_key);

  } else if (startsWith(name, "quad3d")) {

    auto data_to_key = [robot, fun](_T const &m) {
      using Vector13d = Eigen::Matrix<double, 13, 1>;
      Vector13d __x = fun(m);
      // const Eigen::VectorXd &__x = fun(m);
      return std::tuple(Eigen::Vector3d(__x(0), __x(1), __x(2)),
                        Eigen::Quaterniond(__x(3), __x(4), __x(5), __x(6)),
                        Eigen::Vector3d(__x(7), __x(8), __x(9)),
                        Eigen::Vector3d(__x(10), __x(11), __x(12)));
    };

    DYNO_CHECK_EQ(w.size(), 4, AT);
    __SpaceQuad3d space(w(0), w(1), w(2), w(3));
    // out = new NearestNeighborsNigh<_T, SpaceQuad3d>(data_to_key);
    out = new NearestNeighborsNigh<_T, __SpaceQuad3d>(space, data_to_key);

  } else if (startsWith(name, "car1")) {

    auto data_to_key = [robot, fun](_T const &m) {
      Eigen::Vector4d __x = fun(m);
      return std::tuple(Eigen::Vector2d(__x(0), __x(1)), __x(2), __x(3));
    };
    // out = new NearestNeighborsNigh<_T, SpaceCar1>(data_to_key);

    DYNO_CHECK_EQ(w.size(), 3, AT);
    __SpaceCar1 space(w(0), w(1), w(2));
    // out = new NearestNeighborsNigh<_T, SpaceQuad3d>(data_to_key);
    out = new NearestNeighborsNigh<_T, __SpaceCar1>(space, data_to_key);
  }

  CHECK(out, AT);
  return out;
}
// for tdbA* with support for reverse search
template <typename _T>
ompl::NearestNeighbors<_T> *nigh_factory_t(
    const std::string &name,
    const std::shared_ptr<dynobench::Model_robot> &robot,
    bool reverse_search,
    std::function<const Eigen::VectorXd(_T, bool)> fun =
        [](_T m, bool r_search) {if(r_search){
          return m->getLastStateEig();
        }
        else {
          return m->getStateEig();
        }},
    double cost_scale = -1) {
  ompl::NearestNeighbors<_T> *out = nullptr;

  auto &w = robot->distance_weights;
  // CSTR_V(w);

  if (startsWith(name, "unicycle1")) {

    if (cost_scale < 0) {
      auto data_to_key = [robot, fun, reverse_search](_T const &m) {
        Eigen::Vector3d __x = fun(m, reverse_search);
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2));
      };

      DYNO_CHECK_EQ(w.size(), 2, AT);
      __Space space(double(w(0)), double(w(1)));

      out = new NearestNeighborsNigh<_T, __Space>(space, data_to_key);
    } else {
      std::cout << "Warning: State space with cost!" << std::endl;
      auto data_to_key = [robot, fun, reverse_search](_T const &m) {
        Eigen::Vector3d __x = fun(m,reverse_search);
        double c = m->get_cost();
        using Vector1d = Eigen::Matrix<double, 1, 1>;
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2), Vector1d(c));
      };

      DYNO_CHECK_EQ(w.size(), 2, AT);
      __SpaceWithCost space(double(w(0)), double(w(1)), cost_scale);

      out = new NearestNeighborsNigh<_T, __SpaceWithCost>(space, data_to_key);
    }

  } 
  else if (startsWith(name, "unicycle2")) {

    if (cost_scale < 0) {
      auto data_to_key = [robot, fun, reverse_search](_T const &m) {
        using Vector5d = Eigen::Matrix<double, 5, 1>;
        using Vector1d = Eigen::Matrix<double, 1, 1>;
        Vector5d __x = fun(m, reverse_search);
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2),
                          Vector1d(__x(3)), Vector1d(__x(4)));
      };

      DYNO_CHECK_EQ(w.size(), 4, AT);
      __SpaceUni2 space(w(0), w(1), w(2), w(3));
      out = new NearestNeighborsNigh<_T, __SpaceUni2>(space, data_to_key);
    } else {

      auto data_to_key = [robot, fun, reverse_search](_T const &m) {
        using Vector5d = Eigen::Matrix<double, 5, 1>;
        using Vector1d = Eigen::Matrix<double, 1, 1>;
        Vector5d __x = fun(m, reverse_search);
        double c = m->get_cost();
        return std::tuple(Eigen::Vector2d(__x.head(2)), __x(2),
                          Vector1d(__x(3)), Vector1d(__x(4)), Vector1d(c));
      };

      DYNO_CHECK_EQ(w.size(), 4, AT);
      __SpaceUni2WithCost space(w(0), w(1), w(2), w(3), cost_scale);
      out =
          new NearestNeighborsNigh<_T, __SpaceUni2WithCost>(space, data_to_key);
    }
  } else if (startsWith(name, "integrator2")) {
      auto data_to_key = [robot, fun, reverse_search](_T const &m) {
      using Vector4d = Eigen::Matrix<double, 4, 1>;
      Vector4d __x = fun(m, reverse_search);
      return std::tuple(Eigen::Vector2d(__x.head(2)),Eigen::Vector2d(__x(2), __x(3)));
    };

    DYNO_CHECK_EQ(w.size(), 2, AT);
    __SpaceIntegrator2 space(w(0), w(1));
    out =
        new NearestNeighborsNigh<_T, __SpaceIntegrator2>(space, data_to_key);
  } else if (startsWith(name, "car1")) {

    auto data_to_key = [robot, fun, reverse_search](_T const &m) {
      Eigen::Vector4d __x = fun(m, reverse_search);
      return std::tuple(Eigen::Vector2d(__x(0), __x(1)), __x(2), __x(3));
    };
    // out = new NearestNeighborsNigh<_T, SpaceCar1>(data_to_key);

    DYNO_CHECK_EQ(w.size(), 3, AT);
    __SpaceCar1 space(w(0), w(1), w(2));
    out = new NearestNeighborsNigh<_T, __SpaceCar1>(space, data_to_key);
  }

    CHECK(out, AT);
    return out;
  } 
} // namespace dynoplan
