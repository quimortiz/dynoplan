set(${PROJECT_NAME}_FACTORY_TEST
    factory/crocoddyl/pinocchio_model.hpp
    factory/crocoddyl/state.hpp
    factory/crocoddyl/activation.hpp
    factory/crocoddyl/impulse.hpp
    factory/crocoddyl/cost.hpp
    factory/crocoddyl/action.hpp
    factory/solver.hpp
    factory/crocoddyl/pinocchio_model.cpp
    factory/crocoddyl/state.cpp
    factory/crocoddyl/activation.cpp
    factory/crocoddyl/impulse.cpp
    factory/crocoddyl/cost.cpp
    factory/crocoddyl/action.cpp
    factory/solver.cpp)

add_library(mim_unittest SHARED ${${PROJECT_NAME}_FACTORY_TEST})

target_link_libraries(
  mim_unittest PUBLIC mim_solvers Eigen3::Eigen crocoddyl::crocoddyl
                      example-robot-data::example-robot-data)

add_executable(test_solvers test_solvers.cpp)

# add_unit_test(${NAME} ${NAME}.cc) target_link_libraries(${NAME}
# ${PUBLIC_KEYWORD} ${LIB})
#

target_link_libraries(test_solvers PUBLIC mim_unittest)

# if(BUILD_PYTHON_INTERFACE) add_subdirectory(python) endif()
