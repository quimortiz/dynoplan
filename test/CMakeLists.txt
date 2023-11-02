# see /opt/local/share/cmake-3.16/Modules/FindGTest.cmake find_package(boot
# QUIET)
#

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(
  test_optimization
  ./optimization/test_optimization_base.cpp 
  ./optimization/test_optimization_0.cpp 
  ./optimization/test_optimization_1.cpp
  ./optimization/test_optimization_cli.cpp)

add_executable(
  test_dbastar dbastar/test_dbastar_base.cpp dbastar/test_dbastar_0.cpp
               dbastar/test_dbastar_cli.cpp)

add_executable(test_sst sst/test_sst_base.cpp sst/test_sst_0.cpp
                        sst/test_sst_cli.cpp)

add_executable(test_dbrrt dbrrt/test_dbrrt_base.cpp dbrrt/test_dbrrt_0.cpp
                          dbrrt/test_dbrrt_cli.cpp)

add_executable(
  test_primitives
  primitives/test_primitives_base.cpp primitives/test_primitives_0.cpp
  primitives/test_primitives_cli.cpp)

add_executable(
  test_idbastar idbastar/test_idbastar_base.cpp idbastar/test_idbastar_0.cpp
                idbastar/test_idbastar_cli.cpp)

add_executable(test_rrt_to rrt_to/test_rrt_to_base.cpp rrt_to/test_rrt_to_0.cpp
                           rrt_to/test_rrt_to_cli.cpp)

target_link_libraries(test_idbastar idbastar::idbastar
                      Boost::unit_test_framework)

target_link_libraries(test_sst idbastar::sst Boost::unit_test_framework)

target_link_libraries(test_primitives idbastar::motion_primitives
                      Boost::unit_test_framework)
target_link_libraries(test_optimization idbastar::optimization
                      Boost::unit_test_framework)
target_link_libraries(test_dbastar idbastar::dbastar Boost::unit_test_framework)

target_link_libraries(test_rrt_to idbastar::rrt_to Boost::unit_test_framework
                      ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

target_link_libraries(test_dbrrt idbastar::dbrrt Boost::unit_test_framework
                      ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

add_test(NAME test_sst COMMAND test_sst)
add_test(NAME test_rrt_to COMMAND test_rrt_to)
add_test(NAME test_optimization COMMAND test_optimization)
add_test(NAME test_dbastar COMMAND test_dbastar)
add_test(NAME test_idbastar COMMAND test_idbastar)
add_test(NAME test_dbrrt COMMAND test_dbrrt)
