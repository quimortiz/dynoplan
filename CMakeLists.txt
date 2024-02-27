# I have followed the tutorial from here:
# https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

cmake_minimum_required(VERSION 3.5)
project(
  idbastar
  VERSION 0.1.0
  LANGUAGES CXX)

# find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options serialization
                                       stacktrace_basic)
find_package(ompl REQUIRED)

include(FindThreads)

message(STATUS "OMPL_FOUND: ${OMPL_FOUND}")
message(STATUS "OMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPL_LIBRARIES: ${OMPL_LIBRARIES}")
message(STATUS "OMPL_VERSION: ${OMPL_VERSION}")

find_package(fcl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(crocoddyl REQUIRED)

message(STATUS "CROCO_VERSION: ${CROCODDYL_VERSION}")

find_package(Eigen3 REQUIRED)

message(STATUS "FCL_INCLUDE_DIRS: ${FCL_INCLUDE_DIRS}")
message(STATUS "FCL_LIBRARIES: ${FCL_LIBRARIES}")
message(STATUS "FCL_VERSION: ${FCL_VERSION}")

# TODO: check why this is necessary
message(STATUS "Finding LZ4 libraries...")
find_library(
  LZ4_LIBRARIES
  NAMES liblz4.so
  HINTS "lz4/lib")
if(LZ4_LIBRARIES)
  message(STATUS "Found: ${LZ4_LIBRARIES}")
else()
  message(STATUS "Not found: ${LZ4_LIBRARIES}")
  message(FATAL_ERROR "Cannot find required LZ4 libraries")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set(DISABLE_DYNOBENCH_TESTS ON)
add_subdirectory(dynobench)

add_executable(main_rrt_to ./src/ompl/main_rrt_to.cpp)
add_executable(main_primitives ./src/motion_primitives/main_primitives.cpp)
add_executable(from_boost_to_msgpack
               ./src/motion_primitives/from_boost_to_msgpack.cpp)

add_executable(main_sst ./src/ompl/main_sst.cpp)
add_executable(main_optimization ./src/optimization/main_optimization.cpp)
add_executable(main_multirobot_optimization
               ./src/optimization/main_multirobot_optimization.cpp)

add_executable(main_dbastar ./src/dbastar/main_dbastar.cpp)
add_executable(main_tdbastar ./src/tdbastar/main_tdbastar.cpp)
add_executable(main_idbastar ./src/idbastar/main_idbastar.cpp)
add_executable(main_dbrrt ./src/dbrrt/main_dbrrt.cpp)

# target_compile_options(main PRIVATE -Wall -Wextra)

add_library(motion_primitives ./src/motion_primitives/motion_primitives.cpp)

add_library(
  optimization
  ./src/optimization/ocp.cpp ./src/optimization/options.cpp
  ./src/optimization/croco_models.cpp ./src/optimization/generate_ocp.cpp
  ./src/optimization/multirobot_optimization.cpp)

add_library(sst ./src/ompl/sst.cpp ./src/ompl/robots.cpp)
add_library(rrt_to ./src/ompl/rrt_to.cpp ./src/ompl/robots.cpp)

add_library(dbastar ./src/dbastar/dbastar.cpp ./src/dbastar/options.cpp
                    ./src/ompl/robots.cpp ./src/dbastar/heuristics.cpp)

add_library(tdbastar ./src/tdbastar/tdbastar.cpp ./src/ompl/robots.cpp
                     ./src/tdbastar/options.cpp ./src/dbastar/heuristics.cpp)

add_library(
  tdbastar_epsilon
  ./src/tdbastar/tdbastar_epsilon.cpp ./src/ompl/robots.cpp
  ./src/tdbastar/options.cpp ./src/dbastar/heuristics.cpp
  ./src/tdbastar/tdbastar.cpp)

add_library(dbrrt ./src/dbrrt/dbrrt.cpp ./src/ompl/robots.cpp)

add_library(idbastar ./src/idbastar/idbastar.cpp)

add_library(idbastar::motion_primitives ALIAS motion_primitives) # for use in
                                                                 # test folder
add_library(idbastar::optimization ALIAS optimization) # for use in test folder
add_library(idbastar::dbastar ALIAS dbastar) # for use in test folder

add_library(idbastar::dbrrt ALIAS dbrrt) # for use in test folder
add_library(idbastar::rrt_to ALIAS rrt_to) # for use in test folder
add_library(idbastar::idbastar ALIAS idbastar) # for use in test folder
add_library(idbastar::sst ALIAS sst) # for use in test folder

target_include_directories(
  optimization
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${CROCODDYL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_include_directories(
  main_optimization
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_include_directories(
  main_multirobot_optimization
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_include_directories(
  sst
  PUBLIC $<INSTALL_INTERFACE:include> ${OMPL_INCLUDE_DIRS}
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_include_directories(
  rrt_to
  PUBLIC $<INSTALL_INTERFACE:include> ${OMPL_INCLUDE_DIRS}
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_include_directories(
  motion_primitives
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS} ${CROCODDYL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_include_directories(
  dbastar
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_include_directories(
  dbrrt
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

# target_include_directories(main_dbastar PUBLIC $<INSTALL_INTERFACE:include>
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> PRIVATE
# ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src )
#

target_link_libraries(
  idbastar
  PUBLIC dbastar optimization Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

target_link_libraries(
  sst
  PUBLIC optimization Eigen3::Eigen dynobench::dynobench ${OMPL_LIBRARIES}
  PRIVATE fcl ${LZ4_LIBRARIES})

target_link_libraries(
  rrt_to
  PUBLIC optimization Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

target_link_libraries(
  dbastar
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

target_link_libraries(
  dbrrt
  PUBLIC optimization Eigen3::Eigen dynobench::dynobench dbastar
  PRIVATE fcl ${OMPL_LIBRARIES})

target_link_libraries(
  main_dbastar
  PUBLIC Eigen3::Eigen dbastar
  PRIVATE fcl dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
          Boost::serialization ${LZ4_LIBRARIES})

target_link_libraries(
  main_idbastar
  PUBLIC Eigen3::Eigen idbastar
  PRIVATE fcl dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
          Boost::serialization ${LZ4_LIBRARIES})

target_link_libraries(
  main_dbrrt
  PUBLIC dbrrt dbastar Eigen3::Eigen
  PRIVATE fcl dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
          Boost::serialization ${LZ4_LIBRARIES})

target_link_libraries(
  main_sst
  PUBLIC sst Eigen3::Eigen
  PRIVATE fcl dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
          Boost::serialization ${LZ4_LIBRARIES})

target_link_libraries(
  main_rrt_to
  PUBLIC rrt_to Eigen3::Eigen
  PRIVATE fcl dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
          Boost::serialization ${LZ4_LIBRARIES})

target_link_libraries(
  motion_primitives
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE optimization
          ${CROCODDYL_LIBRARIES}
          fcl
          yaml-cpp
          Boost::program_options
          Boost::serialization
          Threads::Threads)

target_link_libraries(
  main_primitives PRIVATE motion_primitives optimization dynobench::dynobench
                          Boost::program_options Boost::serialization)
target_link_libraries(
  from_boost_to_msgpack
  PRIVATE motion_primitives optimization dynobench::dynobench
          Boost::program_options Boost::serialization)

target_link_libraries(
  optimization
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE ${CROCODDYL_LIBRARIES} fcl yaml-cpp Boost::program_options
          Boost::serialization libipopt.so)

target_link_libraries(
  main_optimization
  PRIVATE ${CROCODDYL_LIBRARIES}
          optimization
          Eigen3::Eigen
          dynobench::dynobench
          fcl
          yaml-cpp
          Boost::program_options
          Boost::serialization)

target_link_libraries(
  main_multirobot_optimization
  PRIVATE ${CROCODDYL_LIBRARIES}
          optimization
          Eigen3::Eigen
          dynobench::dynobench
          fcl
          yaml-cpp
          Boost::program_options
          Boost::serialization)

target_link_libraries(
  main_tdbastar
  PUBLIC Eigen3::Eigen tdbastar
  PRIVATE fcl dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
          Boost::serialization ${LZ4_LIBRARIES})

# for tdb-A*
add_library(idbastar::tdbastar ALIAS tdbastar) # for use in test folder
add_library(idbastar::tdbastar_epsilon ALIAS tdbastar_epsilon)

target_include_directories(
  tdbastar
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_link_libraries(
  tdbastar
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

# for tdb-A*-epsilon
add_library(idbastar::tdbastar_epsilon ALIAS tdbastar_epsilon) # for use in test
                                                               # folder

target_include_directories(
  tdbastar_epsilon
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_link_libraries(
  tdbastar_epsilon
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

# target_link_libraries( main_tdbastar PUBLIC Eigen3::Eigen tdbastar PRIVATE fcl
# dynobench::dynobench ${OMPL_LIBRARIES} Boost::program_options
# Boost::serialization ${LZ4_LIBRARIES})

enable_testing()
add_subdirectory(test)
