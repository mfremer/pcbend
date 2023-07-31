cmake_minimum_required(VERSION 3.10)

project(Router)

set(DISCORDE_DIR ${PROJECT_SOURCE_DIR}/../third-party/discorde-tsp)
set(CONCORDE_DIR ${PROJECT_SOURCE_DIR}/../third-party/discorde-tsp/concorde/build)
set(QSOPT_DIR    ${PROJECT_SOURCE_DIR}/../third-party/discorde-tsp/qsopt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(Router
  src/main.cpp
  src/extended_tour.cpp
  src/globals.cpp
  ${PROJECT_SOURCE_DIR}/../helpers/src/FaceInfo.cpp
  ${PROJECT_SOURCE_DIR}/../helpers/src/ModuleInfo.cpp
)

target_link_libraries(Router PUBLIC
  Helpers
  polyclipping
  ${LIBSL_LIBRARIES}
#  discorde_cpp
#  discorde_c
  ${DISCORDE_DIR}/build/source/discorde-cpp-api/libdiscorde_cpp.a
  ${DISCORDE_DIR}/build/source/discorde-c-api/libdiscorde.a
  ${CONCORDE_DIR}/concorde.a
  ${QSOPT_DIR}/qsopt.a)
  
target_include_directories(Router PUBLIC
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/../helpers/src/
  ${PROJECT_SOURCE_DIR}/../third-party/clipper/
  ${LIBSL_INCLUDE_DIR}
  ${DISCORDE_DIR}/source/discorde-cpp-api
  ${DISCORDE_DIR}/source/discorde-c-api
  ${CONCORDE_DIR}
  ${QSOPT_DIR})
