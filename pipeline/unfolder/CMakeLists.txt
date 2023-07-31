cmake_minimum_required(VERSION 3.16)

project(Unfolder)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

################################################################################
# Libigl Options
################################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# Build options
option(LIBIGL_BUILD_TESTS        "Build libigl unit test"                 OFF)
option(LIBIGL_BUILD_TUTORIALS    "Build libigl tutorial"                  OFF)
option(LIBIGL_INSTALL            "Enable installation of libigl targets"  OFF)
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library"           ON)

# Module options
option(LIBIGL_EMBREE              "Build target igl::embree"              OFF)
option(LIBIGL_GLFW                "Build target igl::glfw"                ON)
option(LIBIGL_IMGUI               "Build target igl::imgui"               ON)
option(LIBIGL_OPENGL              "Build target igl::opengl"              ON)
option(LIBIGL_PNG                 "Build target igl::png"                 ON)
option(LIBIGL_PREDICATES          "Build target igl::predicates"          OFF)
option(LIBIGL_XML                 "Build target igl::xml"                 OFF)
option(LIBIGL_COPYLEFT_CGAL       "Build target igl_copyleft::cgal"       OFF)
option(LIBIGL_COPYLEFT_COMISO     "Build target igl_copyleft::comiso"     OFF)
option(LIBIGL_COPYLEFT_CORE       "Build target igl_copyleft::core"       OFF)
option(LIBIGL_COPYLEFT_CORK       "Build target igl_copyleft::cork"       OFF)
option(LIBIGL_COPYLEFT_TETGEN     "Build target igl_copyleft::tetgen"     OFF)
option(LIBIGL_RESTRICTED_MATLAB   "Build target igl_restricted::matlab"   OFF)
option(LIBIGL_RESTRICTED_MOSEK    "Build target igl_restricted::mosek"    OFF)
option(LIBIGL_RESTRICTED_TRIANGLE "Build target igl_restricted::triangle" OFF)

add_definitions ( -DIGL_VIEWER_VIEWER_QUIET=1 ) # this basically shuts down the usage commands in the terminal 

include(libigl)
igl_include(opengl)
igl_include(glfw)
igl_include(imgui)
igl_include(png)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# add_compile_options(-Wall -Wextra -pedantic)

# Add your project files
add_executable(Unfolder 
  src/main.cpp
  src/mesh.cpp
  src/mst.cpp
  src/dualMesh.cpp
  src/triangleHelper.cpp
  src/unfold.cpp
  ${PROJECT_SOURCE_DIR}/../helpers/src/Settings.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  # Helpers
  # polyclipping
  igl::core igl::opengl igl::glfw igl::imgui igl::png
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../helpers/src
)