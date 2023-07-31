cmake_minimum_required(VERSION 3.10)

project(Pipeline)

# list(PREPEND CMAKE_MODULE_PATH
#     ${CMAKE_CURRENT_LIST_DIR}/cmake/
#     ${CMAKE_CURRENT_LIST_DIR}/cmake/igl
#     ${CMAKE_CURRENT_LIST_DIR}/cmake/find
#     ${CMAKE_CURRENT_LIST_DIR}/cmake/recipes/external
# )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic)

add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/discorde-tsp
                 ${PROJECT_SOURCE_DIR}/third-party/discorde-tsp/build)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/clipper
                 ${PROJECT_SOURCE_DIR}/third-party/clipper/build)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/box2d
	               ${PROJECT_SOURCE_DIR}/third-party/box2d/build)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/CDT/CDT
	               ${PROJECT_SOURCE_DIR}/third-party/CDT/CDT/build)

set(LibSL_DIR ${PROJECT_SOURCE_DIR}/third-party/LibSL)
find_package(LibSL REQUIRED)

# set(libigl_DIR ${PROJECT_SOURCE_DIR}/third-party/libigl/install/lib/cmake/igl)
# find_package(libigl REQUIRED)
# find_package(OpenMP)

add_subdirectory(helpers)
add_dependencies(Helpers
  # LibSL
  polyclipping
)

# add_subdirectory(unfolder)
# add_dependencies(Unfolder
#   polyclipping
#   OpenMP::OpenMP_CXX
# )

add_subdirectory(cutter)
add_dependencies(Cutter
  Helpers
  # LibSL
  polyclipping
)

add_subdirectory(placer)
add_dependencies(Placer
  Helpers
  # LibSL
  polyclipping
  box2d
  CDT
)

add_subdirectory(router)
add_dependencies(Router
  Helpers
  # LibSL
  polyclipping
  discorde_cpp
  #  discorde_c
)

add_subdirectory(verifier)
add_dependencies(Verifier
  Helpers
  # LibSL
  polyclipping
)

add_subdirectory(layouter)
add_dependencies(Layout2SVG
  Helpers
  # LibSL
  polyclipping
)

add_subdirectory(orderer)
add_dependencies(Orderer
  Helpers
)