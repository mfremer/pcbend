cmake_minimum_required(VERSION 3.10)

project(Layouter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic)

add_executable(Layout2SVG
  src/main.cpp
  src/Layout.cpp
)

target_link_libraries(Layout2SVG PUBLIC
  Helpers
  polyclipping
  ${LIBSL_LIBRARIES} ${LIBSL_GL_LIBRARIES} ${LIBSL_LINALG_LIBRARIES}
)

target_include_directories(Layout2SVG PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../helpers/src/
  ${PROJECT_SOURCE_DIR}/../third-party/clipper/
  ${LIBSL_INCLUDE_DIR}
)
