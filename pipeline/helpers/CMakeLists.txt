cmake_minimum_required(VERSION 3.10)

project(Helpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic)

add_library(Helpers
  src/FaceInfo.cpp
  src/ModuleInfo.cpp
  src/layout_helpers.cpp
  src/Settings.cpp
  src/Sheet.cpp
  src/svgwriter.cpp
  # src/manas_helper.cpp
)

target_link_libraries(Helpers PUBLIC
  polyclipping
  ${LIBSL_LIBRARIES} ${LIBSL_GL_LIBRARIES} ${LIBSL_LINALG_LIBRARIES}
)
  
target_include_directories(Helpers PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../third-party/clipper/
  ${LIBSL_INCLUDE_DIR}
)
