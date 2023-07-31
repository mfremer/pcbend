cmake_minimum_required(VERSION 3.10)

project(Verifier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic)

add_executable(Verifier
  src/main.cpp
)

target_link_libraries(Verifier PUBLIC
  Helpers
  polyclipping
  ${LIBSL_LIBRARIES} ${LIBSL_GL_LIBRARIES} ${LIBSL_LINALG_LIBRARIES}
)

target_include_directories(Verifier PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../helpers/src/
  ${PROJECT_SOURCE_DIR}/../third-party/clipper/
  ${LIBSL_INCLUDE_DIR}
)