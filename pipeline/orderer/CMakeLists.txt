CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(Orderer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic)

add_executable(Orderer
  src/ordering_leds.cpp
)

target_link_libraries(Orderer PUBLIC
	Helpers
	# polyclipping
	# ${LIBSL_INCLUDE_DIR}
)

target_include_directories(Orderer PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../helpers/src/
#   ${PROJECT_SOURCE_DIR}/../third-party/clipper/
#   ${LIBSL_INCLUDE_DIR}
)