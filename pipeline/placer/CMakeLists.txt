CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(Placer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic)

add_executable(Placer
	src/vorlayout_manas.cpp
	src/helper.cpp
	../third-party/HLBFGS/HLBFGS.cpp
	../third-party/HLBFGS/HLBFGS_BLAS.cpp
	../third-party/HLBFGS/ICFS.cpp
	../third-party/HLBFGS/LineSearch.cpp
	../third-party/HLBFGS/Lite_Sparse_Matrix.cpp
)

add_executable(Ledifier
	src/visualize_layout.cpp
	src/helper.cpp
)

#add_executable(Placer_marco
#	src/vorlayout_marco.cpp
#	src/helper.cpp
#	../third-party/HLBFGS/HLBFGS.cpp
#	../third-party/HLBFGS/HLBFGS_BLAS.cpp
#	../third-party/HLBFGS/ICFS.cpp
#	../third-party/HLBFGS/LineSearch.cpp
#	../third-party/HLBFGS/Lite_Sparse_Matrix.cpp
#)

#TARGET_LINK_LIBRARIES(Placer_marco PUBLIC
#  polyclipping
#  box2d
#  ${LIBSL_LIBRARIES} ${LIBSL_GL_LIBRARIES} ${LIBSL_LINALG_LIBRARIES}
#)

target_link_libraries(Placer PUBLIC
  Helpers
  polyclipping
  ${LIBSL_LIBRARIES} ${LIBSL_GL_LIBRARIES} ${LIBSL_LINALG_LIBRARIES}
  box2d
)

target_link_libraries(Ledifier PUBLIC
  Helpers
  polyclipping
  ${LIBSL_LIBRARIES} ${LIBSL_GL_LIBRARIES} ${LIBSL_LINALG_LIBRARIES}
  box2d
)

target_include_directories(Placer PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../helpers/src/
  ${PROJECT_SOURCE_DIR}/../third-party/clipper/
  ${PROJECT_SOURCE_DIR}/../third-party/HLBFGS/
  ${PROJECT_SOURCE_DIR}/../third-party/box2d/include/box2d/
  ${PROJECT_SOURCE_DIR}/../third-party/CDT/CDT/include/
  ${LIBSL_INCLUDE_DIR}
)

target_include_directories(Ledifier PUBLIC
  # ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../helpers/src/
  ${PROJECT_SOURCE_DIR}/../third-party/clipper/
  ${PROJECT_SOURCE_DIR}/../third-party/HLBFGS/
  ${PROJECT_SOURCE_DIR}/../third-party/box2d/include/box2d/
  ${PROJECT_SOURCE_DIR}/../third-party/CDT/CDT/include/
  ${LIBSL_INCLUDE_DIR}
)
