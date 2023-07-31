cmake_minimum_required(VERSION 3.9)


# =============================================================================
# Project info
# =============================================================================

project("Discorde API")


# =============================================================================
# Projects
# =============================================================================

set(CONCORDE_DIR ${PROJECT_SOURCE_DIR}/concorde/build)

add_subdirectory(source/discorde-c-api)
add_subdirectory(source/discorde-cpp-api)
