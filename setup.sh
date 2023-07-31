#!/bin/bash

THREADS=${1:-1}

# One-time setup
if [ ! -d "pipeline/third-party/discorde-tsp/concorde/build" ]; then
  # git
  git submodule update --init

  # Concorde compilation
  (cd pipeline/third-party/discorde-tsp/concorde;\
  mkdir build; cd build;\
  linux32 ../src/configure --with-qsopt=$(dirname $(dirname `pwd`))/qsopt;
  make -j "$THREADS")

  # LibSL
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -S pipeline/third-party/LibSL -B pipeline/third-party/LibSL/build
  cmake --build pipeline/third-party/LibSL/build -j "$THREADS"
  cmake --install pipeline/third-party/LibSL/build
fi

# cmake configs
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -S pipeline -B pipeline/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DFETCHCONTENT_SOURCE_DIR_LIBIGL=pipeline/third-party/libigl -S pipeline/unfolder -B pipeline/build/unfolder
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DFETCHCONTENT_SOURCE_DIR_LIBIGL=pipeline/third-party/libigl -S pipeline/chamferer -B pipeline/build/chamferer
