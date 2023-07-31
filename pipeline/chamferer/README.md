# README: Repo to perform chamfer operation on the meshes to fabricate their assembly!
## Dependices and how to run
- Relies on CGAL and LIBIGL
- `mkdir release && cd release && cmake -DCMAKE_BUILD_TYPE=Release && make -j`
- `./chamferer [path to input mesh] [path to correspoding sheet file] [path to output file]`

## To make the repo work:
- apply this hack in https://github.com/boostorg/thread/pull/297/commits/74fb0a26099bc51d717f5f154b37231ce7df3e98
[source-directory]/release/_deps/boost-src/boost/thread/pthread/thread_data.hpp