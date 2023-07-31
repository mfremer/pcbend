# Simple unfolder

Simple mesh unfolder implemented using libigl and is based on the work done by Takahashi et. al. on Optimized Topological Surgery for Unfolding 3D Meshes : https://www.u-aizu.ac.jp/~shigeo/research/unfolding/pg2011u-preprint.pdf

# Release folder

- cmake -DCMAKE_BUILD_TYPE:STRING="Release" \
	  -G Ninja \
      -DCMAKE_CXX_FLAGS:STRING="-march=native -m64 -Ofast -flto"  \
      -DLIBIGL_USE_STATIC_LIBRARY=ON\
      -DCMAKE_EXE_LINKER_FLAGS:STRING="-Wl,--allow-multiple-definition" .. 

- ninja
- ./simple-unfolding