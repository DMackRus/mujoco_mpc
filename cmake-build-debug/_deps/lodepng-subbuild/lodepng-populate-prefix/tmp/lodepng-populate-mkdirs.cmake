# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-src"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-build"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix/tmp"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix/src"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
