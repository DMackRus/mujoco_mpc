# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-src"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-build"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix/tmp"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix/src"
  "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/davidrussell/mujoco_mpc/cmake-build-debug/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
