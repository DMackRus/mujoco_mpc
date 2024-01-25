# CMake generated Testfile for 
# Source directory: /home/davidrussell/mujoco_mpc/mjpc/test/ilqg_planner
# Build directory: /home/davidrussell/mujoco_mpc/cmake-build-debug/mjpc/test/ilqg_planner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(iLQGTest.Particle "/home/davidrussell/mujoco_mpc/cmake-build-debug/bin/ilqg_test" "--gtest_filter=iLQGTest.Particle")
set_tests_properties(iLQGTest.Particle PROPERTIES  WORKING_DIRECTORY "/home/davidrussell/mujoco_mpc/mjpc/test" _BACKTRACE_TRIPLES "/opt/clion-2022.3.3/bin/cmake/linux/x64/share/cmake-3.24/Modules/GoogleTest.cmake;400;add_test;/home/davidrussell/mujoco_mpc/mjpc/test/CMakeLists.txt;31;gtest_add_tests;/home/davidrussell/mujoco_mpc/mjpc/test/ilqg_planner/CMakeLists.txt;15;test;/home/davidrussell/mujoco_mpc/mjpc/test/ilqg_planner/CMakeLists.txt;0;")
add_test(iLQGTest.BackwardPass "/home/davidrussell/mujoco_mpc/cmake-build-debug/bin/backward_pass_test" "--gtest_filter=iLQGTest.BackwardPass")
set_tests_properties(iLQGTest.BackwardPass PROPERTIES  WORKING_DIRECTORY "/home/davidrussell/mujoco_mpc/mjpc/test" _BACKTRACE_TRIPLES "/opt/clion-2022.3.3/bin/cmake/linux/x64/share/cmake-3.24/Modules/GoogleTest.cmake;400;add_test;/home/davidrussell/mujoco_mpc/mjpc/test/CMakeLists.txt;31;gtest_add_tests;/home/davidrussell/mujoco_mpc/mjpc/test/ilqg_planner/CMakeLists.txt;18;test;/home/davidrussell/mujoco_mpc/mjpc/test/ilqg_planner/CMakeLists.txt;0;")
