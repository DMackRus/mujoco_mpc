# CMake generated Testfile for 
# Source directory: /home/davidrussell/mujoco_mpc/mjpc/test/planners/robust
# Build directory: /home/davidrussell/mujoco_mpc/cmake-build-debug/mjpc/test/planners/robust
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(RobustPlannerTest.RandomSearch "/home/davidrussell/mujoco_mpc/cmake-build-debug/bin/robust_planner_test" "--gtest_filter=RobustPlannerTest.RandomSearch")
set_tests_properties(RobustPlannerTest.RandomSearch PROPERTIES  WORKING_DIRECTORY "/home/davidrussell/mujoco_mpc/mjpc/test" _BACKTRACE_TRIPLES "/opt/clion-2022.3.3/bin/cmake/linux/x64/share/cmake-3.24/Modules/GoogleTest.cmake;400;add_test;/home/davidrussell/mujoco_mpc/mjpc/test/CMakeLists.txt;31;gtest_add_tests;/home/davidrussell/mujoco_mpc/mjpc/test/planners/robust/CMakeLists.txt;15;test;/home/davidrussell/mujoco_mpc/mjpc/test/planners/robust/CMakeLists.txt;0;")
