cd build
cd _deps
rm -rf mujoco-build
rm -rf mujoco-src
rm -rf mujoco-subbuild
cd ..
cmake --build . --target mjpc
cd ..
./build/bin/mjpc

