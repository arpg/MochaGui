cd ~/code/CarPlanner
mkdir -p build
cd build
cmake .. -DEXPORT_carplanner=ON
make -j8
