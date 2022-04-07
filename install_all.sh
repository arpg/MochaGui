cd ..
sudo apt update

sudo apt install libzmqpp-dev libavahi-compat-libdnssd-dev libprotobuf-dev libprotoc-dev protobuf-compiler
git clone git@github.com:arpg/node
cd node
git checkout fix/install_mochagui
mkdir build
cd build
cmake .. -DEXPORT_Node=ON
make
cd ../..

git clone git@github.com:arpg/calibu
cd calibu
mkdir build
cd build
cmake .. 
make
cd ../..

sudo apt install libopencv-dev
sudo ln -s /usr/include/opencv/opencv2 /usr/include/opencv2
# Needed to create symbolic link from /usr/local/lib/libquat.a to /opt/ros/melodic/lib/libquat.a.
sudo apt install ros-melodic-vrpn-client-ros
git clone git@github.com:arpg/hal
cd hal
mkdir build
cd build
cmake .. -DEXPORT_HAL=ON
make
cd ../..

git clone git@github.com:arpg/cvars
cd cvars
mkdir build
cd build
cmake .. -DEXPORT_CVars=ON
make
cd ../..

git clone git@github.com:arpg/pangolin
cd pangolin
mkdir build
cd build
cmake .. -DEXPORT_Pangolin=ON
make
cd ../..

sudo apt install libglew-dev libdevil-dev
git clone git@github.com:arpg/scenegraph
cd scenegraph
mkdir build
cd build
cmake .. -DEXPORT_SceneGraph=ON
make
cd ../..

git clone git@github.com:arpg/bullet3
cd bullet3
mkdir build
cd build
cmake .. -DUSE_DOUBLE_PRECISION=ON -DBUILD_SHARED_LIBS=ON
make
cd ../..

git clone git@github.com:arpg/carplanner
cd carplanner
git checkout fix/install_mochagui
mkdir build
cd build
cmake .. -DEXPORT_CarPlanner=ON -DBULLET_COLLISION_LIBRARY=../../bullet3/build/src/BulletCollision/libBulletCollision.so -DBULLET_DYNAMICS_LIBRARY=../../bullet3/build/src/BulletDynamics/libBulletDynamics.so -DBULLET_INCLUDE_DIR=../../bullet3/src -DBULLET_MATH_LIBRARY=../../bullet3/build/src/LinearMath/libLinearMath.so -DBULLET_SOFTBODY_LIBRARY=../../bullet3/build/src/BulletSoftBody/libBulletSoftBody.so -Dassimp_INCLUDE_DIRS=/usr/include/assimp/
make
cd ../..

git clone git@github.com:arpg/mochagui
cd mochagui
git checkout fix/install_mochagui
mkdir build
cd build
cmake .. -DBULLET_COLLISION_LIBRARY=../../bullet3/build/src/BulletCollision/libBulletCollision.so -DBULLET_DYNAMICS_LIBRARY=../../bullet3/build/src/BulletDynamics/libBulletDynamics.so -DBULLET_INCLUDE_DIR=../../bullet3/src -DBULLET_MATH_LIBRARY=../../bullet3/build/src/LinearMath/libLinearMath.so -DBULLET_SOFTBODY_LIBRARY=../../bullet3/build/src/BulletSoftBody/libBulletSoftBody.so -Dassimp_INCLUDE_DIRS=/usr/include/assimp/
make
cd ../..
