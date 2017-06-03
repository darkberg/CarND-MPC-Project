#! /bin/bash

git clone https://github.com/karolmajek/CarND-MPC-Project.git
cd CarND-MPC-Project/
sed -i.bak s/sudo/\ /g install-ubuntu.sh
./install-ubuntu.sh
mkdir build
cd build
cmake ..
make
ls -la
ifconfig
./mpc
