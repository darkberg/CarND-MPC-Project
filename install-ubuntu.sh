#! /bin/bash
sudo apt-get install libuv1-dev
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
make 
sudo make install
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
