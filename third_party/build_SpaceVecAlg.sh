# !/bin/bash

# install third party libraries from source when BUILD_THIRD_PARTY=ON

# clone repository and switch to the right tag
git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg
cd SpaceVecAlg
git checkout v1.1.0

mkdir build -p && cd build
cmake .. $@ -DPYTHON_BINDING=OFF
make -j8
make install

if [ $? != 0 ]; then
    echo "Installing with sudo ........."
    sudo make install
fi