# !/bin/bash

# install third party libraries from source when BUILD_THIRD_PARTY=ON

# clone repository and switch to the right tag
git clone https://github.com/gabime/spdlog.git
cd spdlog
git checkout v1.6.0

mkdir build -p && cd build
cmake .. $@
make -j8
make install

if [ $? != 0 ]; then
    echo "Installing with sudo ........."
    sudo make install
fi