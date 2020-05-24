# !/bin/bash

# Updates the name of this project to a new name. Mainly used to create libraries

# check input for project name
if [ -z "$1" ]; then
    echo "New project name required!"
    echo "Usage: bash create_lib.sh <new project name>"
    exit
fi

curr_dir=`pwd`

echo "Creating lib $1"

echo "Updating CMakeLists.txt"
cd $curr_dir
sed -i 's/exp_proj/'$1'/g' CMakeLists.txt

echo "Updating /cmake folder"
cd $curr_dir/cmake
sed -i 's/exp_proj/'$1'/g' config.h.in
sed -i 's/EXP_PROJ/'${1^^}'/g' config.h.in
for file in *.cmake.in ; do mv $file ${file//exp_proj/"$1"} ; done

echo "Updating /src folder"
cd $curr_dir/src
sed -i 's/exp_proj/'$1'/g' exp_class.cc

echo "Updating /include/robotics folder"
cd $curr_dir/include/robotics
mv exp_proj $1
cd $1
sed -i 's/exp_proj/'$1'/g' exp_class.h
sed -i 's/EXP_PROJ/'${1^^}'/g' exp_class.h

echo "Updating /apps folder"
cd $curr_dir/apps
sed -i 's/exp_proj/'$1'/g' CMakeLists.txt
sed -i 's/exp_proj/'$1'/g' app.cc
sed -i 's/EXP_PROJ/'${1^^}'/g' app.cc

echo "Updating /test folder"
cd $curr_dir/test
sed -i 's/exp_proj/'$1'/g' exp_class_test.cc

echo "Updating Doxyfile"
cd $curr_dir
str=`echo -e ${1//_/ } | sed -r 's/\<./\U&/g'`
sed -i "s/Exp Proj/${str}/g" Doxyfile

cd $curr_dir
