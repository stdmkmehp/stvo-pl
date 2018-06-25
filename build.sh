echo "Building 3rdparty/line_descriptor ... "
cd 3rdparty/line_descriptor
rm -rf build
mkdir build
cd build
cmake ..
make -j
cd ../../../

echo "Building StVO-PL ... "
mkdir build
cd build
cmake ..
make -j
