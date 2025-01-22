rm build -rf
mkdir build
cd build
cmake -S .. -DCMAKE_INSTALL_PREFIX=./install
cmake --build . --target install

cd ..
cd adaptor_test/
rm build -rf
mkdir build
cd build
syclbvh_DIR=../../build/install/lib/cmake/syclbvh cmake ..
make
