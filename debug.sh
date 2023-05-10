# rm -r build
cmake -DBUILD_DEBUG=ON -S . -B build
cmake --build build