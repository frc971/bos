git submodule init 
git submodule update 
cmake -DENABLE_CLANG_TIDY=ON -B build -G Ninja . 
cmake --build build
cp -r constants /bos
