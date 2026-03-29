git submodule update --init --depth 1
cmake -DENABLE_CLANG_TIDY=OFF -B build -G Ninja . 
cmake --build build
mkdir -p /bos 
cp -r constants /bos
