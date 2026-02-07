git submodule init 
git submodule update 
cmake -DENABLE_CLANG_TIDY=OFF -DABSL_BUILD_TESTING=OFF -B build -G Ninja . 
cmake --build build
mkdir -p /bos 
cp -r constants /bos
