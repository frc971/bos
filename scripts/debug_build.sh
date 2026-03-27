git submodule init 
git submodule update 
cmake \
  -DENABLE_CLANG_TIDY=OFF \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS_DEBUG="-g -O0" \
  -DCMAKE_CXX_FLAGS="-O0 -g -fno-lto" \
  -B debug_build -G Ninja .
cmake --build debug_build
mkdir -p /bos 
if [ "$(realpath constants)" != "$(realpath /bos/constants 2>/dev/null)" ]; then
  cp -r constants /bos
fi
