git submodule init 
git submodule update 
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DENABLE_CLANG_TIDY=OFF \
  -DABSL_BUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS_DEBUG="-O0 -g -fno-inline" \
  -DCMAKE_C_FLAGS_DEBUG="-O0 -g -fno-inline"
cmake --build build
mkdir -p /bos 
if [ "$(pwd -P)/constants" != "/bos/constants" ]; then
  cp -r constants /bos
fi