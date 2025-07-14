#!/bin/bash
set -eux

DIR=$(pwd)

# 清理旧构建目录
rm -rf build lib
mkdir -p build lib/lib_arm lib/lib_x86 include/protocol src/protocol
rm -rf include/protocol/*.pb.h src/protocol/*.pb.cc

protoc=protoc

# === 生成 patac_hpp proto ===
cd protocol
mkdir -p generate/patac_hpp
$protoc -I=./patac_hpp/ --cpp_out=./generate/patac_hpp/ ./patac_hpp/*.proto
cd $DIR

# === 拷贝生成的 C++ 源码到目标目录 ===
cp protocol/generate/patac_hpp/*.pb.h include/protocol/
cp protocol/generate/patac_hpp/*.pb.cc src/protocol/

# === 编译可执行程序 main_exec ===
cd build
cmake ..
make -j$(nproc)

# === 拷贝生成的动态库 ===
if [ -f lib/libmain_exec.so ]; then
  cp -d lib/*.so* ../lib/lib_x86
fi

cd $DIR
