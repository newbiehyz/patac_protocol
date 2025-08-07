#!/bin/bash

CPP_DIR="./patac_protocol/cpp_gen"
if [ -d "$CPP_DIR" ]; then
    echo "Directory $CPP_DIR exists. Removing it."
    rm -rf "$CPP_DIR"
fi
mkdir -p $CPP_DIR

# 清理旧构建目录
rm -rf patac_protocol/include/protocol/*.pb.h patac_protocol/src/protocol/*.pb.cc

# === 生成 C++ 头文件和源码 ===
./patac_protocol/protoc-3.19.6-linux-x86_64/bin/protoc -I=./patac_protocol/protocol/patac_hpp/ --cpp_out=$CPP_DIR ./patac_protocol/protocol/patac_hpp/*.proto


# === 拷贝生成的 C++ 源码到目标目录 ===
cp patac_protocol/cpp_gen/*.pb.h patac_protocol/include/protocol/
cp patac_protocol/cpp_gen/*.pb.cc patac_protocol/src/protocol/

echo "Protocol buffers (patac_hpp) generated and saved to ${CPP_DIR}"

cd build
cmake ..
make -j$(nproc)
