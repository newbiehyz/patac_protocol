#!/bin/bash


CPP_DIR="./cpp_gen"
PYTHON_DIR="./python_gen"

if [ -d "$CPP_DIR" ]; then
    echo "Directory $CPP_DIR exists. Removing it."
    rm -rf "$CPP_DIR"
fi

if [ -d "$PYTHON_DIR" ]; then
    echo "Directory $PYTHON_DIR exists. Removing it."
    rm -rf "$PYTHON_DIR"
fi
# === 清理旧生成目录，只保留 patac_hpp ===

mkdir $CPP_DIR
mkdir $PYTHON_DIR


# === 生成 C++ 头文件和源码 ===
protoc -I=./protocol/patac_hpp/ --cpp_out=./cpp_gen/ ./protocol/patac_hpp/*.proto

# === 生成 Python 文件 ===
protoc -I=./protocol/patac_hpp/ --python_out=./python_gen/ ./protocol/patac_hpp/*.proto

echo "Protocol buffers (patac_hpp) generated and saved to ${CPP_DIR} and ${PYTHON_DIR}"
