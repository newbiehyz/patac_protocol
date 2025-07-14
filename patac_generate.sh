#!/bin/bash
set -eux

DATE=$(date +%Y%m%d%H%M%S)
OUTDIR="./output/${DATE}"
mkdir -p "${OUTDIR}"

protoc=protoc

# 清理旧生成目录
rm -rf ./protocol/generate
mkdir -p ./protocol/generate/type ./protocol/generate/model ./protocol/generate/interface ./protocol/generate/patac_hpp
mkdir -p include/protocol src/protocol

# === 生成 C++ 头文件和源码 ===
$protoc -I=./protocol/type/ --cpp_out=./protocol/generate/type/ ./protocol/type/*.proto
$protoc -I=./protocol/model/ -I=./protocol/type/ --cpp_out=./protocol/generate/model/ ./protocol/model/*.proto
$protoc -I=./protocol/interface/ -I=./protocol/type/ --cpp_out=./protocol/generate/interface/ ./protocol/interface/*.proto
$protoc -I=./protocol/patac_hpp/ -I=./protocol/type/ --cpp_out=./protocol/generate/patac_hpp/ ./protocol/patac_hpp/*.proto

# === 生成 Python 文件 ===
$protoc -I=./protocol/type/ \
        -I=./protocol/model/ \
        -I=./protocol/interface/ \
        -I=./protocol/patac_hpp/ \
        --python_out=./protocol/generate/ \
        ./protocol/type/*.proto \
        ./protocol/model/*.proto \
        ./protocol/interface/*.proto \
        ./protocol/patac_hpp/*.proto

# === 拷贝以 patac 开头的 proto、pb.h、pb.cc、py 到输出目录 ===
find protocol/ -name "patac*.proto" -exec cp {} "${OUTDIR}/" \;
find protocol/generate/ -name "patac*.pb.h" -exec cp {} "${OUTDIR}/" \;
find protocol/generate/ -name "patac*.pb.cc" -exec cp {} "${OUTDIR}/" \;
find protocol/generate/ -name "patac*.py" -exec cp {} "${OUTDIR}/" \;

echo "Protocol buffers generated and saved to ${OUTDIR}"
