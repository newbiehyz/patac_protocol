#!/bin/bash
set -eux

DATE=$(date +%Y%m%d%H%M%S)
OUTDIR="./output/${DATE}"
mkdir -p "${OUTDIR}"

protoc=protoc

# === 清理旧生成目录，只保留 patac_hpp ===
rm -rf ./protocol/generate
mkdir -p ./protocol/generate/patac_hpp
mkdir -p include/protocol src/protocol

# === 生成 C++ 头文件和源码 ===
$protoc -I=./protocol/patac_hpp/ --cpp_out=./protocol/generate/patac_hpp/ ./protocol/patac_hpp/*.proto

# === 生成 Python 文件 ===
$protoc -I=./protocol/patac_hpp/ --python_out=./protocol/generate/patac_hpp/ ./protocol/patac_hpp/*.proto

# === 拷贝 pb.h / pb.cc 到 include/src 目录 ===
cp ./protocol/generate/patac_hpp/*.pb.h include/protocol/
cp ./protocol/generate/patac_hpp/*.pb.cc src/protocol/

# === 拷贝 patac 系列文件到输出目录 ===
find protocol/patac_hpp/ -name "patac*.proto" -exec cp {} "${OUTDIR}/" \;
find protocol/generate/patac_hpp/ -name "patac*.pb.h" -exec cp {} "${OUTDIR}/" \;
find protocol/generate/patac_hpp/ -name "patac*.pb.cc" -exec cp {} "${OUTDIR}/" \;
find protocol/generate/patac_hpp/ -name "patac*.py"    -exec cp {} "${OUTDIR}/" \;

echo "Protocol buffers (patac_hpp) generated and saved to ${OUTDIR}"
