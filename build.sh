#!/bin/bash

set -eux

DIR=$(pwd)
rm -rf build lib
mkdir -p build lib/lib_arm lib/lib_x86 include/protocol src/protocol
rm -rf include/protocol/*.pb.h src/protocol/*.pb.cc

cd protocol/generate
if [ -d type ]; then
  rm -rf type
fi
mkdir type

# Proto generate protocol api
# protoc=$DIR/../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/bin/protoc
export PATH=/home/hyz/05_LYK/thirdparty/protobuf-3.5.0/bin:$PATH
export LD_LIBRARY_PATH=/home/hyz/05_LYK/thirdparty/protobuf-3.5.0/lib:${LD_LIBRARY_PATH:-}
protoc=protoc


cd ../type
$protoc -I=./ --cpp_out=../generate/type/ *.proto

cd $DIR
cp protocol/generate/type/*.pb.h include/protocol/
cp protocol/generate/type/*.pb.cc src/protocol/

cd protocol/generate
if [ -d model ]; then
  rm -rf model
fi
mkdir model
cd ../model
$protoc -I=./ -I=../type/ --cpp_out=../generate/model/ *.proto
cd $DIR
cp protocol/generate/model/*.pb.h include/protocol/
cp protocol/generate/model/*.pb.cc src/protocol/

cd protocol/generate
if [ -d interface ]; then
  rm -rf interface
fi
mkdir interface
cd ../interface
$protoc -I=./ -I=../type/ --cpp_out=../generate/interface/ *.proto
cd $DIR
cp protocol/generate/interface/*.pb.h include/protocol/
cp protocol/generate/interface/*.pb.cc src/protocol/



cd protocol/generate
if [ -d patac_hpp ]; then
  rm -rf patac_hpp
fi
mkdir patac_hpp
cd ../patac_hpp
$protoc -I=./ -I=../type/ --cpp_out=../generate/patac_hpp/ *.proto
cd $DIR
cp protocol/generate/patac_hpp/*.pb.h include/protocol/
cp protocol/generate/patac_hpp/*.pb.cc src/protocol/

IS_ARM6=false
IS_ARM9=false
IS_ARM10=false
IS_INSTALL=false

for i in "$@"; do
  if [ "$i" = "-platform=arm6" -o "$i" = "-a6" ]; then
    IS_ARM6=true
  elif [ "$i" = "-platform=arm9" -o "$i" = "-a9" ]; then
    IS_ARM9=true
  elif [ "$i" = "-platform=arm10" -o "$i" = "-a10" ]; then
    IS_ARM10=true
  elif [ "$i" = "-install=true" -o "$i" = "-i" ]; then
    IS_INSTALL=true
  fi
done

# Support the arm compile, if the arm plaform need to be supported, please
# enable the CMAKE_COMMAND
CMAKE_COMMAND=""
if [ $IS_ARM6 = true ]; then
  CMAKE_COMMAND=-DTEST_TOOLCHAIN_ROOT=$DIR/../autodrive-arm
elif [ $IS_ARM9 = true ]; then
  CMAKE_COMMAND=-DTEST_TOOLCHAIN_ROOT_9_3_0=$DIR/../autodrive-arm
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../autodrive-arm/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/lib/x86_64-linux-gnu
elif [ $IS_ARM10 = true ]; then
  CMAKE_COMMAND=-DTEST_TOOLCHAIN_ROOT_10_3_0=$DIR/../autodrive-arm
fi

# Check command exit if success
CheckCommand() {
  if [ $? -ne 0 ]; then
    exit 1;
  fi
}

cd build
cmake .. $CMAKE_COMMAND
make -j4
if [ "$IS_ARM10" = "true" -o "$IS_ARM6" = "true" -o "$IS_ARM9" = "true" ]; then
  cp -d lib/*.so* ../lib/lib_arm
else
  
  cp -d lib/*.so* ../lib/lib_x86
fi
cd $DIR

# Strip
if [ $IS_ARM10 ]; then
  STRIP_COMMAND=$DIR/../autodrive-arm/gcc-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-strip
else
  STRIP_COMMAND=strip
fi
#$STRIP_COMMAND lib/*.so*


cd build/bin
./demo