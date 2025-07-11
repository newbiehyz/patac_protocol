cd ~/Downloads
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.5.0/protobuf-all-3.5.0.tar.gz
tar -xzf protobuf-all-3.5.0.tar.gz
cd protobuf-3.5.0

# 放在项目里
./configure --prefix=/home/hyz/05_LYK/thirdparty/protoc-3.5.0

make -j$(nproc)
make install

# 验证安装
/home/hyz/05_LYK/thirdparty/protoc-3.5.0/bin/protoc --version

# 改build.sh和gereater_protocol.sh内的路径，编译
