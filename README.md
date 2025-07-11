## 安装V3.5.0的protobuf
cd ~/Downloads

wget https://github.com/protocolbuffers/protobuf/releases/download/v3.5.0/protobuf-all-3.5.0.tar.gz

tar -xzf protobuf-all-3.5.0.tar.gz

cd protobuf-3.5.0

## 放在项目里
./configure --prefix=/YOUR_INSTALL_PATH/

make -j$(nproc)

make install


## 验证安装
/YOUR_INSTALL_PATH/bin/protoc --version

输出示例应为：libprotoc 3.5.0

改build.sh和gereater_protocol.sh内的路径，编译


## 输出物
*.proto: protocol/patac_hpp

*.pb.h: include/protocol


## DEMO测试serialize和deserialize
sh build.sh

会生成可执行文件直接运行