# 安装V3.5.0的protobuf

```
cd ~/Downloads
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.5.0/protobuf-all-3.5.0.tar.gz
tar -xzf protobuf-all-3.5.0.tar.gz
cd protobuf-3.5.0
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
```
# 验证安装
```
which protoc
# 输出应为 /usr/local/bin/protoc

protoc --version
# 输出应为 libprotoc 3.5.0
```
___

# 使用方法：20250714更新

```
bash generate_patac.sh
```
会在根目录/output/YYYYMMDDHHMSS/目录下生成*.proto, *.pb.cc, *.pb.h, *.py文件

## （可选）DEMO测试serialize和deserialize

```
sh build.sh
```

会生成可执行文件直接运行