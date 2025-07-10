#!/bin/bash

mkdir -p ./generate
mkdir -p ./hobot_generate

protoc=/home/hyz/05_LYK/thirdparty/protobuf-3.5.0/bin/protoc

$protoc -I=./protocol/model/ \
       -I=./protocol/type/ \
       -I=./protocol/interface/ \
       --python_out=./generate/ \
       ./protocol/model/*.proto \
       ./protocol/interface/*.proto \
       ./protocol/type/*.proto

$protoc  -I=./hobot_auto_protocol/ad/ \
        -I=./hobot_auto_protocol/adas_function/ \
        -I=./hobot_auto_protocol/apa/ \
        -I=./hobot_auto_protocol/bole/ \
        -I=./hobot_auto_protocol/calib/ \
        -I=./hobot_auto_protocol/common/ \
        -I=./hobot_auto_protocol/deprecated/ \
        -I=./hobot_auto_protocol/ego_motion/ \
        -I=./hobot_auto_protocol/mono/ \
        -I=./hobot_auto_protocol/ndm/ \
        -I=./hobot_auto_protocol/parking/ \
        -I=./hobot_auto_protocol/perception/ \
        -I=./hobot_auto_protocol/pp_manage/ \
        -I=./hobot_auto_protocol/sensors/ \
        -I=./hobot_auto_protocol/software/ \
        --python_out=./hobot_generate/ \
       ./hobot_auto_protocol/ad/*.proto \
       ./hobot_auto_protocol/adas_function/*.proto \
       ./hobot_auto_protocol/apa/*.proto \
       ./hobot_auto_protocol/bole/*.proto \
       ./hobot_auto_protocol/calib/*.proto \
       ./hobot_auto_protocol/common/*.proto \
       ./hobot_auto_protocol/deprecated/*.proto \
       ./hobot_auto_protocol/ego_motion/*.proto \
       ./hobot_auto_protocol/mono/*.proto \
       ./hobot_auto_protocol/ndm/*.proto \
       ./hobot_auto_protocol/parking/*.proto \
       ./hobot_auto_protocol/perception/*.proto \
       ./hobot_auto_protocol/pp_manage/*.proto \
       ./hobot_auto_protocol/sensors/*.proto \
       ./hobot_auto_protocol/software/*.proto
