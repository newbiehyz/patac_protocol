#!/bin/bash

protoc=protoc

rm -rf ./generate
mkdir -p ./generate

$protoc -I=./protocol/patac_hpp/ \
       --python_out=./generate/ \
       ./protocol/patac_hpp/*.proto
