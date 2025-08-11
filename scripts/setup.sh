#!/bin/bash
set -e
# install tool-chain and python
apt-get install python3 python3-pip cmake gcc g++ make git -y

set +e
ln -s /usr/bin/python /usr/bin/python3
ln -s /usr/bin/pip /usr/bin/pip3
set -e

# install pybind
pwd=`pwd`
mkdir tmp && cd tmp
git clone https://github.com/pybind/pybind11
mkdir -p pybind11/build && cd pybind11/build
rm -rf ./*
cmake ../
make check -j8
make install

cd $pwd
rm -rf tmp
pip install -r requirements.txt
