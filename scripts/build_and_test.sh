#!/bin/bash
set -e
cd ../
cd build
# rebuild
rm CMakeCache.txt
cmake ../
make -j8
# gen stub
stubgen --module falcon_sdk
# generate new stub
mv out/falcon_sdk.pyi ./
# do the test.
pytest --json=./report.json -s falcon_test.py 
