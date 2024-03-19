#! /bin/bash

apt-get update

pip install cython
# downloading a branch of a fork because they fixed python3 stuff in there
git clone --branch python3 https://github.com/pmusau17/range_libc.git
cd ./range_libc/pywrapper

python3 setup.py install
