#!/bin/bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install --executor sequential $@
