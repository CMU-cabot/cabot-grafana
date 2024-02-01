#!/bin/bash

# Copyright (c) 2020  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

ulimit -S -c 0

while getopts "c" arg; do
    case $arg in
        c)
	    ulimit -c unlimited
	    echo 1 | sudo tee /proc/sys/kernel/core_uses_pid
	    echo "/home/developer/core" | sudo tee /proc/sys/kernel/core_pattern
	    ulimit -s 65536
	    ;;
    esac
done
shift $((OPTIND-1))

if [[ $1 == "build" ]]; then
    shift 1
    exec ./build.sh $@
elif [[ $1 == "launch" ]]; then
    shift 1
    exec ./launch.sh $@
fi

exec bash
