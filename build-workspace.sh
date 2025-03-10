#!/bin/bash

# Copyright (c) 2024  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    exit
}

source ./cabot-common/build-utils.sh

function help {
    echo "Usage: $0 [<option>] [<targets>]"
    echo ""
    echo "-h                    show this help"
    echo "-d                    debug build"

    echo "Available services:"
    show_available_services dcfiles
}

debug_build=0
dcfiles=("docker-compose.yaml")

while getopts "hd" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    d)
        debug_build=1
        ;;
    esac
done
shift $((OPTIND-1))
targets=$@

arch=$(uname -m)
if [ $arch != "x86_64" ] && [ $arch != "aarch64" ]; then
    red "Unknown architecture: $arch"
    exit 1
fi

build_workspace dcfiles targets arch debug_build
