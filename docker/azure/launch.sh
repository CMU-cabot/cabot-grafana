#!/bin/bash

# Copyright (c) 2024  Carnegie Mellon University
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

#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Set up environment variables
export GOROOT=$HOME/go/go
export GOPATH=$HOME/go_projects
export PATH=$GOROOT/bin:$GOPATH/bin:$PATH

# Function to install Go if not already installed
install_go() {
    if [[ ! -e $GOROOT ]]; then
        echo "Installing Go..."
        wget https://go.dev/dl/go1.21.0.linux-amd64.tar.gz
        mkdir -p $HOME/go
        tar -C $HOME/go -xzf go1.21.0.linux-amd64.tar.gz
        rm go1.21.0.linux-amd64.tar.gz
        go install github.com/evilmartians/lefthook@latest
    fi
}

# Function to install Node Version Manager (nvm) and Node.js
install_node() {
    if ! command -v nvm &> /dev/null; then
        echo "Installing nvm and Node.js..."
        wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
        source $HOME/.nvm/nvm.sh
        nvm install 20
    fi
}

# Function to build the project
build_project() {
    echo "Building the project..."
    npm install --global yarn
    yarn install --immutable
    cd /home/developer/src/grafana
    make build
    ./bin/linux-amd64/grafana cli plugins install volkovlabs-image-panel
}

# Main script logic
case "$1" in
    build)
        install_go
        install_node
        build_project
        ;;
    launch)
        cd /home/developer/src/grafana
        bin/grafana server
        ;;
    *)
        echo "Usage: $0 {build|launch}"
        exit 1
        ;;
esac
