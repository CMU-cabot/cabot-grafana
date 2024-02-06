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

## TODO: not sure this is the best way
export GOROOT=$HOME/go/go
export GOPATH=$HOME/go_projects
export PATH=$GOROOT/bin:$GOPATH/bin:$PATH

if [[ $1 == "build" ]]; then
    shift 1
    ## TODO: This is fragile
    if [[ ! -e $HOME/go/go ]]; then
	wget https://go.dev/dl/go1.21.0.linux-amd64.tar.gz && \
	    mkdir -p $HOME/go && \
	    tar -C $HOME/go -xzf go1.21.0.linux-amd64.tar.gz
	go install github.com/evilmartians/lefthook@latest
	wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | sh
	source $HOME/.nvm/nvm.sh
	nvm install 20
    fi
    source $HOME/.nvm/nvm.sh
    npm install --global yarn && \
	yarn install --immutable
    cd /home/developer/src/grafana
    make build
    ./bin/linux-amd64/grafana cli plugins install volkovlabs-image-panel
elif [[ $1 == "launch" ]]; then
    shift 1
    cd /home/developer/src/grafana
    source $HOME/.nvm/nvm.sh
    yarn start &
    make run
fi
