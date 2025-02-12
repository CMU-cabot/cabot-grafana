#!/bin/bash

trap ctrl_c INT QUIT TERM

function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function ctrl_c() {
    red "catch the signal"
    if [[ ! -z $dccom ]]; then
	red "$dccom down"
	eval "$dccom down"
    fi
    exit
}

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    "
    echo "-s                    launch server in development mode"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

dcfile="docker-compose.yaml"
profile=server

dccom="docker compose --profile $profile"
eval "$dccom up"
