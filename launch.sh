#!/bin/bash

trap ctrl_c INT QUIT TERM

source util.sh

function ctrl_c() {
    red "catch the signal"
    if [[ ! -z $dccom ]]; then
	red "$dccom down"
	$dccom down
    fi
    exit
}

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    "
    echo "-i                    initial setup"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

initial_setup=0

while getopts "hi" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	i)
	    initial_setup=1
	    ;;
    esac
done
shift $((OPTIND-1))


dccom="docker compose"
eval "$dccom up -d"

if [[ $initial_setup -eq 1 ]]; then
    snore 5
    script -q -c "docker compose exec influxdb ./setup.sh" /dev/null &
    snore 5
    cd ./grafanaconfig
    ./setup.sh
    ./import.sh
fi

while [ 1 -eq 1 ];
do
    snore 1
done
