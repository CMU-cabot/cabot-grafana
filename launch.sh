#!/bin/bash

trap ctrl_c INT QUIT TERM

source util.sh

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
    echo "-s                    launch local server"
    echo "-i                    initial setup the servers"
    echo "-d                    launch server in development mode"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

initial_setup=0
dcfile="docker-compose.yaml"
development=0

while getopts "hisd" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	i)
	    initial_setup=1
	    ;;
	s)
	    dcfile="docker-compose-local.yaml"
	    ;;
	d)
	    development=1
	    ;;
    esac
done
shift $((OPTIND-1))

if [[ $development -eq 1 ]]; then
    dccom="docker compose -f docker-compose-dev.yaml"
    eval "$dccom up -d"    
    curl http://localhost:3000/ --fail > /dev/null 2>&1
    test=$?
    while [[ $test -ne 0 ]]; do
	snore 5
	blue "waiting the grafana server is ready..."
	curl http://localhost:3000/ --fail > /dev/null 2>&1
	test=$?
    done
else
    dccom="docker compose -f $dcfile"
    eval "$dccom up -d"
fi

if [[ $initial_setup -eq 1 ]]; then
    snore 5
    script -q -c "docker compose exec influxdb ./setup.sh" /dev/null &
    snore 5
    cd ./grafanaconfig
    ./setup.sh
    ./import.sh -s datasources.json
    ./import.sh -b dashboard.json
    cd ..
fi

while [ 1 -eq 1 ];
do
    snore 1
done
