#!/bin/bash

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

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    "
    echo "-g                    init grafana"
    echo "-i                    init influxdb"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

init_grafana=0
init_influxdb=0

while getopts "hgi" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	g)
        init_grafana=1
	    ;;
    i)
	    init_influxdb=1
	    ;;
    esac
done
shift $((OPTIND-1))

if [[ -e $scriptdir/.env ]]; then
    source $scriptdir/.env
fi

if [[ $init_influxdb -eq 1 ]]; then
    blue "initialize influxdb server"
    script -q -c "docker compose exec influxdb \
        influx setup \
        --username ${INFLUXDB_USERNAME:-cabot} \
        --password ${INFLUXDB_PASSWORD:-cabot-influxdb} \
        --token ${INFLUXDB_TOKEN:-a54a87f7-73a0-4534-9741-ad7ff4e7d111}\
        --org ${INFLUXDB_ORGANIZATION_NAME:-cabot} \
        --bucket ${INFLUXDB_BUCKET_NAME:-cabot } \
        --force" /dev/null
fi

if [[ $init_grafana -eq 1 ]]; then
    blue "initialize grafana server"
    cd ./grafanaconfig
    ./make-key.sh  >> ../.env
    env | grep API_KEY
    ./import.sh -s datasources.json
    ./import.sh -b dashboard.json
    cd ..
fi
