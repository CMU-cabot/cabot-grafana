#!/bin/bash

source .env

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h        "
    echo "-s <file> import data source"
    echo "-b <file> import daashboard"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

datasource=
dashboard=

while getopts "hs:b:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	s)
	    datasource=$OPTARG
	    ;;
	b)
	    dashboard=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

if [[ $datasource != "" ]]; then
    jq .[0] $datasource > temp.json
    
    curl -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $API_KEY" \
	 -d @temp.json $GRAFANA_HOST/api/datasources > /dev/null
fi


if [[ $dashboard != "" ]]; then
    curl -v -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $API_KEY
" \
	 -d @$dashboard $GRAFANA_HOST/api/dashboards/import > /dev/null
fi
