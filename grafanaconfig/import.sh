#!/bin/bash

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

if [[ -e $scriptdir/../.env ]]; then
    source $scriptdir/../.env
fi

: ${GRAFANA_HOST:=http://localhost:3000}
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
    
    curl -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $GRAFANA_API_KEY" \
	 -d @temp.json $GRAFANA_HOST/api/datasources
	echo ""
fi


if [[ $dashboard != "" ]]; then
    curl -v -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $GRAFANA_API_KEY" \
	 -d @$dashboard $GRAFANA_HOST/api/dashboards/import
	echo ""
fi
