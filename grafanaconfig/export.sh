#!/bin/bash

source common.sh

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h        "
    echo "-s        export data source"
    echo "-l        list dashboards"
    echo "-b <uid>  export specified dashboard"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

datasource=0
list_dashboard=0
dashboard=

while getopts "hslb:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	s)
	    datasource=1
	    ;;
	l)
	    list_dashboard=1
	    ;;
	b)
	    dashboard=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

api_key=$(jq -r .key api-key.txt)


if [[ $datasource -eq 1 ]]; then
    curl -H "Authorization: Bearer $api_key" http://$host/api/datasources
    echo ""
    echo -en "\033[31m" >&2 # red
    echo "You need to add your token by adding 'secureJsonData.json'" >&2
    echo -en "\033[0m" >&2  ## reset color
fi


if [[ $list_dashboard -eq 1 ]]; then
    curl -H "Authorization: Bearer $api_key" "http://$host/api/search?query=&type=dash-db" 2> /dev/null | jq -r '.[] | "\(.title) - \(.uid)"'
fi

if [[ $dashboard != "" ]]; then
    curl -H "Authorization: Bearer $api_key" http://$host/api/dashboards/uid/$dashboard | jq 'del(.meta) | .dashboard.id = null'
fi

