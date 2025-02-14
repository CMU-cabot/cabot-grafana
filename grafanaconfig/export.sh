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

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h        "
    echo "-s        export data source"
    echo "-l        list dashboards"
    echo "-b <uid>  export specified dashboard"
    echo "-D        export all dashboards"
}

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

if [[ -e $scriptdir/../.env ]]; then
    source $scriptdir/../.env
fi

: ${GRAFANA_HOST:=http://localhost:3000}
datasource=0
list_dashboard=0
dashboard=
all_dashboards=0

while getopts "hslb:D" arg; do
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
    D)
        all_dashboards=1
        ;;
    esac
done
shift $((OPTIND-1))

if [[ $datasource -eq 1 ]]; then
    curl -H "Authorization: Bearer $GRAFANA_API_KEY" $GRAFANA_HOST/api/datasources
    echo ""
    echo -en "\033[31m" >&2 # red
    echo "You need to add your token by adding 'secureJsonData.json'" >&2
    echo -en "\033[0m" >&2  ## reset color
fi


if [[ $list_dashboard -eq 1 ]]; then
    curl -H "Authorization: Bearer $GRAFANA_API_KEY" "$GRAFANA_HOST/api/search?query=&type=dash-db" 2> /dev/null | jq -r '.[] | "\(.title) - \(.uid)"'
fi

if [[ $dashboard != "" ]]; then
    curl -H "Authorization: Bearer $GRAFANA_API_KEY" $GRAFANA_HOST/api/dashboards/uid/$dashboard | jq 'del(.meta) | .dashboard.id = null' > $dashboard.json
fi

if [[ $all_dashboards -eq 1 ]]; then
    mkdir -p dashboards
    cd dashboards
    curl -H "Authorization: Bearer $GRAFANA_API_KEY" "$GRAFANA_HOST/api/search?query=&type=dash-db" 2> /dev/null | jq -r '.[] | "\(.uid)"' \
    | while read uid; do
        curl -H "Authorization: Bearer $GRAFANA_API_KEY" $GRAFANA_HOST/api/dashboards/uid/$uid | jq 'del(.meta) | .dashboard.id = null' > $uid.json
      done
fi
