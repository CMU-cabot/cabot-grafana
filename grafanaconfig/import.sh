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
