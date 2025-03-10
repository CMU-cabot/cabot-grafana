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

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

if [[ -e $scriptdir/../.env ]]; then
    source $scriptdir/../.env
fi

function help() {
    echo "Usage: $0 [-h] [-p password]"
    echo "Options:"
    echo "  -h  Show this help message and exit"
    echo "  -p  Password for the admin user"
}

: ${GRAFANA_HOST:=http://localhost:3000}
pass=admin
name=api-key-$(date +%s)
while getopts "hp:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        p)
            pass=$OPTARG
            ;;
    esac
done

curl -s -c cookies.txt -X POST -H "Content-Type: application/json" -d "{\"user\":\"admin\",\"password\":\"$pass\"}" \
     $GRAFANA_HOST/login > /dev/null

curl -s -b cookies.txt -X POST -H "Content-Type: application/json" \
     -d "{\"name\":\"$name\", \"role\": \"Admin\"}" $GRAFANA_HOST/api/auth/keys -o api-key.txt

rm cookies.txt
api_key=$(jq -r .key api-key.txt)
echo "# copy this to your .env"
echo "GRAFANA_API_KEY=$api_key"
rm api-key.txt
