#!/bin/bash

source common.sh

api_key=$(jq -r .key api-key.txt)

echo $api_key

jq .[0] datasources.json > temp.json

curl -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $api_key" \
     -d @temp.json http://$host/api/datasources > /dev/null


curl -v -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $api_key" \
     -d @dashboard.json http://$host/api/dashboards/import > /dev/null
