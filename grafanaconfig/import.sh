#!/bin/bash

source common.sh

api_key=$(jq -r .key api-key.txt)

echo $api_key

jq .[0] datasource.json > temp.json

curl -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $api_key" \
     -d @temp.json http://$host/api/datasources


rm temp.json
