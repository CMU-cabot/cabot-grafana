#!/bin/bash

source common.sh

api_key=$(jq -r .key api-key.txt)

echo $api_key

curl -H "Authorization: Bearer $api_key" http://$host/api/datasources -o datasource.json
