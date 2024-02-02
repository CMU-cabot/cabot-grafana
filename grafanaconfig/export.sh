#!/bin/bash

source common.sh

api_key=$(jq -r .key api-key.txt)

echo $api_key

curl -H "Authorization: Bearer $api_key" http://$host/api/datasources -o datasources.json

curl -H "Authorization: Bearer $api_key" "http://$host/api/search?query=&type=dash-db" -o dash-db.json

jq -r ".[].uid" dash-db.json | while read -r line; do
    curl -H "Authorization: Bearer $api_key" http://$host/api/dashboards/uid/$line > dashboard-$line.json
done
