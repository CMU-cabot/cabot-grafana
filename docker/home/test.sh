#!/bin/bash

host=influxdb
port=8086

TOKEN=a54a87f7-73a0-4534-9741-ad7ff4e7d111
ORGANIZATION_NAME=cabot
BUCKET_NAME=grafana-test


curl -XPOST "http://$host:$port/api/v2/write?org=$ORGANIZATION_NAME&bucket=$BUCKET_NAME&precision=s" \
     -H "Authorization: Token $TOKEN" \
     --data-raw 'weather,location=us-midwest temperature=82'
