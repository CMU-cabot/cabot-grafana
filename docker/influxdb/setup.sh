#!/bin/bash

USERNAME=cabot
PASSWORD=cabot-influxdb
TOKEN=a54a87f7-73a0-4534-9741-ad7ff4e7d111
ORGANIZATION_NAME=cabot
BUCKET_NAME=grafana-test

influx setup \
  --username $USERNAME \
  --password $PASSWORD \
  --token $TOKEN \
  --org $ORGANIZATION_NAME \
  --bucket $BUCKET_NAME \
  --force
