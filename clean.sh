#!/usr/bin/bash

project=$(basename $(pwd))

docker ps -f "name=$project*"
if [[ ! -z $(docker ps -f "name=$project*" -q -a) ]]; then
   docker ps -f "name=$project*" -q -a | xargs docker stop
   docker ps -f "name=$project*" -q -a | xargs docker rm
fi

rm -rf influxdb/*
rm -rf grafanadb/*
rm -rf grafana/data/*
rm -f grafanaconfig/api-key.txt
