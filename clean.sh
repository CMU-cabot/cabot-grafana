#!/usr/bin/bash

project=$(basename $(pwd))

docker ps -f "name=$project*"
if [[ ! -z $(docker ps -f "name=$project*" -q -a) ]]; then
   docker ps -f "name=$project*" -q -a | xargs docker stop
   docker ps -f "name=$project*" -q -a | xargs docker rm
fi

rm -rf influxdb/*
docker compose down grafana-server -v
