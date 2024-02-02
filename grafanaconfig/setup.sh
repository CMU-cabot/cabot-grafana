#!/bin/bash

source common.sh

curl -c cookies.txt -X POST -H "Content-Type: application/json" -d '{"user":"admin","password":"admin"}' \
     http://$host/login


curl -b cookies.txt -X POST -H "Content-Type: application/json" -H "Cookie: grafana_session=<session_id>" \
     -d '{"name":"api-key", "role": "Admin"}' http://$host/api/auth/keys -o api-key.txt
