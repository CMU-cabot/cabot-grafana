#!/bin/bash

host=localhost:3000

if [[ ! -e api-key.txt ]]; then
    curl -c cookies.txt -X POST -H "Content-Type: application/json" -d '{"user":"admin","password":"admin"}' \
	 http://$host/login

    curl -b cookies.txt -X POST -H "Content-Type: application/json" \
	 -d '{"name":"api-key", "role": "Admin"}' http://$host/api/auth/keys -o api-key.txt

    rm cookies.txt
fi
