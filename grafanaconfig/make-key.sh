#!/bin/bash

source .env

function help() {
    echo "Usage: $0 [-h] [-p password]"
    echo "Options:"
    echo "  -h  Show this help message and exit"
    echo "  -p  Password for the admin user"
}

pass=admin
name=api-key-$(date +%s)
while getopts "hp:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        p)
            pass=$OPTARG
            ;;
    esac
done

curl -s -c cookies.txt -X POST -H "Content-Type: application/json" -d "{\"user\":\"admin\",\"password\":\"$pass\"}" \
     $GRAFANA_HOST/login

echo ""

curl -s -b cookies.txt -X POST -H "Content-Type: application/json" \
     -d "{\"name\":\"$name\", \"role\": \"Admin\"}" $GRAFANA_HOST/api/auth/keys -o api-key.txt

rm cookies.txt
api_key=$(jq -r .key api-key.txt)
echo "API_KEY=$api_key"
echo ""
echo "copy this to your .env"
rm api-key.txt
