#!/bin/bash

if [ $(id -u) -eq 0 ]; then
   echo "please do not run as root: $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

## install cabot-grafana
sudo ln -sf $scriptdir /opt/cabot-grafana
docker compose -f "$scriptdir/docker-compose.yaml" up -d

## install cabot-grafana.service
INSTALL_DIR=$HOME/.config/systemd/user
mkdir -p $INSTALL_DIR
cp $scriptdir/cabot-grafana.service $INSTALL_DIR
systemctl --user daemon-reload
systemctl --user enable cabot-grafana

