#!/bin/bash

if [ $(id -u) -eq 0 ]; then
   echo "please do not run as root: $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

## uninstall cabot-grafana.service
systemctl --user disable --now cabot-grafana
INSTALL_DIR=$HOME/.config/systemd/user
rm $INSTALL_DIR/cabot-grafana.service

## uninstall cabot-grafana
sudo rm /opt/cabot-grafana

