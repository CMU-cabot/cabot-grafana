#!/bin/bash

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    red "catch the signal"
    if [[ ! -z $dccom ]]; then
	red "$dccom down"
	$dccom down
    fi
    exit
}

function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}


dccom="docker compose"
eval "$dccom up -d"

snore 5
script -q -c "docker compose exec influxdb ./setup.sh" /dev/null &

while [ 1 -eq 1 ];
do
    snore 1
done
