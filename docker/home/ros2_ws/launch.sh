#!/bin/bash

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

pids=()
termpids=()

trap signal INT TERM

function signal() {
    blue "trap signal "

    # ps -Af
    kill -INT -1
    for pid in ${termpids[@]}; do
	kill -TERM $pid
    done
    for pid in ${pids[@]}; do
	count=0
        while kill -0 $pid 2> /dev/null; do
	    if [[ $count -eq 15 ]]; then
		blue "escalate to SIGTERM $pid"
		com="kill -TERM $pid"
		eval $com
	    fi
	    if [[ $count -eq 30 ]]; then
		blue "escalate to SIGKILL $pid"
		com="kill -KILL $pid"
		eval $com
	    fi
            echo "waiting $0 $pid"
	    # ps -Af
            snore 1
	    count=$((count+1))
        done
    done
    
    exit
}

function err {
    >&2 red "[ERROR] "$@
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


source install/setup.bash

ros2 launch cabot_influxdb_client launch.py &
pids+=($!)
termpids+=($!)

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
