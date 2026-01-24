#!/bin/bash

day=`date '+%Y%m%d_%H%M%S'`
log="/apollo_workspace/data/log/nperf.log.${day}"
touch ${log}
ln -snf nperf.log.${day} /apollo_workspace/data/log/nperf.INFO

while true
do
    # total
    echo >>${log}
    date '+%m-%d %H:%M:%S' >>${log}
    top -p 99999 -bc -w 512 -n 1 | sed '$d' >>${log}
    # processes
    ps aux --sort=-%cpu | head -50 | awk '{if($3!~/^0.*/) {print $0}}' >> ${log}

    echo >>${log}
    if [[ -e /usr/bin/tegrastats ]];then
        timeout 2 /usr/bin/tegrastats --readall >> ${log}
    fi
    if [[ -e /usr/bin/nvidia-smi ]];then
        /usr/bin/nvidia-smi >> ${log}
    fi
    sleep 1
done
