#!/bin/bash

# 基于orin的开启自启动脚本
# 必须用root账户执行
# 使用方法:
#   1. 测试：先sudo su切到root账户，然后 bash rc.local.sh 验证
#   2. 部署：先sudo su切到root账户，cp rc.local.sh /etc/rc.local  && chmod +x /etc/rc.local

# 日志查看: /var/log/apollo_start/rc.local.log.{以时间命名的后缀}

# 基础配置
user="nvidia"
# 环卫写：sweepper，通用能力写：park
env_prefix="sweeper"

# log init
logdir="/var/log/apollo_start"
[ -d ${logdir} ] || mkdir -p ${logdir}
chown 1000:1000 -R ${logdir}
exec >${logdir}/rc.local.log.$(date +%Y%m%d-%H%M%S).$$ 2>&1
#set -x

# 显示系统时间
date

# set up can
echo "begin setup can..."
ip link set can0 type can bitrate 500000
ip link set can1 type can bitrate 500000
ip link set up can0
ip link set up can1


# 挂载盘，根据需要挂载
#sudo mount /dev/nvme0n1p1 /media/nvidia/ssd

# 重启docker，如果您系统docker没有自动启动执行
echo "begin restart docker ..."
systemctl restart docker
chmod 777 /var/run/docker.sock

# 重启授时服务
echo "begin restart chrony service..."
sudo service chrony restart
sleep 5

# 授时触发（以天准为例）：注意这个波特率，根据实际情况修改，比如115200
trigger_pps_cmd="tztek-jetson-tool-cpld-test -d /dev/ttyTHS1 -t 3 -l 232 -b 9600"
$trigger_pps_cmd
sleep 1

# 判断授时生效: 2min
for i in {1..60}; do
    chronyc sources -v | grep "\* GPS"
    if [[ $? -eq 0 ]]; then
        break
    fi
    if [[ $i -eq 60 ]]; then
        # 授时失败
        echo "gpsd not ready"
        exit 1
    fi
    echo "waiting for gpsd..."
    $trigger_pps_cmd
    sleep 2
done
echo "gpsd ready."

# 相机触发（以天准为例）
echo "begin trigger camera..."
#tztek-jetson-tool-cpld-test -d /dev/ttyTHS1 -t 4 -c 8 -f 10 -w 1000 -o 0 
tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 10 1000 >/dev/null 2>&1 &

# orin性能优化（以天准为例）
jetson_clocks

# 定位设备接口，以使用usb ttyACM0为例
chmod 777 /dev/ttyACM0

# 支持core_dump
echo "add core pattern."
echo "1" | tee /proc/sys/kernel/core_uses_pid
echo "/apollo/data/core/%e.core.%p" | tee /proc/sys/kernel/core_pattern

# 重启鉴权服务
#echo "begin stop auth server..."
#/etc/init.d/apollo-auth-server.sh stop
#sleep 5
#echo "begin start auth server..."
#/etc/init.d/apollo-auth-server.sh start
#sleep 20
# 检查鉴权服务是否启动成功，如果成功则继续执行，失败则退出启动程序
function auth_check(){
	result=`bash /etc/init.d/apollo-auth-server.sh check-interface`
	return $?
}
auth_check
if [[ $? -ne 0 ]]; then
    echo "Auth check failed, please check FAQ doc"
    exit 1
fi

# 显示系统时间，如果时间不对，务必修改时间后重新执行这个脚本
date

# 启动docker和launch all
echo "begin start docker container and launch all process ..."
container=`docker ps -a | grep apollo_neo_dev_${env_prefix} | head -1 | awk '{print $1}'`
if [[ -z ${container} ]]; then
    echo "docker container not found"
    exit 1
fi
/bin/bash -c "su ${user} -c \" \
    docker start ${container} && \
    sleep 2 && \
    docker exec -itd -u${user} ${container} /bin/bash -c 'source /opt/apollo/neo/setup.sh && cyber_launch start /apollo_workspace/launch_all.launch' \
\"" || true

echo "All finished."