
SUBNET={$1-192.168.1}

isAlive()
{
  ping -c 1 -t 1 ${1} > /dev/null 2>&1
  [[ $? == 0 ]] && echo "${1} is up"
}


for IP in $SUBNET.{0..255}
do
  isAlive ${IP} &
done
wait
echo "Done"