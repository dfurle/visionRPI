
SUBNET=${1:-192.168.1}

isAlive()
{
  ping -c 1 -t 1 ${1} > /dev/null 2>&1
  if [[ $? == 0 ]]; then
    if [[ "`host ${1} | grep '^Host'`" ]]; then
      echo "${1} is up"
    else
      VAR=`host ${1} | awk '{print $5}' | cut -d. -f1`
      echo "${1} is up: ${VAR}"
    fi
  fi
}

for IP in $SUBNET.{0..255}
do
  isAlive ${IP} &
done
wait
echo "Done"