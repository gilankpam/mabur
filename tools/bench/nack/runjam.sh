#!/usr/bin/env bash
# jammer arms: fresh maburgs.nack per config, host 8822EU jammer on the op channel for DUR s
cd /home/gilankpam/Projects/drone/mabur
G=root@10.18.0.1; D=root@192.168.10.152
DUR=${DUR:-300}; PPS=${PPS:-60}; CH=${CH:-144}
S=$(dirname "$0")
run_arm() {
  local name=$1
  echo "=== $name start $(date +%T) pps=$PPS"
  ssh -o BatchMode=yes $G 'killall maburgs.nack 2>/dev/null; sleep 1; rm -f /tmp/mabur-session; true'
  ssh -o BatchMode=yes $G "setsid /usr/local/bin/maburgs.nack -c /tmp/cfg/$name.toml --loss-sim 8303 </dev/null >/tmp/maburgs.log 2>&1 & sleep 20; cat /tmp/mabur-session; echo"
  ssh -o BatchMode=yes $D 'grep "maburd nack:" /tmp/mabur.log | tail -1' 2>/dev/null
  nohup bash tools/bench/benchjam.sh --channel $CH --pps $PPS --secs $((DUR+5)) > "$S/jam_$name.log" 2>&1 &
  sleep 5
  "$S/gsdelta.sh" "$DUR"
  sleep 8
  ssh -o BatchMode=yes $G 'SD=$(cat /tmp/mabur-session); cp /tmp/maburgs.log $SD/maburgs.log; echo "session $SD"; grep "maburgs nack:" /tmp/maburgs.log | tail -1; grep "^stats:" /tmp/maburgs.log | tail -1 | grep -o "frames\[clean/trunc/drop\]=[0-9/]*"'
  ssh -o BatchMode=yes $D 'grep "maburd nack:" /tmp/mabur.log | tail -1' 2>/dev/null
  echo "=== $name end $(date +%T)"
}
for a in ${ARMS:-A0_control A1_nack_direct12 A1_nack_direct6}; do run_arm $a; done
echo JAM_ARMS_DONE
