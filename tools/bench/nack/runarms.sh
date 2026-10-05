#!/usr/bin/env bash
# bench runner: one arm = fresh maburgs.nack on its config, loss dialed in after link-up
G=root@10.18.0.1; D=root@192.168.10.152
DUR=${DUR:-300}
run_arm() {
  local name=$1
  echo "=== $name start $(date +%T)"
  ssh -o BatchMode=yes $G 'killall maburgs.nack 2>/dev/null; sleep 1; rm -f /tmp/mabur-session; true'
  ssh -o BatchMode=yes $G "setsid /usr/local/bin/maburgs.nack -c /tmp/cfg/$name.toml --loss-sim 8303 </dev/null >/tmp/maburgs.log 2>&1 & sleep 20; python3 /tmp/lossctl.py 's0 eff=1.5 burst=4' 's1 eff=1.5 burst=4'; cat /tmp/mabur-session; echo"
  ssh -o BatchMode=yes $D 'grep "maburd nack:" /tmp/mabur.log | tail -1' 2>/dev/null
  sleep "$DUR"
  ssh -o BatchMode=yes $G 'python3 /tmp/lossctl.py off; SD=$(cat /tmp/mabur-session); cp /tmp/maburgs.log $SD/maburgs.log; echo "session $SD"; grep "maburgs nack:" /tmp/maburgs.log | tail -1; grep "^stats:" /tmp/maburgs.log | tail -1 | grep -o "frames\[clean/trunc/drop\]=[0-9/]*"'
  ssh -o BatchMode=yes $D 'grep "maburd nack:" /tmp/mabur.log | tail -1' 2>/dev/null
  echo "=== $name end $(date +%T)"
}
for a in A0_control A1_nack_direct12 A2_nack_slotted12 A1_nack_direct6 A3_enh_ov50; do run_arm $a; done
echo ARMS_DONE
