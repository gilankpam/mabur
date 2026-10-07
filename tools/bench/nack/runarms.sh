#!/usr/bin/env bash
# loss-sim arms (secondary rig, docs/fec-nack.md "Bench"): one arm = fresh
# maburgs.nack (a MABUR_LOSS_SIM build) on its config, loss dialed in after
# link-up. The jammer runner (runjam.sh) is the primary rig.
G=root@10.18.0.1
DUR=${DUR:-300}
run_arm() {
  local name=$1
  echo "=== $name start $(date +%T)"
  ssh -o BatchMode=yes $G 'killall maburgs.nack 2>/dev/null; sleep 1; rm -f /tmp/mabur-session; true'
  ssh -o BatchMode=yes $G "setsid /usr/local/bin/maburgs.nack -c /tmp/cfg/$name.toml --loss-sim 8303 </dev/null >/tmp/maburgs.log 2>&1 & sleep 20; python3 /tmp/lossctl.py 's0 eff=1.5 burst=4' 's1 eff=1.5 burst=4'; cat /tmp/mabur-session; echo"
  sleep "$DUR"
  ssh -o BatchMode=yes $G 'python3 /tmp/lossctl.py off; SD=$(cat /tmp/mabur-session); cp /tmp/maburgs.log $SD/maburgs.log; echo "session $SD"; grep "^stats:" /tmp/maburgs.log | tail -1 | grep -o "frames\[clean/trunc/drop\]=[0-9/]*"'
  echo "=== $name end $(date +%T)"
}
for a in ${ARMS:-A0_control A1_nack}; do run_arm $a; done
echo ARMS_DONE
