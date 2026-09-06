#!/usr/bin/env bash
# host-side orchestrator: iqpab.sh <cycles-per-arm> <pulse_s> <eff> <climb_wait_s> [armA armB]
# one cycle = set min_iqp on the drone, s0 loss pulse on the GS (cascade down), off, wait for the climb back
N=${1:-4}; D=${2:-3}; EFF=${3:-30}; W=${4:-40}; A=${5:-44}; B=${6:-12}
DR="root@192.168.10.152"; GS="root@10.18.0.1"
sshq(){ ssh -o ConnectTimeout=5 -o BatchMode=yes "$@" 2>&1 | grep -v "^\*\*"; }
OUT=${OUT:-cycles.txt}; : > "$OUT"
i=0
while [ $i -lt $N ]; do
  for arm in $A $B; do
    r=$(sshq $DR "curl -s -m 2 -X POST 'localhost:8301/venc/set?min_iqp=$arm'")
    sleep 1
    line=$(sshq $GS "python3 - <<'PY'
import socket,time,sys
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); a=('127.0.0.1',8390); s.settimeout(1.0)
def cmd(c):
    s.sendto(c.encode(),a)
    try: r=s.recv(1024).decode().strip()
    except Exception as e: r=f'<{e}>'
    return int(time.monotonic()*1000), r
t_on,r_on=cmd('${STREAM:-s0} eff=$EFF burst=3'); time.sleep($D); t_off,r_off=cmd('${STREAM:-s0} off')
print(f'{t_on} {t_off} on={r_on[:30]} off={r_off[:10]}')
PY")
    echo "cycle $i arm=$arm drone=$r $line" | tee -a "$OUT"
    sleep $W
  done
  i=$((i+1))
done
sshq $DR "curl -s -m 2 -X POST 'localhost:8301/venc/set?min_iqp=$A'" >/dev/null
echo DONE
