#!/usr/bin/env bash
# print s0/s1 abandoned, recovered and frames clean/trunc/drop deltas over $1 seconds from the GS stats lines
D=${1:-60}
snap() { ssh -o BatchMode=yes root@10.18.0.1 'grep "^stats:" /tmp/maburgs.log | tail -1' | python3 -c '
import re,sys
l=sys.stdin.read()
m0=re.search(r"s0\[p=(\d+) abn=(\d+) rec=(\d+)",l); m1=re.search(r"s1\[p=(\d+) abn=(\d+) rec=(\d+)",l)
f=re.search(r"frames\[clean/trunc/drop\]=(\d+)/(\d+)/(\d+)",l); op=re.search(r"op=(\S+)",l)
print(m0.group(1),m0.group(2),m0.group(3),m1.group(1),m1.group(2),m1.group(3),f.group(1),f.group(2),f.group(3),op.group(1))'; }
A=($(snap)); sleep "$D"; B=($(snap))
echo "op=${B[9]} over ${D}s: s0 syms=$((B[0]-A[0])) abn=$((B[1]-A[1])) rec=$((B[2]-A[2])) | s1 syms=$((B[3]-A[3])) abn=$((B[4]-A[4])) rec=$((B[5]-A[5])) | frames clean=$((B[6]-A[6])) trunc=$((B[7]-A[7])) drop=$((B[8]-A[8]))"
