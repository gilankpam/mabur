#!/usr/bin/env bash
# THROWAWAY SPIKE: self-signed cert for serve.py --tls, valid for localhost
# and every IPv4 address this host has (so a phone on the LAN can use it).
# usage: ./mkcert.sh [extra-ip-or-name ...]   (NixOS: nix-shell -p openssl)
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p tls
san="DNS:localhost,IP:127.0.0.1"
for ip in $(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1); do san="$san,IP:$ip"; done
for x in "$@"; do
  if [[ $x =~ ^[0-9.]+$ ]]; then san="$san,IP:$x"; else san="$san,DNS:$x"; fi
done
openssl req -x509 -newkey rsa:2048 -nodes -days 365 -subj "/CN=mabur-spike" \
  -addext "subjectAltName=$san" -addext "basicConstraints=critical,CA:FALSE" \
  -keyout tls/key.pem -out tls/cert.pem 2>/dev/null
echo "tls/cert.pem SAN: $san"
