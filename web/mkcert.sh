#!/usr/bin/env bash
# TLS for serve.py --tls. A click-through on a bare self-signed cert is NOT
# enough: Chrome's bypass covers the tab, but the Emscripten pthread pool is
# workers spawned from a worker, and those script fetches still fail the
# cert check ("worker sent an error! undefined" -- app.js maps that to a
# banner, see docs/web-gs.md).
# So: a local CA (made once, reused), trusted on each viewing device, signing
# a server cert for localhost + this host's IPv4s. The CA is name-constrained
# to localhost and private IPv4 ranges, so trusting it can't vouch for any
# public site.
# usage: ./mkcert.sh [extra-ip-or-name ...]   (NixOS: nix-shell -p openssl)
# Output: web/tls/ (this script's own directory + tls/).
# Trust tls/ca.crt: Android Settings > Security > Encryption & credentials >
# Install a certificate > CA certificate; Linux Chrome:
#   certutil -d sql:$HOME/.pki/nssdb -A -t "C,," -n mabur-webgs-ca -i tls/ca.crt
# (nix-shell -p nssTools; remove with certutil ... -D -n mabur-webgs-ca).
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p tls
if [[ ! -f tls/ca.key ]]; then
  openssl req -x509 -newkey rsa:2048 -nodes -days 3650 -subj "/CN=mabur web-gs local CA" \
    -addext "basicConstraints=critical,CA:TRUE,pathlen:0" \
    -addext "keyUsage=critical,keyCertSign,cRLSign" \
    -addext "nameConstraints=critical,permitted;DNS:localhost,permitted;IP:127.0.0.0/255.0.0.0,permitted;IP:10.0.0.0/255.0.0.0,permitted;IP:172.16.0.0/255.240.0.0,permitted;IP:192.168.0.0/255.255.0.0" \
    -keyout tls/ca.key -out tls/ca.crt 2>/dev/null
  echo "new CA: tls/ca.crt (install it on every viewing device)"
fi
san="DNS:localhost,IP:127.0.0.1"
for ip in $(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1); do san="$san,IP:$ip"; done
for x in "$@"; do
  if [[ $x =~ ^[0-9.]+$ ]]; then san="$san,IP:$x"; else san="$san,DNS:$x"; fi
done
openssl req -newkey rsa:2048 -nodes -subj "/CN=mabur-web-gs" -keyout tls/key.pem -out tls/server.csr 2>/dev/null
printf 'subjectAltName=%s\nbasicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\n' "$san" > tls/server.ext
openssl x509 -req -in tls/server.csr -CA tls/ca.crt -CAkey tls/ca.key -CAcreateserial \
  -days 397 -extfile tls/server.ext -out tls/cert.pem 2>/dev/null
rm -f tls/server.csr tls/server.ext
echo "tls/cert.pem (signed by tls/ca.crt) SAN: $san"
