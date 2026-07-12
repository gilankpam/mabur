#!/usr/bin/env bash
# Deploy maburgs to a Radxa (or any systemd aarch64 host): binary + config +
# unit. Usage: gs/bundle/install.sh root@<gs-ip> [path/to/maburgs]
set -euo pipefail
cd "$(dirname "$0")/../.."
HOST=${1:?usage: install.sh root@<gs-ip> [binary]}
BIN=${2:-out/arm64/maburgs}
[ -f "$BIN" ] || { echo "error: $BIN not built (run tools/build-arm64.sh)"; exit 1; }

scp "$BIN" "$HOST:/usr/local/bin/maburgs.new"
scp gs/bundle/maburgs.service "$HOST:/etc/systemd/system/maburgs.service"
# Config: install the default only if none exists (never clobber a tuned one).
ssh "$HOST" "[ -f /etc/maburgs.json ]" || \
  scp gs/bundle/maburgs.default.json "$HOST:/etc/maburgs.json"
ssh "$HOST" 'mv /usr/local/bin/maburgs.new /usr/local/bin/maburgs &&
             chmod +x /usr/local/bin/maburgs &&
             systemctl daemon-reload &&
             systemctl enable --now maburgs &&
             systemctl --no-pager status maburgs | head -5'
