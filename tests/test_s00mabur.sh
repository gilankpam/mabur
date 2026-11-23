#!/bin/sh
# Drives one respawn iteration (`S96mabur step`) of the init loop against
# fake maburd scripts. usage: test_s00mabur.sh <path to S96mabur>
S=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
ROOT=$(mktemp -d) || exit 1
trap 'rm -rf "$ROOT"' EXIT INT TERM
CASE=""

fail() { echo "FAIL $CASE: $*"; exit 1; }

# fake <path> <board|-> <mode> [badmig]: identify answers the board ("-" =
# identify fails); modes: run (record and exit), proven, exit, hang, linked.
fake() {
  cat > "$1" <<EOT
#!/bin/sh
case "\$1" in
  --identify) [ "$2" = - ] && exit 1; echo "version=v1 board=$2"; exit 0 ;;
  --migrate-config) [ "$4" = badmig ] && exit 1; cp "\$2" "\$3"; echo migrated >> "\$3"; exit 0 ;;
esac
echo "$3 trial=\$MABUR_TRIAL" > "$T/ran"
nap() { n=0; while [ \$n -lt \$1 ]; do sleep 1; n=\$((n+1)); done; }
case "$3" in
  run) ;;
  proven) : > "$T/up/linked"; : > "$T/up/proven"; nap 2 ;;
  exit) ;;
  linked) : > "$T/up/linked"; nap 60 ;;
  hang) nap 60 ;;
esac
EOT
  chmod +x "$1"
}

# setup <name> <new board> <new mode> [badmig] [old board]
setup() {
  CASE=$1
  T=$ROOT/$1; mkdir -p "$T/up" "$T/bin"
  export MABUR_DAEMON="$T/bin/maburd" MABUR_CONF="$T/mabur.toml" MABUR_UP="$T/up"
  export MABUR_LOG="$T/log" MABUR_TRIAL_S=3 MABUR_RESPAWN_S=0
  fake "$T/bin/maburd" "${5:-A}" run
  echo old > "$T/mabur.toml"
  cp "$T/bin/maburd" "$T/old.bin"
  fake "$T/up/maburd" "$2" "$3" "$4"
  : > "$T/up/go"
}

step() { sh "$S" step >/dev/null 2>&1; }
result() { cat "$T/up/result" 2>/dev/null; }
expect_result() { [ "$(result)" = "$1" ] || fail "result '$(result)', want '$1'"; }
is_old() {
  cmp -s "$T/bin/maburd" "$T/old.bin" || fail "binary is not the old one"
  [ "$(cat "$T/mabur.toml")" = old ] || fail "config is not the old one"
}
no_go() { [ ! -e "$T/up/go" ] || fail "go not consumed"; }

setup no_go_runs_the_installed_binary A proven; rm "$T/up/go"
step
grep -q '^run ' "$T/ran" || fail "installed binary did not run"
[ ! -e "$T/up/result" ] || fail "result written without go"

setup good_trial_commits A proven
step; no_go
expect_result committed
cmp -s "$T/bin/maburd" "$T/up/maburd" || fail "new binary not installed"
cmp -s "$T/bin/maburd.prev" "$T/old.bin" || fail "binary .prev is not the old one"
[ "$(cat "$T/mabur.toml.prev")" = old ] || fail "config .prev is not the old one"
[ "$(cat "$T/mabur.toml" | tr '\n' ' ')" = "old migrated " ] || fail "config not migrated"
[ ! -e "$T/bin/maburd.new" ] && [ ! -e "$T/mabur.toml.new" ] || fail "leftover .new"

setup trial_that_exits_rolls_back A exit
step; no_go; expect_result "rolled_back exited"; is_old

setup trial_without_link_rolls_back_no_link A hang
step; no_go; expect_result "rolled_back no_link"; is_old
[ -z "$(pgrep -f "$T/bin/maburd")" ] || fail "trial left running"

setup trial_linked_without_video_rolls_back_no_video A linked
step; no_go; expect_result "rolled_back no_video"; is_old

setup wrong_board_is_refused_and_nothing_installed B proven
step; no_go; expect_result "refused wrong_board"; is_old
[ ! -e "$T/bin/maburd.prev" ] || fail ".prev written"

setup unidentifiable_binary_is_refused_wrong_board - proven
step; no_go; expect_result "refused wrong_board"; is_old

setup bad_config_is_refused_and_nothing_installed A proven badmig
step; no_go; expect_result "refused bad_config"; is_old
[ ! -e "$T/bin/maburd.prev" ] || fail ".prev written"

setup stale_proven_does_not_commit_a_failing_trial A exit
: > "$T/up/proven"; : > "$T/up/linked"; echo committed > "$T/up/result"
step; expect_result "rolled_back exited"; is_old

setup trial_runs_with_MABUR_TRIAL_1 A proven
step
grep -q 'trial=1' "$T/ran" || fail "MABUR_TRIAL not 1: $(cat "$T/ran")"

echo "PASS test_s00mabur"
