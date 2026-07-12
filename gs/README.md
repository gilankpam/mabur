# gs/ — ground station (maburgs)

Datapath (this tree) decodes maburd's air frames back to RTP: SBI unpack ->
per-layer RS decode -> FRAG reassembly (`libmabur_common` decode classes),
multi-card union/dedup at the FEC symbol level, UDP RTP out (default
127.0.0.1:5600).

## Modes

**Radio mode** (default): live N-card front-ends, adaptive-link controller, card failover, RTP+control output:

    maburgs -c /etc/maburgs.json

**Dry-run mode**: replays maburd `--dry-run --out` frame files as N virtual cards:

    maburgs -c gs/bundle/maburgs.default.json --dry-run --in frames.bin \
            [--cards N --drop-pct P --seed S] [--out-rtp rtp.bin]

Radio-mode design: `docs/superpowers/specs/2026-07-12-mabur-gs-design.md`.
