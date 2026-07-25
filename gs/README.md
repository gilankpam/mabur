# gs/ — ground station (maburgs)

Datapath (this tree) turns maburd's air frames back into RTP: SBI unpack ->
per-layer sliding-window decode (`libmabur_common` decode classes) -> whole-frame
assembly ordered by `frame_id` (`FrameStream`) -> RFC 7798 packetization
(`RtpPacketizer`), with multi-card union/dedup at the FEC symbol level and UDP
RTP out (default 127.0.0.1:5600).

## Modes

**Radio mode** (default): live N-card front-ends, adaptive-link controller, card failover, RTP+control output:

    maburgs -c /etc/maburgs.json

**Dry-run mode**: replays maburd `--dry-run --out` frame files as N virtual cards:

    maburgs -c gs/bundle/maburgs.default.json --dry-run --in frames.bin \
            [--cards N --drop-pct P --seed S] [--out-rtp rtp.bin]

Radio-mode design: `docs/superpowers/specs/2026-07-12-mabur-gs-design.md`.

## Stats sideport

With `"stats": {"enable": true}` in maburgs.json, maburgs pushes a JSON
datagram (schema v1: cards with per-RF-class signal stats, link{streams,video}, rates + totals) to
`host:port` (default 127.0.0.1:8300) every `interval_ms` (default 500).
Consumers: `tools/maburtop.py` (fullscreen console), PixelPilot OSD
(planned), or `nc -lu 8300` for a raw look. Spec:
`docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md`.
