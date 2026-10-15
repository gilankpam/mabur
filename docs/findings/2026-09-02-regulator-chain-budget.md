# Regulator one-vsync-late chains and the `display.chain_budget` A/B

**Status:** superseded — the bundle ships `chain_budget = 0` since the cut turned out to be most of the clean-link judder (`docs/findings/2026-10-15-regulator-chain-cuts.md`). The regulator line's chain counters are documented in docs/observability.md ("Player stderr lines").

Extracted from docs/observability.md during the 2026-10-11 docs tidy.

## Chains: what they cost (bench 2026-09-02)

These chains are the OTHER kind of one-vsync-late chain, the one the servo
itself creates: one collision (two decodes inside one vblank window)
shifts every following frame a period late until an arrival gap wider
than a period frees a slot. The extra period lands in the `lat:` line's
`reg` segment, so read `chained/held` per second next to `reg` p50.

Bench 2026-09-02 (mcs5 park, agg6+fb6, lead 6, 196 s): 38.9 % of held
frames chained, per-second fraction 7–71 %, longest run 52 frames,
`hold_ema` 13.1 ms with no chain running vs 18.1 ms with one — about
6.5 ms of mean regulator hold (0.39 × 16.7) that the vblank lock does
not need. Collisions come from arrival jitter (AU completion gaps 8–27
ms, ~10 % under 10 ms), not from pair bunching — the AU ring shows
frames completing one per period, not in pairs.

## `display.chain_budget` A/B

Same session, 150 s arms, budget 0 → 6 → 0 → 3, config-only restarts
(`lat:` p50 means over 1 Hz windows, bootstrap 95 % CI vs arm A; A′
reproduced A within noise, e2e −0.2 [−1.2, +0.8]):

| budget | chained | chains/s | cuts/s | hold_ema | reg p50 mean | e2e p50 mean | reg p99 | lat n |
|---|---|---|---|---|---|---|---|---|
| 0 (A) | 39.7 % | 6.9 | 0 | 15.1 | 15.0 | 45.5 | 25.1 | 59.4 |
| 6 (B) | 27.1 % | 6.4 | 0.56 | 13.1 | 12.3 (−2.7 [−3.5, −1.8]) | 43.0 (−2.4 [−3.3, −1.6]) | 24.1 | 58.8 |
| 0 (A′) | 40.6 % | 6.9 | 0 | 15.4 | 15.1 | 45.3 | 25.0 | 59.4 |
| 3 (C) | 19.6 % | 6.1 | 1.44 | 11.9 | 10.7 (−4.3 [−5.1, −3.5]) | 41.6 (−3.9 [−4.8, −3.0]) | 24.3 | 57.9 |

Chains start ~7×/s with a mean length of 3.4 frames, so a budget
only trims the long tail of runs: 6 buys 2.4 ms of median e2e for
0.56 drops/s, 3 buys 3.9 ms for 1.44 drops/s, and budget 1 would be
freshest-wins at ~7 drops/s. reg p99 is untouched by any budget (a
chained frame plus a full phase margin is the p99 whatever the run
length). present_jitter EMA rose 0.56 → 0.90 → 1.37 ms with the
drops. Operator picked **3 as the shipped default** (2026-09-02); 0
restores the unbounded behavior. The counters make either auditable.

## Other regulator-line bench numbers from the same period

- `skips=` (deep-burst slot claims in servo mode): bench steady state
  ~1–1.4/s at the mcs5 park, from fec-batch 4-frame bursts.
- `pdrop=` (paced-mode mailbox drops): steady state ~0.2/s at
  `vsync_lead_ms` 6.
- `heals=` (chain-break slips) fired ~only at startup.
