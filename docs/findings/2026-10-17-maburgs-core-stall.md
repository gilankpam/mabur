# Why does maburgs's core thread leave a frame body in the RX queue for 8-26 ms, five times a minute?

**Status:** investigation, root cause found; fixed by the window sampler
(`docs/inflight-channel-hop.md`, "Verdict engine"). The latency
stamps it was read from are `docs/observability.md` (`au.log`
`t_last`/`t_drain`); the per-window sensor reads belong to the channel
health check in `docs/inflight-channel-hop.md`; the lock lives in devourer
(`third_party/devourer/CLAUDE.md`). Found while measuring the phase lock
(`2026-10-17-phase-lock-spike-2.md`), where these are the only skipped
frames at a 35 ms lock.

## Question

`au.log` shows, in every bench session and under every regulator setting,
one AU about every 5-8 s whose last body waits 8-26 ms between its RX
stamp (`t_last`, the devourer callback's push into `BodyQueue`) and the
core thread's drain (`t_drain`). The core thread's drain is a condvar
wait with a 10 ms cap and a wake on every push, so an idle core thread
takes ~100 µs. What is the core thread doing for 8-26 ms?

## Method

No tracer on the GS image, so everything came from `/proc` with busybox:

- Kernel schedstats, switched on at runtime (`sched_schedstats`) and reset
  per thread (`echo 0 > /proc/<pid>/task/<tid>/sched`), gave the core
  thread's maxima over a 120 s window: run-queue wait 1.2 ms, longest run
  4.1 ms, D-state 0.7 ms, longest voluntary sleep 19.3 ms, against a
  largest stall of 19.8 ms in the same window. Not preempted, not busy:
  asleep in one wait.
- `/proc/<tid>/syscall` sampled ~5 500 times a second shows a sleeping
  thread's syscall and arguments. The stalls are `futex` with
  `FUTEX_WAIT_PRIVATE`, value 2, no timeout — glibc's mutex-lock protocol —
  on two heap addresses, one per card, with the PC in
  `__lll_lock_wait_private`. At every such instant the only other awake
  threads were the libusb event loops, in `ppoll`.
- A capture script matched that signature and immediately read 3 KB of
  the thread's stack at the syscall's SP from `/proc/<pid>/mem`
  (`dd … | od -An -tx8 -v`; busybox `hexdump`'s `1/8 "%016x"` prints
  bytes, not words, which cost two rounds). Return addresses were
  symbolized against the unstripped twin of the deployed binary
  (`build-arm64-br/gs/maburgs`, same `.text`) and the Radxa sysroot's
  libc and libusb.

## Result

Eight of eight captured stacks are the same chain:

    run_radio → ChannelCore::tick → step_hop_edge_and_window_
      → RadioFrontend::read_nhm_busy → RtlJaguar3Device::ReadNhmBusy()+0x3c
      → pthread_mutex_lock(_reg_mu)

`_reg_mu` is devourer's per-device register lock. Its other taker is the
card's own `coex_runtime_loop`, which every 2 s holds it across
`coex_run_5g`, `pwr_track`, `phydm.tick` and three firmware H2C calls —
synchronous USB work that takes 10-26 ms, during which the thread sits in
`ppoll` inside libusb. The core thread meets it because every channel
health window (`hop.window_ms`, 150 ms) it reads each card's NHM busy
counters (4 registers), energy (~8), re-arms NHM (~13) and the tuned
centre — all synchronous USB register I/O under `_reg_mu`, on the video
path.

The arithmetic closes: two cards at one coex hold each per 2 s is one
hold per second; a 10-26 ms hold overlaps a 150 ms window with 7-17 %
probability, so 0.07-0.17 stalls/s predicted against 0.13-0.20 measured.

**Confirmation** (config only, same session): `hop.window_ms` 150 → 2000
cut the stalls from 0.131/s (344 in 2 627 s) to 0.019/s (20 in 1 027 s)
and the queue-wait p99.9 from 11.7 to 6.0 ms. The residual 0.02/s is a
different wait: the core thread inside one of its own synchronous USB
transfers, waiting (libusb's event-waiter condition) for the card's
event loop to reap the completion — the same I/O, without the lock.

Ruled out along the way, each by reading the code or the data:
`LogWriter` (lock-free producer), the AU doorbell (`MSG_DONTWAIT`), control
TX (`tx_async`, non-blocking), the in-flight scout's dwells (no timing
correlation; they run on the scout thread), preemption and CPU contention.

## Answer

The core thread stalls because it performs synchronous USB register I/O
on every channel-health window and that I/O shares a lock with the
card's coex tick. The lock contention is the bulk of it; the I/O itself
is the remainder. The fix is in maburgs, not devourer: the core thread
should not touch USB registers at all. The per-window sensor sampling
(NHM read/arm, energy, tuned centre) belongs on a per-card sampler thread
that publishes a snapshot the core thread reads without blocking — the
pattern the in-flight scout already uses for its dwell records. A
`try_lock`-and-skip would remove the contention but keep the I/O cost on
the video path, so it is not the end state.

Until then every one of these AUs becomes a visible skip once the display
runs without the regulator chain's period of slack (the 35 ms lock), and
at the default settings it is the `t_drain` tail in every recording.
