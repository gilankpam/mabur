// Pure, node-testable logic for the web-GS page (promoted from the
// wasm-spike throwaway). Gate rules = maburplay's (gs/player/src/main.cpp
// sink + RingClient), fixed for the 2-stream space: arm on a complete AU
// carrying VPS+SPS+PPS (sid 0 alone no longer implies parameter sets).

export function nalTypes(u8) {
  const out = [];
  for (let i = 0; i + 3 < u8.length; i++) {
    if (u8[i] === 0 && u8[i + 1] === 0) {
      if (u8[i + 2] === 1) { out.push((u8[i + 3] >> 1) & 0x3f); i += 3; }
      else if (u8[i + 2] === 0 && u8[i + 3] === 1 && i + 4 < u8.length) {
        out.push((u8[i + 4] >> 1) & 0x3f); i += 4;
      }
    }
  }
  return out;
}

const VPS = 32, SPS = 33, PPS = 34, DISCONT = 0x02;

export class Gate {
  constructor() { this.armed = false; }
  onAu({ flags, complete, data }) {
    let reset = false;
    if ((flags & DISCONT) && this.armed) { this.armed = false; reset = true; }
    if (!complete) return { type: null, reset, skip: 'truncated' };
    if (!this.armed) {
      const t = nalTypes(data);
      if (t.includes(VPS) && t.includes(SPS) && t.includes(PPS)) {
        // WebCodecs rejects a non-IRAP first key chunk (Annex-B and hvcC
        // alike), and the drone is GDR: its parameter sets usually ride a
        // TRAIL_R refresh. Arm only on an IRAP (BLA/IDR/CRA, types 16-21).
        if (!t.some((x) => x >= 16 && x <= 21)) return { type: null, reset, skip: 'awaiting-irap' };
        this.armed = true;
        return { type: 'key', reset, skip: null };
      }
      return { type: null, reset, skip: 'gated' };
    }
    return { type: 'delta', reset, skip: null };
  }
  onDecoderError() { this.armed = false; }
}

// GS-requested IDR pacing (spec 2026-09-28, backoff added in the final
// whole-branch review 2026-09-28): ask the moment the gate is unarmed, then
// every retryMs while it stays so (the IDR itself was lost). Polled per AU
// and on decoder errors; no timer -- with no AUs the link is down and the
// drone's link-up path covers it.
//
// Backoff: a device-local decode failure (unsupported codec, reclaimed
// hardware decoder, ...) means the drone's IDR was never the problem, so
// hammering it at retryMs forever just costs the biggest frames on the link
// for nothing. noteKeyFailed()/noteOutput() track consecutive key-frame
// failures since the last decoded output; the effective retry interval
// doubles per failure, capped at maxRetryMs, and collapses back to retryMs
// on the next real output.
export class IdrRequester {
  constructor(retryMs = 300, maxRetryMs = 4800) {
    this.retryMs = retryMs;
    this.maxRetryMs = maxRetryMs;
    this.lastMs = null;
    this.prevMs = null;   // lastMs before the most recent fired poll(); cancel() undoes it
    this.n = 0;            // consecutive key-frame failures since the last output
  }
  noteKeyFailed() { this.n++; }
  noteOutput() { this.n = 0; }
  interval() { return Math.min(this.retryMs * 2 ** this.n, this.maxRetryMs); }
  poll(armed, nowMs) {
    if (armed) {
      // Only wipe the clock on a "clean" re-arm (no failures pending): once
      // backed off (n > 0), a momentary re-arm that then fails again (the
      // exact shape of a permanent local failure -- the gate arms on the
      // bitstream alone, before the decoder has had a chance to reject it)
      // must not reset the wait back to zero, or the backoff never bites.
      if (this.n === 0) this.lastMs = null;
      return false;
    }
    if (this.lastMs !== null && nowMs - this.lastMs < this.interval()) return false;
    this.prevMs = this.lastMs;
    this.lastMs = nowMs;
    return true;
  }
  // onWantIdr reported it didn't actually send (e.g. not live yet): undo the
  // fire just recorded so the next poll() asks again instead of waiting out
  // retryMs/interval for a request that never went anywhere.
  cancel() { this.lastMs = this.prevMs; }
}

export class PtsUnwrap {
  constructor() { this.hi = 0; this.last = null; }
  add(pts) {
    if (this.last !== null && pts < this.last && this.last - pts > 0x80000000) this.hi += 0x100000000;
    this.last = pts;
    return this.hi + pts;
  }
}

export class PeriodEstimator {
  constructor() { this.prev = null; this.deltas = []; }
  add(ptsUs) {
    if (this.prev !== null && ptsUs > this.prev) {
      this.deltas.push(ptsUs - this.prev);
      if (this.deltas.length > 64) this.deltas.shift();
    }
    this.prev = ptsUs;
  }
  periodMs() {
    if (!this.deltas.length) return null;
    return pctl(this.deltas, 0.5) / 1000;
  }
}

export class HitchMeter {
  constructor() { this.last = null; }
  addDraw(tMs, periodMs) {
    const gap = this.last === null ? null : tMs - this.last;
    this.last = tMs;
    return gap !== null && periodMs && gap > 1.5 * periodMs ? gap : null;
  }
}

export function pctl(values, q) {
  if (!values.length) return 0;
  const s = [...values].sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor(s.length * q))];
}

// Annex-B -> 4-byte big-endian length-prefixed NAL units (the hvcC framing
// WebCodecs expects once a `description` is configured).
export function annexbToLengthPrefixed(u8) {
  const starts = [];  // [payloadStart, startCodeStart]
  for (let i = 0; i + 2 < u8.length; i++) {
    if (u8[i] === 0 && u8[i + 1] === 0 && u8[i + 2] === 1) {
      starts.push([i + 3, i > 0 && u8[i - 1] === 0 ? i - 1 : i]);
      i += 2;
    }
  }
  const nals = starts.map(([s], k) => u8.subarray(s, k + 1 < starts.length ? starts[k + 1][1] : u8.length));
  const out = new Uint8Array(nals.reduce((n, x) => n + 4 + x.length, 0));
  let o = 0;
  for (const x of nals) {
    new DataView(out.buffer).setUint32(o, x.length);
    out.set(x, o + 4);
    o += 4 + x.length;
  }
  return out;
}

// Holds the live VideoDecoder; replacing it closes the old one (a rejected
// or reset decoder left open pins a hardware decode slot until GC).
export class DecoderSlot {
  constructor() { this.d = null; }
  replace(next) {
    if (this.d && this.d.state !== 'closed') this.d.close();
    this.d = next;
    return next;
  }
}

// Rolling per-segment latency stats (e.g. 'decode', 'present') over a 1 s
// and a 60 s trailing window, both measured back from `snapshot`'s `now`
// (default: nowFn()). Entries older than 60 s are pruned as they age out
// on add, so the backing arrays never grow unbounded.
export class SegWindow {
  constructor(nowFn) {
    this.nowFn = nowFn || (() => Date.now());
    this.byName = new Map();
  }
  add(name, ms) {
    const now = this.nowFn();
    let arr = this.byName.get(name);
    if (!arr) { arr = []; this.byName.set(name, arr); }
    arr.push({ t: now, v: ms });
    const cutoff60 = now - 60000;
    let drop = 0;
    while (drop < arr.length && arr[drop].t <= cutoff60) drop++;
    if (drop > 0) arr.splice(0, drop);
  }
  // Both windows. O(60 s of samples) -- the page calls it at 1 Hz and for
  // Copy stats, never on the 200 ms tick.
  snapshot(now) {
    if (now === undefined) now = this.nowFn();
    const cutoff1 = now - 1000;
    const cutoff60 = now - 60000;
    const w1 = {}, w60 = {};
    for (const [name, arr] of this.byName) {
      const v1 = [], v60 = [];
      for (const e of arr) {
        if (e.t > cutoff60) v60.push(e.v);
        if (e.t > cutoff1) v1.push(e.v);
      }
      w1[name] = summarize(v1);
      w60[name] = summarize(v60);
    }
    return { w1, w60 };
  }
  // The 1 s window only (all latencyNow needs): walks back from the newest
  // sample, so the cost is ~1 s of samples, not 60 s.
  snapshot1(now) {
    if (now === undefined) now = this.nowFn();
    const cutoff1 = now - 1000;
    const w1 = {};
    for (const [name, arr] of this.byName) {
      const v1 = [];
      for (let i = arr.length - 1; i >= 0 && arr[i].t > cutoff1; i--) v1.push(arr[i].v);
      w1[name] = summarize(v1);
    }
    return { w1 };
  }
}

function summarize(values) {
  // One sort serves p50, p99 and max.
  const s = values.sort((a, b) => a - b);
  const n = s.length;
  const at = (q) => (n ? s[Math.min(n - 1, Math.floor(n * q))] : 0);
  return { p50: at(0.5), p99: at(0.99), max: n ? s[n - 1] : 0, n };
}

// Capture (drone encode-complete) to glass (present) latency, ms:
// cap-to-complete (drone-side, us) plus the two GS-side hop deltas. null
// when the drone didn't report a cap-to-complete offset (absent, or a
// negative sentinel meaning "no measurement").
export function capToGlass({ capToCompleteUs, tEmitMs, tRecvMs, tPresentMs }) {
  if (capToCompleteUs == null || capToCompleteUs < 0) return null;
  return capToCompleteUs / 1000 + (tRecvMs - tEmitMs) + (tPresentMs - tRecvMs);
}

// % of GS->drone RCFs the drone reports having heard, over the delta
// between two stats snapshots. The denominator is `rcf_sent` (RCFs only),
// never `sends`: the drone's Telem.rcf_rx counts RCFs only, and `sends`
// also counts DISC beacons/keep-alives, which would cap the ratio below the
// 95 % pass mark. null when either snapshot is missing, either is missing
// drone_rcf_rx/rcf_sent, rcf_sent didn't advance, or drone_rcf_rx went
// backwards (counter reset, e.g. drone restart) rather than wrapping cleanly.
export function rcfHeardPct(prev, cur) {
  if (!prev || !cur) return null;
  if (prev.drone_rcf_rx == null || cur.drone_rcf_rx == null) return null;
  if (prev.rcf_sent == null || cur.rcf_sent == null) return null;
  const dSent = cur.rcf_sent - prev.rcf_sent;
  if (!(dSent > 0)) return null;
  const dRcf = cur.drone_rcf_rx - prev.drone_rcf_rx;
  if (dRcf < 0) return null;
  return 100 * dRcf / dSent;
}

// F2: dividing adjacent 1 s stats ticks overshoots (100-105 %) because
// drone_rcf_rx (drone Telem, 1 Hz) and rcf_sent (GS-side) are sampled at
// different instants -- the two counters' phase drifts a tick's worth
// relative to each other. Diffing over a ~10-tick (~10 s) window absorbs
// that skew instead of clamping it away. `ring` is the trailing stats
// history, oldest first, one entry per second (the page's existing 60-
// entry stats ring); picks the entry `windowTicks` back from the newest,
// or the oldest available if the ring is shorter than that. null on an
// empty or single-entry ring (rcfHeardPct itself covers the missing-field
// / reset / non-positive-delta cases).
export function rcfHeardPctWindowed(ring, windowTicks = 10) {
  if (!ring || ring.length < 2) return null;
  const cur = ring[ring.length - 1];
  const prev = ring[Math.max(0, ring.length - 1 - windowTicks)];
  return rcfHeardPct(prev, cur);
}

// R12 / F1: the running WASM module keeps the mode/channel/width it was
// started with -- changing the mode radio or the channel/width inputs
// after Connect does nothing but fight the module's actual (unchanged)
// behaviour. Lock those controls for as long as a connect attempt is in
// flight or a module is connected; app.js re-enables them on every path
// that already re-enables the Connect button (error, cancel, module-start
// failure, card lost).
export function controlsLocked({ connecting, connected }) {
  return !!connecting || !!connected;
}

// F3: an rAF callback scheduled just before the tab is hidden fires only
// once the tab becomes visible again, producing a present / capture->glass
// sample of however long the tab was hidden (observed ~18 s outliers on
// the bench). Reject a sample whose rAF timestamp lands more than staleMs
// after the draw that scheduled it -- a gap that large can only mean the
// callback was suspended, never a real frame.
export function isStalePresentSample(drawMs, rafMs, staleMs = 1000) {
  return (rafMs - drawMs) > staleMs;
}

// common/include/mabur/ht40.h's ht40_offset: 1 = HT40+, 2 = HT40-, 0 = no pair.
export function ht40Offset(ch) {
  if (ch >= 36 && ch <= 144 && (ch - 36) % 4 === 0) return ((ch - 36) / 4) % 2 === 0 ? 1 : 2;
  if (ch >= 149 && ch <= 161 && (ch - 149) % 4 === 0) return ((ch - 149) / 4) % 2 === 0 ? 1 : 2;
  return 0;
}

// Pre-connect check of the page's channel/width (strings from the inputs).
// Mirrors the glue's checks that need no config (web_gs.cpp
// channel_width_error): channel an integer in [1,200], width 20|40, 40 only
// on an HT40 pair. The GS-mode "40 MHz rung while tuned 20" check needs the
// ladder and stays in the glue (ERROR bad channel/width: ...). Returns null
// when OK, else a user-facing message.
export function checkChannelWidth(chStr, wStr) {
  const chS = String(chStr ?? '').trim();
  if (!/^\d+$/.test(chS)) return `Channel "${chS}" is not a number.`;
  const ch = Number(chS);
  if (ch < 1 || ch > 200) return `Channel ${ch} is out of range (1–200).`;
  const w = String(wStr ?? '').trim();
  if (w !== '20' && w !== '40') return `Width "${w}" must be 20 or 40 MHz.`;
  if (w === '40' && ht40Offset(ch) === 0) {
    return `Channel ${ch} has no 40 MHz pair — pick 20 MHz or a paired channel (e.g. 136).`;
  }
  return null;
}

// Drops decode-submit records older than maxAgeMs (their frame will never
// be output: decoder replaced/reset, or the chunk was dropped). Mutates and
// returns the map.
export function pruneSubmitted(map, nowMs, maxAgeMs = 1000) {
  for (const [k, v] of map) if (nowMs - v.tSubmit > maxAgeMs) map.delete(k);
  return map;
}

// Drops leading entries <= cutoff from an ascending array of times, in place.
export function trimBefore(times, cutoff) {
  let drop = 0;
  while (drop < times.length && times[drop] <= cutoff) drop++;
  if (drop) times.splice(0, drop);
  return times;
}

// "Copy stats" payload: the core's last 60 s of 1 Hz stats lines (parsed;
// an unparsable line is kept as its raw string) plus the page-side latency
// segments (SegWindow.snapshot(): w1 + w60).
export function copyStatsPayload(statsRing, segSnapshot) {
  const core = statsRing.map((t) => { try { return JSON.parse(t); } catch { return t; } });
  return JSON.stringify({ core, segments: segSnapshot }, null, 1);
}

// WebUSB chooser filters: Realtek's own VID plus TP-Link, whose Archer
// T2U Plus / T4U-class sticks are RTL8821AU (Jaguar1) under 2357:011e/0120/
// 0122. The WASM core (web_main.cpp kCards) picks the exact VID:PID; this
// list only decides what the browser offers and what counts as granted.
export const USB_FILTERS = [{ vendorId: 0x0bda }, { vendorId: 0x2357 }];
export function isCardVendor(vendorId) {
  return USB_FILTERS.some((f) => f.vendorId === vendorId);
}

// The CPE relay's fixed LAN address (mabur-openwrt 90-mabur-lan) + the
// relay's WebSocket port. Connect uses it whenever no address is typed.
export const RELAY_DEFAULT = '10.83.11.1:8311';

// Parses a "host" or "host:port" relay address string into a normalized
// "host:port" (port defaults to 8311), or null if malformed. host is an
// IPv4 dotted quad or a DNS-ish name; port must be 1-65535.
export function parseRelayAddr(s) {
  const m = /^([A-Za-z0-9.-]+)(?::(\d{1,5}))?$/.exec(String(s ?? '').trim());
  if (!m) return null;
  const port = m[2] === undefined ? 8311 : Number(m[2]);
  if (!(port >= 1 && port <= 65535)) return null;
  return `${m[1]}:${port}`;
}

// True when a relay host is one Chrome's Local Network Access exempts from
// mixed-content blocking: a private/loopback/link-local IPv4 literal or a
// `.local` name. A public DNS name that happens to resolve to the LAN is NOT
// exempt (it would need fetch's targetAddressSpace, which WebSocket lacks).
export function isLocalRelayHost(host) {
  const h = String(host ?? '').toLowerCase();
  if (h === 'localhost' || h.endsWith('.local')) return true;
  const m = /^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/.exec(h);
  if (!m) return false;
  const [a, b] = [Number(m[1]), Number(m[2])];
  if (m.slice(1).some((x) => Number(x) > 255)) return false;
  return a === 10 || a === 127 || (a === 172 && b >= 16 && b <= 31) ||
    (a === 192 && b === 168) || (a === 169 && b === 254);
}

// Pre-connect check on the TYPED relay address (empty = the default, always
// fine). An https page may only open the plain ws:// relay socket when the
// host is LNA-exempt (Chrome 142+; verified Chrome 147, docs/web-gs.md).
export function relayBlocker(protocol, typed) {
  const t = String(typed ?? '').trim();
  if (!t) return null;
  const norm = parseRelayAddr(t);
  if (!norm) return 'Enter the CPE relay address as host or host:port.';
  if (protocol === 'https:' && !isLocalRelayHost(norm.slice(0, norm.lastIndexOf(':')))) {
    return 'On the hosted page the relay address must be a private IP (e.g. 10.83.11.1) or a .local name.';
  }
  return null;
}

// The one address a Connect uses: the typed one if any, else the default.
export function relayTarget(typed) {
  const t = String(typed ?? '').trim();
  return t ? parseRelayAddr(t) : RELAY_DEFAULT;
}

// The address field shows when an address was saved (at page load) or when
// a Connect this visit found no relay. Clearing it hides it from next visit.
export function relayFieldVisible(savedAtLoad, openedThisVisit) {
  return !!(savedAtLoad || openedThisVisit);
}

// The typed relay address from `webgs.last`. New key on purpose: builds
// before 2026-09-29 saved relayAddr = the old default for every user.
export function loadRelayCustom(saved) {
  return saved && typeof saved.relayCustom === 'string' ? saved.relayCustom.trim() : '';
}

// Maps a glue `ERROR ...` line (web/src/web_gs.cpp) to user-facing text.
export function errorText(line) {
  if (line.includes('relay cannot tune')) {
    return 'CPE relay could not tune to this channel/width — check the channel is allowed on the CPE (regulatory domain), then press Connect.';
  }
  if (line.includes('relay unreachable')) {
    return 'CPE relay not reachable — check the Ethernet cable and the relay address, then press Connect.';
  }
  if (line.includes('relay owned by another client')) {
    return 'CPE relay is owned by another client (maburgs or another tab). Stop it, wait 2 s, then press Connect.';
  }
  if (line.includes('relay taken by another client')) {
    return 'Another client took over the CPE relay (maburgs or another tab). Stop it, then press Connect.';
  }
  if (line.includes('relay lost')) {
    return 'CPE relay connection lost. Press Connect to restart.';
  }
  if (line.includes('no RTL card')) {
    return 'No RTL8812EU/8812AU/8821AU card found — plug it in and press Connect.';
  }
  if (line.includes('claim failed')) {
    return 'Card busy — maburgs or another tab has it. Close that and press Connect.';
  }
  if (line.includes('card lost')) {
    return 'Card lost (unplugged?). Press Connect to restart.';
  }
  if (line.includes('libusb_init')) {
    return 'WebUSB unavailable in this browser.';
  }
  if (line.includes('unsupported chip')) {
    return 'Unsupported card chip.';
  }
  const bc = line.indexOf('bad config:');
  if (bc >= 0) {
    const rest = line.slice(bc + 'bad config:'.length).trim()
      .replace(/^config:\s*(?:[^:\s]+:\d+:\s*)?/, '');
    return 'Config refused: ' + rest + '. Fix it in Config and press Connect.';
  }
  const bad = line.indexOf('bad channel/width:');
  if (bad >= 0) {
    return 'Channel/width refused: ' + line.slice(bad + 'bad channel/width:'.length).trim() +
      '. Pick another channel/width and press Connect.';
  }
  return line;
}

// SipHash-2-4 (BigInt), byte-identical to common/src/siphash.cpp. Used only
// for the key fingerprint shown beside the Load button; the tags on the
// wire are computed by the WASM core.
const M64 = (1n << 64n) - 1n;
const rotl = (x, b) => ((x << BigInt(b)) | (x >> BigInt(64 - b))) & M64;
function u64le(a, o) { let v = 0n; for (let i = 7; i >= 0; i--) v = (v << 8n) | BigInt(a[o + i]); return v; }
export function siphash24(key, msg) {
  const k0 = u64le(key, 0), k1 = u64le(key, 8);
  let v0 = 0x736f6d6570736575n ^ k0, v1 = 0x646f72616e646f6dn ^ k1;
  let v2 = 0x6c7967656e657261n ^ k0, v3 = 0x7465646279746573n ^ k1;
  const round = () => {
    v0 = (v0 + v1) & M64; v1 = rotl(v1, 13); v1 ^= v0; v0 = rotl(v0, 32);
    v2 = (v2 + v3) & M64; v3 = rotl(v3, 16); v3 ^= v2;
    v0 = (v0 + v3) & M64; v3 = rotl(v3, 21); v3 ^= v0;
    v2 = (v2 + v1) & M64; v1 = rotl(v1, 17); v1 ^= v2; v2 = rotl(v2, 32);
  };
  const n = msg.length, full = n - (n % 8);
  let i = 0;
  for (; i < full; i += 8) { const m = u64le(msg, i); v3 ^= m; round(); round(); v0 ^= m; }
  let b = BigInt(n) << 56n;
  for (let j = 0; i + j < n; j++) b |= BigInt(msg[i + j]) << BigInt(8 * j);
  v3 ^= b; round(); round(); v0 ^= b;
  v2 ^= 0xffn; round(); round(); round(); round();
  return (v0 ^ v1 ^ v2 ^ v3) & M64;
}
export const DEFAULT_KEY_HEX = '6d616275722d64656661756c742d3030';
export function keyFingerprint(hex) {
  if (!hex || hex === DEFAULT_KEY_HEX) return 'default';
  const key = Uint8Array.from(hex.match(/../g), (h) => parseInt(h, 16));
  const h = siphash24(key, new TextEncoder().encode('mabur.key'));
  const b0 = Number(h & 0xffn), b1 = Number((h >> 8n) & 0xffn);
  return b0.toString(16).padStart(2, '0') + b1.toString(16).padStart(2, '0');
}
