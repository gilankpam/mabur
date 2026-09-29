import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ensureLocalNetwork, LNA_DENIED, LNA_UNANSWERED } from '../ui/src/lib/relay_permission.js';

// states: successive query() answers; a thrown entry = browser without LNA.
function rig(states, socketResult = 'open') {
  const opened = [];
  const q = [...states];
  return {
    opened,
    args: (protocol = 'https:') => ({
      protocol, url: 'ws://192.168.1.1:8311',
      query: async () => { const s = q.shift(); if (s instanceof Error) throw s; return { state: s }; },
      openSocket: async (url, ms) => { opened.push([url, ms]); return socketResult; },
    }),
  };
}

test('http origin: no permission dance, no probe socket', async () => {
  const r = rig(['prompt']);
  await ensureLocalNetwork(r.args('http:'));
  assert.equal(r.opened.length, 0);
});

test('browser without the local-network permission: fall through to the core', async () => {
  const r = rig([new TypeError('bad name')]);
  await ensureLocalNetwork(r.args());
  assert.equal(r.opened.length, 0);
});

test('granted: start straight away', async () => {
  const r = rig(['granted']);
  await ensureLocalNetwork(r.args());
  assert.equal(r.opened.length, 0);
});

test('denied: refuse with the site-settings hint, never probe', async () => {
  const r = rig(['denied']);
  await assert.rejects(ensureLocalNetwork(r.args()), { message: LNA_DENIED });
  assert.equal(r.opened.length, 0);
});

test('prompt: probe socket raises it, then the answer decides', async () => {
  // Revert check: without the probe the core starts under a pending prompt and
  // reports "relay unreachable" ~2 s later (Chrome 147 headed, 2026-09-29).
  let r = rig(['prompt', 'granted']);
  await ensureLocalNetwork(r.args());
  assert.deepEqual(r.opened, [['ws://192.168.1.1:8311', 60000]]);
  r = rig(['prompt', 'denied'], 'closed');
  await assert.rejects(ensureLocalNetwork(r.args()), { message: LNA_DENIED });
  r = rig(['prompt', 'prompt'], 'timeout');
  await assert.rejects(ensureLocalNetwork(r.args()), { message: LNA_UNANSWERED });
});
