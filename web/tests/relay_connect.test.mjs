import { test } from 'node:test';
import assert from 'node:assert/strict';
import { HELLO, classifyReply, relayDisplay, connectRelay, LNA_DENIED, LNA_UNANSWERED }
  from '../ui/src/lib/relay_connect.js';

const bytes = (...b) => new Uint8Array(b).buffer;

test('HELLO is magic 0x524D LE, version 3, type 2', () => {
  assert.deepEqual([...HELLO], [0x4D, 0x52, 0x03, 0x02]);
});

test('classifyReply: any v3 mabur message is a relay, including an owned-by-other STATUS', () => {
  // STATUS (type 4) with state byte 3 = "not owner": still a relay (Review Focus 2).
  assert.equal(classifyReply(bytes(0x4D, 0x52, 0x03, 0x04, 0, 0, 3)), 'relay');
  assert.equal(classifyReply(bytes(0x4D, 0x52, 0x03, 0x01)), 'relay');          // FRAME
  assert.deepEqual(classifyReply(bytes(0x4D, 0x52, 0x02, 0x04)), { version: 2 });
  assert.equal(classifyReply(bytes(0x00, 0x52, 0x03, 0x04)), 'none');           // bad magic
  assert.equal(classifyReply(bytes(0x4D, 0x52)), 'none');                       // short
  assert.equal(classifyReply('hello'), 'none');                                 // text frame
});

test('relayDisplay drops the default port only', () => {
  assert.equal(relayDisplay('10.83.11.1:8311'), '10.83.11.1');
  assert.equal(relayDisplay('10.0.0.9:9000'), '10.0.0.9:9000');
});

// states: successive query() answers; an Error entry = browser without LNA.
function rig(states, probeResult) {
  const q = [...states], probes = [];
  return {
    probes,
    args: (protocol = 'https:', addr = '10.83.11.1:8311') => ({
      addr, protocol,
      query: async () => { const s = q.shift(); if (s instanceof Error) throw s; return { state: s }; },
      probe: async (url, ms) => { probes.push([url, ms]); return probeResult; },
    }),
  };
}

test('found: resolves, probes ws://addr once', async () => {
  const r = rig(['granted'], 'relay');
  await connectRelay(r.args());
  assert.deepEqual(r.probes, [['ws://10.83.11.1:8311', 1500]]);
});

test('http: no permission query, short probe', async () => {
  const r = rig([], 'relay');
  await connectRelay(r.args('http:'));
  assert.deepEqual(r.probes, [['ws://10.83.11.1:8311', 1500]]);
});

test('not found names the address and carries the code', async () => {
  const r = rig(['granted'], 'none');
  await assert.rejects(connectRelay(r.args()), (e) => {
    assert.equal(e.code, 'relay-not-found');
    assert.equal(e.message, 'No CPE relay found at 10.83.11.1 — check the Ethernet cable and that the CPE is powered, or enter its address.');
    return true;
  });
  const r2 = rig([], 'none');
  await assert.rejects(connectRelay(r2.args('http:', '10.0.0.9:9000')), /No CPE relay found at 10\.0\.0\.9:9000 /);
});

test('browser without LNA on https: not-found also names the browser block', async () => {
  const r = rig([new TypeError('bad permission name')], 'none');
  await assert.rejects(connectRelay(r.args()), (e) => {
    assert.equal(e.code, 'relay-not-found');
    assert.match(e.message, /Chrome 142\+/);
    return true;
  });
});

test('wrong protocol version', async () => {
  const r = rig(['granted'], { version: 2 });
  await assert.rejects(connectRelay(r.args()), (e) => {
    assert.equal(e.code, 'relay-version');
    assert.equal(e.message, 'CPE relay at 10.83.11.1 runs protocol v2; this page needs v3 — update the CPE firmware.');
    return true;
  });
});

test('denied: refuse without probing', async () => {
  const r = rig(['denied'], 'relay');
  await assert.rejects(connectRelay(r.args()), { message: LNA_DENIED, code: 'lna-denied' });
  assert.equal(r.probes.length, 0);
});

test('prompt: the probe itself raises it and gets the long timeout', async () => {
  // Revert check: a 1.5 s probe would time out under the pending prompt and
  // fail the Connect while Chrome is still asking (Chrome 147 headed).
  const r = rig(['prompt'], 'relay');
  await connectRelay(r.args());
  assert.deepEqual(r.probes, [['ws://10.83.11.1:8311', 60000]]);
});

test('prompt then denied / unanswered', async () => {
  let r = rig(['prompt', 'denied'], 'none');
  await assert.rejects(connectRelay(r.args()), { message: LNA_DENIED });
  r = rig(['prompt', 'prompt'], 'none');
  await assert.rejects(connectRelay(r.args()), { message: LNA_UNANSWERED, code: 'lna-unanswered' });
});

test('prompt answered Allow but no relay: not-found, not the prompt text', async () => {
  // Review Focus 3.
  const r = rig(['prompt', 'granted'], 'none');
  await assert.rejects(connectRelay(r.args()), { code: 'relay-not-found' });
});
