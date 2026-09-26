// THROWAWAY SPIKE: drive.mjs + a Chrome trace (GC + main-thread tasks) of the
// run, with an epoch<->trace-clock sync marker. usage: node trace.mjs <ws> secs query out.json
import fs from 'node:fs';
const [,, url, secs = '30', query = '', out = 'trace.json'] = process.argv;
const ws = new WebSocket(url);
let armed = false, id = 0; const pend = new Map(); const events = [];
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({id: i, method, params})); });
let done;
const finished = new Promise(r => done = r);
ws.onmessage = (m) => {
  const msg = JSON.parse(m.data);
  if (msg.id && pend.has(msg.id)) { pend.get(msg.id)(msg); pend.delete(msg.id); return; }
  if (msg.method === 'Tracing.dataCollected') events.push(...msg.params.value);
  else if (msg.method === 'Tracing.tracingComplete') done();
  else if (msg.method === 'Runtime.consoleAPICalled' && armed) {
    const t = msg.params.args.map(a => a.value ?? a.description).join(' ');
    if (/^(ERR|running|TIMING|SUMMARY|SPIKE|picked|ERROR)/.test(t)) console.log(t);
    if (t === 'DONE' || t.startsWith('ERROR')) finish();
  }
};
async function finish() {
  await send('Runtime.evaluate', {expression: "console.timeStamp('SYNC ' + Date.now())"});
  await send('Tracing.end'); await finished;
  fs.writeFileSync(out, JSON.stringify({traceEvents: events}));
  console.log('trace events', events.length); process.exit(0);
}
ws.onopen = async () => {
  await send('Runtime.enable'); await send('Page.enable');
  await send('Page.navigate', {url: `http://127.0.0.1:8808/index.html?${query}&s=${secs}`});
  await new Promise(r => setTimeout(r, 1500));
  await send('Tracing.start', {traceConfig: {recordMode: 'recordContinuously', includedCategories:
    ['v8.gc', 'disabled-by-default-v8.gc', 'toplevel', 'devtools.timeline', '__metadata']}});
  await send('Runtime.evaluate', {expression: "console.timeStamp('SYNC ' + Date.now())"});
  armed = true;
  await send('Runtime.evaluate', {expression: "document.getElementById('go').click()", userGesture: true});
};
setTimeout(() => { console.log('TIMEOUT'); process.exit(1); }, (Number(secs) + 90) * 1000);
