// THROWAWAY SPIKE: drive viewer.html over CDP -- navigate, click Connect
// (user gesture), record console lines for N s.
// usage: node view_drive.mjs <page-ws-url> <secs> <out.log> [query]
import fs from 'node:fs';
const [,, url, secs = '180', out = 'view.log', query = 'ch=136&w=40'] = process.argv;
const ws = new WebSocket(url);
let armed = false, id = 0; const pend = new Map(); const lines = [];
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({id: i, method, params})); });
const finish = (why) => { fs.writeFileSync(out, lines.join('\n') + '\n'); console.log(`${why}: ${lines.length} lines -> ${out}`); process.exit(0); };
ws.onmessage = (m) => {
  const msg = JSON.parse(m.data);
  if (msg.id && pend.has(msg.id)) { pend.get(msg.id)(msg); pend.delete(msg.id); return; }
  if (msg.method === 'Runtime.consoleAPICalled' && armed) {
    const t = msg.params.args.map(a => a.value ?? a.description).join(' ');
    if (t.startsWith('{"ev"')) return;
    lines.push(`${Date.now()} ${t}`);
    if (/^(ERROR|TIMING|picked)/.test(t)) console.log(t);
    if (/^ERROR (claim|no RTL|requestDevice|not crossOrigin)/.test(t)) setTimeout(() => finish('error'), 500);
  } else if (msg.method === 'Runtime.exceptionThrown' && armed) {
    const t = 'EXC ' + JSON.stringify(msg.params.exceptionDetails).slice(0, 400);
    lines.push(t); console.log(t);
  }
};
ws.onopen = async () => {
  await send('Runtime.enable'); await send('Page.enable');
  await send('Page.navigate', {url: `http://127.0.0.1:8808/viewer.html?${query}`});
  await new Promise(r => setTimeout(r, 2000));
  armed = true;
  await send('Runtime.evaluate', {expression: "document.getElementById('go').click()", userGesture: true});
  setTimeout(() => finish('done'), Number(secs) * 1000);
};
