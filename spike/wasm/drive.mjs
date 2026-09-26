// THROWAWAY SPIKE: drives Chrome over CDP -- clicks the button with a user
// gesture, answers the WebUSB chooser (DeviceAccess domain), streams console.
// usage: node drive.mjs <ws-debugger-url-of-page> [secs]
const ws = new WebSocket(process.argv[2]);
let armed = false; let id = 0; const pend = new Map();
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({id: i, method, params})); });
ws.onmessage = async (m) => {
  const msg = JSON.parse(m.data);
  if (msg.id && pend.has(msg.id)) { pend.get(msg.id)(msg); pend.delete(msg.id); return; }
  if (msg.method === 'DeviceAccess.deviceRequestPrompted') {
    const p = msg.params; console.log('chooser:', JSON.stringify(p.devices));
    if (p.devices.length) await send('DeviceAccess.selectPrompt', {id: p.id, deviceId: p.devices[0].id});
  } else if (msg.method === 'Runtime.consoleAPICalled' && armed) {
    const t = msg.params.args.map(a => a.value ?? a.description).join(' ');
    console.log(t);
    if (t === 'DONE' || t.startsWith('ERROR')) setTimeout(() => process.exit(0), 200);
  } else if (msg.method === 'Runtime.exceptionThrown') {
    console.log('EXC', JSON.stringify(msg.params.exceptionDetails).slice(0, 600));
  }
};
ws.onopen = async () => {
  await send('Runtime.enable'); await send('DeviceAccess.enable');
  await send('Page.enable'); await send('Page.navigate', {url: 'http://127.0.0.1:8808/index.html?s=' + (process.argv[3] || 30)});
  await new Promise(r => setTimeout(r, 1500));
  armed = true;
  const r = await send('Runtime.evaluate', {expression: "document.getElementById('go').click()", userGesture: true});
  console.log('eval', JSON.stringify(r).slice(0, 300));
  const c = await send('Runtime.evaluate', {expression: 'crossOriginIsolated + " " + !!navigator.usb + " " + typeof createRxProbe'});
  console.log('state', JSON.stringify(c.result));
};
setTimeout(() => { console.log('TIMEOUT'); process.exit(1); }, (Number(process.argv[3] || 30) + 90) * 1000);
