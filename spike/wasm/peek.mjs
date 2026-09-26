const ws = new WebSocket(process.argv[2]);
ws.onopen = () => ws.send(JSON.stringify({id: 1, method: 'Runtime.evaluate', params: {expression: "document.getElementById('log').textContent"}}));
ws.onmessage = (m) => { const r = JSON.parse(m.data); if (r.id === 1) { console.log(r.result.result.value); process.exit(0); } };
