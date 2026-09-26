// THROWAWAY SPIKE: host the whole Emscripten runtime (and so every WebUSB
// call libusb proxies to its "main runtime thread") in a dedicated worker,
// off the page's renderer main thread and its Oilpan sweeps.
self.onmessage = ({data}) => {
  if (!self.navigator.usb) { postMessage('ERROR no navigator.usb in worker'); return; }
  importScripts('build-wasm/rxprobe.js');
  createRxProbe({
    arguments: data.args,
    locateFile: (p) => new URL('build-wasm/' + p, self.location).href,
    mainScriptUrlOrBlob: new URL('build-wasm/rxprobe.js', self.location).href,
    print: (t) => postMessage(t),
    printErr: (t) => { if (!t.startsWith('{"ev"')) postMessage('ERR ' + t); },
  });
};
