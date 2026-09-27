// THROWAWAY SPIKE: hosts the Emscripten runtime (and so every WebUSB call
// libusb proxies to its main runtime thread) in a dedicated worker; posts
// each AU to the page as a transferable ArrayBuffer.
self.onmessage = ({ data }) => {
  if (!self.navigator.usb) { postMessage({ t: 'log', line: 'ERROR no navigator.usb in worker' }); return; }
  importScripts('build-wasm/gsweb.js');
  createGsweb({
    arguments: data.args,
    locateFile: (p) => new URL('build-wasm/' + p, self.location).href,
    mainScriptUrlOrBlob: new URL('build-wasm/gsweb.js', self.location).href,
    print: (line) => postMessage({ t: 'log', line }),
    printErr: (line) => { if (!line.startsWith('{"ev"')) postMessage({ t: 'log', line: 'ERR ' + line }); },
    onAu: (buf, pts, sid, flags, complete, tc, hvcc) =>
      postMessage({ t: 'au', buf, pts, sid, flags, complete: !!complete, tc, hvcc },
                  hvcc ? [buf, hvcc] : [buf]),
  });
};
