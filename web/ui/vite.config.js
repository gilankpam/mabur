// mabur web GS page build. Output goes into web/dist NEXT TO the Emscripten
// module (webgs.js/.wasm), so emptyOutDir stays false. In `vite dev` the
// module is served from web/dist by the middleware below, with the COOP/COEP
// headers WASM pthreads need.
import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import fs from 'node:fs';
import path from 'node:path';

const DIST = path.resolve(import.meta.dirname, '../dist');
const COI = {
  'Cross-Origin-Opener-Policy': 'same-origin',
  'Cross-Origin-Embedder-Policy': 'require-corp',
};

function webgsFromDist() {
  return {
    name: 'webgs-from-dist',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const m = req.url && req.url.match(/^\/(webgs\.(?:js|wasm))(?:\?.*)?$/);
        if (!m) return next();
        const f = path.join(DIST, m[1]);
        if (!fs.existsSync(f)) { res.statusCode = 404; res.end('build the WASM module first'); return; }
        res.setHeader('Content-Type', m[1].endsWith('.wasm') ? 'application/wasm' : 'text/javascript');
        for (const [k, v] of Object.entries(COI)) res.setHeader(k, v);
        fs.createReadStream(f).pipe(res);
      });
    },
  };
}

export default defineConfig({
  base: './',
  plugins: [svelte(), webgsFromDist()],
  server: { headers: COI, host: true },
  preview: { headers: COI },
  build: { outDir: DIST, emptyOutDir: false, assetsDir: 'assets', target: 'es2022' },
});
