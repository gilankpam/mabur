// Vite plugin: emits web/dist/sw.js from web/ui/sw/sw.js with this build's
// precache manifest injected. The list comes from Vite's bundle + public/ +
// the Emscripten outputs -- never from listing web/dist, which keeps stale
// hashed bundles (emptyOutDir: false).
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';

const TOKEN = '/*__MABUR_PRECACHE__*/ null';

export function precacheList(bundleNames, publicNames, extraNames) {
  const keep = (f) => f !== 'sw.js' && f !== '.gitkeep' && !f.endsWith('.map');
  return [...new Set([...bundleNames, ...publicNames, ...extraNames].filter(keep))].sort();
}

export function buildId(names, contents) {
  const h = crypto.createHash('sha256');
  for (const n of names) h.update(n + '\n');
  for (const c of contents) h.update(c);
  return h.digest('hex').slice(0, 12);
}

export function injectManifest(src, manifest) {
  if (!src.includes(TOKEN)) throw new Error('sw source lacks the /*__MABUR_PRECACHE__*/ null token');
  return src.replace(TOKEN, JSON.stringify(manifest));
}

export function shouldRegister({ isolated, controllerUrl, ownUrl }) {
  if (!isolated) return true;
  return !!controllerUrl && controllerUrl.split('?')[0] !== ownUrl.split('?')[0];
}

export function maburSw({ srcFile, publicDir, distDir }) {
  return {
    name: 'mabur-sw',
    enforce: 'post',   // run generateBundle after vite:build-html so index.html is already in the bundle
    configureServer(server) {   // vite dev: serve the raw worker (headers come from the dev server)
      server.middlewares.use((req, res, next) => {
        if (!req.url || !/^\/sw\.js(\?.*)?$/.test(req.url)) return next();
        res.setHeader('Content-Type', 'text/javascript');
        res.end(injectManifest(fs.readFileSync(srcFile, 'utf8'), { id: 'dev', files: [] }));
      });
    },
    generateBundle(_opts, bundle) {
      const extra = ['webgs.js', 'webgs.wasm'].filter((f) => fs.existsSync(path.join(distDir, f)));
      const pub = fs.existsSync(publicDir) ? fs.readdirSync(publicDir) : [];
      const files = precacheList(Object.keys(bundle), pub, extra);
      const id = buildId(files, extra.map((f) => fs.readFileSync(path.join(distDir, f))));
      this.emitFile({ type: 'asset', fileName: 'sw.js',
        source: injectManifest(fs.readFileSync(srcFile, 'utf8'), { id, files }) });
    },
  };
}
