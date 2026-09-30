import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { precacheList } from '../ui/sw/plugin.mjs';

// docs/web-gs.md "Install as an app": the manifest and its icons live in
// web/ui/public/, so the sw precache picks them up with no worker change.
const UI = path.resolve(import.meta.dirname, '../ui');
const PUB = path.join(UI, 'public');
const manifest = JSON.parse(fs.readFileSync(path.join(PUB, 'manifest.webmanifest'), 'utf8'));

function pngSize(f) {
  const b = fs.readFileSync(path.join(PUB, f));
  assert.equal(b.toString('latin1', 1, 4), 'PNG', `${f} is not a PNG`);
  return `${b.readUInt32BE(16)}x${b.readUInt32BE(20)}`;
}

test('manifest: fullscreen landscape viewer, relative start_url/scope (works under /mabur/ and serve.py)', () => {
  assert.equal(manifest.display, 'fullscreen');
  assert.equal(manifest.orientation, 'landscape');
  assert.equal(manifest.start_url, './');
  assert.equal(manifest.scope, './');
  assert.ok(manifest.name && manifest.short_name);
  assert.match(manifest.background_color, /^#[0-9a-f]{6}$/i);
  assert.match(manifest.theme_color, /^#[0-9a-f]{6}$/i);
});

test('manifest icons exist with their declared sizes; 192 + 512 any and a 512 maskable', () => {
  for (const ic of manifest.icons) {
    assert.ok(!ic.src.startsWith('/'), `${ic.src} must be relative`);
    assert.equal(pngSize(ic.src), ic.sizes, ic.src);
  }
  const has = (sizes, purpose) => manifest.icons.some((i) => i.sizes === sizes && (i.purpose || 'any') === purpose);
  assert.ok(has('192x192', 'any'));
  assert.ok(has('512x512', 'any'));
  assert.ok(has('512x512', 'maskable'));
});

test('index.html links the manifest, theme-color and the icons it names; all are precached', () => {
  const html = fs.readFileSync(path.join(UI, 'index.html'), 'utf8');
  assert.match(html, /<link rel="manifest" href="manifest\.webmanifest">/);
  assert.match(html, new RegExp(`<meta name="theme-color" content="${manifest.theme_color}">`));
  const hrefs = [...html.matchAll(/<link rel="(?:icon|apple-touch-icon)"[^>]*href="([^"]+)"/g)].map((m) => m[1]);
  assert.ok(hrefs.length >= 2);
  const pub = fs.readdirSync(PUB);
  const pre = precacheList([], pub, []);
  for (const f of ['manifest.webmanifest', ...hrefs, ...manifest.icons.map((i) => i.src)]) {
    assert.ok(pub.includes(f), `${f} missing from public/`);
    assert.ok(pre.includes(f), `${f} not precached`);
  }
});
