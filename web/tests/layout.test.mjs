import test from 'node:test';
import assert from 'node:assert/strict';
import { isMobile, layoutMode, clampFloatPos, dragStarted, keyAction } from '../ui/src/lib/layout.js';

test('mobile detection matches the handoff', () => {
  assert.equal(isMobile(759, 900), true);
  assert.equal(isMobile(760, 900), false);
  assert.equal(isMobile(900, 450), true);    // short landscape
  assert.equal(isMobile(1000, 450), false);
});

test('layout modes', () => {
  assert.equal(layoutMode({ w: 1440, h: 900, fs: false }), 'windowed');
  assert.equal(layoutMode({ w: 1440, h: 900, fs: true }), 'immersive');
  assert.equal(layoutMode({ w: 844, h: 390, fs: false }), 'immersive');
  assert.equal(layoutMode({ w: 390, h: 844, fs: false }), 'portrait');
});

test('clamp_float_pos_after_shrink', () => {
  assert.deepEqual(clampFloatPos({ x: 1200, y: 800 }, { cw: 844, ch: 390 }, 236), { x: 600, y: 342 });
  assert.deepEqual(clampFloatPos({ x: -50, y: -5 }, { cw: 844, ch: 390 }, 236), { x: 8, y: 8 });
});

test('drag threshold 5 px', () => {
  assert.equal(dragStarted(0, 0, 3, 3), false);
  assert.equal(dragStarted(0, 0, 3, 4), true);
});

test('keys: F R S Esc, ignored in inputs and with modifiers', () => {
  const k = (key, tagName = 'BODY', extra = {}) => keyAction({ key, target: { tagName }, ...extra });
  assert.equal(k('f'), 'fs'); assert.equal(k('F'), 'fs');
  assert.equal(k('r'), 'rec'); assert.equal(k('s'), 'stats'); assert.equal(k('Escape'), 'esc');
  assert.equal(k('f', 'INPUT'), null); assert.equal(k('s', 'SELECT'), null);
  assert.equal(k('r', 'BODY', { ctrlKey: true }), null);
  assert.equal(k('x'), null);
});
