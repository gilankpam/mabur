import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { CT_PARAMS, ctForward, CT_FRAG } from '../ui/src/lib/colortrans.js';

const near3 = (got, want, eps = 1e-4) =>
  got.forEach((v, i) => assert.ok(Math.abs(v - want[i]) <= eps, `${got} vs ${want}`));

// tests/test_colortrans.cpp forward_matches_the_glsl_reference: the same
// colortrans3.glsl reference values maburplay's evaluator is pinned to.
test('forward matches the glsl reference', () => {
  near3(ctForward([0, 0, 0]), [0, 0, 0]);
  near3(ctForward([1, 1, 1]), [1, 1, 1]);
  near3(ctForward([0.5, 0.5, 0.5]), [0.853268, 0.853268, 0.853268]);
  near3(ctForward([0.8, 0.2, 0.2]), [1, 0.084757, 0.201865]);
  near3(ctForward([0.3, 0.6, 0.9]), [0.60007, 1, 1]);
});

// Drift guard: a retune of kColorTrans3 (gs/player/src/colortrans.h) must
// reach this page too (docs/colortrans.md).
test('params match colortrans.h kColorTrans3', () => {
  const h = fs.readFileSync(new URL('../../gs/player/src/colortrans.h', import.meta.url), 'utf8');
  const f = (name) => Number(h.match(new RegExp(`float ${name} = (-?[\\d.]+)f;`))[1]);
  assert.equal(CT_PARAMS.yOffset10b, f('y_offset_10b'));
  assert.equal(CT_PARAMS.yOffsetStrength, f('y_offset_strength'));
  assert.equal(CT_PARAMS.blackLift, f('black_lift'));
  assert.equal(CT_PARAMS.gamma, f('gamma'));
  assert.equal(CT_PARAMS.lift, f('lift'));
  assert.equal(CT_PARAMS.gain, f('gain'));
  assert.equal(CT_PARAMS.saturation, f('saturation'));
  const m = h.match(/float matrix\[9\] = \{([^}]*)\}/)[1].split(',').map((t) => Number(t.trim().replace(/f$/, '')));
  assert.deepEqual(CT_PARAMS.matrix, m);
  assert.deepEqual(CT_PARAMS.rgbMult,
    h.match(/float rgb_mult\[3\] = \{([^}]*)\}/)[1].split(',').map((t) => Number(t.trim().replace(/f$/, ''))));
});

test('shader carries the params (column-major matrix)', () => {
  const M = CT_PARAMS.matrix;
  assert.ok(CT_FRAG.includes(`mat3(${[M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]].join(', ')})`));
  assert.ok(CT_FRAG.includes(`const float kGain = ${CT_PARAMS.gain.toFixed(6)};`));
});
