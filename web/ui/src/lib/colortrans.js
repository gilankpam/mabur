// The drone's ColorTrans sensor tuning, reversed on the web GS video
// (docs/colortrans.md). maburplay does it with the VOP2 CRTC LUT; here it is
// a WebGL pass over each decoded VideoFrame. The OSD canvas sits above the
// video canvas, so unlike maburplay nothing needs pre-inverting.
//
// CT_PARAMS is kColorTrans3 (gs/player/src/colortrans.h), and the shader is
// frame_colortrans.cpp's kFrag. web/tests/colortrans.test.mjs pins both to
// the header and to the glsl reference values -- a retune edits all three.
export const CT_PARAMS = Object.freeze({
  yOffset10b: 200.0,
  yOffsetStrength: 0.25,
  blackLift: 0.020,
  matrix: Object.freeze([1.17866031, 0.17460893, 0.01571472,
    -0.11147506, 1.55408099, -0.07362197,
    0.03243649, 0.11275820, 1.22378927]),   // row-major
  gamma: 1.0,
  lift: -0.15,
  gain: 1.75,
  rgbMult: Object.freeze([1.0, 1.0, 1.0]),
  saturation: -22.5,   // mpv units
});

const P = CT_PARAMS;
const YOFF = (P.yOffset10b / 1023) * P.yOffsetStrength;
const SAT = Math.min(3, Math.max(0, 1 + P.saturation / 100));
const clamp01 = (v) => (v < 0 ? 0 : v > 1 ? 1 : v);

// The shader's math, step for step, for the tests. [r,g,b] in/out, [0,1].
export function ctForward([r, g, b]) {
  const M = P.matrix;
  const x = [r - YOFF, g - YOFF, b - YOFF];
  let c = [0, 1, 2].map((i) => clamp01(M[3 * i] * x[0] + M[3 * i + 1] * x[1] + M[3 * i + 2] * x[2] + P.blackLift));
  c = c.map((v, i) => (Math.pow(v, P.gamma) + P.lift) * P.gain * P.rgbMult[i]);
  const l = 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
  return c.map((v) => clamp01(l + (v - l) * SAT));
}

const f = (v) => v.toFixed(6);
const M = P.matrix;
export const CT_FRAG = `precision highp float;
varying vec2 v_uv;
uniform sampler2D tex;
const float kYoff = ${f(YOFF)};
const float kBlackLift = ${f(P.blackLift)};
const mat3 kM = mat3(${[M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]].join(', ')});
const float kGamma = ${f(P.gamma)};
const float kLift = ${f(P.lift)};
const float kGain = ${f(P.gain)};
const vec3 kMult = vec3(${P.rgbMult.map(f).join(', ')});
const float kSat = ${f(SAT)};
void main() {
  vec3 c = texture2D(tex, v_uv).rgb;
  vec3 y = kM * (c - vec3(kYoff)) + vec3(kBlackLift);
  c = clamp(y, 0.0, 1.0);
  y = pow(c, vec3(kGamma));
  y += vec3(kLift);
  y *= kGain;
  y *= kMult;
  float l = dot(y, vec3(0.2126, 0.7152, 0.0722));
  y = mix(vec3(l), y, kSat);
  gl_FragColor = vec4(clamp(y, 0.0, 1.0), 1.0);
}
`;

// Full-screen quad; the texture's first row (the frame's top) is v = 0.
const VERT = `attribute vec2 a_pos;
varying vec2 v_uv;
void main() {
  v_uv = vec2(a_pos.x * 0.5 + 0.5, 0.5 - a_pos.y * 0.5);
  gl_Position = vec4(a_pos, 0.0, 1.0);
}
`;

// Draws VideoFrames through CT_FRAG on its own canvas (a canvas keeps the
// context type it was first asked for, so the flat 2D path uses another
// one). Throws when WebGL or the program is unavailable; `ok` goes false
// on context loss, after which the caller falls back to the 2D path.
export class ColorTransGl {
  constructor(canvas) {
    const gl = canvas.getContext('webgl', { alpha: false, antialias: false, depth: false, stencil: false,
      premultipliedAlpha: false, preserveDrawingBuffer: false });
    if (!gl) throw new Error('WebGL unavailable');
    this.canvas = canvas;
    this.gl = gl;
    this.lost = false;
    canvas.addEventListener('webglcontextlost', (e) => { e.preventDefault(); this.lost = true; });
    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(`shader: ${gl.getShaderInfoLog(s)}`);
      return s;
    };
    const prog = gl.createProgram();
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, VERT));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, CT_FRAG));
    gl.bindAttribLocation(prog, 0, 'a_pos');
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(`link: ${gl.getProgramInfoLog(prog)}`);
    gl.useProgram(prog);
    gl.bindBuffer(gl.ARRAY_BUFFER, gl.createBuffer());
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    gl.bindTexture(gl.TEXTURE_2D, gl.createTexture());
    // NPOT frame: no mips, clamp (WebGL1's NPOT rules).
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.uniform1i(gl.getUniformLocation(prog, 'tex'), 0);
  }

  get ok() { return !this.lost && !this.gl.isContextLost(); }

  draw(frame) {
    const { gl, canvas } = this;
    if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
      canvas.width = frame.displayWidth;
      canvas.height = frame.displayHeight;
    }
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, frame);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  }
}
