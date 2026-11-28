#!/usr/bin/env python3
"""Reference for drone/src/pa_fit: the PA gain curve fitted to the two least-flagged frames of
one probe (one or more captures of the same TXAGC index), and the EVM it predicts at lower indices.
usage: pafit_ref.py BW FILE.hi32 [FILE.hi32 ...]   (numpy; BW is 20 or 40)
Prints: frames own_evm_db residual_db level_db, then the prediction at 0, 4, 8, ... 40 steps below."""
import sys
import numpy as np

BW = int(sys.argv[1]) if len(sys.argv) > 1 else 20
S = 2 if BW == 40 else 1
N, SYM, LTF, NSYM, OS = 64 * S, 80 * S, 656 * S, 20, 4
LL = [1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1, 1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1]
if S == 1:
    USED = np.array([k for k in range(-28, 29) if k != 0]); PIL = np.isin(np.abs(USED), (7, 21))
    HTLTF = np.array([1, 1] + LL + [-1, -1])
else:
    USED = np.array([k for k in range(-58, 59) if abs(k) > 1]); PIL = np.isin(np.abs(USED), (11, 25, 53))
    HTLTF = np.array(LL[:26] + [1] + LL[26:] + [-1, -1, -1, 1] + [-1, 1, 1, -1] + LL[:26] + [1] + LL[26:])
GAMMA = np.where((USED > 0) & (S == 2), 1j, 1)     # the upper half of an HT40 frame is sent rotated by j
BIN, BINOS = USED % N, USED % (N * OS)
KNOTS = np.arange(-25.0, 9.1, 2.0)                 # dB re the frame's rms; flat outside
LAMBDA, ITERS, FLOOR_DB, MIN_LEVEL_DB = 0.05, 10, -27.0, 30.0


def load(f):
    w = np.fromfile(f, '<u4')
    i = ((w >> 12) & 0xfff).astype(np.int32); i[i >= 2048] -= 4096
    q = (w & 0xfff).astype(np.int32); q[q >= 2048] -= 4096
    return (i - np.median(i)) + 1j * (q - np.median(q)), ((w >> 29) & 1) == 0


def frames(x):
    c = np.concatenate(([0], np.cumsum(np.abs(x) ** 2)))
    k = np.arange(len(x)); p = (c[np.minimum(k + 8, len(x))] - c[np.maximum(k - 8, 0)]) / 16
    up = p > 0.1 * np.percentile(p, 90)
    out = []
    for s in np.flatnonzero(up[1:] & ~up[:-1]) + 1:
        e = s + 1
        while e < len(x) and up[e]: e += 1
        if e - s >= 1900 * S and s + LTF + 8 * S + SYM * (NSYM + 1) <= len(x): out.append(int(s))
    return out


def slice_(z):
    d = (np.sign(z.real) + 1j * np.sign(z.imag)) / np.sqrt(2)
    d[:, PIL] = np.sign(z[:, PIL].real)
    return d


def demod(x, s):
    best = None
    for b in range(s + LTF - 12 * S, s + LTF + 8 * S + 1):
        if b + SYM * (NSYM + 1) > len(x): break
        w = np.fft.fft(np.stack([x[b + SYM * m: b + SYM * m + N] for m in range(NSYM + 1)]), axis=1)[:, BIN]
        R = w[1:]; D = slice_(R / (w[0] * HTLTF))
        for _ in range(2):
            h = (R * D.conj()).sum(0) / (np.abs(D) ** 2).sum(0); D = slice_(R / h)
        h = (R * D.conj()).sum(0) / (np.abs(D) ** 2).sum(0)
        e = 10 * np.log10((np.abs(R / h - D) ** 2)[:, ~PIL].sum() / (np.abs(D) ** 2)[:, ~PIL].sum())
        if best is None or e < best[0]: best = (e, R, D)
    return best


def wave(D):
    X = np.zeros((D.shape[0], N * OS), complex); X[:, BINOS] = D * GAMMA
    v = np.fft.ifft(X, axis=1)
    return v / np.sqrt((np.abs(v) ** 2).mean())


def hats(a):
    out = []
    for j, kj in enumerate(KNOTS):
        h = np.zeros_like(a)
        if j: m = (a >= KNOTS[j - 1]) & (a < kj); h[m] = (a[m] - KNOTS[j - 1]) / (kj - KNOTS[j - 1])
        else: h[a < kj] = 1
        if j + 1 < len(KNOTS): m = (a >= kj) & (a < KNOTS[j + 1]); h[m] = (KNOTS[j + 1] - a[m]) / (KNOTS[j + 1] - kj)
        else: h[a >= kj] = 1
        out.append(h)
    return np.array(out)


def env_db(v):
    return 20 * np.log10(np.maximum(np.abs(v), 1e-6))


def gfit(RD):
    J = len(KNOTS); Bs = []
    for R, D in RD:
        v = wave(D); Bs.append(np.fft.fft(hats(env_db(v)) * v, axis=2)[:, :, BINOS] / OS)
    th = np.ones(J, complex)
    for _ in range(ITERS):
        A = []; y = []; Hs = []
        for (R, D), B in zip(RD, Bs):
            M = np.tensordot(th, B, 1); H = (R * M.conj()).sum(0) / (np.abs(M) ** 2).sum(0); Hs.append(H)
            w = 1 / np.sqrt((np.abs(R) ** 2).mean()); A.append((B * H).reshape(J, -1).T * w); y.append(R.ravel() * w)
        A = np.vstack(A); y = np.concatenate(y)
        L = np.diff(np.eye(J), 2, axis=0) * LAMBDA * np.sqrt((np.abs(A) ** 2).sum() / J)
        th = np.linalg.lstsq(np.vstack([A, L]), np.concatenate([y, np.zeros(len(L))]), rcond=None)[0]; th /= th[0]
    num = sum((np.abs(R - np.tensordot(th, B, 1) * H) ** 2).sum() for (R, D), B, H in zip(RD, Bs, Hs))
    return th, 10 * np.log10(num / sum((np.abs(R) ** 2).sum() for R, D in RD))


_rng = np.random.default_rng(1)
_D = (_rng.choice([-1, 1], (200, len(USED))) + 1j * _rng.choice([-1, 1], (200, len(USED)))) / np.sqrt(2)
_D[:, PIL] = _rng.choice([-1, 1], (200, int(PIL.sum())))
_V = wave(_D)


def pred_raw(th, steps):
    y = _V * np.tensordot(th, hats(env_db(_V) - 0.25 * steps), 1)
    Y = np.fft.fft(y, axis=1)[:, BINOS]
    h = (Y * _D.conj()).sum(0) / (np.abs(_D) ** 2).sum(0)
    return 10 * np.log10((np.abs(Y / h - _D) ** 2)[:, ~PIL].sum() / (np.abs(_D) ** 2)[:, ~PIL].sum())


def pa_fit(files, burst=None):
    cand = []
    for ci, f in enumerate(files):
        x, bad = load(f)
        if burst is not None: x = x + burst[:len(x)]
        clip = np.abs(x.real).max() >= 2040 or np.abs(x.imag).max() >= 2040
        for s in frames(x):
            lvl = 10 * np.log10((np.abs(x[s + 700 * S: s + 1800 * S]) ** 2).mean())
            nb = int(bad[s + 640 * S: s + (656 + 80 * (NSYM + 1)) * S].sum())
            if not clip and lvl >= MIN_LEVEL_DB: cand.append((nb, ci, s, x, lvl))
    cand.sort(key=lambda c: c[:3]); use = cand[:2]
    if len(use) < 2: return dict(frames=len(use))
    dem = [demod(c[3], c[2]) for c in use]
    th, res = gfit([(d[1], d[2]) for d in dem])
    own = float(np.mean([d[0] for d in dem])); off = own - pred_raw(th, 0)
    pred = [10 * np.log10(10 ** ((pred_raw(th, k) + off) / 10) + 10 ** (FLOOR_DB / 10)) for k in range(0, 41, 4)]
    return dict(frames=2, own=own, res=res, level=float(np.mean([c[4] for c in use])), pred=pred)


if __name__ == '__main__':
    r = pa_fit(sys.argv[2:])
    if r['frames'] < 2: print('frames', r['frames']); sys.exit()
    print('frames 2 own %.2f residual %.2f level %.2f' % (r['own'], r['res'], r['level']))
    print('pred ' + ' '.join('%.2f' % p for p in r['pred']))
