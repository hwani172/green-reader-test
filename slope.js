/*
 * 그린 읽기 — 계산 부분 (화면과 따로, 컴퓨터에서 node로 시험할 수 있게)
 *
 * 좌표: WebXR 'local' 공간. y = 위(중력 반대), x = 오른쪽, -z = 처음 바라본 앞쪽. 단위 m.
 * 흐름: 카메라 깊이 점 → 세계 좌표(unproject) → 10cm 칸에 평균(Grid) → 칸들로 기울어진 면 맞추기(fitPlane)
 *       → 공·홀 사이 거리, 오르막/내리막, 옆 경사(analyze).
 */
(function (root) {
  'use strict';

  // ── 4×4 행렬 (WebXR처럼 열 우선 Float32Array/배열) ──
  function mat4Invert(m) {
    const a00 = m[0], a01 = m[1], a02 = m[2], a03 = m[3];
    const a10 = m[4], a11 = m[5], a12 = m[6], a13 = m[7];
    const a20 = m[8], a21 = m[9], a22 = m[10], a23 = m[11];
    const a30 = m[12], a31 = m[13], a32 = m[14], a33 = m[15];
    const b00 = a00 * a11 - a01 * a10, b01 = a00 * a12 - a02 * a10;
    const b02 = a00 * a13 - a03 * a10, b03 = a01 * a12 - a02 * a11;
    const b04 = a01 * a13 - a03 * a11, b05 = a02 * a13 - a03 * a12;
    const b06 = a20 * a31 - a21 * a30, b07 = a20 * a32 - a22 * a30;
    const b08 = a20 * a33 - a23 * a30, b09 = a21 * a32 - a22 * a31;
    const b10 = a21 * a33 - a23 * a31, b11 = a22 * a33 - a23 * a32;
    let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (!det) return null;
    det = 1 / det;
    return [
      (a11 * b11 - a12 * b10 + a13 * b09) * det, (a02 * b10 - a01 * b11 - a03 * b09) * det,
      (a31 * b05 - a32 * b04 + a33 * b03) * det, (a22 * b04 - a21 * b05 - a23 * b03) * det,
      (a12 * b08 - a10 * b11 - a13 * b07) * det, (a00 * b11 - a02 * b08 + a03 * b07) * det,
      (a32 * b02 - a30 * b05 - a33 * b01) * det, (a20 * b05 - a22 * b02 + a23 * b01) * det,
      (a10 * b10 - a11 * b08 + a13 * b06) * det, (a01 * b08 - a00 * b10 - a03 * b06) * det,
      (a30 * b04 - a31 * b02 + a33 * b00) * det, (a21 * b02 - a20 * b04 - a23 * b00) * det,
      (a11 * b07 - a10 * b09 - a12 * b06) * det, (a00 * b09 - a01 * b07 + a02 * b06) * det,
      (a31 * b01 - a30 * b03 - a32 * b00) * det, (a20 * b03 - a21 * b01 + a22 * b00) * det,
    ];
  }
  function mat4MulVec(m, x, y, z, w) {
    return [
      m[0] * x + m[4] * y + m[8] * z + m[12] * w,
      m[1] * x + m[5] * y + m[9] * z + m[13] * w,
      m[2] * x + m[6] * y + m[10] * z + m[14] * w,
      m[3] * x + m[7] * y + m[11] * z + m[15] * w,
    ];
  }

  // 화면 위치(nx, ny: 0~1, 왼쪽 위가 0) + 깊이(카메라 면까지 거리, m) → 세계 좌표.
  // invProj = 투영 행렬의 역행렬, camMatrix = 카메라 위치·방향(view.transform.matrix).
  function unproject(nx, ny, depth, invProj, camMatrix) {
    const v = mat4MulVec(invProj, 2 * nx - 1, 1 - 2 * ny, -1, 1);
    const vx = v[0] / v[3], vy = v[1] / v[3], vz = v[2] / v[3]; // 가까운 면 위의 점 (vz < 0)
    const s = depth / -vz;                                       // 깊이는 '카메라 면까지' 거리라 z로 맞춘다
    const w = mat4MulVec(camMatrix, vx * s, vy * s, -depth, 1);
    return [w[0], w[1], w[2]];
  }

  // 세계 좌표 → 화면(0~1)과 카메라 면까지 거리. 카메라 뒤면 null.
  function project(x, y, z, proj, viewMatrix) {
    const c = mat4MulVec(viewMatrix, x, y, z, 1);
    if (c[2] > -0.05) return null;
    const p = mat4MulVec(proj, c[0], c[1], c[2], 1);
    return [(p[0] / p[3] + 1) / 2, (1 - p[1] / p[3]) / 2, -c[2]];
  }

  // ── 10cm 칸에 높이를 모은다 ──
  function Grid(size) {
    this.size = size || 0.1;
    this.map = new Map();
  }
  Grid.prototype.add = function (x, y, z) {
    const s = this.size;
    const key = Math.round(x / s) + ',' + Math.round(z / s);
    let c = this.map.get(key);
    if (!c) { c = { sx: 0, sy: 0, sz: 0, syy: 0, n: 0 }; this.map.set(key, c); }
    c.sx += x; c.sy += y; c.sz += z; c.syy += y * y; c.n++;
  };
  Grid.prototype.clear = function () { this.map.clear(); };
  // 칸마다 평균 위치와 높이 흔들림(표준편차). 점이 적거나 위아래로 넓게 퍼진 칸(깃대·사람 다리)은 뺀다.
  Grid.prototype.count = function () { return this.map.size; };
  Grid.prototype.cells = function (minN, maxStd) {
    const out = [];
    minN = minN || 3; maxStd = maxStd || 0.05;
    for (const c of this.map.values()) {
      if (c.n < minN) continue;
      const my = c.sy / c.n;
      const std = Math.sqrt(Math.max(0, c.syy / c.n - my * my));
      if (std > maxStd) continue;
      out.push({ x: c.sx / c.n, y: my, z: c.sz / c.n, n: c.n, std });
    }
    return out;
  };

  // y = a·x + b·z + c 면을 맞춘다. 튀는 칸(공, 신발, 깃대 밑동)은 세 번 걸러 낸다.
  function fitPlane(cells) {
    let use = cells.slice();
    let res = null;
    for (let iter = 0; iter < 4; iter++) {
      if (use.length < 8) return res;
      // 원점을 가운데로 옮겨 계산을 안정시킨다
      let W = 0, mx = 0, mz = 0;
      for (const p of use) { const w = Math.min(p.n, 30); W += w; mx += w * p.x; mz += w * p.z; }
      mx /= W; mz /= W;
      let sxx = 0, sxz = 0, szz = 0, sx = 0, sz = 0, sxy = 0, szy = 0, sy = 0;
      for (const p of use) {
        const w = Math.min(p.n, 30), x = p.x - mx, z = p.z - mz;
        sxx += w * x * x; sxz += w * x * z; szz += w * z * z; sx += w * x; sz += w * z;
        sxy += w * x * p.y; szy += w * z * p.y; sy += w * p.y;
      }
      // 가운데로 옮겼으니 sx, sz ≈ 0 → a, b는 2×2로 풀린다
      const det = sxx * szz - sxz * sxz;
      if (Math.abs(det) < 1e-9) return res;
      const yc = sy / W;
      const sxy0 = sxy - sx * yc, szy0 = szy - sz * yc;
      const a = (sxy0 * szz - szy0 * sxz) / det;
      const b = (szy0 * sxx - sxy0 * sxz) / det;
      const c = yc - a * mx - b * mz;
      let ss = 0;
      const resid = use.map(p => { const r = p.y - (a * p.x + b * p.z + c); ss += r * r; return r; });
      const rms = Math.sqrt(ss / use.length);
      res = { a, b, c, rms, n: use.length };
      const lim = Math.max(2.5 * rms, 0.015);
      const keep = use.filter((p, i) => Math.abs(resid[i]) <= lim);
      if (keep.length === use.length) break;
      use = keep;
    }
    return res;
  }

  function cellsNear(cells, cx, cz, r) {
    return cells.filter(p => (p.x - cx) ** 2 + (p.z - cz) ** 2 <= r * r);
  }

  // 공→홀 선을 따라 폭 halfWidth, 양 끝을 extend만큼 늘린 띠 안의 칸들
  function cellsInCorridor(cells, ball, hole, halfWidth, extend) {
    const dx = hole[0] - ball[0], dz = hole[2] - ball[2];
    const L = Math.hypot(dx, dz);
    if (L < 0.05) return [];
    const ux = dx / L, uz = dz / L;
    return cells.filter(p => {
      const px = p.x - ball[0], pz = p.z - ball[2];
      const t = px * ux + pz * uz;
      const d = Math.abs(-px * uz + pz * ux);
      return t >= -extend && t <= L + extend && d <= halfWidth;
    });
  }

  // 선을 25cm 토막으로 나눠, 칸이 하나라도 있는 토막의 비율 (그린을 얼마나 비췄나)
  function coverage(cells, ball, hole) {
    const dx = hole[0] - ball[0], dz = hole[2] - ball[2];
    const L = Math.hypot(dx, dz);
    const nb = Math.max(1, Math.ceil(L / 0.25));
    const hit = new Array(nb).fill(false);
    const ux = dx / L, uz = dz / L;
    for (const p of cells) {
      const t = (p.x - ball[0]) * ux + (p.z - ball[2]) * uz;
      const i = Math.floor(t / 0.25);
      if (i >= 0 && i < nb) hit[i] = true;
    }
    return hit.filter(Boolean).length / nb;
  }

  // 면 기울기(a, b)로 공→홀 정보를 만든다.
  function analyze(ball, hole, plane) {
    const dx = hole[0] - ball[0], dz = hole[2] - ball[2];
    const dist = Math.hypot(dx, dz);
    const ux = dx / dist, uz = dz / dist;
    const rx = -uz, rz = ux;                       // 공에서 홀을 볼 때 오른쪽
    const along = plane.a * ux + plane.b * uz;      // 홀 쪽으로 1m 갈 때 오르는 높이(m)
    const side = plane.a * rx + plane.b * rz;       // 오른쪽으로 1m 갈 때 오르는 높이(m)
    const total = Math.hypot(plane.a, plane.b);
    return {
      dist,
      rise: along * dist,           // 홀이 공보다 높은 정도(m), 음수면 내리막
      alongPct: along * 100,        // + 오르막, − 내리막
      sidePct: Math.abs(side) * 100,
      breakTo: side > 0 ? 'left' : 'right', // 오른쪽이 높으면 공은 왼쪽으로 휜다
      slopePct: total * 100,
      fall: total > 0 ? [-plane.a / total, -plane.b / total] : [0, 0], // 물이 흐르는 쪽(x, z)
    };
  }

  // 공→홀 띠 위의 작은 마당들(50cm 간격, 가운데·좌우 50cm)마다 따로 기울기를 맞춘다 → 구슬이 굴곡을 따라 흐르게.
  // 칸이 모자란 마당은 뺀다(그 자리는 띠 전체 면을 쓴다).
  function gradientField(cells, ball, hole, plane) {
    const dx = hole[0] - ball[0], dz = hole[2] - ball[2];
    const L = Math.hypot(dx, dz);
    if (L < 0.05) return [];
    const ux = dx / L, uz = dz / L, out = [];
    for (let u = -0.25; u <= L + 0.25; u += 0.5) {
      for (const v of [-0.5, 0, 0.5]) {
        const x = ball[0] + ux * u - uz * v, z = ball[2] + uz * u + ux * v;
        const f = fitPlane(cellsNear(cells, x, z, 0.6));
        // 띠 전체 면과 너무 다르면(5% 넘게) 잡음으로 보고 버린다
        if (f && f.n >= 10 && (!plane || Math.hypot(f.a - plane.a, f.b - plane.b) < 0.05)) out.push({ x, z, a: f.a, b: f.b });
      }
    }
    return out;
  }
  // 그 자리 기울기: 가까운 마당들을 거리로 섞고, 없으면 띠 전체 면
  function gradAt(field, plane, x, z) {
    let wa = 0, wb = 0, W = 0;
    for (const f of field) {
      const d2 = (f.x - x) ** 2 + (f.z - z) ** 2;
      if (d2 > 0.49) continue;
      const w = 1 / (d2 + 0.04);
      wa += w * f.a; wb += w * f.b; W += w;
    }
    if (W > 0) return [wa / W, wb / W];
    return plane ? [plane.a, plane.b] : [0, 0];
  }

  // ── 시험판 4: 화면에서 찍은 점 4개(공·홀·옆 2개)로 재기 ──

  // 화면 위치(0~1) → 세계 좌표 광선 {o: 카메라 자리, d: 그 화면 점을 지나는 방향(길이 1)}
  function screenRay(nx, ny, invProj, camMatrix) {
    const p = unproject(nx, ny, 1, invProj, camMatrix);
    const o = [camMatrix[12], camMatrix[13], camMatrix[14]];
    const d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]], l = Math.hypot(d[0], d[1], d[2]);
    return { o, d: [d[0] / l, d[1] / l, d[2] / l] };
  }

  // 찍은 점 몇 개([x,y,z], 3개 이상)를 지나는 면 y = a·x + b·z + c. 점이 적으니 걸러 내지 않는다.
  // 점들이 거의 한 줄에 있으면 옆 기울기를 알 수 없어서 null.
  function fitPoints(pts) {
    const n = pts.length;
    if (n < 3) return null;
    let mx = 0, my = 0, mz = 0;
    for (const p of pts) { mx += p[0]; my += p[1]; mz += p[2]; }
    mx /= n; my /= n; mz /= n;
    let sxx = 0, sxz = 0, szz = 0, sxy = 0, szy = 0;
    for (const p of pts) {
      const x = p[0] - mx, y = p[1] - my, z = p[2] - mz;
      sxx += x * x; sxz += x * z; szz += z * z; sxy += x * y; szy += z * y;
    }
    const det = sxx * szz - sxz * sxz;
    if (!(det > 1e-4 * (sxx + szz) ** 2)) return null;
    const a = (sxy * szz - szy * sxz) / det;
    const b = (szy * sxx - sxy * sxz) / det;
    const c = my - a * mx - b * mz;
    let ss = 0;
    for (const p of pts) { const r = p[1] - (a * p[0] + b * p[2] + c); ss += r * r; }
    return { a, b, c, rms: Math.sqrt(ss / n), n };
  }

  // 점 p가 공→홀 선에서 어디쯤인가: u = 공에서 홀 쪽으로 몇 m, v = 선에서 오른쪽으로 몇 m(왼쪽이면 −)
  function lineOffset(ball, hole, p) {
    const dx = hole[0] - ball[0], dz = hole[2] - ball[2], L = Math.hypot(dx, dz) || 1;
    const ux = dx / L, uz = dz / L, px = p[0] - ball[0], pz = p[2] - ball[2];
    return { u: px * ux + pz * uz, v: -px * uz + pz * ux };
  }

  // ── 유도선: 퍼팅스타(hwani172/my golf-putting/src/core.js)와 같은 공 물리 ──
  // 경사로 끌리는 가속 = 9.81·5/7·기울기(구르는 공), 굴림 저항 = 그린 빠르기(스팀프, m)로 정해지는 감속도.
  const G57 = 9.81 * 5 / 7, HOLE_R = 0.054, CUP = 0.108;
  const decelOf = stimpM => 1.83 * 1.83 / (2 * stimpM);   // 스팀프미터: 1.83m/s로 굴린 공이 stimpM 미터 가서 멈춤

  // 공을 (vx, vz)로 굴려 멈출 때까지. 공→홀 선 기준으로: 홀 높이(u = L)를 지날 때 옆으로 얼마나 비켜 있나(cross, 오른쪽 +), 멈춘 곳(uStop).
  function roll(ball, vx, vz, plane, decel, F, path) {
    const dt = 1 / 120;
    let x = ball[0], z = ball[2], cross = null;
    for (let i = 0; i < 120 * 30; i++) {
      vx -= G57 * plane.a * dt; vz -= G57 * plane.b * dt;
      const sp = Math.hypot(vx, vz), d = decel * dt;
      if (sp <= d) break;
      const k = (sp - d) / sp; vx *= k; vz *= k;
      x += vx * dt; z += vz * dt;
      const px = x - ball[0], pz = z - ball[2], u = px * F.ux + pz * F.uz;
      if (cross === null && u >= F.L) cross = -px * F.uz + pz * F.ux;
      if (path) {
        if (i % 4 === 0) path.push([x, z]);
        if (Math.hypot(x - F.hx, z - F.hz) < HOLE_R * 0.7) { path.push([x, z]); break; }   // 홀에 들어감
      }
    }
    return { cross, uStop: (x - ball[0]) * F.ux + (z - ball[2]) * F.uz };
  }

  // 홀을 지나 past(m)만큼 더 가서 멈출 세기로, 홀 한가운데로 들어가는 방향을 찾는다.
  // 돌려줌: aim = 공에서 홀까지 거리만큼 겨냥 방향으로 간 점(그 점을 보고 친다), side = 그 점이 홀에서 옆으로 몇 m(오른쪽 +),
  //        flat = 평지에서 몇 m 치는 세기인가, path = 예상 경로 [[x, z], …]
  function puttLine(ball, hole, plane, stimpM, past) {
    past = past == null ? 0.4 : past;
    const dx = hole[0] - ball[0], dz = hole[2] - ball[2], L = Math.hypot(dx, dz);
    if (L < 0.3) return null;
    const F = { L, ux: dx / L, uz: dz / L, hx: hole[0], hz: hole[2] }, decel = decelOf(stimpM);
    const dirOf = th => [F.ux * Math.cos(th) - F.uz * Math.sin(th), F.uz * Math.cos(th) + F.ux * Math.sin(th)];  // th > 0 → 오른쪽
    const speedFor = th => {
      const d = dirOf(th); let lo = 0.05, hi = 8;
      for (let k = 0; k < 28; k++) { const v = (lo + hi) / 2; if (roll(ball, d[0] * v, d[1] * v, plane, decel, F).uStop < L + past) lo = v; else hi = v; }
      return hi;
    };
    const crossAt = th => { const v = speedFor(th), d = dirOf(th); return roll(ball, d[0] * v, d[1] * v, plane, decel, F).cross; };
    let lo = -0.7, hi = 0.7;
    const cLo = crossAt(lo), cHi = crossAt(hi);
    if (cLo === null || cHi === null || cLo > 0 || cHi < 0) return null;   // 경사가 너무 심해 넣을 길이 없음
    for (let k = 0; k < 30; k++) { const m = (lo + hi) / 2, c = crossAt(m); if (c === null || c < 0) lo = m; else hi = m; }
    const th = (lo + hi) / 2, v = speedFor(th), d = dirOf(th), path = [[ball[0], ball[2]]];
    roll(ball, d[0] * v, d[1] * v, plane, decel, F, path);
    return { th, v, side: L * Math.sin(th), cups: L * Math.sin(th) / CUP, flat: v * v / (2 * decel),
      aim: [ball[0] + d[0] * L, ball[2] + d[1] * L], path };
  }

  const api = { mat4Invert, mat4MulVec, unproject, project, Grid, fitPlane, cellsNear, cellsInCorridor, coverage, analyze, gradientField, gradAt,
    screenRay, fitPoints, lineOffset, puttLine, decelOf, CUP };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.GR = api;
})(this);
