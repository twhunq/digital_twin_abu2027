// Dong hoc robot v3 (thuan du lieu): khop, cay khau co khi (link) + hinh khoi tung khau, FK/IK,
// kich thuoc bao, trong tam, thoi gian khop.
// He truc robot: goc = tam than tren mat tiep xuc banh; x toi truoc, y sang trai, z len (mm).
// "He robot" = he truc gan huong robot nhung KHONG nghieng theo doc; than (body) nghieng pitch quanh goc.
// Mot mo hinh duy nhat cho: mo hinh 3D (robot3d.js), kich thuoc bao + va cham (sim.js),
// kiem tra ket cau (tools/check_mech.mjs) -> hinh ve va mo phong khong the lech nhau.

export const DEG = Math.PI / 180;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

// Kich thuoc vat the (mm)
export const SIZE = { earth: 350, sky: 200, mustika: 210 };

// ---------------------------------------------------------------- khop
// TR: s (truot X), lz (nang Z), w (do mo kep), yaw/pitch (do), len (ong long), roll (lat Sky, do), fing (0..1),
//     cp (khop can bang kep, do), tw (xoay dau cong cu quanh truc dung, do), g (do mo ngon dau cong cu, mm)
// BR: lz (day kep so voi mat san), s (truot X), ly (truot Y), w, pf (mat hut tren day kep), tilt (do), track (0..1)
export function stowJoints(role, R) {
  const r = R[role];
  if (role === 'TR') {
    const st = r.stow;
    return { s: st.slide, lz: st.lift, w: st.open, yaw: st.yaw, pitch: st.pitch, len: st.len, roll: 0, fing: 0, cp: 0, tw: 0, g: r.tool.g_open };
  }
  const st = r.stow;
  return { lz: st.lift, s: st.slide, ly: st.lat, w: st.open, pf: st.plunger, tilt: st.tilt, track: st.track };
}

export function jointLimits(role, R) {
  const r = R[role];
  if (role === 'TR') {
    const c = r.clamp, b = r.boom, t = r.tool;
    return { s: c.slide, lz: c.lift, w: c.open, yaw: b.yaw, pitch: b.pitch, len: b.len, roll: [-180, 180], fing: [0, 1], cp: [-12, 12], tw: [-180, 180], g: [t.g_min, t.g_open] };
  }
  const g = r.gripper;
  return { lz: g.lift, s: g.slide, ly: g.lat, w: g.open, pf: g.plunger, tilt: r.tower.tilt, track: [0, 1] };
}

// Gioi han toc do / gia toc tung khop TINH TU PHAN CUNG (robot_spec.json: motors + actuators).
// lin: v = k * omega / n * r ; F_ra = T * n * eta / (r * k) ; a = (F_ra - m g vert - 0.05 m g (1 - vert)) / m
// rot: v = rpm / n * 6 (do/s) ; a = (T * n * eta - grav) / I (rad/s2 -> do/s2). Ca hai bi chan boi cap_v, cap_a.
const G = 9.81;
// Khoi luong tung khau tu BOM (kg) va trong tam cua khau (tam hinh hoc cac khoi, theo the tich) trong he khau
export function linkMass(role, R) {
  const bom = R[role].bom || {}, out = {};
  for (const [k, v] of Object.entries(bom)) if (Array.isArray(v)) out[k] = v.reduce((a, it) => a + it[1], 0);
  return out;
}
const CC = new WeakMap();
function linkCentroids(role, R) {
  let c = CC.get(R);
  if (!c) CC.set(R, (c = {}));
  if (c[role]) return c[role];
  const out = {};
  for (const l of links(role, R)) {
    let V = 0, x = 0, y = 0, z = 0;
    for (const p of l.prims) {
      const q = primBox(p, R), v = Math.max(1, (q.b[0] - q.a[0]) * (q.b[1] - q.a[1]) * (q.b[2] - q.a[2]));
      V += v; x += (v * (q.a[0] + q.b[0])) / 2; y += (v * (q.a[1] + q.b[1])) / 2; z += (v * (q.a[2] + q.b[2])) / 2;
    }
    out[l.name] = V ? [x / V, y / V, z / V] : [0, 0, 0];
  }
  return (c[role] = out);
}
// Tai cua 1 bo truyen dong: lin -> khoi luong keo m (kg); rot -> momen quan tinh I (kg.m2) + momen trong luc grav (N.m)
export function actuatorLoad(role, R, act) {
  if (act.kind === 'lin' && act.links) {
    const lm = linkMass(role, R);
    const m = act.links.reduce((a, k) => a + (lm[k] || 0), 0) + 0.5 * (act.half || []).reduce((a, k) => a + (lm[k] || 0), 0) + (act.pay || 0);
    return { m };
  }
  if (act.kind === 'rot' && act.links && act.axis) {
    const lm = linkMass(role, R), cen = linkCentroids(role, R), j = { ...stowJoints(role, R), ...(act.pose || {}) };
    const F = frames(role, R, j, 0), b = R.TR.boom, piv = M.ap(F.boomP, [0, 0, 0]);
    const pts = act.links.map((k) => [lm[k] || 0, M.ap(F[k], cen[k])]);
    const tip = M.ap(F.boomP, [j.len, 0, 0]);
    pts.push([act.pay || 0, [tip[0], tip[1], tip[2] - b.tool_drop - 105]]);
    let I = 0, grav = 0;
    for (const [m, p] of pts) {
      if (act.axis === 'yaw') I += m * (((p[0] - b.turret[0]) / 1000) ** 2 + ((p[1] - b.turret[1]) / 1000) ** 2);
      else { const dx = (p[0] - piv[0]) / 1000, dz = (p[2] - piv[2]) / 1000; I += m * (dx * dx + dz * dz); grav += m * G * dx; }
    }
    return { I, grav: Math.abs(grav) };
  }
  return { m: act.m, I: act.I, grav: act.grav || 0 };
}
export function actuatorLimit(R, act, role) {
  const mo = R.motors[act.motor], w = (mo.rpm * 2 * Math.PI) / 60, L = role ? actuatorLoad(role, R, act) : { m: act.m, I: act.I, grav: act.grav || 0 };
  if (act.kind === 'lin') {
    const r = act.r / 1000, vmax = ((act.k * w) / act.n) * r * 1000, m = L.m;
    const F = (mo.T * act.n * act.eta) / (r * act.k), load = m * G * (act.vert || 0) + 0.05 * m * G * (1 - (act.vert || 0));
    const amax = ((F - load) / m) * 1000;
    return { v: Math.min(act.cap_v, vmax), a: Math.min(act.cap_a, amax), vmax, amax, F, load, m };
  }
  const vmax = (mo.rpm / act.n) * 6, Tout = mo.T * act.n * act.eta, amax = ((Tout - L.grav) / L.I) / DEG;
  return { v: Math.min(act.cap_v, vmax), a: Math.min(act.cap_a, amax), vmax, amax, Tout, load: L.grav, I: L.I };
}
const LIMC = new WeakMap();
function jointLimitsVA(role, R) {
  let c = LIMC.get(R);
  if (!c) LIMC.set(R, (c = {}));
  if (c[role]) return c[role];
  const r = R[role], v = {}, a = {};
  for (const [k, act] of Object.entries(r.actuators || {})) { if (k === 'note') continue; const L = actuatorLimit(R, act, role); v[k] = L.v; a[k] = L.a; }
  if (role === 'TR') { v.fing = 1 / r.tool.t_fingers; a.fing = 20; }
  else { v.track = 1 / r.tracks.t_deploy; a.track = 20; }
  return (c[role] = { v, a });
}
export const jointVA = jointLimitsVA;

// Profile hinh thang dong bo nhieu khop: tra ve thoi gian T va ti le tang toc c (0 < c <= 0.5) sao cho
// MOI khop deu khong vuot v_max, a_max cua no (khop dai nhat quyet dinh T).
export function profile(role, R, from, to) {
  const { v, a } = jointLimitsVA(role, R), J = [];
  for (const k of Object.keys(to)) {
    if (from[k] == null || v[k] == null) continue;
    let d = Math.abs(to[k] - from[k]);
    if (k === 'yaw') d = Math.abs(((to[k] - from[k] + 540) % 360) - 180);
    if (d > 1e-6) J.push({ d, v: v[k], a: a[k] });
  }
  if (!J.length) return { T: 0, c: 0.5 };
  let T = 0;
  for (const j of J) T = Math.max(T, j.d >= (j.v * j.v) / j.a ? j.d / j.v + j.v / j.a : 2 * Math.sqrt(j.d / j.a));
  for (let it = 0; it < 60; it++) {
    let lo = 0, hi = 0.5;
    for (const j of J) {
      const q = j.d / (j.a * T * T); // c (1 - c) >= q
      lo = Math.max(lo, q >= 0.25 ? 0.5 : (1 - Math.sqrt(1 - 4 * q)) / 2);
      hi = Math.min(hi, 1 - j.d / (j.v * T));
    }
    if (lo <= hi + 1e-9) return { T, c: Math.max(lo, 1e-3) };
    T *= 1.03;
  }
  return { T, c: 0.5 };
}
// Vi tri chuan hoa (0..1) theo profile hinh thang, u = t / T
export function trapS(u, c) {
  if (u <= 0) return 0;
  if (u >= 1) return 1;
  const vp = 1 / (1 - c);
  if (u < c) return (0.5 * vp * u * u) / c;
  if (u <= 1 - c) return 0.5 * vp * c + vp * (u - c);
  return 1 - (0.5 * vp * (1 - u) * (1 - u)) / c;
}
// Thoi gian de moi khop di tu `from` toi `to` (dong thoi, profile hinh thang) - s
export function jointTime(role, R, from, to) { return profile(role, R, from, to).T; }

// ---------------------------------------------------------------- dai so 3x4 (affine cung)
// M = [r00 r01 r02 tx, r10 r11 r12 ty, r20 r21 r22 tz]
export const M = {
  I: () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
  T: (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z],
  RY: (a) => { const c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0]; }, // quanh +y; RY(-p): mui x ngang len p
  RZ: (a) => { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0]; },
  mul(A, B, ...rest) {
    const o = new Array(12);
    for (let i = 0; i < 3; i++) {
      for (let k = 0; k < 3; k++) o[i * 4 + k] = A[i * 4] * B[k] + A[i * 4 + 1] * B[4 + k] + A[i * 4 + 2] * B[8 + k];
      o[i * 4 + 3] = A[i * 4] * B[3] + A[i * 4 + 1] * B[7] + A[i * 4 + 2] * B[11] + A[i * 4 + 3];
    }
    return rest.length ? M.mul(o, ...rest) : o;
  },
  ap: (A, p) => [A[0] * p[0] + A[1] * p[1] + A[2] * p[2] + A[3], A[4] * p[0] + A[5] * p[1] + A[6] * p[2] + A[7], A[8] * p[0] + A[9] * p[1] + A[10] * p[2] + A[11]],
  apv: (A, p) => [A[0] * p[0] + A[1] * p[1] + A[2] * p[2], A[4] * p[0] + A[5] * p[1] + A[6] * p[2], A[8] * p[0] + A[9] * p[1] + A[10] * p[2]],
  inv(A) { // nghich dao phep bien doi cung
    const r = [A[0], A[4], A[8], A[1], A[5], A[9], A[2], A[6], A[10]];
    const t = [A[3], A[7], A[11]];
    return [r[0], r[1], r[2], -(r[0] * t[0] + r[1] * t[1] + r[2] * t[2]), r[3], r[4], r[5], -(r[3] * t[0] + r[4] * t[1] + r[5] * t[2]), r[6], r[7], r[8], -(r[6] * t[0] + r[7] * t[1] + r[8] * t[2])];
  },
};

// ---------------------------------------------------------------- hinh hoc co cau (khau + khoi)
// Khoi: {k:'box', a:[x0,y0,z0], b:[x1,y1,z1], m, rad?, ry?(do, xoay quanh y qua tam)} | {k:'cyl', ax, c, r, h, m, r2?}
//       {k:'mecanum', d, w, n, hand} | {k:'track'}. Toa do theo khau.
// Khau: {name, parent, prims, conn:[khau phai tiep xuc co hoc], free:[khau duoc phep long nhau (truot/ong long)]}
const box = (a, b, m, o = {}) => ({ k: 'box', a, b, m, ...o });
const cyl = (ax, c, r, h, m, o = {}) => ({ k: 'cyl', ax, c, r, h, m, ...o });
const ys = (sg, a, b) => (sg > 0 ? [a, b] : [-b, -a]); // doan [a, b] theo phia y
const bxs = (sg, [x0, x1], [a, b], [z0, z1], m, o) => box([x0, ys(sg, a, b)[0], z0], [x1, ys(sg, a, b)[1], z1], m, o);

// TR: kep chu U tren ban truot ong long 3 tang; kep can bang bang ray cung (tam ao o day kep);
// can cau ong long 3 doan; dau cong cu: 2 ngon song song co dia xoay (lat Sky quanh tam khoi) + coc hut.
function trLinks(R) {
  const T = R.TR, c = T.chassis, w = T.wheels, cl = T.clamp, b = T.boom, tl = T.tool;
  const L = c.length / 2, W = c.width / 2, zt = c.z_top, [tx, ty] = b.turret;
  const P0 = [cl.x0, cl.z0]; // tam ao khop can bang (day kep)
  const links = [];
  const add = (l) => { links.push({ conn: l.parent ? [l.parent] : [], free: [], ...l }); };
  // than: sàn sau kin + 2 dam doc (chu U mo phia truoc) + vo banh
  const body = [
    box([-L, -254, c.z_bottom], [-60, 254, zt + 6], 'alu', { rad: 6, name: 'sàn sau' }),
    box([-L + 5, -250, zt + 6], [-270, 250, zt + 56], 'panel', { rad: 8, name: 'hộp điều khiển' }),
    cyl('z', [-290, 110, zt + 66], 20, 20, 'stop'),
    cyl('z', [tx, ty, (zt + 6 + b.pivot_z - 80) / 2], b.column_r, b.pivot_z - 80 - zt - 6, 'aluDark', { name: 'cột mâm xoay' }),
  ];
  for (const sg of [-1, 1]) {
    body.push(bxs(sg, [-L, L], [246, W], [156, zt + 6], 'alu', { rad: 5, name: 'dầm dọc' }));
    body.push(bxs(sg, [-L + 10, L - 10], [W - 11, W], [30, 156], 'team', { rad: 5, name: 'vỏ truyền động' }));
    body.push(bxs(sg, [140, L - 5], [246, 254], [40, 156], 'aluDark', { name: 'tấm đỡ bánh' }));
    body.push(bxs(sg, [-300, 320], [256, 284], [zt + 6, zt + 26], 'steel', { rad: 2, name: 'ray X cố định' }));
    for (const sx of [-1, 1]) body.push(cyl('y', [sx * w.x, sg * (W + 250) / 2, w.diameter / 2], 12, W - 250, 'steel', { name: 'trục bánh' }));
  }
  add({ name: 'body', parent: null, prims: body });
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) add({ name: `wheel${sx}${sy}`, parent: 'body', prims: [{ k: 'mecanum', d: w.diameter, w: w.width, n: w.rollers, hand: sx * sy }] });
  // ban truot X ong long: ray co dinh (than) -> tang giua (s/2) -> tang trong (s) mang dam ngang + 2 tru Z
  const mid = [], sled = [];
  for (const sg of [-1, 1]) {
    mid.push(bxs(sg, [-300, 120], [257, 283], [zt + 26, zt + 51], 'alu', { rad: 2, name: 'tầng giữa ray X' }));
    sled.push(bxs(sg, [-330, -90], [258, 282], [zt + 51, zt + 76], 'act', { rad: 2, name: 'tầng trong ray X' }));
    sled.push(bxs(sg, [-120, -100], [278, 298], [zt + 106, 700], 'extrusion', { rad: 3, name: 'trụ Z' }));
    sled.push(bxs(sg, [-145, -120], [281, 295], [zt + 106, zt + 160], 'act', { name: 'gân trụ Z' }));
  }
  sled.push(box([-125, -298, zt + 76], [-95, 298, zt + 106], 'act', { rad: 4, name: 'dầm ngang bàn trượt' }));
  add({ name: 'sledMid', parent: 'body', prims: mid, free: ['body'] });
  add({ name: 'sled', parent: 'sledMid', prims: sled, free: ['sledMid'] });
  // xe con Z: con truot tren 2 tru + 2 tam ben mang ray cung. Ray cung co tam = tam day kep (tam ao cua khop
  // can bang), nam hoan toan phia sau mat sau khoi (khong vuong cot Storage ben canh).
  const RA = 520, A0 = 114, A1 = 134, zc = [];
  for (const sg of [-1, 1]) {
    zc.push(bxs(sg, [-100, -86], [276, 300], [392, 452], 'act', { name: 'con trượt Z' }));
    zc.push(bxs(sg, [-270, -86], [268, 276], [370, 505], 'aluDark', { rad: 3, name: 'tấm mang ray cung' }));
    const n = 6;
    for (let i = 0; i < n; i++) {
      const a = (A0 + ((i + 0.5) / n) * (A1 - A0)) * DEG, cx = P0[0] + RA * Math.cos(a), cz = P0[1] + RA * Math.sin(a);
      zc.push(bxs(sg, [cx - 26, cx + 26], [260, 268], [cz - 11, cz + 11], 'steel', { ry: 90 - a / DEG, name: 'ray cung cân bằng' }));
    }
  }
  add({ name: 'zcar', parent: 'sled', prims: zc, free: ['sled'] });
  // cum kep can bang: dam Y ngay sau mat sau khoi + 2 tai + guoc truot tren ray cung
  const lv = [box([-69, -248, 395], [-55, 248, 425], 'act', { rad: 3, name: 'dầm dẫn hướng hàm (Y)' })];
  const as = 130 * DEG, sx0 = P0[0] + RA * Math.cos(as), sz0 = P0[1] + RA * Math.sin(as);
  for (const sg of [-1, 1]) {
    lv.push(bxs(sg, [-240, -55], [248, 256], [380, 440], 'alu', { rad: 3, name: 'tai kẹp' }));
    lv.push(bxs(sg, [sx0 - 30, sx0 + 30], [256, 260], [sz0 - 8, sz0 + 8], 'steel', { ry: 90 - 130, name: 'guốc trượt ray cung' }));
  }
  add({ name: 'clampLevel', parent: 'zcar', prims: lv, free: ['zcar'] });
  for (const sg of [-1, 1]) add({
    name: sg > 0 ? 'jawP' : 'jawN', parent: 'clampLevel', free: ['clampLevel'],
    prims: [
      bxs(sg, [-20, 280], [5, 13], [cl.z0, cl.z0 + cl.jaw_h], 'alu', { rad: 2, name: 'hàm' }),
      bxs(sg, [-10, 270], [0, 5], [cl.z0 + 20, cl.z0 + cl.jaw_h - 20], 'pad', { name: 'đệm hàm' }),
      bxs(sg, [-55, -20], [1, 13], [395, 425], 'act', { name: 'tay nối hàm' }),
      bxs(sg, [-20, 40], [5, 13], [cl.z0 + cl.jaw_h - 60, cl.z0 + cl.jaw_h], 'act', { name: 'vát dẫn hướng' }),
    ],
  });
  // can cau: mam xoay -> chac nang (truc nang lech truoc pivot_off) -> can ong long 4 doan -> chac treo dau cong cu
  const po = b.pivot_off || 0;
  add({
    name: 'turret', parent: 'body', prims: [
      cyl('z', [0, 0, b.pivot_z - 65], 65, 30, 'act', { name: 'ổ xoay' }),
      box([-45, 40, b.pivot_z - 60], [po + 30, 52, b.pivot_z - 50], 'aluDark', { name: 'đế chạc' }),
      box([-45, -52, b.pivot_z - 60], [po + 30, -40, b.pivot_z - 50], 'aluDark', { name: 'đế chạc' }),
      box([po - 30, 40, b.pivot_z - 50], [po + 30, 52, b.pivot_z + 32], 'aluDark', { name: 'chạc nâng' }),
      box([po - 30, -52, b.pivot_z - 50], [po + 30, -40, b.pivot_z + 32], 'aluDark', { name: 'chạc nâng' }),
      cyl('y', [po, 70, b.pivot_z], 30, 36, 'steel', { name: 'động cơ + hộp số nâng cần' }),
    ],
  });
  const L0 = b.len[0]; // chieu dai can khi thu gon (tu truc nang toi chot co tay)
  // can ong long 5 doan (moi doan truot e/4): chong lan toi thieu >= 120 mm o hanh trinh toi da
  add({ name: 'boomP', parent: 'turret', prims: [cyl('y', [0, 35, 0], 26, 10, 'black', { name: 'ngõng trục nâng' }), cyl('y', [0, -35, 0], 26, 10, 'black', { name: 'ngõng trục nâng' }), box([-60, -30, -30], [L0 - 60, 30, 30], 'alu', { rad: 4, name: 'cần ngoài' })] });
  add({ name: 'boomM', parent: 'boomP', free: ['boomP'], prims: [box([-52, -25, -25], [L0 - 60, 25, 25], 'aluDark', { rad: 3, name: 'cần giữa 1' })] });
  add({ name: 'boomN', parent: 'boomM', free: ['boomM', 'boomP'], prims: [box([-44, -20, -20], [L0 - 60, 20, 20], 'alu', { rad: 3, name: 'cần giữa 2' })] });
  add({ name: 'boomO', parent: 'boomN', free: ['boomN', 'boomM', 'boomP'], prims: [box([-36, -15.5, -15.5], [L0 - 60, 15.5, 15.5], 'aluDark', { rad: 3, name: 'cần giữa 3' })] });
  add({
    name: 'boomI', parent: 'boomO', free: ['boomO', 'boomN', 'boomM', 'boomP'], prims: [
      box([-28, -11, -11], [L0 - 60, 11, 11], 'alu', { rad: 2, name: 'cần trong' }),
      box([L0 - 80, 13, -13], [L0 + 12, 19, 13], 'act', { name: 'chạc treo đầu' }), box([L0 - 80, -19, -13], [L0 + 12, -13, 13], 'act', { name: 'chạc treo đầu' }),
      cyl('y', [L0, 0, 0], 8, 42, 'steel', { name: 'chốt cổ tay' }),
    ],
  });
  const D = b.tool_drop, pz = -D - tl.pad_z;
  add({
    name: 'tool', parent: 'boomI', prims: [
      cyl('z', [0, 0, -(D - 27) / 2 + 6], 12, D - 15, 'black', { name: 'thân cổ tay' }),
      cyl('z', [0, 0, -(D - 52)], 22, 24, 'act', { name: 'động cơ xoay cổ tay' }),
      box([-30, -(tl.g_open / 2 + 10), -(D - 5)], [30, tl.g_open / 2 + 10, -(D - 27)], 'aluDark', { rad: 3, name: 'ray ngón' }),
    ],
  });
  add({ name: 'cup', parent: 'tool', free: ['tool'], prims: [cyl('z', [0, 0, -(D - 14)], 8, 16, 'hub'), cyl('z', [0, 0, -D + 5], 26, 10, 'cup', { r2: 20, name: 'cốc hút' })] });
  for (const sg of [-1, 1]) {
    const f = sg > 0 ? 'fP' : 'fN';
    // ngon 2 doan: doan tren tren con truot, doan duoi (mang dia xoay) truot xuong 45 mm khi lat Sky -> goc khoi khong cham ray
    add({ name: f, parent: 'tool', free: ['tool'], prims: [bxs(sg, [-22, 22], [4, 26], [-(D - 2), -(D - 28)], 'act', { name: 'con trượt ngón' }), bxs(sg, [-14, 14], [10, 18], [pz + 10, -(D - 18)], 'tpu', { rad: 3, name: 'ngón trên' })] });
    add({ name: f + 'lo', parent: f, free: [f], prims: [bxs(sg, [-11, 11], [18, 24], [pz - 32, pz + 70], 'aluDark', { rad: 2, name: 'ngón dưới (trượt)' })] });
    add({ name: f + 'pad', parent: f + 'lo', prims: [cyl('y', [0, sg * 5, 0], 28, 10, 'rubber', { name: 'đĩa xoay' }), cyl('y', [0, sg * 14, 0], 8, 8, 'act', { name: 'trục đĩa' })] });
  }
  return links;
}

// BR: xich ha duoc; thap 3 tang ong long tren khop nghieng tu can bang; xe con nang tu do tren tang trong;
// ban truot Y; ban truot X ong long 3 tang; tam dau cong cu mang 2 ham (truot Y) + tru hut tren cot ong long 4 tang.
function brLinks(R) {
  const B = R.BR, c = B.chassis, w = B.wheels, T = B.tracks, tw = B.tower, g = B.gripper;
  const L = c.length / 2, zt = c.z_top, PV = brPivot(R);
  const links = [];
  const add = (l) => { links.push({ conn: l.parent ? [l.parent] : [], free: [], ...l }); };
  const body = [
    box([-300, -100, c.z_bottom], [200, 100, zt - 10], 'panel', { rad: 10, name: 'hộp thân' }),
    box([200, -100, 125], [300, 100, zt - 10], 'panel', { rad: 8, name: 'hộp thân trước' }),
    box([-L, -240, zt - 10], [L, 240, zt], 'aluDark', { rad: 4, name: 'tấm sàn' }),
    box([-L, -240, c.z_bottom + 8], [-300, 240, zt - 10], 'extrusion', { rad: 6, name: 'khung sau' }),
    box([300, -240, 150], [L, 240, zt - 10], 'extrusion', { rad: 6, name: 'khung trước' }),
    box([L - 3, -85, 155], [L, 85, 185], 'team'), box([-L, -85, 95], [-L + 3, 85, 155], 'team'),
    box([40, -110, zt], [200, 110, zt + 50], 'steel', { rad: 10, name: 'pin' }),
    box([-60, -180, zt], [40, -60, zt + 60], 'pcb', { rad: 8, name: 'hộp điều khiển' }),
    cyl('z', [312, 0, 170], 26, 36, 'black', { name: 'LiDAR (cản trước)' }), cyl('z', [312, 0, 170], 27, 10, 'lens'),
    cyl('z', [-300, 200, zt + 10], 18, 20, 'stop'),
    cyl('y', [PV[0], 0, PV[2]], 10, 316, 'steel', { name: 'chốt khớp nghiêng' }),
  ];
  for (const sg of [-1, 1]) {
    body.push(bxs(sg, [-300, 200], [232, 240], [c.z_bottom + 8, zt - 10], 'extrusion', { name: 'khung hông' }));
    body.push(bxs(sg, [200, 300], [232, 240], [125, zt - 10], 'extrusion', { name: 'khung hông' }));
    body.push(bxs(sg, [PV[0] - 15, PV[0] + 15], [146, 158], [zt, PV[2] + 15], 'act', { name: 'tai khớp nghiêng' }));
    for (const sx of [-1, 1]) {
      body.push(cyl('y', [sx * w.x, sg * (100 + 80 / 2), w.diameter / 2], 26, 80, 'steel', { name: 'động cơ bánh' }));
      body.push(bxs(sg, [sx * 160 - 20, sx * 160 + 20], [240, 246], [70, 150], 'act', { name: 'ray hạ xích' }));
    }
  }
  add({ name: 'body', parent: null, prims: body });
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) add({ name: `wheel${sx}${sy}`, parent: 'body', prims: [{ k: 'mecanum', d: w.diameter, w: w.width, n: w.rollers, hand: sx * sy }] });
  for (const sg of [-1, 1]) {
    const pr = [{ k: 'track' }];
    for (const px of [-300, 200]) pr.push(cyl('y', [px, 0, 50], 28, T.width + 6, 'hub'));
    for (const sx of [-1, 1]) pr.push(box([sx * 160 - 15, sg > 0 ? -46 : 34, 70], [sx * 160 + 15, sg > 0 ? -34 : 46, 150], 'steel', { name: 'con trượt hạ xích' }));
    add({ name: sg > 0 ? 'trackP' : 'trackN', parent: 'body', free: ['body'], prims: pr });
  }
  // cum nghieng: de thap + tang 1 (co dinh tren de)
  const lv = [box([PV[0] - 60, -140, PV[2] + 12], [PV[0] + 60, 140, PV[2] + 27], 'act', { rad: 5, name: 'đế tháp' })];
  const s1 = [PV[2] + 27, tw.base[1] + tw.stage1_h];
  for (const sg of [-1, 1]) {
    lv.push(bxs(sg, [PV[0] - 15, PV[0] + 15], [128, 140], [PV[2] - 12, PV[2] + 12], 'act', { name: 'bản lề tháp' }));
    lv.push(bxs(sg, [PV[0] - 20, PV[0] + 20], [120, 140], s1, 'extrusion', { rad: 3, name: 'cột tầng 1' }));
    lv.push(bxs(sg, [PV[0] - 60, PV[0] - 20], [122, 138], [PV[2] + 27, PV[2] + 130], 'act', { name: 'gân chân tháp' }));
  }
  add({ name: 'level', parent: 'body', free: ['body'], prims: lv });
  const m2 = [box([PV[0] - 28, -118, PV[2] + 35], [PV[0] - 18, 118, PV[2] + 50], 'aluDark', { name: 'giằng tầng 2' })];
  const m3 = [box([PV[0] - 26, -104, s1[1] - 12], [PV[0] - 16, 104, s1[1]], 'aluDark', { name: 'giằng tầng 3' })];
  for (const sg of [-1, 1]) {
    m2.push(bxs(sg, [PV[0] - 18, PV[0] + 18], [104, 120], [PV[2] + 35, s1[1]], 'aluDark', { rad: 2, name: 'cột tầng 2' }));
    m3.push(bxs(sg, [PV[0] - 16, PV[0] + 16], [88, 104], [PV[2] + 40, s1[1]], 'alu', { rad: 2, name: 'cột tầng 3' }));
  }
  add({ name: 'mast2', parent: 'level', free: ['level'], prims: m2 });
  add({ name: 'mast3', parent: 'mast2', free: ['mast2'], prims: m3 });
  // xe con (z = lz): mat tren = lz + jaw_h + top_margin
  const zc0 = g.jaw_h - 20, zc1 = g.jaw_h + tw.top_margin, bx = PV[0];
  const car = [
    box([bx + 22, -130, zc0], [bx + 40, 130, zc1], 'act', { rad: 4, name: 'xe con' }),
    box([bx + 40, -300, zc0 + 5], [bx + 56, 300, zc1 - 5], 'steel', { name: 'ray Y' }),
    box([bx + 22, 130, zc0 - 30], [bx + 40, 176, zc1 - 4], 'panel', { rad: 4, name: 'camera RGB-D' }),
    cyl('x', [bx + 41.5, 153, zc0 - 8], 9, 3, 'lens'),
  ];
  for (const sg of [-1, 1]) car.push(bxs(sg, [bx + 10, bx + 22], [74, 88], [zc0 + 5, zc1 - 5], 'steel', { name: 'con lăn xe con' }));
  add({ name: 'car', parent: 'mast3', conn: ['mast3'], free: ['mast3'], prims: car });
  const yc = [box([bx + 56, -196, zc0 + 8], [bx + 72, 196, zc1 - 8], 'act', { rad: 3, name: 'dầm bàn trượt Y' })];
  const xm = [], xi = [];
  const cx = g.x0, pl = cx - 225; // tam day kep khi s = 0; mat sau tam dau cong cu
  for (const sg of [-1, 1]) {
    yc.push(bxs(sg, [bx + 56, 310], [180, 196], [g.jaw_h + 2, zc1], 'steel', { name: 'ray X cố định' }));
    xm.push(bxs(sg, [-70, 315], [196, 208], [g.jaw_h + 2, zc1], 'aluDark', { name: 'tầng giữa ray X' }));
    xi.push(bxs(sg, [pl - 10, 315], [208, 220], [g.jaw_h + 2, zc1], 'alu', { name: 'tầng trong ray X' }));
    xi.push(bxs(sg, [pl, pl + 13], [220, 250], [g.jaw_h - 18, zc1], 'act', { name: 'tai tấm đầu' }));
    xi.push(bxs(sg, [pl + 13, pl + 23], [40, 250], [g.jaw_h - 38, g.jaw_h - 18], 'steel', { name: 'ray hàm (Y)' }));
  }
  xi.push(box([pl, -250, 205], [pl + 13, 250, g.jaw_h], 'aluDark', { rad: 3, name: 'tấm đầu công cụ' }));
  xi.push(box([pl + 13, -25, 60], [pl + 21, 25, zc1], 'extrusion', { name: 'cột hút tầng 1' }));
  add({ name: 'ycar', parent: 'car', free: ['car'], prims: yc });
  add({ name: 'xmid', parent: 'ycar', free: ['ycar'], prims: xm });
  add({ name: 'xin', parent: 'xmid', free: ['xmid'], prims: xi });
  let prev = 'xin';
  for (const [i, nm] of ['pmB', 'pmC', 'pmD'].entries()) {
    add({ name: nm, parent: prev, free: [prev, 'xin'], prims: [box([pl + 21 + i * 6, -25, 60], [pl + 27 + i * 6, 25, zc1], i === 2 ? 'alu' : 'aluDark', { name: 'cột hút tầng ' + (i + 2) })] });
    prev = nm;
  }
  add({
    name: 'parm', parent: 'pmD', conn: ['pmD'], free: ['pmD'], prims: [
      box([pl + 39, -20, 18], [pl + 47, 20, 35], 'act', { name: 'con trượt tay hút' }),
      box([pl + 47, -12, 22], [cx + 12, 12, 34], 'alu', { rad: 3, name: 'tay hút (ống nhôm 24×12)' }),
      cyl('z', [cx, 0, 25], 8, 10, 'hub'),
      cyl('z', [cx, 0, 10], 30, 20, 'cup', { r2: 22, name: 'cốc hút' }),
    ],
  });
  for (const sg of [-1, 1]) add({
    name: sg > 0 ? 'jawP' : 'jawN', parent: 'xin', free: ['xin'],
    prims: [
      bxs(sg, [cx - 150, cx + 150], [5, 13], [0, g.jaw_h], 'extrusion', { rad: 2, name: 'hàm' }),
      bxs(sg, [cx - 140, cx + 140], [0, 5], [15, g.jaw_h - 15], 'pad', { name: 'đệm hàm' }),
      bxs(sg, [pl + 23, cx - 150], [1, 13], [g.jaw_h - 42, g.jaw_h - 14], 'act', { name: 'tay nối hàm' }),
    ],
  });
  return links;
}

const LCACHE = new WeakMap();
export function links(role, R) {
  let c = LCACHE.get(R);
  if (!c) LCACHE.set(R, (c = {}));
  return (c[role] ??= role === 'TR' ? trLinks(R) : brLinks(R));
}

// Do ha khoi (mm) cua ngon duoi khi 2 dia lat Sky: 0 o dau/cuoi, 45 o giua (goc khoi nho len 41 mm)
export const flipDrop = (roll = 0) => 45 * Math.sin((Math.abs(roll % 90) / 90) * Math.PI);
export function brPivot(R) { return [R.BR.tower.base[0], 0, R.BR.chassis.z_top + 20]; }
// Do mo rong cac tang ong long (free lift): thap BR, cot hut BR, can TR
export function brMast(R, j) {
  const g = R.BR.gripper, tw = R.BR.tower;
  const e = Math.max(0, j.lz + g.jaw_h + tw.top_margin - (tw.base[1] + tw.stage1_h));
  const ep = Math.max(0, j.pf + 35 - (g.jaw_h + tw.top_margin));
  return { e, ep };
}

// Ma tran tung khau trong he robot (khong nghieng). pitch: goc nghieng than (rad, mui len > 0)
export function frames(role, R, j, pitch = 0, spin = 0) {
  const F = {};
  if (role === 'TR') {
    const T = R.TR, cl = T.clamp, b = T.boom, w = T.wheels, P0 = [cl.x0, 0, cl.z0];
    F.body = M.RY(-pitch);
    for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) F[`wheel${sx}${sy}`] = M.mul(F.body, M.T(sx * w.x, sy * w.y, w.diameter / 2), M.RY(spin));
    F.sledMid = M.mul(F.body, M.T(j.s / 2, 0, 0));
    F.sled = M.mul(F.body, M.T(j.s, 0, 0));
    F.zcar = M.mul(F.sled, M.T(0, 0, j.lz));
    F.clampLevel = M.mul(F.zcar, M.T(...P0), M.RY(-(j.cp || 0) * DEG), M.T(-P0[0], 0, -P0[2]));
    F.jawP = M.mul(F.clampLevel, M.T(0, j.w / 2, 0));
    F.jawN = M.mul(F.clampLevel, M.T(0, -j.w / 2, 0));
    F.turret = M.mul(F.body, M.T(b.turret[0], b.turret[1], 0), M.RZ(j.yaw * DEG));
    F.boomP = M.mul(F.turret, M.T(b.pivot_off || 0, 0, b.pivot_z), M.RY(-j.pitch * DEG));
    const e = j.len - T.boom.len[0];
    F.boomM = M.mul(F.boomP, M.T(e / 4, 0, 0));
    F.boomN = M.mul(F.boomP, M.T(e / 2, 0, 0));
    F.boomO = M.mul(F.boomP, M.T((3 * e) / 4, 0, 0));
    F.boomI = M.mul(F.boomP, M.T(e, 0, 0));
    const tip = M.ap(F.boomP, [j.len, 0, 0]);
    F.tool = M.mul(M.T(...tip), M.RZ((j.yaw + (j.tw || 0)) * DEG));
    F.cup = F.tool;
    const pz = -b.tool_drop - T.tool.pad_z, gg = j.g ?? T.tool.g_open, dr = flipDrop(j.roll);
    F.fP = M.mul(F.tool, M.T(0, gg / 2, 0));
    F.fN = M.mul(F.tool, M.T(0, -gg / 2, 0));
    F.fPlo = M.mul(F.fP, M.T(0, 0, -dr));
    F.fNlo = M.mul(F.fN, M.T(0, 0, -dr));
    F.fPpad = M.mul(F.fPlo, M.T(0, 0, pz), M.RY((j.roll || 0) * DEG));
    F.fNpad = M.mul(F.fNlo, M.T(0, 0, pz), M.RY((j.roll || 0) * DEG));
  } else {
    const B = R.BR, w = B.wheels, T = B.tracks, PV = brPivot(R), lift = T.drop * (j.track || 0);
    F.body = M.mul(M.RY(-pitch), M.T(0, 0, lift));
    for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) F[`wheel${sx}${sy}`] = M.mul(F.body, M.T(sx * w.x, sy * w.y, w.diameter / 2), M.RY(spin));
    const up = (T.raise ?? 40) * (1 - (j.track || 0)) - lift; // xich thu: day xich cao hon mat san `raise` mm (chi banh mecanum cham san)
    F.trackP = M.mul(F.body, M.T(0, T.y, up));
    F.trackN = M.mul(F.body, M.T(0, -T.y, up));
    F.level = M.mul(F.body, M.T(...PV), M.RY(-(j.tilt || 0) * DEG), M.T(-PV[0], 0, -PV[2]));
    const { e, ep } = brMast(R, j);
    F.mast2 = M.mul(F.level, M.T(0, 0, e / 2));
    F.mast3 = M.mul(F.level, M.T(0, 0, e));
    F.car = M.mul(F.level, M.T(0, 0, j.lz));
    F.ycar = M.mul(F.car, M.T(0, j.ly, 0));
    F.xmid = M.mul(F.ycar, M.T(j.s / 2, 0, 0));
    F.xin = M.mul(F.ycar, M.T(j.s, 0, 0));
    F.pmB = M.mul(F.xin, M.T(0, 0, ep / 3));
    F.pmC = M.mul(F.xin, M.T(0, 0, (2 * ep) / 3));
    F.pmD = M.mul(F.xin, M.T(0, 0, ep));
    F.parm = M.mul(F.xin, M.T(0, 0, j.pf));
    F.jawP = M.mul(F.xin, M.T(0, j.w / 2, 0));
    F.jawN = M.mul(F.xin, M.T(0, -j.w / 2, 0));
  }
  return F;
}

// Hop bao (trong he khau) cua 1 khoi
export function primBox(p, R) {
  if (p.k === 'box') return { a: p.a, b: p.b, ry: p.ry || 0 };
  if (p.k === 'cyl') {
    const r = Math.max(p.r, p.r2 ?? p.r), h = p.h / 2, [x, y, z] = p.c;
    const e = p.ax === 'x' ? [h, r, r] : p.ax === 'y' ? [r, h, r] : [r, r, h];
    return { a: [x - e[0], y - e[1], z - e[2]], b: [x + e[0], y + e[1], z + e[2]], ry: 0 };
  }
  if (p.k === 'mecanum') return { a: [-p.d / 2, -p.w / 2, -p.d / 2], b: [p.d / 2, p.w / 2, p.d / 2], ry: 0 };
  const T = R.BR.tracks; // xich
  return { a: [-360, -T.width / 2, 0], b: [330, T.width / 2, T.front_rise], ry: 0 };
}
// Hinh hop co huong (OBB) trong he robot: {c, u:[3 truc], e:[3 nua canh]}
export function primOBB(p, Mx, R, grow = 0) {
  const q = primBox(p, R);
  let c = [(q.a[0] + q.b[0]) / 2, (q.a[1] + q.b[1]) / 2, (q.a[2] + q.b[2]) / 2];
  let Ml = Mx;
  if (q.ry) Ml = M.mul(Mx, M.T(...c), M.RY(q.ry * DEG), M.T(-c[0], -c[1], -c[2]));
  return {
    c: M.ap(Ml, c),
    u: [M.apv(Ml, [1, 0, 0]), M.apv(Ml, [0, 1, 0]), M.apv(Ml, [0, 0, 1])],
    e: [(q.b[0] - q.a[0]) / 2 + grow, (q.b[1] - q.a[1]) / 2 + grow, (q.b[2] - q.a[2]) / 2 + grow],
  };
}
const obbAABB = (o) => {
  const h = [0, 1, 2].map((k) => Math.abs(o.u[0][k]) * o.e[0] + Math.abs(o.u[1][k]) * o.e[1] + Math.abs(o.u[2][k]) * o.e[2]);
  return { x0: o.c[0] - h[0], x1: o.c[0] + h[0], y0: o.c[1] - h[1], y1: o.c[1] + h[1], z0: o.c[2] - h[2], z1: o.c[2] + h[2] };
};

// ---------------------------------------------------------------- FK diem thao tac (he than) - sim dung
export function trClampBase(R, j) { // tam day cua khoi duoi trong kep TR (= tam ao khop can bang)
  const c = R.TR.clamp;
  return [c.x0 + j.s, 0, c.z0 + j.lz];
}
export function trBoomTip(R, j) {
  const b = R.TR.boom, cp = Math.cos(j.pitch * DEG), sp = Math.sin(j.pitch * DEG), cy = Math.cos(j.yaw * DEG), sy = Math.sin(j.yaw * DEG), h = (b.pivot_off || 0) + j.len * cp;
  return [b.turret[0] + h * cy, b.turret[1] + h * sy, b.pivot_z + j.len * sp];
}
// IK can cau: dau can (tip) toi diem p (he than). Tra ve khop + ok
export function trBoomIK(R, p) {
  const b = R.TR.boom, dx = p[0] - b.turret[0], dy = p[1] - b.turret[1], dz = p[2] - b.pivot_z;
  const h = Math.hypot(dx, dy) - (b.pivot_off || 0), len = Math.hypot(h, dz);
  const yaw = Math.atan2(dy, dx) / DEG, pitch = Math.atan2(dz, h) / DEG;
  const ok = len >= b.len[0] - 1 && len <= b.len[1] + 1 && pitch >= b.pitch[0] - 0.5 && pitch <= b.pitch[1] + 0.5;
  return { yaw, pitch: clamp(pitch, b.pitch[0], b.pitch[1]), len: clamp(len, b.len[0], b.len[1]), ok, need: { len, pitch } };
}
export function brGripBase(R, j) { // tam day kep BR trong he thap (thang dung nho khop nghieng)
  const g = R.BR.gripper;
  return [g.x0 + j.s, j.ly, j.lz];
}
// He thap BR (goc tai goc robot, thang dung) <-> he robot: xoay quanh chot nghieng + nang than khi ha xich
function brLevelM(R, j, pitch) {
  const PV = brPivot(R), lift = R.BR.tracks.drop * (j.track || 0);
  return M.mul(M.RY(-pitch), M.T(0, 0, lift), M.T(...PV), M.RY(-(j.tilt || 0) * DEG), M.T(-PV[0], 0, -PV[2]));
}
export const brLevelToRobot = (R, j, pitch, p) => M.ap(brLevelM(R, j, pitch), p);
export const brRobotToLevel = (R, j, pitch, p) => M.ap(M.inv(brLevelM(R, j, pitch)), p);

// ---------------------------------------------------------------- kich thuoc bao, va cham
// items: [{kind, hold}] hold: TR 'clamp'|'boom'; BR 'clamp'|'vac'. Hop trong HE ROBOT (khong nghieng).
export function parts(role, R, j, items = [], pitch = 0) {
  const F = frames(role, R, j, pitch), out = [];
  for (const l of links(role, R)) {
    const Mx = F[l.name];
    for (const p of l.prims) {
      const bb = obbAABB(primOBB(p, Mx, R));
      bb.name = p.name === 'hàm' ? 'hàm' : p.name || l.name; bb.link = l.name;
      out.push(bb);
    }
  }
  const bx = (c, h, z0, name) => out.push({ x0: c[0] - h, x1: c[0] + h, y0: c[1] - h, y1: c[1] + h, z0, z1: z0 + 2 * h, name });
  if (role === 'TR') {
    const base = M.ap(F.body, trClampBase(R, j)), tip = M.ap(F.boomP, [j.len, 0, 0]);
    let zc = base[2];
    for (const it of items) {
      const s = SIZE[it.kind];
      if (it.hold === 'clamp') { bx(base, s / 2, zc, 'vật:' + it.kind); zc += s; } else bx(tip, s / 2, tip[2] - R.TR.boom.tool_drop - s, 'vật:' + it.kind);
    }
  } else {
    const base = brLevelToRobot(R, j, pitch, brGripBase(R, j));
    let zc = base[2];
    for (const it of items) {
      const s = SIZE[it.kind];
      if (it.hold === 'clamp') { bx(base, s / 2, zc, 'vật:' + it.kind); zc += s; } else bx(base, s / 2, base[2] + j.pf - s, 'vật:' + it.kind);
    }
  }
  return out;
}
// Hop co huong (he robot) cua cac chi tiet co cau (khong gom than / banh / xich) - dung lay mau hinh chieu
export function mechOBBs(role, R, j, pitch = 0) {
  const F = frames(role, R, j, pitch), out = [];
  for (const l of links(role, R)) if (!/^(body|wheel|track)/.test(l.name)) for (const p of l.prims) out.push(primOBB(p, F[l.name], R));
  return out;
}
export function envelope(role, R, j, items = [], pitch = 0) {
  const box = { x0: Infinity, x1: -Infinity, y0: Infinity, y1: -Infinity, z1: -Infinity };
  for (const p of parts(role, R, j, items, pitch)) { box.x0 = Math.min(box.x0, p.x0); box.x1 = Math.max(box.x1, p.x1); box.y0 = Math.min(box.y0, p.y0); box.y1 = Math.max(box.y1, p.y1); box.z1 = Math.max(box.z1, p.z1); }
  return { ...box, L: box.x1 - box.x0, W: box.y1 - box.y0, H: box.z1 };
}

// Trong tam (he robot, khong nghieng) + tong khoi luong (kg) tu BOM tung khau, tinh ca vat dang mang
export function cog(role, R, j, items = []) {
  const MASS = { earth: 0.6, sky: 0.22, mustika: 0.27 };
  const lm = linkMass(role, R), cen = linkCentroids(role, R), F = frames(role, R, j, 0);
  let m = 0, x = 0, y = 0, z = 0;
  const add = (kg, p) => { m += kg; x += kg * p[0]; y += kg * p[1]; z += kg * p[2]; };
  for (const [k, kg] of Object.entries(lm)) if (F[k]) add(kg, M.ap(F[k], cen[k]));
  for (const b of parts(role, R, j, items)) if (b.name.startsWith('vật:')) add(MASS[b.name.slice(4)], [(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, (b.z0 + b.z1) / 2]);
  return { m, p: [x / m, y / m, z / m] };
}

// Goc nghieng toi han (do) truoc khi lat quanh mep da giac chong do theo huong x (+ toi / - lui)
export function tipAngle(role, R, j, items = []) {
  const { p } = cog(role, R, j, items);
  const r = R[role];
  const xf = role === 'BR' ? r.tracks.length / 2 : r.wheels.x, xb = -xf;
  return { fwd: Math.atan2(xf - p[0], p[2]) / DEG, back: Math.atan2(p[0] - xb, p[2]) / DEG, cog: p };
}

// ---------------------------------------------------------------- he truc robot <-> san
// pose: {x, y, z (cao do mat tiep xuc tai tam), h (rad), pitch (rad, mui ngang len > 0)}
export function toWorld(pose, p, level = false) {
  const cp = level ? 1 : Math.cos(pose.pitch), sp = level ? 0 : Math.sin(pose.pitch);
  const fx = p[0] * cp - p[2] * sp, fz = p[0] * sp + p[2] * cp;
  const ch = Math.cos(pose.h), sh = Math.sin(pose.h);
  return [pose.x + fx * ch - p[1] * sh, pose.y + fx * sh + p[1] * ch, pose.z + fz];
}
export function toRobot(pose, w, level = false) {
  const dx = w[0] - pose.x, dy = w[1] - pose.y, dz = w[2] - pose.z;
  const ch = Math.cos(pose.h), sh = Math.sin(pose.h);
  const fx = dx * ch + dy * sh, py = -dx * sh + dy * ch;
  const cp = level ? 1 : Math.cos(pose.pitch), sp = level ? 0 : Math.sin(pose.pitch);
  return [fx * cp + dz * sp, py, -fx * sp + dz * cp];
}
