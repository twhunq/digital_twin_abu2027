// Mo phong tran dau v2 - muc co cau (kinematic): moi thao tac la chuoi pha khop voi thoi gian tinh tu
// toc do co cau trong robot_spec.json; vat duoc mang dung vi tri dau kep / dau hut (FK); kiem tra luat,
// kich thuoc bao, va cham than robot voi san / khoi / robot khac; sai so dat khoi co the chinh.
// TR va BR KHONG trao doi thong tin (11.10): BR chi "nhin" Transfer Area, TR chi "nhin" san.

import { Geo, rectCenter } from './geo.js';
import * as K from './robots.js';
import { TR_NODES, TR_EDGES, BR_NODES, BR_EDGES, PIVOTS, SPOT_STAND, SPOT_STAND_L1, TA_SLOTS, STORE_SLIDE, SKY_PLAN, HANDOVER, DEFAULT_PARAMS, PLANS } from './plans.js';

const SIZE = K.SIZE, DEG = K.DEG;
const TEAMS = ['red', 'blue'];
const UP = { red: '+z', blue: '-z' };
const TEAM_VN = { red: 'Đỏ', blue: 'Xanh' };
const angDiff = (a, b) => { let d = b - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return d; };
const hOf = (it) => SIZE[it.kind];

export function applyKnobs(R, knobs = {}) {
  const c = JSON.parse(JSON.stringify(R));
  for (const [path, v] of Object.entries(knobs)) {
    const ks = path.split('.'), last = ks.pop();
    let o = c;
    for (const k of ks) o = o[k];
    o[last] = v;
  }
  return c;
}
function rng(seed) { // mulberry32
  let a = seed >>> 0;
  return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

export class World {
  constructor(spec, cfg, robotSpec) {
    this.spec = spec;
    this.geo = new Geo(spec);
    this.t = 0;
    this.T = spec.scoring.match_seconds;
    this.over = false;
    this.events = [];
    this.violations = [];
    this.collisions = [];
    this.items = [];
    this.locks = {};
    this._vKeys = {};
    this.spots = Object.fromEntries(spec.building_spots.map((s) => [s.id, { def: s, items: [], claimedBy: null }]));
    this.teams = {};
    for (const team of TEAMS) {
      const c = cfg[team] || {};
      const params = { ...DEFAULT_PARAMS, ...(c.params || {}) };
      this.teams[team] = {
        team, planKey: c.plan || 'E', plan: PLANS[c.plan || 'E'], params, R: applyKnobs(robotSpec, c.knobs),
        rand: rng(params.seed * 7919 + (team === 'red' ? 1 : 2)),
        transfer: 0, delivered: { E: 0, S: 0 }, lost: { E: 0, S: 0 }, mandate: false, mandateT: null, mustikaT: null,
        mustikaHanded: false, mustikaReady: false, trBlocked: false, storage: [], ta: [], misplaced: 0, requeue: [],
      };
    }
    this._initObjects();
    this.robots = [];
    for (const team of TEAMS) {
      this.robots.push(this._makeRobot(team, 'TR', 'T_S1')); // TR o R1 (sat bien), BR o R2 (gan cau thang): duong di khong cat nhau
      this.robots.push(this._makeRobot(team, 'BR', 'B_S2'));
    }
    for (const r of this.robots) {
      r.gen = r.role === 'TR' ? this._trBrain(r) : this._brBrain(r);
      const env = K.envelope(r.role, r.R, r.j, []);
      r.startEnv = r.env = env;
      if (env.L > 700.5 || env.W > 700.5 || env.H > 700.5) this._viol(r, 'start', `kích thước xuất phát ${Math.round(env.L)}×${Math.round(env.W)}×${Math.round(env.H)} > 700³ (11.4)`);
    }
    this.log(null, 'Bắt đầu trận: Đỏ = ' + this.teams.red.plan.name + ' | Xanh = ' + this.teams.blue.plan.name);
  }

  // ------------------------------------------------------------------ khoi tao
  _initObjects() {
    const S = this.spec, g = this.geo;
    let n = 0;
    const es = S.initial_layout.earth_storage_default;
    for (const team of TEAMS) {
      const tm = this.teams[team], cols = [];
      es.rows_y.forEach((y, row) => es.red_columns_x.forEach((lx, i) => cols.push({ i, row, lx, x: g.mx(team, lx), y, items: [] })));
      for (const col of cols) for (let k = 0; k < es.stack; k++) {
        const it = { id: 'E' + (n++), kind: 'earth', team, x: col.x, y: col.y, z: k * SIZE.earth, yaw: 0, pitch: 0, where: 'storage', holder: null };
        col.items.push(it); this.items.push(it);
      }
      tm.storage = cols;
      tm.ta = TA_SLOTS.map((s) => ({ id: s.id, x: g.mx(team, s.x), y: s.y, br: s.br, items: [] }));
    }
    const sg = S.initial_layout.sky_grid, [ix0, , , iy1] = sg.inner_rect;
    this.skyCells = {};
    for (const b of sg.blocks) {
      const it = { id: `S${b.r}${b.c}`, kind: 'sky', team: null, x: ix0 + b.c * sg.cell + sg.cell / 2, y: iy1 - b.r * sg.cell - sg.cell / 2, z: 0, yaw: 0, pitch: 0, dir: b.red_dir, where: 'grid', cell: [b.r, b.c], holder: null };
      this.skyCells[`${b.r},${b.c}`] = it; this.items.push(it);
    }
    const mp = S.pillars.find((p) => p.id === 'MUSTIKA_PILLAR');
    this.ball = { id: 'MUSTIKA', kind: 'mustika', team: null, x: mp.center[0], y: mp.center[1], z: this._ballZ(mp), yaw: 0, pitch: 0, where: 'pillar', holder: null };
    this.items.push(this.ball);
  }
  _ballZ(p) { const r = SIZE.mustika / 2, a = p.cup_diameter / 2; return p.z0 + p.height + Math.sqrt(r * r - a * a) - r; }

  _makeRobot(team, role, startNode) {
    const tm = this.teams[team];
    const graph = this._buildGraph(team, role);
    const [x, y, h] = graph.nodes[startNode];
    const r = { id: `${team}-${role}`, team, role, R: tm.R, x, y, z: 0, h, pitch: 0, j: K.stowJoints(role, tm.R), carry: [], node: startNode, graph, task: null, label: 'Sẵn sàng', dist: 0, wheel: 0, track: 0, ignore: new Set(), climbing: false };
    this._setPose(r);
    return r;
  }

  _buildGraph(team, role) {
    const g = this.geo, W = (p) => [g.mx(team, p[0]), p[1], p[2] == null ? null : g.mh(team, p[2])];
    const nodes = {}, edges = [];
    for (const [k, p] of Object.entries(role === 'TR' ? TR_NODES : BR_NODES)) nodes[k] = W(p);
    for (const e of role === 'TR' ? TR_EDGES : BR_EDGES) edges.push({ a: e[0], b: e[1], kind: e[2] || 'flat' });
    if (role === 'TR') {
      // diem dung lay Earth o Storage: cot 1..4 tiep can tu phia nam (+y), cot sat tuong (0) tu phia dong (-x)
      const es = this.spec.initial_layout.earth_storage_default, d = this.teams[team].R.TR.clamp.x0 + STORE_SLIDE;
      es.rows_y.forEach((y, row) => es.red_columns_x.forEach((lx, i) => {
        const k = `STO_${i}_${row}`;
        nodes[k] = i === 0 ? W([lx + d, y, Math.PI]) : W([lx, y - d, Math.PI / 2]);
        edges.push({ a: k, b: 'STO_HUB', kind: 'flat' });
        // hang gan (row 0), cot 1..4: di thang tu chan doc (bo duong vong qua STO_HUB, ~0,6 m / chuyen)
        if (row === 0 && i > 0) edges.push({ a: k, b: 'RP_BOT', kind: 'flat' });
      }));
      for (let i = 1; i < 5; i++) for (let jj = i + 1; jj < 5; jj++) for (const row of [0, 1]) edges.push({ a: `STO_${i}_${row}`, b: `STO_${jj}_${row}`, kind: 'flat' });
    }
    const adj = {};
    for (const k of Object.keys(nodes)) adj[k] = [];
    for (const e of edges) {
      const [ax, ay] = nodes[e.a], [bx, by] = nodes[e.b];
      e.len = Math.hypot(bx - ax, by - ay);
      e.rise = Math.abs(g.floorZ(bx, by) - g.floorZ(ax, ay));
      adj[e.a].push({ to: e.b, e }); adj[e.b].push({ to: e.a, e });
    }
    return { nodes, adj };
  }

  log(team, msg) { this.events.push({ t: this.t, team, msg }); }

  // ------------------------------------------------------------------ tu the robot tren nen san
  // Mat tiep xuc (banh / xich): lay diem tua cao nhat o nua truoc va nua sau.
  _poseAt(r, x, y, h) {
    const g = this.geo, c = Math.cos(h), s = Math.sin(h);
    if (r.role === 'TR') { // 4 banh cung: moi banh cham diem cao nhat duoi vet banh
      const W = r.R.TR.wheels, rw = W.diameter / 2;
      const foot = (u0) => { let z = -1e9; for (let u = u0 - rw; u <= u0 + rw + 1e-6; u += rw / 4) z = Math.max(z, g.floorZ(x + u * c, y + u * s)); return z; };
      const zf = foot(W.x), zb = foot(-W.x), pitch = Math.atan2(zf - zb, 2 * W.x);
      return { x, y, h, pitch, z: (zf + zb) / 2 };
    }
    { // BR: day xich thang (mui vat len 150 mm tu x = 200, duoi vat tu x = -300) tua len bao loi tren cua nen
      const pts = [];
      for (let u = -360; u <= 330 + 1e-6; u += 10) {
        const lift = u > 200 ? ((u - 200) / 130) * 150 : u < -300 ? -300 - u : 0;
        pts.push([u, g.floorZ(x + u * c, y + u * s) - lift]);
      }
      const hull = [];
      for (const p of pts) {
        while (hull.length >= 2) { const a = hull[hull.length - 2], b = hull[hull.length - 1]; if ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0) hull.pop(); else break; }
        hull.push(p);
      }
      let i = 0;
      while (i < hull.length - 2 && hull[i + 1][0] <= 0) i++;
      const a = hull[i], b = hull[i + 1], pitch = Math.atan2(b[1] - a[1], b[0] - a[0]);
      return { x, y, h, pitch, z: a[1] + (0 - a[0]) * Math.tan(pitch) };
    }
  }
  _setPose(r) {
    const p = this._poseAt(r, r.x, r.y, r.h);
    r.z = p.z; r.pitch = p.pitch;
    if (r.role === 'BR') r.j.tilt = Math.max(-30, Math.min(30, -p.pitch / DEG)); // thap tu can bang
    else r.j.cp = Math.max(-12, Math.min(12, -p.pitch / DEG));                       // kep TR tu can bang
  }
  pose(r) { return { x: r.x, y: r.y, z: r.z, h: r.h, pitch: r.pitch }; }

  // ------------------------------------------------------------------ duong di
  _speed(r) { const d = r.R[r.role].drive; return { v: d.v_max * 1000, a: d.a_max * 1000, wr: d.turn_rate }; }
  _climbDur(r, e) { const T = r.R.BR.tracks; return 2 * T.t_deploy + Math.hypot(e.len, e.rise) / (T.v_climb * 1000) + T.t_crest; }
  _edgeCost(r, e) {
    const { v, a } = this._speed(r);
    if (e.kind === 'stairs') return r.role === 'BR' ? this._climbDur(r, e) : Infinity;
    if (e.kind === 'ramp') { const d = r.R.TR.drive; return e.len / (d.v_ramp * 1000) + d.v_ramp / d.a_ramp; }
    return e.len / v + v / a;
  }
  _route(r, goal) {
    const { adj } = r.graph, dist = { [r.node]: 0 }, prev = {}, done = new Set();
    while (true) {
      let u = null, best = Infinity;
      for (const [k, d] of Object.entries(dist)) if (!done.has(k) && d < best) { best = d; u = k; }
      if (u == null) return null;
      if (u === goal) break;
      done.add(u);
      for (const { to, e } of adj[u]) {
        const c = this._edgeCost(r, e);
        if (!isFinite(c)) continue;
        if (dist[to] == null || best + c < dist[to]) { dist[to] = best + c; prev[to] = { u, e }; }
      }
    }
    const path = [];
    for (let k = goal; k !== r.node; k = prev[k].u) path.unshift({ from: prev[k].u, to: k, e: prev[k].e });
    return { path, cost: dist[goal] };
  }
  routeCost(r, goal) { if (r.node === goal) return 0; const rt = this._route(r, goal); return rt ? rt.cost : Infinity; }

  _moveTask(r, goal) {
    const { v, a, wr } = this._speed(r);
    const N = r.graph.nodes, segs = [];
    let h = r.h;
    const turn = (x, y, h1) => { const d = Math.abs(angDiff(h, h1)); if (d > 0.03) segs.push({ kind: 'turn', x0: x, y0: y, x1: x, y1: y, h0: h, h1, dur: d / wr }); h = h1; };
    const route = r.node === goal ? { path: [] } : this._route(r, goal);
    if (!route) return null;
    // nang kep truoc khi leo cau thang (BR) / len doc (TR): ham khong cham bac, mat doc
    const climbLz = 440 + (r.carry.some((i) => i.kind === 'mustika') ? 70 : 0);
    const needLift = r.role === 'BR' ? (route.path.some((st) => st.e.kind === 'stairs') && r.j.lz < climbLz - 10 ? climbLz : 0) // dau kep (thap tu can bang) khong quet vao than khi nghieng 27 do
      : (route.path.some((st) => st.e.kind === 'ramp') && r.j.lz < 40 ? 40 : 0);
    if (needLift) { const to = { lz: needLift }; segs.push({ kind: 'joint', x0: r.x, y0: r.y, x1: r.x, y1: r.y, h0: h, h1: h, from: null, to, dur: K.jointTime(r.role, r.R, r.j, to) }); }
    if (needLift && r.role === 'BR') { // ha tru hut sat vat dang mang (robot khong vuot 1200 mm khi xich nang than)
      const vac = r.carry.find((i) => i.hold === 'vac'), clampH = r.carry.filter((i) => i.hold === 'clamp').reduce((a, i) => a + hOf(i), 0);
      const pfT = Math.max(r.R.BR.gripper.plunger[0], vac ? clampH + hOf(vac) : clampH + 10);
      if (pfT < r.j.pf - 1) segs.push({ kind: 'joint', x0: r.x, y0: r.y, x1: r.x, y1: r.y, h0: h, h1: h, from: null, to: { pf: pfT }, dur: K.jointTime('BR', r.R, r.j, { pf: pfT }) });
    }
    route.path.forEach((st, i) => {
      const [x0, y0] = N[st.from], [x1, y1] = N[st.to], d = st.e.len, dir = Math.atan2(y1 - y0, x1 - x0), k = st.e.kind;
      if (k === 'flat') {
        const nxt = route.path[i + 1];
        let h1 = h;
        if (!nxt) h1 = N[st.to][2] ?? h;
        else if (nxt.e.kind !== 'flat' && !PIVOTS.has(st.to)) { const [nx0, ny0] = N[nxt.from], [nx1, ny1] = N[nxt.to]; h1 = Math.atan2(ny1 - ny0, nx1 - nx0); }
        if (PIVOTS.has(st.from)) turn(x0, y0, h1);
        const trap = d >= (v * v) / a ? d / v + v / a : 2 * Math.sqrt(d / a);
        segs.push({ kind: k, x0, y0, x1, y1, h0: h, h1, dur: Math.max(trap, Math.abs(angDiff(h, h1)) / wr), d, v, a });
        h = h1;
      } else {
        // doc / cau thang: di thang theo huong canh (lui neu can)
        const fwd = Math.abs(angDiff(h, dir)) < Math.PI / 2 ? dir : dir + Math.PI;
        turn(x0, y0, fwd);
        let dur;
        if (k === 'ramp') { const dr = r.R.TR.drive, vr = dr.v_ramp * 1000, ar = dr.a_ramp * 1000; dur = d / vr + vr / ar; }
        else dur = this._climbDur(r, st.e);
        segs.push({ kind: k, x0, y0, x1, y1, h0: fwd, h1: fwd, dur, lock: k === 'stairs' ? 'stairs' : null });
      }
    });
    const hf = N[goal][2];
    if (hf != null) turn(N[goal][0], N[goal][1], hf);
    return { type: 'move', goal, segs: this._chain(r, segs), i: 0, tau: 0, label: 'Di chuyển → ' + goal };
  }
  // Lap quy dao lien mach (kieu may CNC): gop cac doan phang / doc / xoay lien tiep thanh 1 doan 'path'. Van toc qua moi goc
  // gioi han theo do gat cua goc (v_j = vmax (1 + cos goc) / 2), quet xuoi / nguoc de ton trong gia toc tung manh (doc: a_ramp),
  // than xoay trong luc chay (mecanum); manh doc giu huong co dinh nen phan xoay dat vao manh phang truoc do.
  _chain(r, segs) {
    const D = r.R[r.role].drive, out = [];
    let run = [];
    const flush = () => {
      if (!run.length) return;
      // xoay tai cho giua cac manh -> chuyen vao manh phang lien truoc (neu co) hoac lien sau
      const pieces = [];
      let pendH = null;
      for (const sg of run) {
        if (sg.kind === 'turn') {
          const last = pieces[pieces.length - 1];
          if (last && last.kind === 'flat') last.h1 = sg.h1; else pendH = sg;
          continue;
        }
        const pts = sg.kind === 'poly' ? sg.pts : [[sg.x0, sg.y0], [sg.x1, sg.y1]];
        for (let i = 1; i < pts.length; i++) {
          const ramp = sg.kind === 'ramp';
          pieces.push({ kind: ramp ? 'ramp' : 'flat', x0: pts[i - 1][0], y0: pts[i - 1][1], x1: pts[i][0], y1: pts[i][1], h0: sg.h0, h1: i === pts.length - 1 ? sg.h1 : sg.h0,
            v: (ramp ? D.v_ramp : D.v_max) * 1000, a: (ramp ? D.a_ramp : D.a_max) * 1000 });
        }
      }
      if (pendH && !pieces.length) { out.push(pendH); run = []; return; }
      if (pendH) { if (pieces[0].kind === 'flat') pieces[0].h0 = pendH.h0; else out.push(pendH); }
      for (let i = 1; i < pieces.length; i++) pieces[i].h0 = pieces[i - 1].h1;
      // manh doc can dung huong ngay khi vao: manh phang truoc do xoay xong; neu khong co -> xoay tai cho
      for (let i = 0; i < pieces.length; i++) if (pieces[i].kind === 'ramp' && i > 0 && pieces[i - 1].kind === 'flat') pieces[i - 1].h1 = pieces[i].h0;
      for (const pc of pieces) pc.L = Math.hypot(pc.x1 - pc.x0, pc.y1 - pc.y0);
      const n = pieces.length;
      // profile van toc (vj: van toc tai cac moi noi; stop[i] = true -> dung han tai moi noi i)
      const stop = new Array(n + 1).fill(false);
      const profileAll = () => {
        const vj = new Array(n + 1).fill(0);
        for (let i = 1; i < n; i++) {
          if (stop[i]) continue;
          const A = pieces[i - 1], B = pieces[i], ca = ((A.x1 - A.x0) * (B.x1 - B.x0) + (A.y1 - A.y0) * (B.y1 - B.y0)) / Math.max(1e-9, A.L * B.L);
          vj[i] = Math.min(A.v, B.v) * Math.max(0, (1 + ca) / 2);
        }
        for (let i = 0; i < n; i++) vj[i + 1] = Math.min(vj[i + 1], Math.sqrt(vj[i] ** 2 + 2 * pieces[i].a * pieces[i].L));
        for (let i = n - 1; i >= 0; i--) vj[i] = Math.min(vj[i], Math.sqrt(vj[i + 1] ** 2 + 2 * pieces[i].a * pieces[i].L));
        for (let i = 0; i < n; i++) {
          const pc = pieces[i], vs = vj[i], ve = vj[i + 1], a = pc.a;
          const vp = Math.min(pc.v, Math.sqrt((2 * a * pc.L + vs * vs + ve * ve) / 2));
          const ta = (vp - vs) / a, td = (vp - ve) / a, da = (vp * vp - vs * vs) / (2 * a), dd = (vp * vp - ve * ve) / (2 * a);
          Object.assign(pc, { vs, ve, vp, ta, td, tc: Math.max(0, (pc.L - da - dd) / Math.max(1e-9, vp)), da, dd });
          pc.tm = pc.ta + pc.tc + pc.td;
        }
      };
      profileAll();
      // Xep lich xoay than: moi quang phang (giua cac manh doc) doi huong tu hs -> he. Thu lan luot: manh dai truoc, cua so
      // [ca manh | 40% dau | 40% cuoi]; kiem hinh chieu than voi tuong / bac / barrier / cot / khoi tren san. Khong phuong an nao sach
      // -> dung han cuoi quang, xoay tai cho (nhu diem xoay cu).
      const wr = D.turn_rate * 0.8;
      // hold: 'end' -> dung xoay tai cho o cuoi quang; 'start' -> xoay tai cho truoc khi chay
      const setRun = (k, e, hs, he, q, f0, f1, hold) => {
        for (let i = k; i <= e; i++) {
          const pc = pieces[i];
          pc.h0 = i <= q ? hs : he; pc.h1 = i < q ? hs : he; pc.hold = 0; pc.holdStart = false; pc.f0 = 0; pc.f1 = 1;
          if (i === q) { pc.f0 = f0; pc.f1 = f1; }
        }
        if (typeof hold === 'number') { // xoay tai cho o moi noi `hold` (k..e+1)
          for (let i = k; i <= e; i++) { pieces[i].h0 = pieces[i].h1 = i < hold ? hs : he; }
          if (hold <= e) { const pc = pieces[hold]; pc.h0 = hs; pc.hold = Math.abs(angDiff(hs, he)) / wr + 0.2; pc.holdStart = true; }
          else { const pc = pieces[e]; pc.h1 = he; pc.hold = Math.abs(angDiff(hs, he)) / wr + 0.2; }
        }
      };
      const runs = [];
      for (let k = 0; k < n; ) {
        if (pieces[k].kind === 'ramp') { pieces[k].hold = 0; pieces[k].f0 = 0; pieces[k].f1 = 1; k++; continue; }
        let e = k; while (e + 1 < n && pieces[e + 1].kind === 'flat') e++;
        runs.push([k, e, pieces[k].h0, pieces[e].h1]); k = e + 1;
      }
      for (const [k, e, hs, he] of runs) {
        const dh = Math.abs(angDiff(hs, he));
        if (dh < 1e-3) { setRun(k, e, hs, hs, -1, 0, 1, false); continue; }
        const need = dh / wr + 0.2, order = [];
        for (let q = k; q <= e; q++) order.push(q);
        order.sort((x, y) => pieces[y].L - pieces[x].L);
        let ok = false;
        for (const q of order) {
          const pq = pieces[q], v0 = pq.v;
          for (const [f0, f1] of [[0, 1], [0, 0.8], [0.2, 1], [0, 0.6], [0.4, 1], [0, 0.4], [0.6, 1]]) {
            // cua so ngan hon thoi gian xoay -> giam van toc manh (toi ~1/6) de xoay kip, van re hon dung han xoay tai cho
            for (let it = 0; (f1 - f0) * pq.tm < need && it < 8; it++) { pq.v *= 0.8; profileAll(); }
            if ((f1 - f0) * pq.tm >= need) {
              setRun(k, e, hs, he, q, f0, f1, false);
              if (this._runClear(r, pieces, k, e)) { ok = true; break; }
            }
            if (pq.v !== v0) { pq.v = v0; profileAll(); }
          }
          if (ok) break;
        }
        const juncs = [e + 1, k]; for (let m = e; m > k; m--) juncs.push(m);
        for (const m of juncs) {
          if (ok) break;
          setRun(k, e, hs, he, -1, 0, 1, m);
          stop[m] = true; profileAll();
          if (this._runClear(r, pieces, k, e)) ok = true; else stop[m] = false;
        }
        if (!ok) { setRun(k, e, hs, he, order[0], 0, 1, false); profileAll(); } // khong phuong an nao sach: xoay suot doan dai nhat
      }
      profileAll();
      let t = 0;
      for (const pc of pieces) { pc.t0 = t; pc.dur = pc.tm + (pc.hold || 0); t += pc.dur; }
      const f = pieces[0], l = pieces[n - 1];
      out.push({ kind: 'path', pieces, dur: t, d: pieces.reduce((a2, pc) => a2 + pc.L, 0), x0: f.x0, y0: f.y0, x1: l.x1, y1: l.y1, h0: f.h0, h1: l.h1 });
      run = [];
    };
    for (const sg of segs) {
      if (sg.kind === 'flat' || sg.kind === 'ramp' || sg.kind === 'poly' || sg.kind === 'turn') run.push(sg); // moi lenh xoay deu duoc xep lich + kiem
      else { flush(); out.push(sg); }
    }
    flush();
    return out;
  }
  // Vi tri + huong tai thoi diem u (tu dau manh): tinh tien theo profile vs -> vp -> ve, sau do (neu co) dung xoay tai cho
  _piecePose(pc, u) {
    if (pc.holdStart && pc.hold) {
      if (u < pc.hold) return { x: pc.x0, y: pc.y0, h: pc.h0 + angDiff(pc.h0, pc.h1) * K.trapS(u / pc.hold, 0.3) };
      return this._piecePose({ ...pc, holdStart: false, hold: 0, h0: pc.h1 }, u - pc.hold);
    }
    const um = Math.min(u, pc.tm);
    let dist;
    if (um < pc.ta) dist = pc.vs * um + 0.5 * ((pc.vp - pc.vs) / Math.max(1e-9, pc.ta)) * um * um;
    else if (um < pc.ta + pc.tc) dist = pc.da + pc.vp * (um - pc.ta);
    else { const w = um - pc.ta - pc.tc; dist = pc.da + pc.vp * pc.tc + pc.vp * w - 0.5 * ((pc.vp - pc.ve) / Math.max(1e-9, pc.td)) * w * w; }
    const k = pc.L > 0 ? Math.min(1, Math.max(0, dist / pc.L)) : 1;
    let h = pc.h1;
    if (Math.abs(angDiff(pc.h0, pc.h1)) > 1e-3) {
      if (pc.hold) h = u <= pc.tm ? pc.h0 : pc.h0 + angDiff(pc.h0, pc.h1) * K.trapS((u - pc.tm) / pc.hold, 0.3);
      else { const r0 = pc.f0 * pc.tm, r1 = pc.f1 * pc.tm; h = pc.h0 + angDiff(pc.h0, pc.h1) * K.trapS((u - r0) / Math.max(1e-9, r1 - r0), 0.3); }
    }
    return { x: pc.x0 + (pc.x1 - pc.x0) * k, y: pc.y0 + (pc.y1 - pc.y0) * k, h };
  }
  // Quang phang k..e sach vat can voi lich xoay hien tai? (than than robot + 30 mm, lay mau 0,05 s)
  _runClear(r, pieces, k, e) {
    const g = this.geo, c = r.R[r.role].chassis, L = c.length / 2, W = c.width / 2, R2 = Math.hypot(L, W) + 50;
    // chi xet vat can gan quang duong
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = k; i <= e; i++) { const p = pieces[i]; x0 = Math.min(x0, p.x0, p.x1); x1 = Math.max(x1, p.x0, p.x1); y0 = Math.min(y0, p.y0, p.y1); y1 = Math.max(y1, p.y0, p.y1); }
    x0 -= R2; x1 += R2; y0 -= R2; y1 += R2;
    const items = this.items.filter((it) => !it.holder && it.where !== 'lost' && it.x > x0 - 200 && it.x < x1 + 200 && it.y > y0 - 200 && it.y < y1 + 200);
    // chi tiet co cau + vat dang mang nho ra ngoai khung (can do ngang, dau cong cu...): khop dung yen khi chay -> tinh 1 lan
    const over = K.parts(r.role, r.R, r.j, r.carry).filter((p) => !/^(body|wheel|track)/.test(p.link || '') && (p.x0 < -L || p.x1 > L || p.y0 < -W || p.y1 > W));
    const R3 = over.reduce((m, p) => Math.max(m, Math.hypot(Math.max(-p.x0, p.x1), Math.max(-p.y0, p.y1))), 0);
    const items2 = R3 > 0 ? this.items.filter((it) => !it.holder && it.where !== 'lost' && it.x > x0 - R3 - 200 && it.x < x1 + R3 + 200 && it.y > y0 - R3 - 200 && it.y < y1 + R3 + 200) : [];
    const overHit = (q, zc) => {
      const ch = Math.cos(q.h), sh = Math.sin(q.h);
      for (const p of over) {
        const cs = [[p.x0, p.y0], [p.x1, p.y0], [p.x0, p.y1], [p.x1, p.y1]].map(([a, b]) => [q.x + a * ch - b * sh, q.y + a * sh + b * ch]);
        const wx0 = Math.min(...cs.map((c) => c[0])) - 10, wx1 = Math.max(...cs.map((c) => c[0])) + 10, wy0 = Math.min(...cs.map((c) => c[1])) - 10, wy1 = Math.max(...cs.map((c) => c[1])) + 10;
        const z0 = zc + p.z0, z1 = zc + p.z1;
        for (const it of items2) { const hs = hOf(it) / 2; if (wx1 > it.x - hs && wx0 < it.x + hs && wy1 > it.y - hs && wy0 < it.y + hs && z1 > it.z + 5 && z0 < it.z + 2 * hs) return true; }
        for (const c of [...cs, [(wx0 + wx1) / 2, (wy0 + wy1) / 2]]) if (this._obstacleHit(c[0], c[1], z0, z1)) return true;
      }
      return false;
    };
    for (let i = k; i <= e; i++) {
      const pc = pieces[i];
      if (Math.abs(angDiff(pc.h0, pc.h1)) < 1e-3) continue; // huong khong doi: tuyen goc da thiet ke khong va cham
      let u0 = 0, u1 = pc.tm + (pc.hold || 0);
      if (pc.hold) { if (pc.holdStart) u1 = pc.hold; else u0 = pc.tm; }
      else { u0 = Math.max(0, pc.f0 * pc.tm - 0.05); u1 = Math.min(pc.tm, pc.f1 * pc.tm + 0.05); }
      for (let u = u0; u <= u1 + 1e-9; u += 0.03) {
        const q = this._piecePose(pc, u), ch = Math.cos(q.h), sh = Math.sin(q.h), zc = g.floorZ(q.x, q.y);
        if (over.length && overHit(q, zc)) return false;
        for (let a = -L; a <= L + 1e-6; a += L / 7) for (let b = -W; b <= W + 1e-6; b += W / 8) { // cung luoi voi _chassisSamples
          if (r.role === 'TR' && a > c.u_cut.x0 && Math.abs(b) < c.u_cut.half_w) continue;
          const x = q.x + a * ch - b * sh, y = q.y + a * sh + b * ch;
          if (g.floorZ(x, y) > zc + 30) return false;
          if (g.inOpponent(r.team, x, y) || (r.role === 'TR' && g.trForbidden(x, y))) return false; // luat 6.2.1, 6.2.2
          if (this._obstacleHit(x, y, zc, zc + c.z_top)) return false;
          if (items.length && this._itemHit(x, y, zc, zc + c.z_top, new Set(), items)) return false;
        }
      }
    }
    return true;
  }
  // Thoi diem (tu dau manh) robot di duoc quang duong `dist` tren manh (profile vs -> vp -> ve)
  _pieceTime(pc, dist) {
    if (dist <= 0) return 0;
    if (dist < pc.da) { const aa = (pc.vp - pc.vs) / Math.max(1e-9, pc.ta); return aa > 1e-9 ? (-pc.vs + Math.sqrt(pc.vs * pc.vs + 2 * aa * dist)) / aa : dist / Math.max(1e-9, pc.vs); }
    if (dist < pc.da + pc.vp * pc.tc) return pc.ta + (dist - pc.da) / pc.vp;
    const w0 = dist - pc.da - pc.vp * pc.tc, ad = (pc.vp - pc.ve) / Math.max(1e-9, pc.td), q = pc.vp * pc.vp - 2 * ad * w0;
    return pc.ta + pc.tc + (ad > 1e-9 ? (pc.vp - Math.sqrt(Math.max(0, q))) / ad : w0 / pc.vp);
  }
  // (cu) Gop cac doan thang lien tiep khong xoay than thanh 1 quy dao: 1 profile van toc, qua goc khong dung han
  // (+0,15 s moi goc cua) - dung voi mecanum bam duong lien tuc.
  _blend(segs, v, a) {
    const out = [];
    for (const s of segs) {
      const prev = out[out.length - 1];
      const straight = s.kind === 'flat' && Math.abs(angDiff(s.h0, s.h1)) < 1e-3;
      if (straight && prev && (prev.kind === 'poly' || (prev.kind === 'flat' && Math.abs(angDiff(prev.h0, prev.h1)) < 1e-3)) && Math.abs(angDiff(prev.h1, s.h0)) < 1e-3) {
        const P = prev.kind === 'poly' ? prev : { kind: 'poly', pts: [[prev.x0, prev.y0], [prev.x1, prev.y1]], h0: prev.h0, h1: prev.h1, x0: prev.x0, y0: prev.y0 };
        P.pts.push([s.x1, s.y1]); P.x1 = s.x1; P.y1 = s.y1;
        out[out.length - 1] = P;
        continue;
      }
      out.push(s);
    }
    for (const s of out) if (s.kind === 'poly') {
      s.cum = [0];
      for (let i = 1; i < s.pts.length; i++) s.cum.push(s.cum[i - 1] + Math.hypot(s.pts[i][0] - s.pts[i - 1][0], s.pts[i][1] - s.pts[i - 1][1]));
      s.d = s.cum[s.cum.length - 1]; s.v = v; s.a = a;
      s.dur = (s.d >= (v * v) / a ? s.d / v + v / a : 2 * Math.sqrt(s.d / a)) + 0.15 * (s.pts.length - 2);
    }
    return out;
  }

  // ------------------------------------------------------------------ vong lap
  step(dt) {
    if (this.over) return;
    const dtt = Math.min(dt, this.T - this.t);
    this.t += dtt;
    this._tick = (this._tick || 0) + 1;
    const order = this._tick % 2 ? this.robots : [...this.robots].reverse();
    for (const r of order) this._advance(r, dtt);
    for (const r of this.robots) this._syncCarry(r);
    if (this._tick % 4 === 0) { this._checkRules(); this._checkPhysics(); }
    if (this.t >= this.T - 1e-9) this._end();
  }

  _advance(r, dt) {
    let budget = dt;
    for (let guard = 0; guard < 40 && budget > 1e-9; guard++) {
      if (!r.task) {
        const n = r.gen.next();
        r.task = n.done ? { type: 'idle', label: 'Hoàn tất kế hoạch' } : n.value;
        if (r.task.type === 'move' && !r.task.segs) r.task = this._moveTask(r, r.task.goal) || { type: 'idle', label: 'Không có đường tới ' + r.task.goal };
        r.label = r.task.label || '';
      }
      budget = this._run(r, r.task, budget);
      if (r.task.finished) r.task = null; else break;
    }
  }

  _run(r, task, budget) {
    if (task.type === 'idle') return 0;
    if (task.type === 'wait') {
      if (task.until()) { task.finished = true; return budget; }
      if (task.max != null) {
        const use = Math.min(task.max - (task.el || 0), budget);
        task.el = (task.el || 0) + use;
        if (task.el >= task.max - 1e-9) { task.finished = true; return budget - use; }
      }
      return 0;
    }
    if (task.type === 'act') return this._runAct(r, task, budget);
    // move
    while (budget > 1e-9 && task.i < task.segs.length) {
      const s = task.segs[task.i];
      if (s.lock && task.tau === 0) {
        const key = r.team + ':' + s.lock, holder = this.locks[key];
        if (holder && holder !== r.id) { r.label = 'Chờ lối lên trống'; return 0; }
        this.locks[key] = r.id;
      }
      r.climbing = s.kind === 'stairs';
      const use = Math.min(s.dur - task.tau, budget);
      task.tau += use; budget -= use;
      const px = r.x, py = r.y;
      this._segPose(r, s, task.tau);
      const ds = Math.hypot(r.x - px, r.y - py);
      r.wheel += ds / (r.R[r.role].wheels.diameter / 2);
      r.track += ds;
      if (s.kind === 'stairs') { const T = r.R.BR.tracks; r.j.track = Math.max(0, Math.min(1, task.tau / T.t_deploy, (s.dur - task.tau) / T.t_deploy)); }
      if (task.tau >= s.dur - 1e-9) { task.i++; task.tau = 0; r.dist += s.d ?? Math.hypot(s.x1 - s.x0, s.y1 - s.y0); r.climbing = false; if (r.role === 'BR') r.j.track = 0; }
    }
    if (task.i >= task.segs.length) {
      r.node = task.goal;
      const [x, y, h] = r.graph.nodes[task.goal];
      r.x = x; r.y = y; if (h != null) r.h = h;
      this._setPose(r);
      for (const key of Object.keys(this.locks)) if (this.locks[key] === r.id) delete this.locks[key];
      task.finished = true;
    }
    return budget;
  }

  _segPose(r, s, tau) {
    let f;
    if (s.kind === 'path') {
      let i = 0;
      while (i < s.pieces.length - 1 && tau > s.pieces[i].t0 + s.pieces[i].dur) i++;
      const pc = s.pieces[i], q = this._piecePose(pc, Math.max(0, Math.min(pc.dur, tau - pc.t0)));
      r.x = q.x; r.y = q.y; r.h = q.h; r._pieceKind = pc.kind;
      this._setPose(r);
      return;
    }
    if (s.kind === 'poly') {
      const { d, v, a } = s, T = s.dur, ta = v / a;
      const tt = Math.min(tau, T) * ((d >= v * ta ? d / v + ta : 2 * Math.sqrt(d / a)) / T); // nen thoi gian cho goc cua
      let dist;
      if (d >= v * ta) { const tc = d / v - ta; dist = tt < ta ? 0.5 * a * tt * tt : tt < ta + tc ? 0.5 * a * ta * ta + v * (tt - ta) : d - 0.5 * a * Math.max(0, d / v + ta - tt) ** 2; }
      else { const th = Math.sqrt(d / a); dist = tt < th ? 0.5 * a * tt * tt : d - 0.5 * a * Math.max(0, 2 * th - tt) ** 2; }
      dist = Math.min(d, Math.max(0, dist));
      let i = 1; while (i < s.cum.length - 1 && s.cum[i] < dist) i++;
      const k = (dist - s.cum[i - 1]) / Math.max(1e-9, s.cum[i] - s.cum[i - 1]);
      r.x = s.pts[i - 1][0] + (s.pts[i][0] - s.pts[i - 1][0]) * k; r.y = s.pts[i - 1][1] + (s.pts[i][1] - s.pts[i - 1][1]) * k; r.h = s.h0;
      this._setPose(r);
      return;
    }
    if (s.kind === 'joint') {
      s.from ??= { ...r.j };
      s.c ??= K.profile(r.role, r.R, s.from, s.to).c;
      const k = K.trapS(tau / s.dur, s.c);
      for (const [j, v] of Object.entries(s.to)) r.j[j] = s.from[j] + (v - s.from[j]) * k;
      return;
    }
    if (s.kind === 'flat') {
      const { d, v, a } = s, ta = v / a;
      let dist;
      if (d >= v * ta) {
        const tc = d / v - ta;
        if (tau < ta) dist = 0.5 * a * tau * tau;
        else if (tau < ta + tc) dist = 0.5 * a * ta * ta + v * (tau - ta);
        else { const td = Math.min(tau - ta - tc, ta); dist = 0.5 * a * ta * ta + v * tc + v * td - 0.5 * a * td * td; }
        f = tau >= d / v + ta ? 1 : dist / d;
      } else {
        const th = Math.sqrt(d / a), tm = 2 * th;
        dist = tau < th ? 0.5 * a * tau * tau : d - 0.5 * a * Math.max(0, tm - tau) ** 2;
        f = d > 0 ? Math.min(1, dist / d) : 1;
      }
      // xoay than o toc do xoay toi da ngay dau doan (mecanum xoay + tinh tien dong thoi), xong som truoc khi qua chuong ngai
      const tRot = Math.min(s.dur, Math.abs(angDiff(s.h0, s.h1)) / (this._speed(r).wr * 0.8) + 0.2);
      r.h = s.h0 + angDiff(s.h0, s.h1) * K.trapS(Math.min(1, tau / tRot), 0.3);
    } else {
      f = s.dur > 0 ? tau / s.dur : 1;
      r.h = s.kind === 'turn' ? s.h0 + angDiff(s.h0, s.h1) * f : s.h1;
    }
    r.x = s.x0 + (s.x1 - s.x0) * f;
    r.y = s.y0 + (s.y1 - s.y0) * f;
    this._setPose(r);
  }

  // Thao tac = chuoi pha. Moi pha: {to: {khop: gia tri} | () => {...}, t: giay giu them, done(), tick(f), label}
  _runAct(r, task, budget) {
    task.pi ??= 0;
    while (budget > 1e-9 && task.pi < task.phases.length) {
      const ph = task.phases[task.pi];
      if (!task.from) {
        task.from = { ...r.j };
        task.to = (typeof ph.to === 'function' ? ph.to() : ph.to) || {};
        const lim = K.jointLimits(r.role, r.R); // khop khong duoc vuot hanh trinh thiet ke
        for (const [k, v] of Object.entries(task.to)) {
          if (!lim[k] || k === 'yaw') continue;
          if (v < lim[k][0] - 1 || v > lim[k][1] + 1) this._coll(r, 'joint', `khớp ${k} = ${Math.round(v)} ngoài hành trình [${lim[k][0]}, ${lim[k][1]}] (${r.label})`);
          task.to[k] = Math.min(lim[k][1], Math.max(lim[k][0], v));
        }
        const pr = K.profile(r.role, r.R, task.from, task.to);
        task.move = pr.T; task.c = pr.c;
        task.dur = Math.max(task.move + (ph.t || 0), ph.tmin || 0);
        task.tau = 0;
        if (ph.label) r.label = ph.label;
      }
      const use = Math.min(task.dur - task.tau, budget);
      task.tau += use; budget -= use;
      const f = task.move > 0 ? K.trapS(task.tau / task.move, task.c) : 1; // profile hinh thang: khong vuot v_max, a_max
      for (const [k, v] of Object.entries(task.to)) { const a = task.from[k]; r.j[k] = a + (v - a) * f; }
      ph.tick?.(f);
      if (task.tau >= task.dur - 1e-9) {
        if (r.j.yaw != null) r.j.yaw = ((r.j.yaw % 360) + 540) % 360 - 180;
        ph.done?.(); task.pi++; task.from = null;
      }
    }
    if (task.pi >= task.phases.length) { task.finished = true; r.ignore.clear(); }
    return budget;
  }

  // Vi tri vat dang mang theo FK co cau. Luc vua gap, do lech thuc te giua vat va diem kep (trong he cua
  // co cau) duoc ghi lai va giu nguyen suot qua trinh mang -> vat khong bao gio "nhay" khi gap / nha.
  _syncCarry(r) {
    const P = this.pose(r), held = new Set();
    r._offs ??= new Map();
    for (const it of r.carry) if (it.holder !== r.id && !this._dbg) { this._dbg = true; this.log(null, `BUG carry ${r.id} ${it.id} holder=${it.holder} where=${it.where} task=${r.label}`); }
    const put = (it, p, yaw) => {
      const key = it.hold, c = Math.cos(yaw), s = Math.sin(yaw);
      let o = r._offs.get(it.id);
      if (!o || o.key !== key) {
        const dx = it.x - p[0], dy = it.y - p[1];
        o = { key, off: [dx * c + dy * s, -dx * s + dy * c, it.z - p[2]], yaw: (it.yaw ?? yaw) - yaw };
        r._offs.set(it.id, o);
        // BR kep Mustika o nua tren bong (day ham cao hon day bong 65 mm) la co chu dich; lech khac > 40 mm la loi
        const byDesign = r.role === 'BR' && it.kind === 'mustika' && Math.abs(o.off[2] + 65) < 5 && Math.hypot(o.off[0], o.off[1]) < 10;
        if (!byDesign && Math.hypot(...o.off) > 40) this.log(null, `BUG ${r.id} gắp ${it.id} lệch ${Math.round(Math.hypot(...o.off))} mm (${r.label})`);
      }
      held.add(it.id);
      it.x = p[0] + o.off[0] * c - o.off[1] * s; it.y = p[1] + o.off[0] * s + o.off[1] * c; it.z = p[2] + o.off[2];
      it.yaw = yaw + o.yaw; it.pitch = 0;
    };
    if (r.role === 'TR') {
      const R = r.R, base = K.toWorld(P, K.trClampBase(R, r.j)), tip = K.toWorld(P, K.trBoomTip(R, r.j));
      const toolYaw = r.h + (r.j.yaw + (r.j.tw || 0)) * DEG;
      let zc = base[2], top = -Infinity;
      for (const it of r.carry) {
        if (it.hold === 'boom') put(it, [tip[0], tip[1], tip[2] - R.TR.boom.tool_drop - hOf(it) - (it.flip ? K.flipDrop(r.j.roll) : 0)], toolYaw);
        else { put(it, [base[0], base[1], zc], r.h); if (it.z < top) it.z = top; top = it.z + hOf(it); zc += hOf(it); } // chong thang dung (khop can bang), khoi tren tua mat khoi duoi
      }
    } else {
      const lvl = (p) => K.toWorld(P, K.brLevelToRobot(r.R, r.j, r.pitch, p), true);
      const b0 = K.brGripBase(r.R, r.j);
      let zc = b0[2], top = -Infinity;
      for (const it of r.carry) {
        if (it.hold === 'clamp') { put(it, lvl([b0[0], b0[1], zc]), r.h); if (it.z < top) it.z = top; top = it.z + hOf(it); zc += hOf(it); } else put(it, lvl([b0[0], b0[1], b0[2] + r.j.pf - hOf(it)]), r.h);
      }
    }
    for (const id of [...r._offs.keys()]) if (!held.has(id)) r._offs.delete(id);
  }

  // ------------------------------------------------------------------ luat + vat ly
  _viol(r, type, msg) {
    const key = r.id + type;
    if (this._vKeys[key] != null && this.t - this._vKeys[key] < 3) return;
    this._vKeys[key] = this.t;
    this.violations.push({ t: this.t, robot: r.id, type, msg, pose: [Math.round(r.x), Math.round(r.y), Math.round(r.h / DEG)], task: r.label });
    this.log(r.team, '⚠ VI PHẠM ' + r.role + ': ' + msg);
  }
  _coll(r, type, msg) {
    const key = r.id + 'c' + type;
    if (this._vKeys[key] != null && this.t - this._vKeys[key] < 3) return;
    this._vKeys[key] = this.t;
    this.collisions.push({ t: this.t, robot: r.id, type, msg, pose: [Math.round(r.x), Math.round(r.y), Math.round(r.h / DEG)], task: r.label });
    this.log(r.team, '✖ VA CHẠM ' + r.role + ': ' + msg);
  }
  _boxSamples(r, b, step, level = true) {
    const P = this.pose(r), out = [];
    const sx = Math.max(10, Math.min(step, (b.x1 - b.x0) / 2 || step)), sy = Math.max(10, Math.min(step, (b.y1 - b.y0) / 2 || step));
    for (let x = b.x0; x <= b.x1 + 1e-6; x += sx) for (let y = b.y0; y <= b.y1 + 1e-6; y += sy) {
      const w = K.toWorld(P, [x, y, (b.z0 + b.z1) / 2], level);
      out.push(w);
    }
    return out;
  }
  // Than robot (khong gom co cau): TR la khung chu U (bo khoang trong phia truoc)
  _chassisSamples(r, P = this.pose(r)) {
    const c = r.R[r.role].chassis, L = c.length / 2, W = c.width / 2, out = [];
    const u = r.role === 'TR' ? c.u_cut : null;
    for (let x = -L; x <= L + 1e-6; x += L / 7) for (let y = -W; y <= W + 1e-6; y += W / 8) {
      if (u && x > u.x0 && Math.abs(y) < u.half_w) continue;
      out.push([...K.toWorld({ ...P, pitch: 0 }, [x, y, 0]), x]);
    }
    return out;
  }
  _checkRules() {
    const g = this.geo;
    for (const r of this.robots) {
      const items = r.carry.map((it) => ({ kind: it.kind, hold: it.hold }));
      const env = K.envelope(r.role, r.R, r.j, items, r.pitch);
      r.env = env;
      const hz = [env.L, env.W].sort((a, b) => b - a);
      if (hz[0] > 1400.5 || hz[1] > 1000.5 || env.H > 1200.5) this._viol(r, 'size', `kích thước ${Math.round(env.L)}×${Math.round(env.W)}×${Math.round(env.H)} vượt 1400×1000×1200 (11.5–11.6)`);
      if (r.carry.filter((i) => i.kind !== 'mustika').length > (r.role === 'TR' ? 3 : 2)) this._viol(r, 'carry', 'mang quá số khối (4.4.2)');
      const pts = this._chassisSamples(r);
      const P = this.pose(r);
      for (const o of K.mechOBBs(r.role, r.R, r.j, r.pitch)) {
        const nx = Math.max(1, Math.ceil((2 * o.e[0]) / 70)), ny = Math.max(1, Math.ceil((2 * o.e[1]) / 70));
        for (let a = 0; a <= nx; a++) for (let b = 0; b <= ny; b++) {
          const fx = (2 * a / nx - 1) * o.e[0], fy = (2 * b / ny - 1) * o.e[1];
          pts.push(K.toWorld(P, [o.c[0] + o.u[0][0] * fx + o.u[1][0] * fy, o.c[1] + o.u[0][1] * fx + o.u[1][1] * fy, o.c[2]], true));
        }
      }
      for (const b of K.parts(r.role, r.R, r.j, items, r.pitch)) if (b.name.startsWith('vật:')) pts.push(...this._boxSamples(r, b, 70));
      if (r.role === 'TR' && pts.some(([x, y]) => g.trForbidden(x, y))) this._viol(r, 'zone_tr', 'hình chiếu TR lấn vào L1 ngoài Transfer Area (6.2.1)');
      const bad = pts.find(([x, y]) => g.inOpponent(r.team, x, y));
      if (bad) this._viol(r, 'zone_opp', `hình chiếu lấn lãnh thổ riêng đối phương (6.2.2) tại (${Math.round(bad[0])}, ${Math.round(bad[1])})`);
    }
  }
  _obstacleHit(x, y, z0, z1) {
    for (const f of this.spec.fences) {
      const t = f.thickness / 2, [ax, ay] = f.from, [bx, by] = f.to;
      const inside = ax === bx ? Math.abs(x - ax) <= t && y >= Math.min(ay, by) && y <= Math.max(ay, by) : Math.abs(y - ay) <= t && x >= Math.min(ax, bx) && x <= Math.max(ax, bx);
      if (inside && z1 > f.z0 && z0 < f.z0 + f.height) return f.kind === 'center_divider' ? 'hàng rào giữa' : 'barrier L1';
    }
    for (const p of this.spec.pillars) if (Math.hypot(x - p.center[0], y - p.center[1]) <= p.diameter / 2 && z1 > p.z0 && z0 < p.z0 + p.height) return 'cột trụ';
    return null;
  }
  _itemHit(x, y, z0, z1, skip, list = this.items) {
    for (const it of list) {
      if (it.holder || skip.has(it.id) || it.where === 'lost') continue;
      const s = hOf(it) / 2;
      if (Math.abs(x - it.x) < s && Math.abs(y - it.y) < s && z1 > it.z + 5 && z0 < it.z + 2 * s) return it;
    }
    return null;
  }
  _checkPhysics() {
    const g = this.geo;
    for (const r of this.robots) {
      const c = r.R[r.role].chassis, z0 = r.z + c.z_bottom, z1 = r.z + c.z_top;
      const seg = r.task?.segs?.[r.task.i];
      const near = this.items.filter((it) => !it.holder && Math.abs(it.x - r.x) < 1800 && Math.abs(it.y - r.y) < 1800);
      for (const [x, y, , lx] of this._chassisSamples(r)) {
        if (!r.climbing && (seg?.kind === 'path' ? r._pieceKind : seg?.kind) !== 'ramp' && Math.abs(r.pitch) < 0.02 && g.floorZ(x, y) > z0 + (r.role === 'BR' ? (lx > 300 ? 105 : lx > 200 ? 80 : 0) : 0) + 5) { this._coll(r, 'wall', `thân chạm bậc/tường tại (${Math.round(x)}, ${Math.round(y)})`); break; }
        const ob = this._obstacleHit(x, y, z0, z1);
        if (ob) { this._coll(r, 'obs', `thân chạm ${ob} tại (${Math.round(x)}, ${Math.round(y)})`); break; }
        const it = this._itemHit(x, y, z0, z1, r.ignore, near);
        if (it) { this._coll(r, 'item', `thân đè/đẩy ${it.kind === 'earth' ? 'Earth' : it.kind === 'sky' ? 'Sky' : 'Mustika'} ${it.id} (${it.where} @${Math.round(it.x)},${Math.round(it.y)},${Math.round(it.z)}) (cấm đẩy khối, 6.1)`); break; }
      }
      if (r.role === 'BR') { // ham kep khong duoc dam vao than
        const g = r.R.BR.gripper, jx0 = g.x0 + r.j.s - g.jaw_depth / 2;
        if (jx0 < c.length / 2 - 5 && r.j.lz < c.z_top - 5) this._coll(r, 'jawbody', `hàm kẹp đâm vào thân (s=${Math.round(r.j.s)}, lz=${Math.round(r.j.lz)})`);
      }
      for (const b of K.parts(r.role, r.R, r.j, [], r.pitch)) {
        if (b.name !== 'hàm') continue;
        for (const [x, y, z] of this._boxSamples(r, { ...b, z1: b.z0 }, 100)) if (z < g.floorZ(x, y) - 8) { this._coll(r, 'floor', `hàm kẹp cắm xuống sàn ${Math.round(g.floorZ(x, y) - z)} mm tại (${Math.round(x)}, ${Math.round(y)}), lz=${Math.round(r.j.lz)}`); break; }
        const half = (b.z1 - b.z0) / 2;
        for (const [x, y, zc] of this._boxSamples(r, { ...b, x0: b.x0 + 20, x1: b.x1 - 20 }, 60)) {
          const ob = this._obstacleHit(x, y, zc - half + 2, zc + half);
          if (ob) { this._coll(r, 'jawobs', `hàm kẹp chạm ${ob} tại (${Math.round(x)}, ${Math.round(y)})`); break; }
          const it = this._itemHit(x, y, zc - 40, zc + 40, r.ignore, near);
          if (it) { this._coll(r, 'jaw', `hàm kẹp chạm khối ${it.id} (${it.where} @${Math.round(it.x)},${Math.round(it.y)},${Math.round(it.z)}) tại (${Math.round(x)}, ${Math.round(y)}, ${Math.round(zc)}), w=${Math.round(r.j.w)}`); break; }
        }
      }
    }
    for (let i = 0; i < this.robots.length; i++) for (let k = i + 1; k < this.robots.length; k++) {
      const a = this.robots[i], b = this.robots[k];
      if (Math.hypot(a.x - b.x, a.y - b.y) > 1000 || Math.abs(a.z - b.z) > 300) continue;
      const cb = b.R[b.role].chassis, P = { ...this.pose(b), pitch: 0 };
      const hit = this._chassisSamples(a).some((w) => { const p = K.toRobot(P, w); return Math.abs(p[0]) < cb.length / 2 && Math.abs(p[1]) < cb.width / 2; });
      if (hit) this._coll(a, 'robot', `thân va chạm ${b.id}`);
    }
  }

  // ------------------------------------------------------------------ TA / spot / storage
  taEmpty(team) { return this.teams[team].ta.every((s) => s.items.length === 0); }
  _lane(team, id) { return this.teams[team].ta.find((s) => s.id === id); }
  _laneTop(l) { return l.items.reduce((a, i) => a + hOf(i), 600); }
  claimSpot(team, prefs) {
    for (const lid of prefs) {
      const id = this.geo.spotId(team, lid), sp = this.spots[id];
      if (sp && !sp.claimedBy && sp.items.length === 0) { sp.claimedBy = team; return { id, lid }; }
    }
    return null;
  }
  _completeTowers(team) {
    const out = [];
    for (const [id, sp] of Object.entries(this.spots)) {
      const it = sp.items;
      if (it.length === 3 && it[0].kind === 'earth' && it[1].kind === 'earth' && it[2].kind === 'sky' && it[2].dir === UP[team]) out.push({ id, shared: sp.def.shared });
    }
    return out;
  }
  _checkMandate(team) {
    const tm = this.teams[team];
    if (tm.mandate) return;
    const ct = this._completeTowers(team);
    if (ct.length >= 2 && ct.some((c) => c.shared)) {
      tm.mandate = true; tm.mandateT = this.t;
      this.log(team, `✔ Đạt SANCTUARY MANDATE (${ct.map((c) => c.id).join(', ')}) — được lấy Mustika`);
    }
  }
  _mustikaGone(team) {
    const b = this.ball;
    if (b.where === 'central') return true;
    if (b.holder) { const h = this.robots.find((r) => r.id === b.holder); return h.team !== team; }
    return false;
  }
  _nodePose(r, node) { const [x, y, h] = r.graph.nodes[node]; return this._poseAt(r, x, y, h ?? r.h); }
  _standFree(r, node, skip = new Set()) {
    const P = this._nodePose(r, node), c = r.R[r.role].chassis;
    return !this._chassisSamples(r, P).some(([sx, sy]) => this._itemHit(sx, sy, P.z + c.z_bottom, P.z + c.z_top, skip));
  }

  // ------------------------------------------------------------------ BRAIN: TR
  _demand(team) {
    const tm = this.teams[team];
    let before = 0, all = 0, mustikaPending = false;
    for (const g of tm.plan.goals) {
      if (g.mustika) { if (!tm.mustikaHanded && !this._mustikaGone(team)) mustikaPending = true; continue; }
      all++; if (!mustikaPending) before++;
    }
    const tw = mustikaPending ? before : all;
    const dE = tm.delivered.E - tm.lost.E, dS = tm.delivered.S - tm.lost.S; // khoi roi phai giao bu
    let eU = 0, sU = 0, next = null;
    // gom Earth: giao du Earth cho cac thap truoc Mustika roi moi giao Sky -> BR xay cap Earth thap sau trong luc TR di lay Sky,
    // sau do BR gap ca 2 Sky 1 luot (dat len 2 thap)
    const nb = tm.params.tr_earth_batch ? Math.min(2, tw) : 0; // chi gom cho 2 thap dau (2 thap dat Mandate)
    const seq = [...Array(2 * nb).fill('E'), ...Array(nb).fill('S'), ...Array.from({ length: tw - nb }, () => ['E', 'E', 'S']).flat()];
    for (const k of seq) { if (k === 'E' && eU < dE) eU++; else if (k === 'S' && sU < dS) sU++; else { next = k; break; } }
    const skyLeft = this._skyAvail(team), eLeft = tm.storage.reduce((a, c) => a + c.items.length, 0);
    const pendE = Math.min(eLeft, Math.max(0, 2 * tw - dE)), pendS = Math.min(skyLeft, Math.max(0, tw - dS));
    return { pendE, pendS, next, eLeft, mustikaNext: mustikaPending && pendE === 0 && pendS === 0, allE: Math.min(eLeft, Math.max(0, 2 * all - dE)), allS: Math.min(skyLeft, Math.max(0, all - dS)) };
  }
  _skyCell(team, [r, c]) { return this.skyCells[`${r},${team === 'red' ? c : 4 - c}`]; }
  _skyAvail(team) { return SKY_PLAN.filter((p) => this._skyCell(team, p.rc)?.where === 'grid').length; }

  * _trBrain(r) {
    const tm = this.teams[r.team], P = tm.params;
    while (!this.over) {
      const d = this._demand(r.team), before = tm.delivered.E + tm.delivered.S + (tm.mustikaHanded ? 1 : 0);
      if (d.mustikaNext) { yield* this._trMustika(r); continue; }
      if (P.tr_mixed && d.pendE >= 2 && d.pendS >= 1 && !r.carry.length && this._skyAvail(r.team) > 0) { yield* this._trMixedTrip(r, 2); if (tm.delivered.E + tm.delivered.S + (tm.mustikaHanded ? 1 : 0) > before) continue; }
      if (P.tr_sky_first && tm.delivered.S === 0 && !r.carry.length && d.allS > 0) {
        const br = this.robots.find((x) => x.team === r.team && x.role === 'BR');
        if (r.node === 'T_S1') yield { type: 'wait', until: () => Math.hypot(br.x - r.x, br.y - r.y) > 1600, max: 4, label: 'Chờ BR rời ô xuất phát' };
        yield* this._trSkyTrip(r, Math.min(3, d.allS));
      }
      else if (d.pendE > 0 && (d.next !== 'S' || d.pendS === 0)) yield* this._trEarthTrip(r, Math.min(3, Math.max(d.pendE, d.allE)));
      else if (d.pendS > 0) yield* this._trSkyTrip(r, Math.min(3, d.allS));
      else if (d.pendE > 0) yield* this._trEarthTrip(r, Math.min(3, Math.max(d.pendE, d.allE)));
      else if (P.extra_delivery && d.eLeft > 0 && this.T - this.t > 25 && this.taEmpty(r.team)) yield* this._trEarthTrip(r, Math.min(3, d.eLeft), true);
      const after = tm.delivered.E + tm.delivered.S + (tm.mustikaHanded ? 1 : 0);
      if (after === before) yield { type: 'wait', until: () => this.over, max: 1, label: 'TR chờ (chưa có việc theo kế hoạch)' };
    }
  }

  _boomTo(r, tipWorld, zBody = null) {
    const p = K.toRobot(this.pose(r), tipWorld), items = r.carry.map((it) => ({ kind: it.kind, hold: it.hold }));
    if (zBody != null) p[2] = zBody;
    // robot luon <= 1200 mm (luat 11.5), tinh ca goc nghieng than (dung tren doc 9,7 do: dau can duoi ra truoc cao them ~150 mm)
    let ik = K.trBoomIK(r.R, p);
    for (let k = 0; k < 6; k++) {
      const H = K.envelope('TR', r.R, { ...r.j, yaw: ik.yaw, pitch: ik.pitch, len: ik.len }, items, r.pitch).H;
      if (H <= 1199.5) break;
      p[2] -= H - 1198; ik = K.trBoomIK(r.R, p);
    }
    return { yaw: this._yawVia(r, ik.yaw), pitch: ik.pitch, len: ik.len };
  }
  // Mam xoay chon chieu quay khong quet can qua L1 (6.2.1) / lanh tho doi phuong: tra ve goc dich "khong cuon"
  _yawVia(r, target) {
    const cur = r.j.yaw, d = ((target - cur + 540) % 360) - 180, alt = d > 0 ? d - 360 : d + 360;
    const P = this.pose(r), b = r.R.TR.boom, bad = (deg) => {
      const tip = K.toWorld(P, [b.turret[0] + 900 * Math.cos(deg * DEG), b.turret[1] + 900 * Math.sin(deg * DEG), b.pivot_z]);
      return this.geo.trForbidden(tip[0], tip[1]) || this.geo.inOpponent(r.team, tip[0], tip[1]);
    };
    const sweeps = (delta) => { for (let f = 0.1; f < 1; f += 0.1) if (bad(cur + delta * f)) return true; return false; };
    return cur + (Math.abs(d) > 20 && sweeps(d) && !sweeps(alt) ? alt : d);
  }
  _clampTo(r, bottomWorld) {
    const c = r.R.TR.clamp, p = K.toRobot(this.pose(r), bottomWorld);
    return { s: p[0] - c.x0, lz: p[2] - c.z0, lat: p[1] };
  }
  // Di chuyen can an toan: neu phai xoay mam > 40 do thi dung can len (pitch 55, rut ngan) roi moi xoay (gioi han rong 1000 mm)
  // Do cao dau can (he than) de dau cong cu + vat dang cam di qua tren ham kep / chong khoi trong kep
  _safeZ(r) {
    const T = r.R.TR, cl = T.clamp, b = T.boom;
    const inClamp = r.carry.filter((i) => i.hold === 'clamp').reduce((a, i) => a + hOf(i), 0);
    const held = r.carry.find((i) => i.hold === 'boom');
    const below = Math.max(b.tool_drop + T.tool.pad_z + 32, b.tool_drop + (held ? hOf(held) : 0)) + 15;
    let z = cl.z0 + r.j.lz + Math.max(cl.jaw_h, inClamp) + below;
    if (r.node === 'RP_TOP' || r.node === 'TA_FRONT') for (const l of this.teams[r.team].ta) if (l.items.length) z = Math.max(z, this._laneTop(l) - r.z + below);
    return z;
  }
  _liftPhase(r, label) {
    return { to: () => { const cur = K.trBoomTip(r.R, r.j), z = this._safeZ(r); if (cur[2] >= z - 1) return {}; const s = K.trBoomIK(r.R, [cur[0], cur[1], z]); return { pitch: s.pitch, len: s.len }; }, label };
  }
  // 2 ngon vuong goc voi can (khong ngon nao nam duoi ong can khi can ngoc len)
  _twPerpPhase(r) { return { to: () => { const tw = r.j.tw || 0, t = Math.round(tw / 180) * 180; return Math.abs(t - tw) > 0.5 ? { tw: t } : {}; } }; }
  _boomPhases(r, tipFn, label, twFn = null) {
    const body = () => K.toRobot(this.pose(r), tipFn());
    const high = () => { const t = body(); return this._boomTo(r, tipFn(), Math.max(t[2], this._safeZ(r))); };
    const withTw = (o) => (twFn ? { ...o, tw: twFn(o.yaw) } : o);
    return [
      this._liftPhase(r, label),
      this._twPerpPhase(r),
      { to: () => ({ yaw: high().yaw }) },
      { to: () => withTw(high()) },
      { to: () => withTw(this._boomTo(r, tipFn())) },
    ];
  }
  // Goc xoay dau cong cu (do) de huong dau = goc dich (rad, the gioi) theo modulo `mod` (do), gan tw hien tai nhat
  _tw(r, yawDeg, target, mod = 90) {
    let d = (target - r.h) / DEG - yawDeg - (r.j.tw || 0);
    d = ((((d % mod) + mod) % mod) + mod / 2) % mod - mod / 2;
    let tw = (r.j.tw || 0) + d;
    while (tw > 180) tw -= mod;
    while (tw < -180) tw += mod;
    return tw;
  }
  // Huong dau cong cu de dat vat dang giu (lech huong vat so voi dau luc gap) thang hang goc the gioi `target`
  _twItem(r, it, target) { return (yawDeg) => { const o = r._offs?.get(it.id); return this._tw(r, yawDeg, target - (o ? o.yaw : 0), 90); }; }
  _stowPhases(r) {
    return [
      this._liftPhase(r),
      this._twPerpPhase(r),
      { to: () => { const s = this._stowTR(r); return Math.abs(s.yaw - r.j.yaw) > 30 ? { pitch: s.pitch, len: s.len } : {}; } },
      { to: () => ({ yaw: this._stowTR(r).yaw }) },
      { to: () => this._stowTR(r) },
    ];
  }
  // Tu the cat can: dang giu vat -> dung cao phia sau; kep dang co vat -> nga ra sau (tranh chong trong kep); con lai -> gap ve truoc (khung 700)
  // Do can cao ben hong phia ngoai san (tai doc: ra ngoai mep doc, khong tren L1): vat treo cao hon cot Storage 700 mm
  _sidePark(r) { return { yaw: r.team === 'red' ? -90 : 90, pitch: 45, len: 560 }; }
  _stowTR(r) {
    const T = r.R.TR, pick = (o) => ({ yaw: this._yawVia(r, o.yaw), pitch: o.pitch, len: o.len });
    if (r.carry.some((i) => i.kind === 'mustika')) return pick({ yaw: r.team === 'red' ? -90 : 90, pitch: 45, len: 560 }); // bong do ben hong, cao hon tru Z
    if (r.carry.some((i) => i.hold === 'boom')) {
      const h = r.carry.filter((i) => i.hold === 'clamp').reduce((a, i) => a + hOf(i), 0);
      if (h > 450 || r._willClampEarth) return pick(this._sidePark(r)); // Sky tren can + cot Earth: do cao ben hong (day Sky 749 > dinh cot 700)
      return pick(T.carry_stow_front); // tren chong Sky thap: de phia truoc-tren
    }
    // chong Earth 700 mm: can do ben hong; san doi Xanh la anh guong -> lat dau yaw (dau cong cu luon ve phia ngoai hang Storage)
    if (r.carry.some((i) => i.hold === 'clamp' && i.kind === 'earth')) return pick(r.team === 'red' ? T.carry_stow : { ...T.carry_stow, yaw: -T.carry_stow.yaw });
    return pick(T.stow_high); // can dung cao phia truoc tren kep: khong vuong ban truot / chong khoi
  }

  // Can cau nhac vat (dinh cot Storage / Sky tren luoi); dest: 'rack0' | 'rack1' | null (giu tren can)
  _trBoomPick(r, it, dest, orient) {
    const R = r.R.TR, top = it.z + hOf(it), tipAt = [it.x, it.y, top + R.boom.tool_drop], above = [it.x, it.y, top + R.boom.tool_drop + 120];
    r.ignore.add(it.id);
    const flip = orient && it.kind === 'sky' && it.dir !== UP[r.team];
    const DIRA = { '+x': 0, '-x': Math.PI, '+y': Math.PI / 2, '-y': -Math.PI / 2 };
    // lat Sky: truc 2 dia xoay (ngang dau) vuong goc huong nua mau -> dau cong cu song song huong do (mod 180)
    const twFn = flip && DIRA[it.dir] != null ? (yawDeg) => this._tw(r, yawDeg, (it.yaw || 0) + DIRA[it.dir], 180) : (yawDeg) => this._tw(r, yawDeg, it.yaw || 0, 90);
    const phases = [
      { to: () => (Math.abs(r.pitch) < 0.01 && r.j.lz > 0 && !r.carry.some((i) => i.hold === 'clamp') ? { lz: 0 } : {}) },
      ...this._boomPhases(r, () => above, 'Cần cẩu xoay tới ' + (it.kind === 'sky' ? 'Sky' : 'Earth'), twFn),
      { to: () => this._boomTo(r, tipAt), label: 'Hạ đầu hút' },
      { to: { g: hOf(it) }, t: R.tool.t_vac, label: 'Hút + 2 đĩa kẹp mặt bên', done: () => { this._detach(it); it.holder = r.id; it.hold = 'boom'; r.carry.push(it); } },
      { to: () => this._boomTo(r, above) },
    ];
    if (flip) {
      const from = it.dir;
      // kep can nap Sky -> mo ham + day ban truot cung luc voi luc lat (2 cum doc lap)
      phases.push({ to: dest === 'clamp' ? { roll: 90, s: 100, lz: 0, w: SIZE.sky + 70 } : { roll: 90 }, label: '2 đĩa xoay lật Sky 90° quanh tâm — nửa màu đội lên trên', tick: (f) => { it.flip = { from, to: UP[r.team], f }; }, done: () => { it.dir = UP[r.team]; it.flip = null; r.j.roll = 0; } });
    }
    if (dest === 'clamp') { // xep Sky vao kep chu U (tren san trong long U hoac chong len Sky dang kep)
      const inClamp = r.carry.filter((i) => i.hold === 'clamp');
      // mat tren THUC cua chong trong kep (khong phai danh nghia) + bu do lech z cua Sky dang cam -> khong lun vao khoi duoi
      const zTop = () => { const w0 = K.toWorld(this.pose(r), K.trClampBase(r.R, r.j)), o = r._offs?.get(it.id); return [w0[0], w0[1], Math.max(w0[2], ...inClamp.map((i) => i.z + hOf(i))) - (o ? o.off[2] : 0)]; };
      // 2 dia giu 2 mat ben y (ngon vuong goc can) -> ham mo 260 de ngon lot vao; nha, rut ngon len roi moi dong ham
      if (!flip) phases.push({ to: { s: 100, lz: 0, w: SIZE.sky + 70 }, label: 'Mở kẹp 270 chờ Sky (bàn trượt ra 100 mm, trong tầm cần)' });
      phases.push(...this._boomPhases(r, () => { const z = zTop(); return [z[0], z[1], z[2] + hOf(it) + R.boom.tool_drop + 60]; }, 'Cần cẩu xếp Sky vào kẹp chữ U'));
      phases.push({ to: () => { const z = zTop(); return this._boomTo(r, [z[0], z[1], z[2] + hOf(it) + R.boom.tool_drop + 3]); } });
      phases.push({ to: { g: SIZE.sky + 12 }, t: R.tool.t_release, label: 'Nhả hút, 2 đĩa lùi khỏi mặt khối', done: () => { it.hold = 'clamp'; r.carry.sort((x, y) => (x.hold === 'clamp' ? 0 : 1) - (y.hold === 'clamp' ? 0 : 1)); } });
      phases.push(this._liftPhase(r, 'Rút ngón lên khỏi vùng hàm'));
      phases.push({ to: { w: SIZE.sky, g: R.tool.g_open }, t: R.clamp.t_grip, label: 'Kẹp giữ chồng Sky' });
    }
    phases.push(...this._stowPhases(r));
    if (dest === 'clamp') phases.push({ to: { s: 0 }, label: 'Thu kẹp về khung chữ U' });
    return { type: 'act', label: 'Cần cẩu gắp', phases };
  }
  _trClampPick(r, col, n) {
    if (n < 1) return { type: 'wait', until: () => true };
    const R = r.R.TR, bottom = col.items[col.items.length - n], target = [col.x, col.y, bottom.z];
    for (const it of col.items) r.ignore.add(it.id);
    let sol;
    return {
      type: 'act', label: n === 2 ? 'Kẹp cột 2 Earth' : 'Kẹp 1 Earth',
      phases: [
        ...(() => {
          const pk = () => (r.carry.some((i) => i.hold === 'boom') ? this._sidePark(r) : r.team === 'red' ? R.carry_stow : { ...R.carry_stow, yaw: -R.carry_stow.yaw });
          return [this._liftPhase(r, 'Cần cẩu đỗ bên hông (nhường chỗ chồng Earth)'), this._twPerpPhase(r), { to: () => (Math.abs(pk().yaw - r.j.yaw) > 30 ? { pitch: pk().pitch, len: pk().len } : {}) }, { to: () => ({ yaw: this._yawVia(r, pk().yaw) }) }, { to: () => ({ yaw: this._yawVia(r, pk().yaw), pitch: pk().pitch, len: pk().len }) }];
        })(),
        { to: { w: R.clamp.open_storage }, label: 'Đặt độ mở kẹp 400 (hàm vừa khe 50 mm giữa các cột)' },
        { to: () => { sol = this._clampTo(r, target); sol.lz = Math.max(sol.lz, bottom.z - R.clamp.z0 + 2); return { s: sol.s, lz: sol.lz }; }, label: 'Bàn trượt đưa kẹp ôm cột' },
        { to: { w: SIZE.earth }, t: R.clamp.t_grip, label: 'Kẹp', done: () => { const got = col.items.splice(col.items.length - n, n); for (const it of got) { this._detach(it); it.holder = r.id; it.hold = 'clamp'; r.carry.push(it); } } },
        { to: () => ({ lz: sol.lz + 20 }), label: 'Nhấc khỏi sàn 20 mm' },
        { to: { s: 0 }, label: 'Thu kẹp về khung chữ U' },
      ],
    };
  }
  _trClampDrop(r, lane) {
    const R = r.R.TR, stack = r.carry.filter((i) => i.hold === 'clamp'), target = [lane.x, lane.y, this._laneTop(lane)];
    for (const it of lane.items) r.ignore.add(it.id);
    let sol;
    return {
      type: 'act', label: 'Thả vào Transfer Area',
      phases: [
        { to: () => { sol = this._clampTo(r, target); return { s: sol.s, lz: Math.max(r.j.lz, sol.lz + 15) }; }, label: 'Vươn kẹp qua mép TA vào ' + lane.id },
        { to: () => ({ lz: sol.lz }), label: 'Hạ khối xuống sàn TA' },
        { to: { w: R.clamp.open[1] }, label: 'Nhả kẹp', done: () => this._toLane(r, stack, lane) },
        { to: () => ({ lz: Math.min(R.clamp.lift[1], sol.lz + 40) }), label: 'Nhấc hàm khỏi sàn TA' },
        { to: () => ({ s: 0, lz: Math.min(R.clamp.lift[1], Math.max(sol.lz + 40, 40)) }), label: 'Thu kẹp' },
        { to: { lz: 40 } },
      ],
    };
  }
  _trBoomPlace(r, it, lane) {
    const o = r._offs?.get(it.id), R = r.R.TR, top = this._laneTop(lane) + hOf(it) - (o ? o.off[2] : 0) + 2, tipAt = [lane.x, lane.y, top + R.boom.tool_drop], above = [lane.x, lane.y, top + R.boom.tool_drop + 100];
    for (const x of lane.items) r.ignore.add(x.id);
    const phases = [];
    phases.push(...this._boomPhases(r, () => above, 'Cần cẩu đưa vật vào ' + lane.id, this._twItem(r, it, 0)));
    phases.push({ to: () => this._boomTo(r, tipAt) });
    phases.push({ to: { g: R.tool.g_open }, t: R.tool.t_release, label: 'Nhả hút + mở đĩa', done: () => this._toLane(r, [it], lane) });
    phases.push({ to: () => this._boomTo(r, above) });
    phases.push(...this._stowPhases(r));
    return { type: 'act', label: 'Cần cẩu thả vào TA', phases };
  }
  _toLane(r, grp, lane) {
    const tm = this.teams[r.team];
    for (const it of grp) {
      r.carry.splice(r.carry.indexOf(it), 1);
      it.holder = null; it.hold = null; it.where = 'ta';
      it.z = this._laneTop(lane); it.pitch = 0; // giu x, y, huong luc nha (dau kep da can dung tam lan)
      lane.items.push(it);
      tm.transfer += this.spec.scoring.transfer_per_block;
      tm.delivered[it.kind === 'earth' ? 'E' : 'S']++;
    }
    this.log(r.team, `TR thả ${grp.length} ${grp[0].kind === 'earth' ? 'Earth' : 'Sky'} vào ${lane.id} (+${5 * grp.length} transfer)`);
  }
  _detach(it) {
    if (it.where === 'grid') for (const [k, v] of Object.entries(this.skyCells)) if (v === it) delete this.skyCells[k];
    if (it.where === 'storage') for (const tm of Object.values(this.teams)) for (const c of tm.storage) { const i = c.items.indexOf(it); if (i >= 0) c.items.splice(i, 1); }
    it.where = 'robot';
  }

  // Chon cot cho kep va (tuy chon) cot cho can cau tu cung diem dung; kiem tra than khong de len khoi
  _storagePlan(r, n) {
    const tm = this.teams[r.team];
    const order = [...tm.storage].sort((a, b) => a.row - b.row || b.i - a.i); // hang gan truoc, dong -> tay
    for (const col of order) {
      if (!col.items.length) continue;
      const node = `STO_${col.i}_${col.row}`;
      if (!this._standFree(r, node, new Set(col.items.map((i) => i.id)))) continue;
      const take = Math.min(n, col.items.length, 2);
      let boom = null;
      if (false && n - take >= 1 && !r.carry.some((i) => i.hold === 'boom')) {
        const P = this._nodePose(r, node);
        const cands = order.filter((c) => c !== col && c.items.length).map((c) => {
          const top = c.items[c.items.length - 1], ik = K.trBoomIK(r.R, K.toRobot(P, [c.x, c.y, top.z + SIZE.earth + r.R.TR.boom.tool_drop + 120]));
          return { c, ok: ik.ok, single: c.items.length === 1 };
        }).filter((x) => x.ok);
        boom = (cands.find((x) => x.single) || cands[0])?.c || null;
      }
      return { col, node, take, boom };
    }
    return null;
  }

  * _trEarthTrip(r, n, extra = false) {
    if (Math.min(n, 2, 3 - r.carry.length) < 1 || r.carry.some((i) => i.hold === 'clamp')) return;
    const plan = this._storagePlan(r, Math.min(n, 2, 3 - r.carry.length));
    if (!plan) return;
    yield { type: 'move', goal: plan.node };
    if (plan.boom && plan.boom.items.length) yield this._trBoomPick(r, plan.boom.items[plan.boom.items.length - 1], null, false);
    if (plan.col.items.length) yield this._trClampPick(r, plan.col, Math.min(plan.take, plan.col.items.length));
    if (r.carry.length) yield* this._trDeliver(r, extra);
  }

  // Chuyen hon hop: 1 Sky len can (lat mau, giu tren can) roi 2 Earth vao kep -> moi chuyen du 1 thap (E, E, S)
  * _trMixedTrip(r, nE) {
    const P = this.teams[r.team].params, br = this.robots.find((x) => x.team === r.team && x.role === 'BR');
    if (r.node === 'T_S1') yield { type: 'wait', until: () => Math.hypot(br.x - r.x, br.y - r.y) > 1600, max: 4, label: 'Chờ BR rời ô xuất phát' };
    for (const sp of SKY_PLAN) {
      const it = this._skyCell(r.team, sp.rc);
      if (!it || it.where !== 'grid' || !this._standFree(r, sp.st, new Set([it.id]))) continue;
      yield { type: 'move', goal: sp.st };
      if (it.where !== 'grid') continue;
      r._willClampEarth = true;
      yield this._trBoomPick(r, it, null, P.tr_orient_sky);
      break;
    }
    yield* this._trEarthTrip(r, nE);
    r._willClampEarth = false;
    if (r.carry.length) yield* this._trDeliver(r);
  }
  * _trSkyTrip(r, n) {
    const P = this.teams[r.team].params;
    // lan Sky (T1) chua toi da 2: chi lay 2 Sky vao kep, de can cau ranh cho chuyen Earth sau (3 khoi/chuyen)
    n = Math.min(n, 2, 3 - r.carry.length);
    let got = 0;
    for (const sp of SKY_PLAN) {
      if (got >= n) break;
      const it = this._skyCell(r.team, sp.rc);
      if (!it || it.where !== 'grid') continue;
      if (!this._standFree(r, sp.st, new Set([it.id]))) continue;
      if (!K.trBoomIK(r.R, K.toRobot(this._nodePose(r, sp.st), [it.x, it.y, SIZE.sky + r.R.TR.boom.tool_drop + 120])).ok) continue;
      yield { type: 'move', goal: sp.st };
      if (it.where !== 'grid') continue;
      const inClamp = r.carry.filter((i) => i.hold === 'clamp').length;
      const dest = inClamp < 1 ? 'clamp' : null; // Sky 1 xep vao kep chu U; Sky 2 giu tren can, tha chong len Sky 1 (bot 1 lan xep kep)
      yield this._trBoomPick(r, it, dest, P.tr_orient_sky);
      if (r.carry.includes(it)) got++;
    }
    if (r.carry.length) yield* this._trDeliver(r);
  }

  // Giao: T2 (gan doc) = lan Earth, tha tu RP_TOP; T1 (gan cau thang) = lan Sky, tha tu TA_FRONT
  // (TA_FRONT chi dung duoc khi T2 trong vi than TR che T2). Tach lan theo loai de TA khong bao gio ket.
  * _trDeliver(r, extra = false, all = false) {
    const tm = this.teams[r.team];
    yield { type: 'move', goal: 'RP_TOP' };
    const T1 = this._lane(r.team, 'T1'), T2 = this._lane(r.team, 'T2');
    // kep chi tha vao lan trong (tru Z khong du cao de chong); can cau chong duoc 1 khoi len khoi cung loai
    // vat tren can chi chong len 1 khoi cung loai (khong nam mot minh chan lan); kep tha vao lan trong
    const fits = (lane, grp, kind) => !lane.resBR && (kind === 'boom' ? (lane.items.length === 1 && lane.items[0].kind === grp[0].kind) || (lane === T1 && !lane.items.length) : lane.items.length === 0 && grp.length <= 2);
    // T2 (gan doc) toi tu RP_TOP; T1 (xa) chi toi duoc khi T2 trong (kep di qua tren T2). Dem 2 lan: khi ca 2 trong,
    // giao lan xa T1 truoc de T2 con trong cho chuyen sau -> TR khong phai cho BR don TA.
    const option = (u) => {
      const t1 = !T2.items.length && fits(T1, u.grp, u.kind), t2 = fits(T2, u.grp, u.kind);
      if (u.kind === 'boom' && T1.items.length && t1) return { u, lane: T1, node: 'TA_FRONT' };
      if (t1 && (this.teams[r.team].params.ta_buffer || u.grp[0].kind === 'sky') && !T1.items.length) return { u, lane: T1, node: 'TA_FRONT' };
      if (t2) return { u, lane: T2, node: 'RP_TOP' };
      return t1 ? { u, lane: T1, node: 'TA_FRONT' } : null;
    };
    while (r.carry.length && !this.over) {
      const clampStack = r.carry.filter((i) => i.hold === 'clamp'), boomIt = r.carry.find((i) => i.hold === 'boom');
      const units = [];
      if (clampStack.length) units.push({ kind: 'clamp', grp: clampStack });
      if (boomIt) { const u = { kind: 'boom', grp: [boomIt] }; if (boomIt.kind === 'sky' && clampStack.some((i) => i.kind === 'earth') && !T1.items.length && !T2.items.length) units.unshift(u); else units.push(u); }
      const choice = units.map(option).find(Boolean);
      if (!choice) {
        if (!all && !clampStack.length && boomIt?.kind === 'sky') break; // giu Sky tren can cho chuyen sau
        tm.trBlocked = true;
        if (r.node !== 'RP_TOP') yield { type: 'move', goal: 'RP_TOP' };
        yield { type: 'wait', until: () => this.over || units.some(option), label: 'Chờ Transfer Area có chỗ trống' };
        tm.trBlocked = false;
        continue;
      }
      if (r.node !== choice.node) {
        if (tm.taBusy === 'TR') tm.taBusy = null;
        // TA_FRONT: banh truoc da len TA -> chi lan vao khi kep BR khong o trong TA, giu quyen ngay tu luc lan banh
        if (choice.node === 'TA_FRONT') { yield { type: 'wait', until: () => tm.taBusy !== 'BR' || this.over, label: 'Chờ kẹp BR rút khỏi TA' }; tm.taBusy = 'TR'; }
        yield { type: 'move', goal: choice.node };
      }
      // quan sat: khong dua kep/can vao TA khi kep BR dang o trong TA (khong lien lac, chi nhin)
      if (tm.taBusy !== 'TR') { yield { type: 'wait', until: () => tm.taBusy !== 'BR' || this.over, label: 'Chờ kẹp BR rút khỏi TA' }; tm.taBusy = 'TR'; }
      yield choice.u.kind === 'clamp' ? this._trClampDrop(r, choice.lane) : this._trBoomPlace(r, choice.u.grp[0], choice.lane);
      // giu quyen vao TA neu con vat tha tiep tai cung diem dung (BR khong chen vao giua 2 lan tha)
      const rest = [];
      if (r.carry.some((i) => i.hold === 'clamp')) rest.push({ kind: 'clamp', grp: r.carry.filter((i) => i.hold === 'clamp') });
      if (r.carry.some((i) => i.hold === 'boom')) rest.push({ kind: 'boom', grp: [r.carry.find((i) => i.hold === 'boom')] });
      if (!rest.map(option).find((c) => c && c.node === r.node)) tm.taBusy = null;
    }
    if (tm.taBusy === 'TR') tm.taBusy = null;
    if (r.node === 'TA_FRONT') yield { type: 'move', goal: 'RP_TOP' };
    if (extra) this.log(r.team, 'TR hoàn tất chuyến giao thêm');
  }

  * _trMustika(r) {
    const tm = this.teams[r.team], R = r.R.TR, b = this.ball;
    if (r.carry.length) yield* this._trDeliver(r, false, true); // can cau phai trong truoc khi gap Mustika
    while (true) {
      yield { type: 'move', goal: 'MU_ST' };
      yield { type: 'wait', until: () => tm.mandate || this._mustikaGone(r.team) || this.over, max: 20, label: 'TR chờ tại Mustika Pillar (chưa đủ Mandate — 4.5.1)' };
      if (tm.mandate || this._mustikaGone(r.team) || this.over) break;
      // Mandate sap dat (2 thap chi con <= 2 khoi): o lai giu cho canh cot Mustika thay vi di lay them Earth
      const near = Object.values(this.spots).filter((sp) => sp.claimedBy === r.team).map((sp) => Math.min(3, sp.items.length)).sort((a, b) => b - a).slice(0, 2).reduce((a, b) => a + b, 0) >= 4;
      if (near) continue;
      const left = tm.storage.reduce((a, c) => a + c.items.length, 0);
      if (!left || !this.taEmpty(r.team)) continue;
      yield* this._trEarthTrip(r, Math.min(3, left), true);
    }
    if (this._mustikaGone(r.team) || this.over) { tm.mustikaHanded = true; return; }
    yield { type: 'wait', until: () => { if (this.over || this._mustikaGone(r.team)) return true; if (!this.mustikaLock || this.mustikaLock === r.id) { this.mustikaLock = r.id; return true; } return false; }, label: 'Chờ đầu gắp đối phương rời Mustika Pillar' };
    if (this._mustikaGone(r.team) || this.over) { tm.mustikaHanded = true; return; }
    const top = b.z + SIZE.mustika, tipAt = [b.x, b.y, top + R.boom.tool_drop], above = [b.x, b.y, top + R.boom.tool_drop + 120];
    r.ignore.add(b.id);
    yield {
      type: 'act', label: 'Cần cẩu gắp Mustika',
      phases: [
        ...this._boomPhases(r, () => above, 'Cần cẩu vươn tới Mustika Pillar'),
        { to: () => this._boomTo(r, tipAt) },
        { to: { fing: 1, g: SIZE.mustika }, t: R.tool.t_vac, label: '2 đĩa ôm xích đạo bóng + hút', done: () => { if (b.where !== 'pillar') return; this._detach(b); b.holder = r.id; b.hold = 'boom'; r.carry.push(b); this.log(r.team, 'TR lấy Mustika'); } },
        { to: () => this._boomTo(r, above) },
        ...this._stowPhases(r),
      ],
    };
    if (this.mustikaLock === r.id) this.mustikaLock = null;
    if (b.holder !== r.id) { tm.mustikaHanded = true; return; }
    yield { type: 'move', goal: 'RP_TOP' };
    const hx = this.geo.mx(r.team, HANDOVER.x), hy = HANDOVER.y;
    // giu bong cao hon san TA 160 mm: BR om bong ma day ham van cao hon noc than (khong phai vuon xa)
    // 2 dia cua TR nam vuong goc chieu dong ham BR (BR dung o H_TA_F)
    const brH = this.geo.mh(r.team, BR_NODES.H_TA_F[2]);
    yield { type: 'wait', until: () => this.teams[r.team].ta.every((l) => l.items.length <= 1) || this.over, max: 25, label: 'Chờ lối trên Transfer Area (chồng ≤ 1 khối) để đưa bóng' };
    yield { type: 'act', label: 'Đưa Mustika vào không gian TA', phases: this._boomPhases(r, () => [hx, hy, 600 + HANDOVER.z + SIZE.mustika + R.boom.tool_drop - 20], 'Đưa Mustika vào dải trống cạnh T2 (trong TA)', (yawDeg) => this._tw(r, yawDeg, brH - Math.PI / 2, 180)) };
    tm.mustikaReady = true;
    yield { type: 'wait', until: () => b.holder !== r.id || this.over, label: 'TR giữ Mustika trong không gian TA, chờ BR nhận' };
    tm.mustikaReady = false; tm.mustikaHanded = true;
    yield { type: 'act', label: 'Thu cần', phases: [{ to: { fing: 0, g: R.tool.g_open }, label: 'Mở đĩa, nhả Mustika' }, ...this._stowPhases(r), { done: () => { tm.trClear = true; } }] };
  }

  // ------------------------------------------------------------------ BRAIN: BR
  _brTo(r, bottomWorld) {
    const g = r.R.BR.gripper, p = K.brRobotToLevel(r.R, r.j, r.pitch, K.toRobot(this.pose(r), bottomWorld, true));
    return { lz: p[2], s: p[0] - g.x0, ly: p[1] };
  }

  // Ke hoach gap toi da 2 khoi tu TA theo vat ly kep + hut:
  //  - ca chong 2 khoi cua 1 lan (kep khoi duoi + tru hut giu khoi tren)
  //  - hut khoi tren cua lan A roi kep khoi don cua lan B (vat hut nam tren vat kep)
  //  - 1 khoi: hut khoi tren, hoac kep lan chi co 1 khoi
  _brPickPlan(team, queue) {
    const lanes = this.teams[team].ta.filter((l) => l.items.length);
    const cands = [];
    for (const L of lanes) {
      if (L.items.length === 2) cands.push([{ lane: L, mode: 'both', items: [...L.items] }]);
      cands.push([{ lane: L, mode: 'vac', items: [L.items[L.items.length - 1]] }]);
      if (L.items.length === 1) cands.push([{ lane: L, mode: 'clamp', items: [L.items[0]] }]);
      for (const M of lanes) if (M !== L && M.items.length === 1) cands.push([{ lane: L, mode: 'vac', items: [L.items[L.items.length - 1]] }, { lane: M, mode: 'clamp', items: [M.items[0]] }]);
    }
    let best = null;
    for (const acts of cands) {
      const clampItems = acts.flatMap((a) => (a.mode === 'clamp' ? a.items : a.mode === 'both' ? [a.items[0]] : []));
      const vacItems = acts.flatMap((a) => (a.mode === 'vac' ? a.items : a.mode === 'both' ? [a.items[1]] : []));
      const order = [...clampItems, ...vacItems]; // thu tu dat
      const used = new Set(), assign = [];
      for (const it of order) {
        const k = it.kind === 'earth' ? 'E' : 'S';
        const qi = queue.findIndex((q, i) => !used.has(i) && q.k === k && queue.slice(0, i).every((p, pi) => p.spot.id !== q.spot.id || used.has(pi)));
        if (qi < 0) break;
        used.add(qi); assign.push({ it, q: queue[qi], qi });
      }
      if (assign.length !== order.length) continue;
      if (!this._comboFits(team, assign, clampItems.length, vacItems.length)) continue;
      const score = assign.length * 100 - assign.reduce((a, x) => a + x.qi, 0);
      if (!best || score > best.score) best = { acts, assign, score };
    }
    return best;
  }

  // Kiem tra chieu cao: khi dat vat trong kep ma tru hut con giu vat phia tren, robot phai <= 1200 mm
  _comboFits(team, assign, nClamp, nVac) {
    if (!nClamp || !nVac) return true;
    const R = this.teams[team].R.BR, P = this.teams[team].params;
    const clampA = assign[0], vacA = assign[assign.length - 1], sp = this.spots[clampA.q.spot.id];
    const onL2 = sp.def.level === 'L2', fromL1 = onL2 && P.br_l2_from_l1 && SPOT_STAND_L1[clampA.q.spot.lid];
    const rel = (fromL1 ? 300 : 0) + sp.items.reduce((a, i) => a + hOf(i), 0);
    const top = rel + 30 + hOf(clampA.it) + hOf(vacA.it) + 10 + 60;
    return top <= 1200 && rel + 30 + R.gripper.jaw_h + R.tower.top_margin <= 1200;
  }

  * _brBrain(r) {
    const tm = this.teams[r.team], P = tm.params, goals = [...tm.plan.goals];
    yield { type: 'move', goal: 'H_TA_F' };
    this.log(r.team, 'BR đã lên L1 (xích leo cầu thang, tháp tự cân bằng)');
    const queue = [];
    let gi = 0, extra = 0;
    while (true) {
      if (tm.requeue.length) { queue.unshift(...tm.requeue); tm.requeue = []; }
      const maxT = (P.br_lookahead ? 2 : 1) + extra;
      while (gi < goals.length && goals[gi].tower && new Set(queue.map((q) => q.spot.id)).size < maxT) {
        const spot = this.claimSpot(r.team, goals[gi].tower); gi++;
        if (spot) { queue.push({ spot, k: 'E' }, { spot, k: 'E' }, { spot, k: 'S' }); this.log(r.team, `BR nhận mục tiêu tháp tại ${spot.id}`); }
      }
      if (!queue.length) {
        if (gi < goals.length && goals[gi].mustika) { gi++; yield* this._brMustika(r); continue; }
        if (gi >= goals.length) { yield { type: 'wait', until: () => this.over, label: 'BR hoàn tất kế hoạch' }; return; }
        continue;
      }
      const nextTower = () => goals.findIndex((g, j) => j >= gi && g.tower);
      const blocked = () => tm.trBlocked && !this._brPickPlan(r.team, queue) && nextTower() >= 0;
      if (tm.taBusy === 'TR' || !this._brPickPlan(r.team, queue)) { // tay TR con trong TA: TA dang thay doi, chua chot ke hoach
        if (!['H_TA_N', 'H_TA_F'].includes(r.node)) yield { type: 'move', goal: 'H_TA_F' };
        yield { type: 'wait', until: () => (tm.taBusy !== 'TR' && !!this._brPickPlan(r.team, queue)) || blocked() || this.over, label: `BR chờ TR giao ${queue[0].k === 'E' ? 'Earth' : 'Sky'}` };
      }
      if (this.over) return;
      const plan = this._brPickPlan(r.team, queue);
      if (!plan) {
        const j = nextTower();
        if (j > gi) goals.splice(gi, 0, goals.splice(j, 1)[0]); // mo thap sau Mustika de go ket TA
        extra++;
        continue;
      }
      extra = 0;
      for (const a of plan.assign) queue.splice(queue.indexOf(a.q), 1);
      for (const a of plan.acts) a.lane.resBR = true; // dat cho: TR khong tha them khoi vao lan BR sap gap (BR nhin thay khoi TR vua tha)
      for (const a of plan.acts) {
        if (r.node !== a.lane.br) yield { type: 'move', goal: a.lane.br };
        yield { type: 'wait', until: () => tm.taBusy !== 'TR' || this.over, label: 'Chờ TR rút tay khỏi TA' };
        tm.taBusy = 'BR';
        yield this._brPickAct(r, a, plan.assign);
        tm.taBusy = null; a.lane.resBR = false;
      }
      // dat: vat trong kep truoc (duoi len), vat hut sau
      while (r.carry.length && !this.over) {
        const clampIt = r.carry.filter((i) => i.hold === 'clamp'), vacIt = r.carry.find((i) => i.hold === 'vac');
        const it = clampIt[0] || vacIt, q = it.q, sp = this.spots[q.spot.id];
        const fromL1 = sp.def.level === 'L2' && it.kind === 'earth' && P.br_l2_from_l1 && SPOT_STAND_L1[q.spot.lid];
        const stand = fromL1 ? SPOT_STAND_L1[q.spot.lid] : SPOT_STAND[q.spot.lid];
        if (r.node !== stand) yield { type: 'move', goal: stand };
        const pair = it.hold === 'clamp' && vacIt && vacIt.kind === 'earth' && it.kind === 'earth' && vacIt.q.spot.id === q.spot.id && P.br_pair_place && clampIt.length === 1;
        yield this._brPlaceAct(r, it, sp, pair ? vacIt : null);
      }
    }
  }

  // Trinh tu tiep can kha thi cho dau gap chu C cua BR (ham noi khung dung phia sau, tay tru hut vuon tu tren):
  //  1) vuon X toi S_PRE (ham ra khoi mui than, chua toi vat) o do cao mang  2) dich Y  3) vuon X toi vat
  //  4) ha Z (ham truot doc 2 mat ben vat)  -> gap / tha ->  5) nang ham qua dinh khoi neu dang thap  6) rut X
  // Tiep can: (1) nang kep tren mat dich (camera / cam bien mau lam viec song song), (2) vuon X + dich Y dong thoi o do cao an toan,
  // (3) ha kep. Moi pha la 1 profile hinh thang dong bo nhieu khop.
  _brApproach(r, target, extra = {}, tSense = 0) {
    let sol;
    return [
      { to: () => {
          sol = this._brTo(r, target());
          const lz = Math.min(r.R.BR.gripper.lift[1], Math.max(r.R.BR.carry_lift, sol.lz + 30)), o = { lz, ...extra };
          if (o.pf != null && !r.carry.some((i) => i.hold === 'vac')) o.pf = Math.min(o.pf, 1195 - 35 - Math.max(lz, r.j.lz));
          return o;
        }, tmin: tSense, label: tSense ? 'Nâng kẹp + camera nhận dạng vật' : 'Nâng kẹp lên trên mặt đích (kẹp còn thu)' },
      // X + Y dong thoi toi diem mui ham con cach mat sau vat 360 mm, sau do truot thang (ham dong truc voi vat)
      { to: () => ({ s: Math.max(Math.min(r.j.s, sol.s), sol.s - 360), ly: sol.ly, lz: Math.max(sol.lz + 30, r.j.lz) }), label: 'Vươn X + dịch Y (hàm chưa tới vật)' },
      { to: () => ({ s: sol.s }), label: 'Bàn trượt X đưa kẹp tới vật (hàm trượt dọc 2 mặt bên)' },
      { to: () => ({ lz: sol.lz }), label: 'Hạ kẹp' },
    ];
  }
  _brRetract(r, stackTopWorld, side = SIZE.earth) { // side: canh khoi nam giua 2 ham
    const R = r.R.BR, g = R.gripper;
    return [
      { to: () => {
          if (r.j.lz >= R.chassis.z_top + 5) return {};
          const rel = this._brTo(r, [0, 0, stackTopWorld()]).lz, lz = Math.min(g.lift[1], Math.max(R.carry_lift, rel + 10));
          const held = r.carry.some((i) => i.hold === 'vac');
          return held ? { lz: Math.min(lz, 1200 - 60 - r.j.pf) } : { lz, pf: g.plunger[0] };
        }, label: 'Nâng hàm qua đỉnh khối' },
      // ham con ngang canh khoi (dat tren spot cao, khong nang qua dinh): X va Y chay dong bo -> ham roi khoi sau khi X lui (canh + 20) mm,
      // luc do Y moi di duoc (canh + 20)/s hanh trinh; vuot khe ho ham -> rut X truoc roi moi dich Y
      { to: () => (r.j.lz < this._brTo(r, [0, 0, stackTopWorld()]).lz + 5 && Math.min(1, (side + 20) / Math.max(1, r.j.s)) * Math.abs(r.j.ly) > (r.j.w - side) / 2 - 3 ? { s: 0 } : { s: 0, ly: 0 }), label: 'Rút kẹp về (X + Y đồng thời)' },
      { to: () => (Math.abs(r.j.ly) > 0.5 ? { ly: 0 } : {}) },
      { to: () => (r.carry.some((i) => i.hold === 'vac') ? {} : { lz: R.carry_lift, pf: g.plunger[1] - 10 }) },
    ];
  }

  _brPickAct(r, a, assign) {
    const R = r.R.BR, g = R.gripper, L = a.lane;
    for (const it of L.items) r.ignore.add(it.id);
    const tag = (it) => { it.q = assign.find((x) => x.it === it)?.q; };
    const phases = []; // camera nhan dang vat trong luc nang kep (pha dau cua _brApproach)
    const vac = r.carry.find((i) => i.hold === 'vac');
    if (a.mode === 'vac') {
      const it = a.items[0], topZ = it.z + hOf(it);
      let pf;
      phases.push(...this._brApproach(r, () => { const rel = this._brTo(r, [it.x, it.y, topZ]).lz; const lz = Math.max(0, rel - (g.plunger[1] - 40)); pf = rel - lz; return [it.x, it.y, topZ - pf]; }, { w: g.open[1], pf: g.plunger[1] - 10 }, R.sensing.t_detect));
      phases.push({ to: () => ({ pf }), t: g.t_vac, label: 'Hạ trụ hút + hút', done: () => { L.items.pop(); this._detach(it); it.holder = r.id; it.hold = 'vac'; tag(it); r.carry.push(it); } });
      phases.push({ to: { pf: g.plunger[1] - 10 }, label: 'Trụ hút nhấc vật lên trên vùng kẹp' });
      phases.push({ to: () => ({ lz: Math.min(1200 - 60 - r.j.pf, Math.max(r.j.lz, R.carry_lift)) }) });
    } else {
      const bottom = a.mode === 'both' ? L.items[0] : a.items[0];
      const pfA = vac ? Math.max(r.j.pf, hOf(bottom) + hOf(vac) + 30) : Math.max(r.j.pf, L.items.reduce((s2, i) => s2 + hOf(i), 0) + 30);
      phases.push(...this._brApproach(r, () => [bottom.x, bottom.y, bottom.z + 3], { w: g.open[1], pf: Math.min(g.plunger[1], pfA) }, R.sensing.t_detect));
      if (a.mode === 'both') phases.push({ to: { pf: hOf(L.items[0]) + hOf(L.items[1]) }, t: g.t_vac, label: 'Trụ hút giữ khối trên' });
      phases.push({
        to: { w: hOf(bottom) }, t: g.t_grip, label: 'Kẹp',
        done: () => {
          const got = L.items.splice(0, a.mode === 'both' ? 2 : 1);
          got.forEach((it, i) => { this._detach(it); it.holder = r.id; it.hold = a.mode === 'both' && i === 1 ? 'vac' : 'clamp'; tag(it); r.carry.push(it); });
          r.carry.sort((x, y) => (x.hold === 'clamp' ? 0 : 1) - (y.hold === 'clamp' ? 0 : 1));
        },
      });
      if (vac) phases.push({ to: () => ({ pf: hOf(bottom) + hOf(vac) }), label: 'Hạ trụ hút: vật trên tựa sát vật trong kẹp' });
      phases.push({ to: () => ({ lz: Math.min(g.lift[1], r.j.lz + R.carry_lift) }), label: 'Nâng khối lên cao độ mang' });
    }
    phases.push({ to: { s: 0 }, label: 'Rút kẹp về' });
    phases.push({ to: { ly: 0 } });
    return { type: 'act', label: 'BR gắp từ ' + L.id, phases };
  }

  _brPlaceAct(r, it, sp, alsoVac) {
    const tm = this.teams[r.team], R = r.R.BR, g = R.gripper, P = tm.params;
    const [sx, sy] = rectCenter(sp.def.rect), lvl = sp.def.level === 'L2' ? 900 : 600;
    const baseZ = lvl + sp.items.reduce((a, i) => a + hOf(i), 0);
    for (const x of sp.items) r.ignore.add(x.id);
    const gauss = () => { const u = Math.max(1e-9, tm.rand()), v = tm.rand(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
    const err = [gauss() * P.pos_sigma, gauss() * P.pos_sigma], tx = sx + err[0], ty = sy + err[1];
    const place = (x, z) => {
      const q = x.q, below = sp.items[sp.items.length - 1];
      const bad = !below ? (Math.abs(tx - sx) > 75 || Math.abs(ty - sy) > 75 ? 'khối nằm ngoài Building Spot 500×500' : null)
        : (Math.abs(tx - below.x) > (x.kind === 'sky' ? 75 : 110) || Math.abs(ty - below.y) > (x.kind === 'sky' ? 75 : 110) ? 'lệch khỏi mặt khối bên dưới' : null);
      r.carry.splice(r.carry.indexOf(x), 1);
      x.holder = null; x.hold = null; x.placedBy = r.team; delete x.q;
      if (x.kind === 'sky' && x.dir !== UP[r.team]) x.dir = UP[r.team];
      if (bad) {
        const off = [Math.sign(err[0] || 1) * 260, Math.sign(err[1] || 1) * 260];
        x.where = 'lost'; x.x = tx + off[0]; x.y = ty + off[1]; x.z = this.geo.floorZ(x.x, x.y); x.pitch = 0;
        tm.misplaced++; tm.lost[x.kind === 'earth' ? 'E' : 'S']++;
        tm.requeue.push(q);
        this.log(r.team, '⚠ BR đặt lệch ' + Math.round(Math.hypot(...err)) + ' mm tại ' + q.spot.id + ' → ' + bad + '; khối rơi, cần giao bù');
        return false;
      }
      x.where = 'spot'; x.spot = q.spot.id; x.z = z; x.pitch = 0; // giu x, y, huong luc nha
      sp.items.push(x);
      return true;
    };
    const after = () => { if (sp.items.length === 3) this.log(r.team, 'Hoàn thành tháp tại ' + sp.def.id + ' (' + sp.def.level + (sp.def.shared ? ', shared' : '') + ')'); this._checkMandate(r.team); };
    const stackTop = () => lvl + sp.items.reduce((a, i) => a + hOf(i), 0);
    const phases = []; // cam bien mau can spot trong luc nang kep
    const vacHeld = r.carry.find((i) => i.hold === 'vac');
    if (it.hold === 'clamp') {
      const offZ = () => (r._offs?.get(it.id)?.off[2] || 0) - 2; // bu do lech luc gap: day khoi cham mat do, khong lun
      phases.push(...this._brApproach(r, () => [tx, ty, baseZ - offZ()], vacHeld ? { pf: hOf(it) + hOf(vacHeld) + (alsoVac ? 0 : 10) } : {}, R.sensing.t_align));
      phases.push({ to: { w: g.open_release }, label: alsoVac ? 'Đặt cặp Earth 2 tầng' : 'Nhả kẹp (mở 390: hàm không chạm barrier)', done: () => { if (place(it, baseZ) && alsoVac) place(alsoVac, baseZ + hOf(it)); after(); } });
    } else {
      phases.push({ to: { pf: hOf(it), w: g.open_release }, label: 'Trụ hút hạ vật vào lòng hàm (đáy vật ngang đáy hàm)' });
      phases.push(...this._brApproach(r, () => [tx, ty, baseZ - ((r._offs?.get(it.id)?.off[2] || 0) - 2)], {}, R.sensing.t_align));
      phases.push({ t: g.t_release, label: 'Nhả hút', done: () => { place(it, baseZ); after(); } });
    }
    // con giu vat tren tru hut sau khi tha vat trong kep: khong nang ham qua dinh (robot > 1200), rut thang ra
    phases.push(...this._brRetract(r, stackTop, hOf(it)));
    return { type: 'act', label: 'BR đặt khối', phases };
  }

  * _brMustika(r) {
    const tm = this.teams[r.team], R = r.R.BR, g = R.gripper, b = this.ball;
    if (r.node !== 'H_MU') yield { type: 'move', goal: 'H_MU' };
    yield { type: 'wait', until: () => tm.mustikaReady || this._mustikaGone(r.team) || this.over, label: 'BR chờ TR mang Mustika tới TA' };
    if (!tm.mustikaReady || this.over) return;
    if (r.node !== 'H_MU') yield { type: 'move', goal: 'H_MU' };
    const tr = this.robots.find((x) => x.team === r.team && x.role === 'TR');
    let sol;
    r.ignore.add(b.id);
    tm.trClear = false;
    yield {
      type: 'act', label: 'Nhận Mustika trong Transfer Area',
      phases: [
        { t: R.sensing.t_detect, label: 'Camera nhận dạng Mustika' },
        // kep o nua tren bong (day ham cach tam bong 40 mm) de khi dat, day ham van cao hon dinh cot 800 mm
        ...this._brApproach(r, () => [b.x, b.y, b.z + SIZE.mustika / 2 - 40], { w: g.open[1], pf: g.plunger[1] }),
        { to: { w: SIZE.mustika }, t: g.t_grip, label: 'Hàm đệm mềm kẹp xích đạo bóng (cốc hút đỗ cao, tránh đầu TR)', done: () => { tr.carry.splice(tr.carry.indexOf(b), 1); b.holder = r.id; b.hold = 'clamp'; r.carry.push(b); this.log(r.team, 'BR nhận Mustika từ TR'); } },
      ],
    };
    yield { type: 'wait', until: () => tm.trClear || this.over, label: 'Chờ TR mở đĩa và rút cần khỏi bóng' };
    yield {
      type: 'act', label: 'Giữ chắc Mustika',
      phases: [
        { to: { pf: SIZE.mustika / 2 + 40 }, t: g.t_vac, label: 'Hạ cốc hút lên đỉnh bóng + hút' },
        { to: () => ({ lz: Math.max(r.j.lz + 40, R.carry_lift + 80) }), label: 'Nâng bóng cao hơn thân (đáy bóng thấp hơn đáy hàm 65 mm)' },
        { to: { s: 0 } }, { to: { ly: 0, lz: 450 } },
      ],
    };
    yield { type: 'move', goal: 'PILLAR' };
    const cp = this.spec.pillars.find((p) => p.id === 'CENTRAL_PILLAR'), seat = this._ballZ(cp);
    yield {
      type: 'act', label: 'Đặt Mustika lên Central Pillar',
      phases: [
        { t: R.sensing.t_align, label: 'Căn tâm Central Pillar' },
        // day bong phai cao hon mieng cot 10 mm khi truot ngang: day ham = day bong + 65
        { to: () => { sol = this._brTo(r, [cp.center[0], cp.center[1], seat + SIZE.mustika / 2 - 40]); return { lz: sol.lz + 61 }; }, label: 'Nâng bóng cao hơn đỉnh cột 800 mm' },
        { to: () => ({ ly: sol.ly }) },
        { to: () => ({ s: sol.s }), label: 'Đưa bóng tới trên hõm D180' },
        { to: () => ({ lz: sol.lz }), label: 'Hạ bóng vào hõm (đáy hàm vẫn cao hơn đỉnh cột 14 mm)' },
        { to: { w: g.open[1] }, t: g.t_release, label: 'Nhả bóng vào hõm D180', done: () => { r.carry.splice(r.carry.indexOf(b), 1); b.holder = null; b.hold = null; b.where = 'central'; b.placedBy = r.team; b.x = cp.center[0]; b.y = cp.center[1]; b.z = seat; tm.mustikaT = this.t; this.log(r.team, `★ Mustika đã đặt lên Central Pillar${tm.mandate ? ' (+250)' : ' — CHƯA đủ Mandate'}`); } },
        { to: () => ({ lz: sol.lz + 60 }) },
        { to: { s: 0 } },
        { to: { ly: 0, lz: R.carry_lift } },
      ],
    };
  }

  // ------------------------------------------------------------------ diem
  score(team) {
    const tm = this.teams[team], S = this.spec.scoring;
    const out = { transfer: tm.transfer, towers: [], towerPts: 0, mustika: 0, skyUp: 0, blocksL1: 0, blocksL2: 0, misplaced: tm.misplaced };
    for (const [id, sp] of Object.entries(this.spots)) {
      const tbl = S[sp.def.level], it = sp.items;
      let pts = 0, n = 0;
      if (it[0]?.kind === 'earth') {
        if (it[0].placedBy === team) { pts += tbl.earth_bottom; n++; }
        if (it[1]?.kind === 'earth') {
          if (it[1].placedBy === team) { pts += tbl.earth_middle; n++; }
          if (it[2]?.kind === 'sky' && it[2].dir === UP[team]) { pts += tbl.sky; n++; out.skyUp++; }
        }
      }
      if (n) {
        out.towers.push({ id, level: sp.def.level, shared: sp.def.shared, pts, complete: it.length === 3 && it[2].dir === UP[team] });
        out.towerPts += pts;
        if (sp.def.level === 'L2') out.blocksL2 += n; else out.blocksL1 += n;
      }
    }
    if (this.ball.where === 'central' && this.ball.placedBy === team && tm.mandate) out.mustika = S.mustika;
    out.total = out.transfer + out.towerPts + out.mustika;
    out.mandateT = tm.mandateT; out.mustikaT = tm.mustikaT; out.plan = tm.planKey;
    return out;
  }

  _end() {
    this.over = true;
    for (const r of this.robots) { r.label = 'Dừng (hết 180 s)'; if (r.carry.length) this.log(r.team, `${r.role} còn giữ ${r.carry.length} vật khi hết giờ → không tính điểm (8.6.1)`); }
    const a = this.score('red'), b = this.score('blue');
    let win = a.total > b.total ? 'red' : b.total > a.total ? 'blue' : null, why = 'tổng điểm';
    if (!win) {
      const tb = [[(s) => (s.mustika ? 1 : 0), 'Mustika'], [(s) => s.skyUp, 'số Sky đúng màu'], [(s) => s.blocksL2, 'số khối trên L2'], [(s) => s.blocksL1, 'số khối trên L1']];
      for (const [f, name] of tb) if (f(a) !== f(b)) { win = f(a) > f(b) ? 'red' : 'blue'; why = 'tie-break: ' + name; break; }
    }
    this.result = { red: a, blue: b, winner: win, why };
    this.log(null, `HẾT GIỜ — Đỏ ${a.total} : ${b.total} Xanh → ${win ? 'Thắng: ' + TEAM_VN[win] + ' (' + why + ')' : 'Hòa — trọng tài quyết định (9.3.5)'}`);
  }
}

export function runMatch(spec, cfg, robotSpec, dt = 0.05) {
  const w = new World(spec, cfg, robotSpec);
  while (!w.over) w.step(dt);
  return w;
}
export { TEAM_VN, UP };
