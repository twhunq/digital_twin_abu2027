// Dung canh 3D tu field_spec.json va dong bo vat the dong tu World (sim.js).
// Quy uoc: toa do san (x, y, z) mm -> three.js (m): X = (x-5500)/1000, Y = z/1000, Z = -(y-5500)/1000.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { buildTR, buildBR } from './robot3d.js';
import * as K from './robots.js';

const C = (rgb) => new THREE.Color(`rgb(${rgb[0]},${rgb[1]},${rgb[2]})`);
const mm = (v) => v / 1000;
const TEAM_RGB = { red: [223, 34, 34], blue: [50, 0, 255] };
const disposeChildren = (g) => { for (const c of [...g.children]) { c.geometry?.dispose(); c.material?.dispose(); g.remove(c); } };

export class FieldView {
  constructor(container, spec) {
    this.spec = spec;
    this.container = container;
    this.cx = spec.field.size[0] / 2;
    this.cy = spec.field.size[1] / 2;
    this.pickables = [];
    this.labels = [];
    this.itemMeshes = new Map();
    this.robotMeshes = new Map();

    const r = (this.renderer = new THREE.WebGLRenderer({ antialias: true }));
    r.setPixelRatio(Math.min(2, window.devicePixelRatio));
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(r.domElement);
    this.css = new CSS2DRenderer();
    this.css.domElement.className = 'css2d';
    container.appendChild(this.css.domElement);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1b1f27);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.05, 200);
    this.controls = new OrbitControls(this.camera, this.css.domElement);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI * 0.495;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x445066, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(-6, 14, 8);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -8, right: 8, top: 8, bottom: -8, near: 1, far: 40 });
    sun.shadow.bias = -0.0005;
    this.scene.add(sun);

    this.field = new THREE.Group();
    this.scene.add(this.field);
    this.dyn = new THREE.Group();
    this.scene.add(this.dyn);
    this._buildField();
    this._buildGrid();
    this.setView('iso');

    this.pathLines = new THREE.Group();
    this.scene.add(this.pathLines);
    this.footprints = new THREE.Group();
    this.scene.add(this.footprints);

    this.raycaster = new THREE.Raycaster();
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
  }

  V(x, y, z = 0) { return new THREE.Vector3((x - this.cx) / 1000, z / 1000, -(y - this.cy) / 1000); }
  toField(p) { return { x: p.x * 1000 + this.cx, y: -p.z * 1000 + this.cy, z: p.y * 1000 }; }

  // hop theo hinh chu nhat mat bang [x0,y0,x1,y1] tu z0 den z1 (mm)
  box(rect, z0, z1, color, opts = {}) {
    const [x0, y0, x1, y1] = rect;
    const g = new THREE.BoxGeometry((x1 - x0) / 1000, Math.max(0.0005, (z1 - z0) / 1000), (y1 - y0) / 1000);
    const m = new THREE.MeshStandardMaterial({ color: color instanceof THREE.Color ? color : C(color), roughness: 0.85, metalness: 0, ...(opts.mat || {}) });
    const mesh = new THREE.Mesh(g, m);
    mesh.position.copy(this.V((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2));
    mesh.castShadow = opts.cast ?? true;
    mesh.receiveShadow = true;
    (opts.parent || this.field).add(mesh);
    if (opts.pick !== false) this.pickables.push(mesh);
    return mesh;
  }
  paint(rect, z, color, lift = 1) { // lop son mong tren mat, nang nhe tranh z-fighting
    return this.box(rect, z + lift - 1, z + lift + 1, color, { cast: false, mat: { polygonOffset: true, polygonOffsetFactor: -1 } });
  }
  label(text, x, y, z, cls = 'lbl') {
    const d = document.createElement('div');
    d.className = cls; d.textContent = text;
    const o = new CSS2DObject(d);
    o.position.copy(this.V(x, y, z));
    this.field.add(o);
    this.labels.push(o);
    return o;
  }

  _buildField() {
    const S = this.spec, W = S.field.size[0], H = S.field.size[1];
    const A = Object.fromEntries(S.areas.map((a) => [a.id, a]));
    // nen nha thi dau
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), new THREE.MeshStandardMaterial({ color: 0x2a2f38, roughness: 1 }));
    floor.rotation.x = -Math.PI / 2; floor.position.y = -0.021; floor.receiveShadow = true;
    this.scene.add(floor);
    // Ground Area (2 nua) day 20 mm
    this.box(A.GROUND_RED.rect, -20, 0, A.GROUND_RED.color, { cast: false });
    this.box(A.GROUND_BLUE.rect, -20, 0, A.GROUND_BLUE.color, { cast: false });
    // vung son tren mat dat
    for (const a of S.areas) {
      if (a.level !== 'ground' || !a.kind) continue;
      this.paint(a.rect, 0, a.color);
      if (a.outline) this._outline(a.rect, 3, a.outline);
    }
    // luoi 5x5 cua Ground Shared Area
    const sg = S.initial_layout.sky_grid, [ix0, iy0, ix1, iy1] = sg.inner_rect;
    const pts = [];
    for (let i = 0; i <= 5; i++) {
      const x = ix0 + i * sg.cell, y = iy0 + i * sg.cell;
      pts.push(this.V(x, iy0, 3), this.V(x, iy1, 3), this.V(ix0, y, 3), this.V(ix1, y, 3));
    }
    this.field.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x9a947c })));
    this.paint([ix0 + 2 * sg.cell, iy0 + 2 * sg.cell, ix0 + 3 * sg.cell, iy0 + 3 * sg.cell], 0, [150, 100, 20], 3);

    // L1: than + mat son
    const L1 = S.levels.L1, L2 = S.levels.L2, wall = S.walls.L1_side_wall_color;
    this.box(L1.rect, 0, L1.z - 2, wall);
    for (const a of S.areas) if (a.level === 'L1' && a.kind !== 'transfer') this.paint(a.rect, L1.z, a.color, a.kind ? 3 : 1);
    // Transfer Area, cau thang, doc
    for (const id of ['TA_RED', 'TA_BLUE']) { this.box(A[id].rect, 0, L1.z - 2, wall); this.paint(A[id].rect, L1.z, A[id].color); }
    for (const st of S.transitions.stairs_L1) for (const s of st.steps) this.box([st.x[0], s.y[0], st.x[1], s.y[1]], 0, s.z, st.color);
    for (const rp of S.transitions.ramps) this._ramp(rp);
    // L2
    this.box(L2.rect, L1.z, L2.z, S.walls.L2_side_wall_color);
    for (const st of S.transitions.stairs_L2) this.box(st.rect, L1.z, st.z, st.color);
    for (const s of S.building_spots) {
      const z = S.levels[s.level].z;
      this.paint(s.rect, z, S.building_spot_color, 5);
    }
    // hang rao / barrier
    for (const f of S.fences) {
      const [ax, ay] = f.from, [bx, by] = f.to, t = f.thickness / 2;
      const rect = ax === bx ? [ax - t, Math.min(ay, by), ax + t, Math.max(ay, by)] : [Math.min(ax, bx), ay - t, Math.max(ax, bx), ay + t];
      this.box(rect, f.z0, f.z0 + f.height, f.kind === 'center_divider' ? S.walls.fence_color : S.walls.barrier_color);
    }
    const b = S.field.boundary, t = b.thickness;
    for (const rc of [[-t, -t, W + t, 0], [-t, H, W + t, H + t], [-t, 0, 0, H], [W, 0, W + t, H]]) this.box(rc, -20, b.height, b.color);
    // cot
    for (const p of S.pillars) {
      const h = p.height / 1000, R = p.diameter / 2000;
      const cyl = new THREE.Mesh(new THREE.CylinderGeometry(R, R, h, 40), new THREE.MeshStandardMaterial({ color: C(p.color), roughness: 0.6 }));
      cyl.position.copy(this.V(p.center[0], p.center[1], p.z0 + p.height / 2));
      cyl.castShadow = true; this.field.add(cyl); this.pickables.push(cyl);
      const cup = new THREE.Mesh(new THREE.CircleGeometry(p.cup_diameter / 2000, 32), new THREE.MeshStandardMaterial({ color: 0x1a1208 }));
      cup.rotation.x = -Math.PI / 2; cup.position.copy(this.V(p.center[0], p.center[1], p.z0 + p.height + 1));
      this.field.add(cup);
    }

    // nhan
    const L = (t, id, dz = 40) => { const a = A[id]; this.label(t, (a.rect[0] + a.rect[2]) / 2, (a.rect[1] + a.rect[3]) / 2, (a.level === 'ground' ? 0 : a.level === 'L1' ? 600 : 900) + dz); };
    L('Storage', 'STORAGE_RED', 800); L('Storage', 'STORAGE_BLUE', 800);
    L('Start R1', 'SZ_R1'); L('Start R2', 'SZ_R2'); L('Start B2', 'SZ_B2'); L('Start B1', 'SZ_B1');
    L('Ground Shared', 'GROUND_SHARED', 350); L('Mustika Pillar', 'MUSTIKA_SHARED', 800);
    L('Transfer Area', 'TA_RED', 60); L('Transfer Area', 'TA_BLUE', 60);
    L('L1 Retry', 'L1_RETRY_RED'); L('L1 Retry', 'L1_RETRY_BLUE');
    L('L1 Shared', 'L1_SHARED_NEAR'); L('L1 Shared', 'L1_SHARED_FAR');
    L('L2 (shared) · Central Pillar', 'L2_AREA', 1000);
    this.label('Ramp 3500', 2000, 6400, 450); this.label('Ramp 3500', 9000, 6400, 450);
    this.label('Stairs 3×150', 2000, 3200, 500); this.label('Stairs 3×150', 9000, 3200, 500);
    for (const s of S.building_spots) this.label(s.id, (s.rect[0] + s.rect[2]) / 2, (s.rect[1] + s.rect[3]) / 2, S.levels[s.level].z + 30, 'lbl spot');
  }

  _outline(rect, z, color) {
    const [x0, y0, x1, y1] = rect;
    const g = new THREE.BufferGeometry().setFromPoints([this.V(x0, y0, z), this.V(x1, y0, z), this.V(x1, y1, z), this.V(x0, y1, z), this.V(x0, y0, z)]);
    this.field.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color: C(color) })));
  }

  _ramp(rp) {
    const [x0, x1] = rp.x, v = [
      [x0, rp.y_top, 0], [x1, rp.y_top, 0], [x0, rp.y_top, rp.z_top], [x1, rp.y_top, rp.z_top], [x0, rp.y_bottom, 0], [x1, rp.y_bottom, 0],
    ].map(([x, y, z]) => this.V(x, y, z));
    const tri = [[2, 3, 4], [3, 5, 4], [0, 4, 5], [0, 5, 1], [0, 2, 4], [1, 5, 3], [0, 1, 3], [0, 3, 2]];
    const pos = [];
    for (const t of tri) for (const i of t) pos.push(v[i].x, v[i].y, v[i].z);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: C(rp.color), roughness: 0.85, side: THREE.DoubleSide }));
    m.castShadow = true; m.receiveShadow = true;
    this.field.add(m); this.pickables.push(m);
  }

  _buildGrid() {
    const pts = [], W = this.spec.field.size[0];
    for (let i = 0; i <= 11; i++) {
      pts.push(this.V(i * 1000, 0, 4), this.V(i * 1000, W, 4), this.V(0, i * 1000, 4), this.V(W, i * 1000, 4));
    }
    this.grid = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.18 }));
    this.grid.visible = false;
    this.scene.add(this.grid);
    this.axes = new THREE.Group();
    const ax = (to, col) => this.axes.add(new THREE.ArrowHelper(to.clone().sub(this.V(0, 0, 0)).normalize(), this.V(0, 0, 5), 1.2, col, 0.25, 0.12));
    ax(this.V(1000, 0, 0), 0xff4040); ax(this.V(0, 1000, 0), 0x40ff40); ax(this.V(0, 0, 1000), 0x4080ff);
    const o = document.createElement('div'); o.className = 'lbl origin'; o.textContent = 'O (0,0) · x→ · y↑';
    const lo = new CSS2DObject(o); lo.position.copy(this.V(0, 0, 300)); this.axes.add(lo);
    this.scene.add(this.axes);
  }

  setView(name, target) {
    const c = this.camera, ctl = this.controls;
    const views = {
      iso: [this.V(-3800, -6400, 11800), this.V(5500, 5200, 300)],
      top: [this.V(5500, 5499, 17500), this.V(5500, 5500, 0)],
      near: [this.V(5500, -5200, 4200), this.V(5500, 4500, 600)],
      red: [this.V(-4500, 5500, 5200), this.V(5000, 5500, 600)],
      ta: [this.V(-600, 1500, 2600), this.V(2200, 4300, 600)],
      l2: [this.V(2400, 2600, 3400), this.V(5500, 5500, 900)],
      mustika: [this.V(3800, 8300, 1600), this.V(5500, 9750, 500)],
      pillar: [this.V(3600, 3900, 2600), this.V(5500, 5500, 1600)],
    };
    const v = views[name] || views.iso;
    c.position.copy(v[0]); ctl.target.copy(v[1]); ctl.update();
    this.follow = null;
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h); this.css.setSize(w, h);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  // ------------------------------------------------------------------ vat the dong
  _itemMesh(it) {
    const S = this.spec.objects;
    let obj;
    if (it.kind === 'earth') {
      const s = S.earth_block.size / 1000;
      const g = new THREE.BoxGeometry(s, s, s);
      obj = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: C(TEAM_RGB[it.team]), roughness: 0.7 }));
      obj.add(new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({ color: 0x111111 })));
      obj.castShadow = true;
    } else if (it.kind === 'sky') {
      const s = S.sky_block.size / 1000;
      obj = new THREE.Group();
      for (const [sgn, col] of [[1, S.sky_block.face_red], [-1, S.sky_block.face_blue]]) {
        const g = new THREE.BoxGeometry(s / 2, s, s);
        const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: C(col), roughness: 0.6 }));
        m.position.x = (sgn * s) / 4; m.castShadow = true;
        m.add(new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({ color: 0x111111 })));
        obj.add(m);
      }
    } else {
      const cv = document.createElement('canvas'); cv.width = 256; cv.height = 128;
      const x = cv.getContext('2d');
      const band = ['#f2c200', '#1d4fa0', '#f2c200', '#1d4fa0', '#f2c200', '#1d4fa0'];
      band.forEach((c, i) => { x.fillStyle = c; x.fillRect(0, (i * 128) / band.length, 256, 128 / band.length + 1); });
      const tex = new THREE.CanvasTexture(cv); tex.colorSpace = THREE.SRGBColorSpace;
      obj = new THREE.Mesh(new THREE.SphereGeometry(S.mustika.diameter / 2000, 32, 20), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.5 }));
      obj.castShadow = true;
    }
    this.dyn.add(obj);
    return obj;
  }

  _robotMesh(r) {
    const m = r.role === 'TR' ? buildTR(r.team, r.R) : buildBR(r.team, r.R);
    this.dyn.add(m.root);
    return m;
  }

  sync(world, opts) {
    const DIR = { '+x': [1, 0, 0], '-x': [-1, 0, 0], '+y': [0, 0, -1], '-y': [0, 0, 1], '+z': [0, 1, 0], '-z': [0, -1, 0] };
    const q = (d) => new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), new THREE.Vector3(...DIR[d]));
    for (const it of world.items) {
      let m = this.itemMeshes.get(it.id);
      if (!m) { m = this._itemMesh(it); this.itemMeshes.set(it.id, m); }
      const h = K.SIZE[it.kind];
      m.position.copy(this.V(it.x, it.y, it.z + h / 2));
      if (it.kind === 'sky') { // huong nua mau (dir) tinh trong he khoi; khoi xoay quanh truc dung theo yaw
        const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), it.yaw || 0);
        if (it.flip) { // 2 dia xoay lat khoi 90 do quanh truc (from x to) qua tam khoi
          const a = new THREE.Vector3(...DIR[it.flip.from]), b = new THREE.Vector3(...DIR[it.flip.to]);
          const qa = new THREE.Quaternion().setFromAxisAngle(a.clone().cross(b).normalize(), (it.flip.f * Math.PI) / 2);
          m.quaternion.copy(qy).multiply(qa).multiply(q(it.flip.from));
        } else m.quaternion.copy(qy).multiply(q(it.dir));
      } else if (it.kind === 'earth') { m.rotation.order = 'YZX'; m.rotation.set(0, it.yaw || 0, it.pitch || 0); }
      m.visible = it.where !== 'lost' || true;
      if (m.material) m.material.opacity = 1;
    }
    for (const r of world.robots) {
      let rm = this.robotMeshes.get(r.id);
      if (!rm) { rm = this._robotMesh(r); this.robotMeshes.set(r.id, rm); }
      rm.root.position.copy(this.V(r.x, r.y, r.z));
      rm.root.rotation.set(0, r.h, 0);
      rm.update(r);
      rm.lbl.textContent = r.role + (r.carry.length ? ' ·' + r.carry.length : '');
    }
    this._paths(world, opts.paths);
    this._foot(world, opts.foot);
    this._env(world, opts.env);
  }

  _paths(world, on) {
    disposeChildren(this.pathLines);
    if (!on) return;
    for (const r of world.robots) {
      const t = r.task;
      if (!t || t.type !== 'move' || !t.segs) continue;
      const pts = [this.V(r.x, r.y, r.z + 30)];
      for (let i = t.i; i < t.segs.length; i++) { const s = t.segs[i]; for (const [x, y] of s.pts ? s.pts.slice(1) : [[s.x1, s.y1]]) pts.push(this.V(x, y, world.geo.floorZ(x, y) + 30)); }
      if (pts.length < 2) continue;
      const col = r.team === 'red' ? 0xff6b6b : 0x8f84ff;
      const ln = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color: col, dashSize: 0.12, gapSize: 0.08 }));
      ln.computeLineDistances();
      this.pathLines.add(ln);
    }
  }

  // Hinh chieu dung that cua robot (than + co cau + vat mang), vang = hop le, do = vua vi pham
  _foot(world, on) {
    disposeChildren(this.footprints);
    if (!on) return;
    for (const r of world.robots) {
      const items = r.carry.map((i) => ({ kind: i.kind, hold: i.hold }));
      const bad = world.violations.some((v) => v.robot === r.id && world.t - v.t < 1.5);
      const col = bad ? 0xff2222 : 0xfff27a;
      const P = { x: r.x, y: r.y, z: r.z, h: r.h, pitch: 0 }, zf = world.geo.floorZ(r.x, r.y) + 6;
      const c = r.R[r.role].chassis;
      const boxes = [{ x0: -c.length / 2, x1: c.length / 2, y0: -c.width / 2, y1: c.width / 2 }, ...K.parts(r.role, r.R, r.j, items)];
      const pts = [];
      for (const b of boxes) {
        const k = [[b.x0, b.y0], [b.x1, b.y0], [b.x1, b.y1], [b.x0, b.y1], [b.x0, b.y0]].map(([x, y]) => { const w = K.toWorld(P, [x, y, 0], true); return this.V(w[0], w[1], zf); });
        for (let i = 0; i < 4; i++) pts.push(k[i], k[i + 1]);
      }
      this.footprints.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.8 })));
    }
  }
  // Khung kich thuoc hien thoi (he truc robot) so voi gioi han 1400 x 1000 x 1200 (xanh la / do) va 700^3 luc xuat phat
  _env(world, on) {
    this.envBoxes ??= new THREE.Group(); if (!this.envBoxes.parent) this.scene.add(this.envBoxes);
    disposeChildren(this.envBoxes);
    if (!on) return;
    for (const r of world.robots) {
      const e = r.env || K.envelope(r.role, r.R, r.j, []);
      const hz = [e.L, e.W].sort((a, b) => b - a), ok = hz[0] <= 1400.5 && hz[1] <= 1000.5 && e.H <= 1200.5;
      const start = world.t < 0.05;
      const lim = start ? { x0: -350, x1: 350, y0: -350, y1: 350, z1: 700 } : e;
      const g = new THREE.BoxGeometry(mm(lim.x1 - lim.x0), mm(lim.z1), mm(lim.y1 - lim.y0));
      const ln = new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({ color: start ? 0x39c5ff : ok ? 0x3ecf8e : 0xff3030 }));
      g.dispose();
      const P = { x: r.x, y: r.y, z: r.z, h: r.h, pitch: 0 }, cw = K.toWorld(P, [(lim.x0 + lim.x1) / 2, (lim.y0 + lim.y1) / 2, 0], true);
      ln.position.copy(this.V(cw[0], cw[1], cw[2] + lim.z1 / 2));
      ln.rotation.y = r.h;
      this.envBoxes.add(ln);
    }
  }

  followRobot(world, id) { this.follow = id; }
  // Camera can canh mot robot (xuong co cau)
  closeUp(world, id) {
    const r = world.robots.find((x) => x.id === id); if (!r) return;
    const c = Math.cos(r.h), s = Math.sin(r.h);
    this.camera.position.copy(this.V(r.x + 1400 * c - 1500 * s, r.y + 1400 * s + 1500 * c, r.z + 1300));
    this.controls.target.copy(this.V(r.x + 200 * c, r.y + 200 * s, r.z + 500));
    this.follow = id; this.controls.update();
  }
  render(world) {
    if (this.follow && world) {
      const r = world.robots.find((x) => x.id === this.follow);
      if (r) { // camera di theo robot, giu nguyen goc nhin
        const p = this.V(r.x, r.y, r.z + 400), d = p.clone().sub(this.controls.target).multiplyScalar(0.15);
        this.controls.target.add(d); this.camera.position.add(d);
      }
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.css.render(this.scene, this.camera);
  }

  pick(ev) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const m = new THREE.Vector2(((ev.clientX - rect.left) / rect.width) * 2 - 1, -((ev.clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(m, this.camera);
    const hit = this.raycaster.intersectObjects(this.pickables, false)[0];
    return hit ? this.toField(hit.point) : null;
  }
  setLabels(on) { for (const l of this.labels) l.visible = on; }
  setGrid(on) { this.grid.visible = on; this.axes.visible = on; }
}
