// Mo hinh 3D co khop cho TR v3 va BR v3. Hinh khoi va cay khau lay tu robots.js (K.links / K.frames) -
// dung chung voi kich thuoc bao, va cham cua sim va tools/check_mech.mjs: hinh ve khong the lech co cau.
// Moi khau la 1 nhom cung, chi dat ma tran (khong co chi tiet nao bi keo gian): ong long = cac doan truot long nhau.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import * as K from './robots.js';

const mm = (v) => v / 1000;
// he truc robot (x toi, y trai, z len) -> three cuc bo (x, z, -y)
const L = (x, y, z) => new THREE.Vector3(mm(x), mm(z), -mm(y));
const TEAM = { red: 0xdf2222, blue: 0x3200ff };
const S = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);
const Si = S.clone().invert();

const MAT = {
  alu: new THREE.MeshStandardMaterial({ color: 0xb8c0ca, metalness: 0.62, roughness: 0.31 }),
  aluDark: new THREE.MeshStandardMaterial({ color: 0x5a636e, metalness: 0.7, roughness: 0.34 }),
  extrusion: new THREE.MeshStandardMaterial({ color: 0x89939e, metalness: 0.78, roughness: 0.27 }),
  steel: new THREE.MeshStandardMaterial({ color: 0x343a42, metalness: 0.82, roughness: 0.3 }),
  rubber: new THREE.MeshStandardMaterial({ color: 0x17191d, roughness: 0.92 }),
  roller: new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.7 }),
  hub: new THREE.MeshStandardMaterial({ color: 0x9aa0a8, metalness: 0.8, roughness: 0.3 }),
  act: new THREE.MeshStandardMaterial({ color: 0xc48732, metalness: 0.58, roughness: 0.34 }),
  panel: new THREE.MeshStandardMaterial({ color: 0x252a31, metalness: 0.36, roughness: 0.68 }),
  pad: new THREE.MeshStandardMaterial({ color: 0x3a3f46, roughness: 0.95 }),
  black: new THREE.MeshStandardMaterial({ color: 0x0d0f12, roughness: 0.6 }),
  pcb: new THREE.MeshStandardMaterial({ color: 0x1d5e3a, roughness: 0.6 }),
  lens: new THREE.MeshStandardMaterial({ color: 0x0a0a10, metalness: 0.9, roughness: 0.1 }),
  stop: new THREE.MeshStandardMaterial({ color: 0xff1a1a, emissive: 0x440000, roughness: 0.4 }),
  cup: new THREE.MeshStandardMaterial({ color: 0x25282d, roughness: 0.8 }),
  tpu: new THREE.MeshStandardMaterial({ color: 0xf0f0ea, roughness: 0.7 }),
};
const teamMat = (team) => new THREE.MeshStandardMaterial({ color: TEAM[team], metalness: 0.2, roughness: 0.45 });

// Banh mecanum: moay-o + 2 dia + n con lan nghieng 45 do (hand = +1 / -1); truc banh = y robot
function mecanum(d, w, n, hand) {
  const g = new THREE.Group(), R = d / 2;
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(mm(R * 0.55), mm(R * 0.55), mm(w * 0.85), 20), MAT.hub);
  hub.rotation.x = Math.PI / 2; g.add(hub);
  for (const s of [-1, 1]) {
    const plate = new THREE.Mesh(new THREE.CylinderGeometry(mm(R * 0.82), mm(R * 0.82), mm(4), 24), MAT.aluDark);
    plate.rotation.x = Math.PI / 2; plate.position.z = mm((s * w) / 2 - s * 2); g.add(plate);
  }
  const rl = new THREE.CylinderGeometry(mm(R * 0.2), mm(R * 0.2), mm(w * 0.95), 10);
  for (let i = 0; i < n; i++) {
    const th = (i / n) * Math.PI * 2;
    const roller = new THREE.Mesh(rl, MAT.roller);
    roller.position.set(mm(R * 0.8 * Math.cos(th)), mm(R * 0.8 * Math.sin(th)), 0);
    const axis = new THREE.Vector3(-Math.sin(th) * Math.SQRT1_2, Math.cos(th) * Math.SQRT1_2, hand * Math.SQRT1_2).normalize();
    roller.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
    roller.castShadow = true;
    g.add(roller);
  }
  return g;
}

// Xich: bien dang profile trong mat phang x-z robot, ep dai theo y (tam y = 0)
function trackMesh(T) {
  const s = new THREE.Shape();
  const P = [[-300, 0], [200, 0], [270, 40], [315, 110], [330, T.front_rise - 20], [300, T.front_rise], [220, T.front_rise], [-300, T.height - 10], [-345, T.height - 40], [-360, 75], [-345, 35]];
  s.moveTo(mm(P[0][0]), mm(P[0][1]));
  for (const [x, z] of P.slice(1)) s.lineTo(mm(x), mm(z));
  s.closePath();
  const geo = new THREE.ExtrudeGeometry(s, { depth: mm(T.width - 8), bevelEnabled: true, bevelThickness: mm(4), bevelSize: mm(4), bevelSegments: 1 });
  geo.translate(0, 0, -mm(T.width - 8) / 2);
  const cv = document.createElement('canvas'); cv.width = 64; cv.height = 16;
  const c = cv.getContext('2d'); c.fillStyle = '#1a1c20'; c.fillRect(0, 0, 64, 16); c.fillStyle = '#3a3e45';
  for (let i = 0; i < 64; i += 8) c.fillRect(i, 0, 3, 16);
  const tex = new THREE.CanvasTexture(cv); tex.wrapS = tex.wrapT = THREE.RepeatWrapping; tex.repeat.set(40, 1);
  const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ map: tex, roughness: 0.9 })); m.castShadow = true;
  return { m, tex };
}

function primMesh(p, team, R, texOut) {
  const mat = p.m === 'team' ? teamMat(team) : MAT[p.m] || MAT.alu;
  let m;
  if (p.k === 'box') {
    const sx = mm(p.b[0] - p.a[0]), sy = mm(p.b[2] - p.a[2]), sz = mm(p.b[1] - p.a[1]);
    const rad = p.rad ? Math.min(mm(p.rad), sx / 2.2, sy / 2.2, sz / 2.2) : 0;
    m = new THREE.Mesh(rad > 0.0004 ? new RoundedBoxGeometry(sx, sy, sz, 2, rad) : new THREE.BoxGeometry(sx, sy, sz), mat);
    m.position.copy(L((p.a[0] + p.b[0]) / 2, (p.a[1] + p.b[1]) / 2, (p.a[2] + p.b[2]) / 2));
    if (p.ry) m.rotation.z = -p.ry * K.DEG;
  } else if (p.k === 'cyl') {
    m = new THREE.Mesh(new THREE.CylinderGeometry(mm(p.r2 ?? p.r), mm(p.r), mm(p.h), p.seg || 24), mat);
    if (p.ax === 'x') m.rotation.z = -Math.PI / 2;
    if (p.ax === 'y') m.rotation.x = Math.PI / 2;
    m.position.copy(L(...p.c));
  } else if (p.k === 'mecanum') {
    return mecanum(p.d, p.w, p.n, p.hand);
  } else {
    const t = trackMesh(R.BR.tracks); texOut.push(t.tex); return t.m;
  }
  m.castShadow = true; m.receiveShadow = true;
  return m;
}

function label(parent, text, team, z) {
  const d = document.createElement('div'); d.className = `rlbl ${team}`; d.textContent = text;
  const o = new CSS2DObject(d); o.position.copy(L(0, 0, z)); parent.add(o);
  return d;
}

function build(role, team, R) {
  const root = new THREE.Group(), groups = {}, texs = [];
  for (const l of K.links(role, R)) {
    const g = new THREE.Group(); g.matrixAutoUpdate = false; root.add(g); groups[l.name] = g;
    for (const p of l.prims) g.add(primMesh(p, team, R, texs));
  }
  const lbl = label(root, role, team, role === 'TR' ? 1250 : 1400);
  const tmp = new THREE.Matrix4();
  return {
    root, lbl, groups,
    update(r) {
      const F = K.frames(role, R, r.j, r.pitch, r.wheel);
      for (const [name, g] of Object.entries(groups)) {
        const A = F[name];
        tmp.set(A[0], A[1], A[2], mm(A[3]), A[4], A[5], A[6], mm(A[7]), A[8], A[9], A[10], mm(A[11]), 0, 0, 0, 1);
        g.matrix.multiplyMatrices(S, tmp).multiply(Si);
        g.matrixWorldNeedsUpdate = true;
      }
      for (const t of texs) t.offset.x = -r.track / 120;
    },
  };
}

export const buildTR = (team, R) => build('TR', team, R);
export const buildBR = (team, R) => build('BR', team, R);
