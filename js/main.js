import { FieldView } from './render.js';
import { World, runMatch, TEAM_VN } from './sim.js';
import { PLANS, DEFAULT_PARAMS, SPEC_KNOBS } from './plans.js';
import { inRect } from './geo.js';
import * as K from './robots.js';

const $ = (id) => document.getElementById(id);
const [spec, RSPEC] = await Promise.all([fetch('field_spec.json').then((r) => r.json()), fetch('robot_spec.json').then((r) => r.json())]);
const view = new FieldView($('view'), spec);

const FLAGS = [
  ['tr_sky_first', 'TR gom Sky ở chuyến đầu (lưới Sky sát ô xuất phát)'],
  ['tr_orient_sky', 'TR xoay Sky đúng màu trước khi giao (cổ tay cần cẩu)'],
  ['br_l2_from_l1', 'BR đặt Earth lên L2 khi đứng trên L1 (cần FAQ)'],
  ['br_pair_place', 'BR đặt cặp Earth 2 tầng 1 lần (cần FAQ)'],
  ['br_lookahead', 'BR mang kết hợp khối của 2 tháp'],
  ['extra_delivery', 'TR giao thêm Earth lấy điểm Transfer'],
];
const get = (o, path) => path.split('.').reduce((a, k) => a[k], o);
const JOINT_VN = {
  TR: { s: 'Bàn trượt X ống lồng (mm)', lz: 'Trục Z kẹp (mm)', w: 'Độ mở kẹp (mm)', yaw: 'Mâm xoay cần (°)', pitch: 'Góc nâng cần (°)', len: 'Cần ống lồng 5 đoạn (mm)', roll: '2 đĩa lật Sky (°)', fing: 'Ôm Mustika (0–1)', cp: 'Khớp cân bằng kẹp – ray cung (°)', tw: 'Xoay đầu công cụ (°)', g: 'Độ mở 2 ngón (mm)' },
  BR: { lz: 'Xe con trên tháp 3 tầng – đáy hàm (mm)', s: 'Bàn trượt X ống lồng (mm)', ly: 'Bàn trượt Y (mm)', w: 'Độ mở kẹp (mm)', pf: 'Cốc hút trên cột 4 tầng (mm)', tilt: 'Khớp nghiêng tháp (°)', track: 'Xích hạ (0–1)' },
};

const cfg = { red: { plan: 'E', params: { ...DEFAULT_PARAMS }, knobs: {} }, blue: { plan: 'B', params: { ...DEFAULT_PARAMS }, knobs: {} } };
let world, playing = false, speed = 1, acc = 0, logN = 0, paramTeam = 'red', sel = 'red-BR', workshop = false;
const opts = { paths: true, foot: true, env: false };

// ---------------------------------------------------------------- UI dung dong
for (const s of ['planRed', 'planBlue']) {
  for (const [k, p] of Object.entries(PLANS)) $(s).add(new Option(p.name, k));
  $(s).value = cfg[s === 'planRed' ? 'red' : 'blue'].plan;
  $(s).onchange = () => { cfg[s === 'planRed' ? 'red' : 'blue'].plan = $(s).value; reset(); };
}
for (const s of [1, 2, 4, 8]) {
  const b = document.createElement('button'); b.textContent = '×' + s; b.className = s === 1 ? 'on' : '';
  b.onclick = () => { speed = s; for (const x of $('speeds').children) x.classList.toggle('on', x === b); };
  $('speeds').appendChild(b);
}
for (const [k, t] of [['iso', 'Toàn cảnh'], ['top', 'Nhìn từ trên'], ['near', 'Khán đài'], ['ta', 'Transfer Area'], ['l2', 'L2'], ['mustika', 'Mustika Pillar'], ['pillar', 'Central Pillar']]) {
  const b = document.createElement('button'); b.textContent = t; b.onclick = () => view.setView(k); $('views').appendChild(b);
}
for (const k of ['red-TR', 'red-BR', 'blue-TR', 'blue-BR']) {
  const b = document.createElement('button'); b.textContent = 'Cận cảnh ' + k.replace('red', 'Đỏ').replace('blue', 'Xanh');
  b.onclick = () => { sel = k; view.closeUp(world, k); updatePanels(true); }; $('views').appendChild(b);
}

function renderParams() {
  const T = cfg[paramTeam], box = $('params');
  box.innerHTML = '';
  for (const [path, label, min, max, step, unit] of SPEC_KNOBS) {
    const v = T.knobs[path] ?? get(RSPEC, path);
    const d = document.createElement('div'); d.className = 'param';
    d.innerHTML = `<label>${label}</label><output>${v} ${unit}</output><input type="range" min="${min}" max="${max}" step="${step}" value="${v}">`;
    const inp = d.querySelector('input'), out = d.querySelector('output');
    inp.oninput = () => { out.textContent = `${inp.value} ${unit}`; };
    inp.onchange = () => setKnob(path, +inp.value);
    box.appendChild(d);
  }
  const sg = document.createElement('div'); sg.className = 'param';
  sg.innerHTML = `<label>Sai số đặt khối BR σ (khối lệch > 75 mm rơi khỏi spot)</label><output>${T.params.pos_sigma} mm</output><input type="range" min="0" max="45" step="1" value="${T.params.pos_sigma}">`;
  sg.querySelector('input').oninput = (e) => { sg.querySelector('output').textContent = e.target.value + ' mm'; };
  sg.querySelector('input').onchange = (e) => setParam('pos_sigma', +e.target.value);
  box.appendChild(sg);
  for (const [k, label] of FLAGS) {
    const d = document.createElement('label'); d.className = 'chk';
    d.innerHTML = `<input type="checkbox" ${T.params[k] ? 'checked' : ''}> ${label}`;
    d.querySelector('input').onchange = (e) => setParam(k, e.target.checked);
    box.appendChild(d);
  }
}
const linked = () => $('linkParams').checked && paramTeam === 'red';
function setKnob(path, v) { cfg[paramTeam].knobs[path] = v; if (linked()) cfg.blue.knobs[path] = v; reset(); }
function setParam(k, v) { cfg[paramTeam].params[k] = v; if (linked()) cfg.blue.params[k] = v; reset(); }
for (const b of document.querySelectorAll('.tabs button')) {
  b.onclick = () => { paramTeam = b.dataset.team; for (const x of document.querySelectorAll('.tabs button')) x.classList.toggle('on', x === b); renderParams(); };
}
$('linkParams').onchange = () => { if ($('linkParams').checked) { cfg.blue.params = { ...cfg.red.params }; cfg.blue.knobs = { ...cfg.red.knobs }; reset(); } };

$('play').onclick = () => togglePlay();
$('reset').onclick = () => reset();
$('optLabels').onchange = (e) => view.setLabels(e.target.checked);
$('optGrid').onchange = (e) => view.setGrid(e.target.checked);
$('optPaths').onchange = (e) => { opts.paths = e.target.checked; };
$('optFoot').onchange = (e) => { opts.foot = e.target.checked; };
$('optEnv').onchange = (e) => { opts.env = e.target.checked; };
$('optLight').onchange = (e) => { document.documentElement.dataset.theme = e.target.checked ? 'light' : 'dark'; view.scene.background.set(e.target.checked ? 0xdfe3ea : 0x1b1f27); };
addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.code === 'Space') { e.preventDefault(); togglePlay(); }
  if (e.key === 'r' || e.key === 'R') reset();
});

function togglePlay() {
  if (workshop) toggleWorkshop();
  if (world.over) reset();
  playing = !playing;
  $('play').textContent = playing ? '❚❚ Tạm dừng' : '▶ Chạy';
}
function reset() {
  world = new World(spec, JSON.parse(JSON.stringify(cfg)), RSPEC);
  acc = 0; logN = 0; $('log').innerHTML = '';
  playing = false; $('play').textContent = '▶ Chạy';
  if (workshop) buildWorkshop();
  updatePanels(true);
}

// ---------------------------------------------------------------- xuong co cau: dung tran, keo tung khop
$('workshop').onclick = () => toggleWorkshop();
$('wsRobot').onchange = () => { sel = $('wsRobot').value; view.closeUp(world, sel); buildWorkshop(); };
function toggleWorkshop() {
  workshop = !workshop;
  $('workshop').classList.toggle('on', workshop);
  $('wsPanel').hidden = !workshop;
  if (workshop) { playing = false; $('play').textContent = '▶ Chạy'; $('optEnv').checked = opts.env = true; $('wsRobot').value = sel; view.closeUp(world, sel); buildWorkshop(); }
}
function buildWorkshop() {
  const r = world.robots.find((x) => x.id === sel), lim = K.jointLimits(r.role, r.R), box = $('wsJoints');
  box.innerHTML = '';
  for (const [k, [a, b]] of Object.entries(lim)) {
    if (!(k in r.j)) continue;
    const step = b - a <= 2 ? 0.05 : 1;
    const d = document.createElement('div'); d.className = 'param';
    d.innerHTML = `<label>${JOINT_VN[r.role][k] || k}</label><output>${(+r.j[k]).toFixed(step < 1 ? 2 : 0)}</output><input type="range" min="${a}" max="${b}" step="${step}" value="${r.j[k]}">`;
    d.querySelector('input').oninput = (e) => { r.j[k] = +e.target.value; d.querySelector('output').textContent = (+e.target.value).toFixed(step < 1 ? 2 : 0); updatePanels(true); };
    box.appendChild(d);
  }
}

// ---------------------------------------------------------------- bang diem, kiem tra robot, log
function teamCard(team) {
  const s = world.over ? world.result[team] : world.score(team), tm = world.teams[team];
  const tw = s.towers.map((t) => `${t.id} ${t.pts}${t.complete ? ' ✔' : ''}`).join(' · ') || '—';
  return `<div class="card ${team}">
    <div class="kv"><span>Đội ${TEAM_VN[team]} — kế hoạch ${tm.planKey}</span><b>${s.total}</b></div>
    <div class="kv"><span>Transfer (5/khối)</span><span>${s.transfer}</span></div>
    <div class="kv"><span>Khối trên tháp</span><span>${s.towerPts}</span></div>
    <div class="kv"><span>Mustika</span><span>${s.mustika}</span></div>
    <div class="kv"><span>Sanctuary Mandate</span><span>${tm.mandate ? '✔ ' + tm.mandateT.toFixed(1) + ' s' : 'chưa'}</span></div>
    <div class="kv"><span>Khối đặt trượt</span><span>${s.misplaced}</span></div>
    <div class="tw">${tw}</div></div>`;
}
function inspector() {
  const r = world.robots.find((x) => x.id === sel);
  const items = r.carry.map((i) => ({ kind: i.kind, hold: i.hold }));
  const e = K.envelope(r.role, r.R, r.j, items), hz = [e.L, e.W].sort((a, b) => b - a);
  const okS = hz[0] <= 1400.5 && hz[1] <= 1000.5 && e.H <= 1200.5;
  const cg = K.cog(r.role, r.R, r.j, items), tip = K.tipAngle(r.role, r.R, r.j, items);
  const st = r.startEnv;
  const jrows = Object.entries(r.j).map(([k, v]) => `<div class="kv"><span>${JOINT_VN[r.role][k] || k}</span><span>${(+v).toFixed(Math.abs(v) < 2 && k !== 'lz' ? 2 : 0)}</span></div>`).join('');
  return `<div class="card ${r.team}">
    <div class="kv"><span><b>${TEAM_VN[r.team]} ${r.role}</b> — ${r.R[r.role].name.split('—')[1] || ''}</span></div>
    <div class="tw">${r.label}</div>
    <div class="kv"><span>Kích thước hiện thời L×W×H</span><span class="${okS ? 'ok' : 'bad'}">${Math.round(e.L)}×${Math.round(e.W)}×${Math.round(e.H)}</span></div>
    <div class="kv"><span>Lúc xuất phát (≤ 700³)</span><span class="${st.L <= 700.5 && st.W <= 700.5 && st.H <= 700.5 ? 'ok' : 'bad'}">${Math.round(st.L)}×${Math.round(st.W)}×${Math.round(st.H)}</span></div>
    <div class="kv"><span>Khối lượng (kể cả vật mang)</span><span>${cg.m.toFixed(1)} kg ${cg.m <= 50 ? '' : '⚠'}</span></div>
    <div class="kv"><span>Trọng tâm (x, z)</span><span>${Math.round(cg.p[0])}, ${Math.round(cg.p[2])} mm</span></div>
    <div class="kv"><span>Góc lật tới / lui · đang nghiêng</span><span>${tip.fwd.toFixed(0)}° / ${tip.back.toFixed(0)}° · ${(r.pitch / K.DEG).toFixed(1)}°</span></div>
    <details><summary>Khớp</summary>${jrows}</details></div>`;
}
function updatePanels() {
  const t = world.t, left = Math.max(0, world.T - t);
  $('timer').textContent = `${Math.floor(left / 60)}:${String(Math.floor(left % 60)).padStart(2, '0')}`;
  const a = world.over ? world.result.red : world.score('red'), b = world.over ? world.result.blue : world.score('blue');
  $('sRed').textContent = a.total; $('sBlue').textContent = b.total;
  $('board').innerHTML = teamCard('red') + teamCard('blue') + (world.over ? `<div class="card"><b>${world.result.winner ? 'Thắng: ' + TEAM_VN[world.result.winner] : 'Hòa'}</b> <span class="note">(${world.result.why})</span></div>` : '');
  $('insp').innerHTML = inspector();
  $('robots').innerHTML = world.robots.map((r) => `<div class="robot" data-id="${r.id}"><b style="color:var(--${r.team}-t)">${TEAM_VN[r.team]} ${r.role}</b> ${r.label}${r.carry.length ? ` <span class="note">[${r.carry.map((i) => (i.kind === 'earth' ? 'E' : i.kind === 'sky' ? 'S' : 'M') + (i.hold === 'boom' ? '↑' : i.hold === 'vac' ? '◎' : '')).join('')}]</span>` : ''}</div>`).join('');
  for (const d of $('robots').children) d.onclick = () => { sel = d.dataset.id; updatePanels(); };
  const nv = world.violations.length, nc = world.collisions.length;
  $('viol').className = 'card viol' + (nv || nc ? ' bad' : '');
  $('viol').innerHTML = `${nv ? `<b>${nv}</b> vi phạm luật` : '0 vi phạm luật'} · ${nc ? `<b>${nc}</b> va chạm vật lý` : '0 va chạm vật lý'}<div class="note">Luật: vùng hoạt động 6.2, kích thước 11.4–11.6, số khối 4.4.2 · Vật lý: thân–tường/bậc/rào/cột/khối, hàm kẹp–khối/sàn, robot–robot</div>`;
  if (world.events.length > logN) {
    const frag = document.createDocumentFragment();
    for (; logN < world.events.length; logN++) {
      const e = world.events[logN], d = document.createElement('div');
      d.className = /VI PHẠM|VA CHẠM|lệch/.test(e.msg) ? 'warn' : e.team || '';
      d.textContent = `${e.t.toFixed(1).padStart(5)}s ${e.msg}`;
      frag.appendChild(d);
    }
    $('log').appendChild(frag);
    $('log').scrollTop = $('log').scrollHeight;
  }
}

// ---------------------------------------------------------------- so sanh chien thuat
$('compare').onclick = () => {
  $('cmp').showModal();
  $('cmpBody').textContent = `Đang mô phỏng ${Object.keys(PLANS).length ** 2} trận…`;
  setTimeout(() => {
    const keys = Object.keys(PLANS);
    let html = '<table><tr><th>Đỏ \\ Xanh</th>' + keys.map((k) => `<th>${k}</th>`).join('') + '</tr>';
    for (const a of keys) {
      html += `<tr><th title="${PLANS[a].name}">${a}</th>`;
      for (const b of keys) {
        const w = runMatch(spec, { red: { ...cfg.red, plan: a }, blue: { ...cfg.blue, plan: b } }, RSPEC, 0.05), r = w.result;
        html += `<td class="${r.winner === 'red' ? 'win' : ''}">${r.red.total} : ${r.blue.total}</td>`;
      }
      html += '</tr>';
    }
    html += '</table><p class="note">' + keys.map((k) => `<b>${k}</b> = ${PLANS[k].name}`).join('<br>') + '<br>Ô xanh lá = Đỏ thắng.</p>';
    $('cmpBody').innerHTML = html;
  }, 30);
};

// ---------------------------------------------------------------- di chuot: toa do + khu vuc
function zoneAt(p) {
  const lvl = p.z > 880 ? 'L2' : p.z > 580 ? 'L1' : 'ground';
  const spot = spec.building_spots.find((s) => s.level === lvl && inRect(s.rect, p.x, p.y));
  if (spot) return 'Building Spot ' + spot.id;
  const hits = spec.areas.filter((a) => a.level === lvl && inRect(a.rect, p.x, p.y));
  const best = hits.sort((u, v) => (u.rect[2] - u.rect[0]) * (u.rect[3] - u.rect[1]) - (v.rect[2] - v.rect[0]) * (v.rect[3] - v.rect[1]))[0];
  if (best) return best.name;
  if (lvl !== 'ground') return lvl === 'L1' ? 'Level 1' : 'Level 2';
  return p.z > 5 ? 'Cầu thang / dốc' : 'Ground Area';
}
$('view').addEventListener('pointermove', (e) => {
  const p = view.pick(e);
  if (!p) { $('hoverXY').textContent = '—'; $('hoverZone').textContent = '—'; return; }
  $('hoverXY').textContent = `x ${Math.round(p.x)} · y ${Math.round(p.y)} · z ${Math.round(p.z)} mm`;
  $('hoverZone').textContent = zoneAt(p);
});

// ---------------------------------------------------------------- vong lap
const Q = new URLSearchParams(location.search);
for (const team of ['red', 'blue']) if (PLANS[Q.get(team)]) { cfg[team].plan = Q.get(team); $(team === 'red' ? 'planRed' : 'planBlue').value = Q.get(team); }
reset();
renderParams();
if (Q.get('t')) { const T = Math.min(+Q.get('t'), world.T); while (world.t < T - 1e-9 && !world.over) world.step(1 / 60); }
if (Q.get('sel')) sel = Q.get('sel');
if (Q.get('env') === '1') { opts.env = true; $('optEnv').checked = true; }
if (Q.get('view')) view.setView(Q.get('view'));
if (Q.get('close')) { sel = Q.get('close'); view.closeUp(world, sel); }
if (Q.get('play') === '1') togglePlay();
updatePanels(true);
let last = performance.now(), lastUI = 0;
function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (playing && !world.over) {
    acc += dt * speed;
    let n = 0;
    while (acc >= 1 / 60 && n++ < 600) { world.step(1 / 60); acc -= 1 / 60; }
    if (world.over) { playing = false; $('play').textContent = '▶ Chạy lại'; updatePanels(true); }
  }
  view.sync(world, opts);
  view.render(world);
  if (now - lastUI > 150) { updatePanels(); lastUI = now; }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
window.__twin = { get world() { return world; }, view, cfg };
