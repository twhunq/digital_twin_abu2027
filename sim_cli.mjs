// Chay mo phong khong giao dien (Node >= 18):
//   node digital_twin/sim_cli.mjs                  -> bang chien thuat + ma tran doi dau
//   node digital_twin/sim_cli.mjs E B --log        -> 1 tran Do=E, Xanh=B, in nhat ky
//   node digital_twin/sim_cli.mjs --sens [--md]    -> do nhay co cau (Do = E, Xanh = D)
//   node digital_twin/sim_cli.mjs --md             -> bang Markdown
//   --set TR.drive.v_max=2.5   ghi de thong so co cau (robot_spec.json) cho ca hai doi
//   --param pos_sigma=25       ghi de tuy chon chien thuat cho ca hai doi
import { readFileSync } from 'node:fs';
import { runMatch } from './js/sim.js';
import { PLANS } from './js/plans.js';

const spec = JSON.parse(readFileSync(new URL('./field_spec.json', import.meta.url), 'utf-8'));
const R = JSON.parse(readFileSync(new URL('./robot_spec.json', import.meta.url), 'utf-8'));
const args = process.argv.slice(2);
const plans = args.filter((a) => PLANS[a]);
const fmt = (t) => (t == null ? '—' : t.toFixed(1) + ' s');
const val = (v) => (v === 'true' ? true : v === 'false' ? false : +v);
const knobs = {}, params = {};
args.forEach((a, i) => {
  if (a === '--set') { const [k, v] = args[i + 1].split('='); knobs[k] = val(v); }
  if (a === '--param') { const [k, v] = args[i + 1].split('='); params[k] = val(v); }
});
const team = (plan, k = {}, p = {}) => ({ plan, knobs: { ...knobs, ...k }, params: { ...params, ...p } });
const run = (a, b) => runMatch(spec, { red: a, blue: b }, R);
const md = args.includes('--md');

if (args.includes('--sens')) {
  const cases = [
    ['Thiết kế v3 mặc định', {}, {}],
    ['BR leo xích 0,45 m/s (nhanh)', { 'BR.tracks.v_climb': 0.45 }, {}],
    ['BR leo xích 0,20 m/s (chậm)', { 'BR.tracks.v_climb': 0.2 }, {}],
    ['BR tháp nâng trần 350 mm/s (thiết kế ~700)', { 'BR.actuators.lz.cap_v': 350 }, {}],
    ['BR nhận dạng + căn spot nhanh (0,2 s)', { 'BR.sensing.t_detect': 0.2, 'BR.sensing.t_align': 0.2 }, {}],
    ['BR vận tốc 2,0 m/s', { 'BR.drive.v_max': 2.0 }, {}],
    ['TR vận tốc 3,0 m/s', { 'TR.drive.v_max': 3.0 }, {}],
    ['TR ống lồng + mâm xoay chậm 50%', { 'TR.actuators.len.cap_v': 450, 'TR.actuators.yaw.cap_v': 150 }, {}],
    ['TR bàn trượt X chậm 50%', { 'TR.actuators.s.cap_v': 400 }, {}],
    ['KHÔNG đặt Earth lên L2 từ L1', {}, { br_l2_from_l1: false }],
    ['KHÔNG đặt cặp Earth 1 lần', {}, { br_pair_place: false }],
    ['KHÔNG mang kết hợp', {}, { br_lookahead: false }],
    ['Sai số đặt khối σ = 30 mm', {}, { pos_sigma: 30 }],
    ['Sai số đặt khối σ = 45 mm', {}, { pos_sigma: 45 }],
  ];
  const rows = cases.map(([name, k, p]) => {
    const w = run(team('E', k, p), team('D'));
    const r = w.result.red;
    return { name, total: r.total, mandate: fmt(r.mandateT), mustika: fmt(r.mustikaT), towers: r.towers.filter((t) => t.complete).length, lech: r.misplaced, viol: w.violations.length, coll: w.collisions.length };
  });
  if (md) {
    console.log('| Thay đổi (Đỏ chơi E, Xanh chơi D) | Điểm Đỏ | Đạt Mandate | Mustika lên cột | Tháp hoàn chỉnh | Khối đặt trượt | Vi phạm / Va chạm |');
    console.log('|---|---:|---:|---:|---:|---:|---:|');
    for (const r of rows) console.log(`| ${r.name} | **${r.total}** | ${r.mandate} | ${r.mustika} | ${r.towers} | ${r.lech} | ${r.viol} / ${r.coll} |`);
  } else console.table(rows);
  process.exit(0);
}

if (plans.length === 2) {
  const w = run(team(plans[0]), team(plans[1]));
  if (args.includes('--log')) for (const e of w.events) console.log(`${e.t.toFixed(1).padStart(6)}  ${(e.team || '').padEnd(4)}  ${e.msg}`);
  for (const t of ['red', 'blue']) {
    const s = w.result[t];
    console.log(`${t.padEnd(4)} plan ${s.plan}: total ${s.total} = transfer ${s.transfer} + towers ${s.towerPts} + mustika ${s.mustika} | mandate ${fmt(s.mandateT)} | mustika ${fmt(s.mustikaT)} | towers ${s.towers.map((x) => `${x.id}:${x.pts}${x.complete ? '' : '*'}`).join(' ')} | lệch ${s.misplaced}`);
  }
  console.log('violations:', w.violations.length, JSON.stringify(w.violations.slice(0, 4)));
  console.log('collisions:', w.collisions.length, JSON.stringify(w.collisions.slice(0, 6)));
  for (const r of w.robots) console.log(`${r.id.padEnd(8)} quãng đường ${(r.dist / 1000).toFixed(1)} m, kích thước xuất phát ${Math.round(r.startEnv.L)}×${Math.round(r.startEnv.W)}×${Math.round(r.startEnv.H)}`);
} else {
  const keys = Object.keys(PLANS), rows = [];
  for (const a of keys) {
    const w = run(team(a), team('D')), r = w.result.red;
    rows.push({ plan: a, name: PLANS[a].name, total: r.total, transfer: r.transfer, towers: r.towerPts, mustika: r.mustika, nT: r.towers.filter((t) => t.complete).length, mandate: fmt(r.mandateT), mustikaT: fmt(r.mustikaT), viol: w.violations.length, coll: w.collisions.length });
  }
  if (md) {
    console.log('| Chiến thuật đội Đỏ (Xanh chơi D) | Tổng | Transfer | Tháp | Mustika | Tháp hoàn chỉnh | Đạt Mandate | Mustika lên cột | Vi phạm / Va chạm |');
    console.log('|---|---:|---:|---:|---:|---:|---:|---:|---:|');
    for (const r of rows) console.log(`| ${r.name} | **${r.total}** | ${r.transfer} | ${r.towers} | ${r.mustika} | ${r.nT} | ${r.mandate} | ${r.mustikaT} | ${r.viol} / ${r.coll} |`);
  } else console.table(rows);
  console.log('\nMa trận điểm Đỏ (hàng) vs Xanh (cột):');
  const m = {};
  for (const a of keys) { m[a] = {}; for (const b of keys) { const w = run(team(a), team(b)); m[a][b] = `${w.result.red.total}:${w.result.blue.total}`; } }
  console.table(m);
}
