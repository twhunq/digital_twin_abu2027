// Hinh hoc san: do cao nen, vung, doi xung Do <-> Xanh. Thuan du lieu, khong phu thuoc three.js
// => dung chung cho trinh duyet va Node (sim_cli.mjs).

export const inRect = (r, x, y) => x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3];
export const rectCenter = (r) => [(r[0] + r[2]) / 2, (r[1] + r[3]) / 2];

export class Geo {
  constructor(spec) {
    this.spec = spec;
    this.W = spec.field.size[0];
    this.H = spec.field.size[1];
    this.cx = spec.field.center_line_x;
    this.areas = Object.fromEntries(spec.areas.map((a) => [a.id, a]));
    this.spots = Object.fromEntries(spec.building_spots.map((s) => [s.id, s]));
    this.L1 = spec.levels.L1;
    this.L2 = spec.levels.L2;
    this.shared = spec.areas.filter((a) => a.kind === 'shared').map((a) => a.rect);
  }

  // Do cao mat nen (mm) tai diem (x, y) - dung cho robot di tren san.
  floorZ(x, y) {
    const T = this.spec.transitions;
    if (inRect(this.L2.rect, x, y)) return this.L2.z;
    for (const s of T.stairs_L2) if (inRect(s.rect, x, y)) return s.z;
    if (inRect(this.L1.rect, x, y)) return this.L1.z;
    for (const id of ['TA_RED', 'TA_BLUE']) if (inRect(this.areas[id].rect, x, y)) return this.L1.z;
    for (const st of T.stairs_L1) {
      if (x < st.x[0] || x > st.x[1]) continue;
      for (const s of st.steps) if (y >= s.y[0] && y <= s.y[1]) return s.z;
    }
    for (const rp of T.ramps) {
      if (x < rp.x[0] || x > rp.x[1]) continue;
      if (y >= rp.y_top && y <= rp.y_bottom) {
        const t = (y - rp.y_top) / (rp.y_bottom - rp.y_top);
        return rp.z_top + (rp.z_bottom - rp.z_top) * t;
      }
    }
    return 0;
  }

  // Doi xung qua duong giua x = 5500 (team-local -> world).
  mx(team, x) { return team === 'red' ? x : this.W - x; }
  mh(team, h) { return team === 'red' ? h : Math.PI - h; }
  // Ten Building Spot theo goc nhin doi Do -> ten that tren san cua doi team.
  spotId(team, localId) {
    if (team === 'red') return localId;
    const map = { L1_R_NEAR: 'L1_B_NEAR', L1_R_FAR: 'L1_B_FAR', L2_NL: 'L2_NR', L2_FL: 'L2_FR', L2_NR: 'L2_NL', L2_FR: 'L2_FL' };
    return map[localId] || localId;
  }
  isShared(x, y) { return this.shared.some((r) => inRect(r, x, y)); }
  // Diem (x,y) nam trong lanh tho rieng cua doi phuong?
  inOpponent(team, x, y) {
    const opp = team === 'red' ? x > this.cx + 1 : x < this.cx - 1;
    return opp && !this.isShared(x, y);
  }
  // TR bi cam: moi phan L1/L2 ngoai Transfer Area (hinh chieu dung).
  trForbidden(x, y) {
    return inRect(this.L1.rect, x, y) && x > this.L1.rect[0] + 0.5 && x < this.L1.rect[2] - 0.5 && y > this.L1.rect[1] + 0.5 && y < this.L1.rect[3] - 0.5;
  }
}

// Mau diem trong hinh chieu robot (hinh chu nhat xoay): lui = back, toi = front (mm tinh tu tam).
export function footprintSamples(x, y, heading, back, front, halfW, step = 50) {
  const c = Math.cos(heading), s = Math.sin(heading);
  const pts = [];
  for (let u = -back; u <= front + 1e-6; u += step) {
    for (let v = -halfW; v <= halfW + 1e-6; v += step) pts.push([x + u * c - v * s, y + u * s + v * c]);
  }
  return pts;
}
