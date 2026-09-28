// Do thi duong di v2 (toa do goc nhin doi Do, mm), cac o Transfer Area, tham so chien thuat.
// Doi Xanh dung cung du lieu, lay doi xung qua x = 5500 (Geo.mx / Geo.mh).
// Moi diem dung da duoc kiem tra voi dong hoc co cau trong robot_spec.json (tools/check_robot.mjs).

const D = Math.PI / 180;

// ---- TR: nut [x, y, heading khi dung (rad) | null] ----
export const TR_NODES = {
  T_S1: [350, 350, 90 * D],
  T_S2: [1150, 350, 90 * D],
  G_A: [1150, 1250, null],
  W0: [350, 1500, null],               // tu o R1 di thang len hanh lang phia tay
  T_E: [2400, 450, null],              // di doc bien gan, khong xoay than canh BR luc xuat phat
  W1: [900, 2300, null],               // hanh lang phia tay doc (x < 1500)
  W2: [900, 8750, null],
  STO_HUB: [1000, 9250, null],
  RP_BOT: [2000, 8750, -90 * D],       // chan doc (z = 0)
  RP_TOP: [2000, 5000, -90 * D],       // tren doc, mui cach mep TA 25 mm - khong cham Ground
  TA_FRONT: [2000, 4700, -90 * D],     // banh truoc da len TA (TR duoc vao TA), chi khi lan xa trong
  MU_APP: [2600, 9350, null],
  MU_ST: [4700, 9750, 0],              // canh Mustika Pillar, can cau vuon toi bong
  SK_A: [4575, 1250, 0],               // ngoai Ground Shared Area: Sky cot 0 nam thang truoc mui (can khong quet qua ham kep)
  SK_A0: [4575, 1650, 0],
  SK_A4: [4575, 850, 0],
  SK_B: [4800, 1250, 0],               // trong Ground Shared Area (khi cot 0 da trong): cot 1 va 2
  SK_B0: [4800, 1650, 0],
  SK_B1: [4800, 1450, 0],
  SK_B3: [4800, 1050, 0],
  SK_B4: [4800, 850, 0],
};
export const TR_EDGES = [
  ['T_S1', 'W0'], ['W0', 'W1'], ['W0', 'G_A'], ['T_S2', 'G_A'], ['G_A', 'W1'], ['W1', 'W2'], ['W2', 'RP_BOT'], ['W2', 'STO_HUB'], ['STO_HUB', 'RP_BOT'],
  ['RP_BOT', 'RP_TOP', 'ramp'], ['RP_TOP', 'TA_FRONT'], ['RP_BOT', 'MU_APP'], ['STO_HUB', 'MU_APP'], ['MU_APP', 'MU_ST'],
  ['T_S2', 'T_E'], ['T_E', 'SK_A'], ['T_E', 'G_A'], ['G_A', 'SK_A'], ['SK_A', 'SK_B'], ['G_A', 'SK_B'],
  ['SK_A', 'SK_A0'], ['SK_A', 'SK_A4'], ['SK_B', 'SK_B1'], ['SK_B', 'SK_B3'], ['SK_B1', 'SK_B0'], ['SK_B3', 'SK_B4'], ['SK_A0', 'SK_B0'], ['SK_A4', 'SK_B4'],
];

// ---- BR ----
export const BR_NODES = {
  B_S1: [350, 350, 90 * D],
  B_S2: [1150, 350, 90 * D],
  B_ST_BOT: [2000, 2300, 90 * D],
  B_ST_IN: [2000, 2560, 90 * D],       // xich da ha, mui sat co bac 1
  B_TA_IN: [2000, 3860, 90 * D],       // da len TA
  B_TA: [2000, 4150, null],
  H_TA_N: [2660, 3980, 180 * D],       // lay o gan (T1) - than lan 160 mm vao TA qua cua mo, cach dau barrier 10 mm
  H_TA_F: [2660, 4320, 180 * D],       // lay o xa (T2)
  H_MU: [2840, 4320, 180 * D],         // nhan Mustika: lui 180 mm de bong (x 2345) khong nam tren than BR
  H_X: [3050, 4150, null],             // diem thoat khoi TA (tranh dau barrier L1)
  H_N: [3100, 3500, null],
  H_F: [3100, 7400, null],
  SP_L1_R_NEAR: [2870, 3325, -90 * D],   // mui than cach mep thap 60 mm (du cho sai so dinh vi)
  SP_L1_R_FAR: [2870, 7675, 90 * D],
  SP_L1_SH_NEAR: [4760, 2870, 0],
  SP_L1_SH_FAR: [4760, 8130, 0],
  SP_L2_NL_L1: [3530, 4250, 0],        // dat Earth len spot L2 tu L1 (than cach tuong L2 130 mm)
  SP_L2_FL_L1: [3530, 6750, 0],
  L2A: [3200, 5500, 0],
  L2P: [3480, 5500, 0],                // mui sat co bac L2
  L2Q: [4380, 5500, 0],                // da len L2
  L2C: [4550, 5500, 0],
  L2E: [6450, 5500, null],
  SP_L2_NL: [4370, 4950, -90 * D],
  SP_L2_FL: [4370, 6050, 90 * D],
  SP_L2_NR: [6630, 4950, -90 * D],
  SP_L2_FR: [6630, 6050, 90 * D],
  PILLAR: [4850, 5500, 0],
};
export const BR_EDGES = [
  ['B_S1', 'B_ST_BOT'], ['B_S2', 'B_ST_BOT'], ['B_ST_BOT', 'B_ST_IN'], ['B_ST_IN', 'B_TA_IN', 'stairs'], ['B_TA_IN', 'B_TA'],
  ['B_TA', 'H_TA_N'], ['B_TA', 'H_TA_F'], ['H_TA_N', 'H_TA_F'], ['H_TA_F', 'H_MU'], ['H_MU', 'H_X'],
  ['H_TA_N', 'H_X'], ['H_TA_F', 'H_X'], ['H_X', 'H_N'], ['H_X', 'H_F'], ['H_X', 'L2A'], ['H_N', 'L2A'], ['H_F', 'L2A'],
  ['H_N', 'SP_L1_R_NEAR'], ['H_N', 'SP_L1_SH_NEAR'], ['H_N', 'SP_L2_NL_L1'], ['H_F', 'SP_L1_R_FAR'], ['H_F', 'SP_L1_SH_FAR'], ['H_F', 'SP_L2_FL_L1'],
  ['L2A', 'L2P'], ['L2P', 'L2Q', 'stairs'], ['L2Q', 'L2C'],
  ['L2C', 'SP_L2_NL'], ['L2C', 'SP_L2_FL'], ['L2C', 'PILLAR'], ['SP_L2_NL', 'SP_L2_NR'], ['SP_L2_FL', 'SP_L2_FR'],
  ['SP_L2_NR', 'L2E'], ['SP_L2_FR', 'L2E'],
];
// Nut an toan de xoay tai cho
export const PIVOTS = new Set(['T_E', 'W0', 'G_A', 'H_X', 'W1', 'W2', 'STO_HUB', 'MU_APP', 'B_TA', 'H_N', 'H_F', 'L2A', 'L2C', 'L2E', 'PILLAR']);
// Building Spot (ten goc nhin Do) -> diem dung cua BR. *_L1: dat Earth len L2 khi dung tren L1
export const SPOT_STAND = {
  L1_R_NEAR: 'SP_L1_R_NEAR', L1_R_FAR: 'SP_L1_R_FAR', L1_SH_NEAR: 'SP_L1_SH_NEAR', L1_SH_FAR: 'SP_L1_SH_FAR',
  L2_NL: 'SP_L2_NL', L2_FL: 'SP_L2_FL', L2_NR: 'SP_L2_NR', L2_FR: 'SP_L2_FR',
};
export const SPOT_STAND_L1 = { L2_NL: 'SP_L2_NL_L1', L2_FL: 'SP_L2_FL_L1' };

// Transfer Area v2: 2 lan doc truc giua TA (x = 2000). T1 gan cau thang, T2 gan doc. Moi lan chong toi da 2 khoi.
export const TA_SLOTS = [
  { id: 'T1', x: 2000, y: 3900, br: 'H_TA_N' },
  { id: 'T2', x: 2000, y: 4460, br: 'H_TA_F' },
];

// Storage v2 (doi tu xep): 5 cot x 2 hang, buoc cot 400 (khe 50 mm cho ham kep 13 mm), hang y 10190 / 10690.
export const STORE_SLIDE = 400; // do vuon kep khi lay o Storage
// Diem ban giao Mustika: dai trong ben canh lan T2 (bong khong o tren chong khoi -> robot khong vuot 1200 mm)
export const HANDOVER = { x: 2345, y: 4300, z: 160 }; // z: day bong cao hon san TA

// Sky: thu tu uu tien (r, c) theo goc nhin Do va diem dung tuong ung
export const SKY_PLAN = [
  { rc: [2, 0], st: 'SK_A' }, { rc: [0, 0], st: 'SK_A0' }, { rc: [4, 0], st: 'SK_A4' },
  { rc: [1, 1], st: 'SK_B1' }, { rc: [3, 1], st: 'SK_B3' }, { rc: [0, 2], st: 'SK_B0' }, { rc: [4, 2], st: 'SK_B4' },
];

// ---- Tuy chon chien thuat / ky thuat (co the chinh tren giao dien) ----
export const DEFAULT_PARAMS = {
  br_l2_from_l1: true,   // dat Earth len L2 khi dung tren L1 (3.5.4 chi bat buoc BR o L2 khi dat Sky) - can FAQ
  br_pair_place: true,   // dat cap Earth 2 tang trong 1 lan - can FAQ
  br_lookahead: true,    // mang ket hop khoi cua 2 thap
  tr_orient_sky: true,   // TR xoay Sky dung mau truoc khi giao
  tr_sky_first: true,    // chuyen dau gom Sky (luoi Sky nam sat o xuat phat)
  extra_delivery: true,  // TR giao them Earth lay diem Transfer
  pos_sigma: 12,         // sai so dat khoi cua BR (mm, do lech chuan) - khoi lech > 75 mm roi khoi spot
  seed: 1,
};
// Ghi de thong so robot tu giao dien: [duong dan trong robot_spec.json, nhan, min, max, buoc, don vi]
export const SPEC_KNOBS = [
  ['TR.drive.v_max', 'TR vận tốc tối đa', 0.8, 3.5, 0.1, 'm/s'],
  ['TR.drive.v_ramp', 'TR vận tốc trên dốc', 0.5, 2.5, 0.1, 'm/s'],
  ['TR.clamp.slide_v', 'TR bàn trượt X', 200, 1200, 50, 'mm/s'],
  ['TR.boom.yaw_v', 'TR mâm xoay cần', 90, 540, 10, '°/s'],
  ['TR.boom.len_v', 'TR ống lồng', 200, 1200, 50, 'mm/s'],
  ['BR.drive.v_max', 'BR vận tốc tối đa', 0.5, 2.5, 0.1, 'm/s'],
  ['BR.tracks.v_climb', 'BR leo bằng xích', 0.1, 0.6, 0.02, 'm/s'],
  ['BR.gripper.lift_v', 'BR tháp nâng', 200, 1200, 50, 'mm/s'],
  ['BR.gripper.slide_v', 'BR bàn trượt X', 200, 1200, 50, 'mm/s'],
  ['BR.sensing.t_detect', 'BR nhận dạng vật ở TA', 0.1, 2, 0.1, 's'],
  ['BR.sensing.t_align', 'BR căn spot bằng cảm biến màu', 0.1, 2, 0.1, 's'],
];

// ---- Chien thuat: danh sach muc tieu cua BR (ten spot theo goc nhin Do) ----
const L2W = ['L2_NL', 'L2_FL'], L2E = ['L2_NR', 'L2_FR'], L2ANY = [...L2W, ...L2E];
export const PLANS = {
  A: {
    name: 'A · An toàn: 2 tháp L1 → Mustika → thêm L1/L2',
    goals: [
      { tower: ['L1_R_NEAR'] }, { tower: ['L1_SH_NEAR', 'L1_SH_FAR', ...L2ANY] }, { mustika: true },
      { tower: ['L1_R_FAR'] }, { tower: L2ANY }, { tower: L2ANY }, { tower: ['L1_SH_FAR', 'L1_SH_NEAR'] },
    ],
  },
  B: {
    name: 'B · Cân bằng: L1 + L2 → Mustika → thêm L2',
    goals: [
      { tower: ['L1_R_NEAR'] }, { tower: L2ANY }, { mustika: true },
      { tower: L2ANY }, { tower: ['L1_R_FAR'] }, { tower: L2ANY }, { tower: ['L1_SH_NEAR', 'L1_SH_FAR'] },
    ],
  },
  C: {
    name: 'C · Tấn công: 2 tháp L2 → Mustika → L2',
    goals: [
      { tower: L2ANY }, { tower: L2ANY }, { mustika: true },
      { tower: L2ANY }, { tower: ['L1_R_NEAR'] }, { tower: ['L1_R_FAR'] },
    ],
  },
  D: {
    name: 'D · Không Mustika: dồn tháp',
    goals: [
      { tower: ['L1_R_NEAR'] }, { tower: L2ANY }, { tower: L2ANY }, { tower: ['L1_R_FAR'] }, { tower: L2ANY },
      { tower: ['L1_SH_NEAR', 'L1_SH_FAR'] }, { tower: L2ANY },
    ],
  },
  E: {
    name: 'E · Đua Mandate bằng L1 → Mustika → dồn L2',
    goals: [
      { tower: ['L1_R_NEAR'] }, { tower: ['L1_SH_NEAR', 'L1_SH_FAR', ...L2ANY] }, { mustika: true },
      { tower: L2W }, { tower: L2W }, { tower: L2E }, { tower: ['L1_R_FAR'] },
    ],
  },
};
