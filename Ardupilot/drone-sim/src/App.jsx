import { useState, useMemo, useEffect, useRef } from "react";

/* ═══════════════════════════════════════════════════════════════
   MORPHING EDF QUADCOPTER — ArduPilot Tuning Simulator
   D2842-2800KV 70mm EDF on 4S · 35KG Servos · Carbon Fiber Frame
   Configurations: X, O, T, H
   ═══════════════════════════════════════════════════════════════ */

const G = 9.80665;

/* ── EDF Motor Database ── */
const MOTORS = {
  "D2842_2800KV_4S": {
    label: "D2842 2800KV 70mm EDF (4S)",
    kv: 2800, weight_g: 150, resistance_mohm: 42, voltage_s: 4, nominal_v: 14.8,
    full_v: 16.8, peak_a: 65, max_power: 962,
    table: [
      { throttle: 20, thrust_g: 200, torque_nm: 0.02, current_a: 4.0, rpm: 8400, power_w: 59, eff: 3.39 },
      { throttle: 30, thrust_g: 420, torque_nm: 0.05, current_a: 9.5, rpm: 12600, power_w: 141, eff: 2.98 },
      { throttle: 40, thrust_g: 680, torque_nm: 0.09, current_a: 16.0, rpm: 16800, power_w: 237, eff: 2.87 },
      { throttle: 50, thrust_g: 950, torque_nm: 0.13, current_a: 24.0, rpm: 21000, power_w: 355, eff: 2.68 },
      { throttle: 60, thrust_g: 1200, torque_nm: 0.17, current_a: 33.0, rpm: 25200, power_w: 488, eff: 2.46 },
      { throttle: 70, thrust_g: 1440, torque_nm: 0.22, current_a: 42.0, rpm: 29400, power_w: 622, eff: 2.32 },
      { throttle: 80, thrust_g: 1620, torque_nm: 0.26, current_a: 51.0, rpm: 33600, power_w: 755, eff: 2.15 },
      { throttle: 90, thrust_g: 1750, torque_nm: 0.30, current_a: 58.0, rpm: 37800, power_w: 858, eff: 2.04 },
      { throttle: 100, thrust_g: 1820, torque_nm: 0.33, current_a: 65.0, rpm: 42000, power_w: 962, eff: 1.89 },
    ],
  },
  "POWERFUN_2800KV_4S": {
    label: "Powerfun 70mm Pro 2800KV (4S)",
    kv: 2800, weight_g: 180, resistance_mohm: 35, voltage_s: 4, nominal_v: 14.8,
    full_v: 16.8, peak_a: 75, max_power: 1260,
    table: [
      { throttle: 20, thrust_g: 240, torque_nm: 0.03, current_a: 5.0, rpm: 8400, power_w: 74, eff: 3.24 },
      { throttle: 30, thrust_g: 500, torque_nm: 0.06, current_a: 11.0, rpm: 12600, power_w: 163, eff: 3.07 },
      { throttle: 40, thrust_g: 800, torque_nm: 0.10, current_a: 18.5, rpm: 16800, power_w: 274, eff: 2.92 },
      { throttle: 50, thrust_g: 1100, torque_nm: 0.15, current_a: 28.0, rpm: 21000, power_w: 414, eff: 2.66 },
      { throttle: 60, thrust_g: 1380, torque_nm: 0.20, current_a: 38.0, rpm: 25200, power_w: 562, eff: 2.46 },
      { throttle: 70, thrust_g: 1580, torque_nm: 0.25, current_a: 48.0, rpm: 29400, power_w: 710, eff: 2.23 },
      { throttle: 80, thrust_g: 1720, torque_nm: 0.29, current_a: 57.0, rpm: 33600, power_w: 844, eff: 2.04 },
      { throttle: 90, thrust_g: 1800, torque_nm: 0.32, current_a: 66.0, rpm: 37800, power_w: 977, eff: 1.84 },
      { throttle: 100, thrust_g: 1820, torque_nm: 0.33, current_a: 75.0, rpm: 42000, power_w: 1260, eff: 1.44 },
    ],
  },
};

const BATTERIES = {
  "4S_3500": { label: "4S 3500mAh 35C", capacity_mah: 3500, cells: 4, nominal_v: 14.8, full_v: 16.8, weight_g: 380, discharge_c: 35, max_a: 122.5, energy_wh: 51.8 },
  "4S_5000": { label: "4S 5000mAh 50C", capacity_mah: 5000, cells: 4, nominal_v: 14.8, full_v: 16.8, weight_g: 540, discharge_c: 50, max_a: 250, energy_wh: 74.0 },
  "4S_6600": { label: "4S 6600mAh 35C", capacity_mah: 6600, cells: 4, nominal_v: 14.8, full_v: 16.8, weight_g: 680, discharge_c: 35, max_a: 231, energy_wh: 97.7 },
};

/* ── Morphing Configurations ── */
const CONFIGS = {
  X: { label: "X-config", servo: [0, 0, 0, 0], desc: "Standard quadcopter", color: "#3b82f6", stability: "high", group: "X" },
  O: { label: "O-config", servo: [90, 90, 90, 90], desc: "Compact circular", color: "#22c55e", stability: "medium", group: "O" },
  Y: { label: "Y-config", servo: [0, 0, 45, -45], desc: "Y — rear pair folds", color: "#ef4444", stability: "medium", group: "Y" },
  T: { label: "T-config", servo: [-45, 45, 45, -45], desc: "T — all 4 arms move", color: "#f59e0b", stability: "medium", group: "T" },
  H: { label: "H-config", servo: [45, -45, 45, -45], desc: "H-shape parallel", color: "#a78bfa", stability: "medium-high", group: "H" },
};

// T-shape variants for comparison (all count as "T" in cycles)
const T_VARIANTS = {
  T1: { label: "T1: front move", servo: [-45, 45, 0, 0], desc: "Front 2 servos spread", color: "#f59e0b" },
  T2: { label: "T2: rear move", servo: [0, 0, 45, -45], desc: "Rear 2 servos fold", color: "#fb923c" },
  T3: { label: "T3: all move", servo: [-45, 45, 45, -45], desc: "All 4 servos", color: "#ea580c" },
};

const BASE_HEADINGS_DEG = [45, 135, 225, 315];

function interp(table, throttle) {
  if (throttle <= table[0].throttle) return { ...table[0] };
  if (throttle >= table[table.length - 1].throttle) return { ...table[table.length - 1] };
  for (let i = 0; i < table.length - 1; i++) {
    if (throttle >= table[i].throttle && throttle <= table[i + 1].throttle) {
      const t = (throttle - table[i].throttle) / (table[i + 1].throttle - table[i].throttle);
      const r = {};
      for (const k of Object.keys(table[i])) r[k] = table[i][k] + t * (table[i + 1][k] - table[i][k]);
      return r;
    }
  }
  return { ...table[table.length - 1] };
}

function makeSweep(table, steps = 80) {
  const mn = table[0].throttle, mx = table[table.length - 1].throttle, out = [];
  for (let i = 0; i <= steps; i++) {
    const t = mn + (mx - mn) * i / steps, d = interp(table, t);
    out.push({
      throttle: +t.toFixed(1), thrust_g: +d.thrust_g.toFixed(0),
      thrust_kg: +(d.thrust_g / 1000).toFixed(3), current_a: +d.current_a.toFixed(1),
      power_w: +d.power_w.toFixed(0), rpm: +d.rpm.toFixed(0),
      torque_nm: +d.torque_nm.toFixed(3), eff: +d.eff.toFixed(2)
    });
  }
  return out;
}

function computeEdfPositions(servoDeg) {
  const armInner = 0.159, armOuter = 0.159;
  return servoDeg.map((sd, i) => {
    const bh = BASE_HEADINGS_DEG[i] * Math.PI / 180;
    const sx = armInner * Math.cos(bh);
    const sy = armInner * Math.sin(bh);
    const oh = bh + sd * Math.PI / 180;
    return { x: sx + armOuter * Math.cos(oh), y: sy + armOuter * Math.sin(oh), sx, sy };
  });
}

function computeInertia(mass, positions) {
  let ixx = 0, iyy = 0, izz = 0;
  const mPerArm = mass / 4;
  positions.forEach(p => {
    ixx += mPerArm * p.y * p.y;
    iyy += mPerArm * p.x * p.x;
    izz += mPerArm * (p.x * p.x + p.y * p.y);
  });
  return { ixx, iyy, izz };
}

function momentArms(positions) {
  const yArms = positions.map(p => Math.abs(p.y));
  const xArms = positions.map(p => Math.abs(p.x));
  return {
    rollArm: yArms.reduce((a, b) => a + b, 0) / 4,
    pitchArm: xArms.reduce((a, b) => a + b, 0) / 4,
    avgArm: positions.map(p => Math.sqrt(p.x * p.x + p.y * p.y)).reduce((a, b) => a + b, 0) / 4,
  };
}

/* ═══ Shape Visual Data ═══ */
const SHAPE_VISUAL_DATA = {
  X: { label: "X", desc: "Standard quadcopter", servos: [0, 0, 0, 0], moving: [],
       cgOffset: 0.0, Ixx: 0.0404, Iyy: 0.0404, rollAuth: 224.9, pitchAuth: 224.9,
       servosMove: "0 of 4", bestFor: "Stable hover, precision" },
  O: { label: "O", desc: "Compact circular", servos: [90, 90, 90, 90], moving: [0, 1, 2, 3],
       cgOffset: 0.0, Ixx: 0.0202, Iyy: 0.0202, rollAuth: 112.4, pitchAuth: 112.4,
       servosMove: "4 of 4", bestFor: "Confined spaces" },
  Y: { label: "Y", desc: "Rear pair folds inward", servos: [0, 0, 45, -45], moving: [2, 3],
       cgOffset: 23.3, Ixx: 0.0497, Iyy: 0.0253, rollAuth: 248.1, pitchAuth: 168.6,
       servosMove: "2 of 4", bestFor: "Forward cruise" },
  T: { label: "T", desc: "All 4 arms move", servos: [-45, 45, 45, -45], moving: [0, 1, 2, 3],
       cgOffset: 79.5, Ixx: 0.0345, Iyy: 0.0345, rollAuth: 191.9, pitchAuth: 191.9,
       servosMove: "4 of 4", bestFor: "Extreme asymmetric" },
  H: { label: "H", desc: "Parallel front/rear pairs", servos: [45, -45, 45, -45], moving: [0, 1, 2, 3],
       cgOffset: 0.0, Ixx: 0.0589, Iyy: 0.0101, rollAuth: 271.4, pitchAuth: 112.4,
       servosMove: "4 of 4", bestFor: "Wide stable platform" },
};

function ShapeDiagram({ config }) {
  const ARM_IN = 159, ARM_OUT = 159;
  const HEADINGS = [45, 135, 225, 315];
  const positions = HEADINGS.map((hdg, i) => {
    const bh = (hdg * Math.PI) / 180;
    const sx = ARM_IN * Math.cos(bh);
    const sy = ARM_IN * Math.sin(bh);
    const oh = bh + (config.servos[i] * Math.PI) / 180;
    return {
      jointX: sx, jointY: sy,
      edfX: sx + ARM_OUT * Math.cos(oh),
      edfY: sy + ARM_OUT * Math.sin(oh),
    };
  });
  const size = 220, center = size / 2, scale = 0.30;
  const toScreen = (x, y) => ({ sx: center + x * scale, sy: center - y * scale });
  const cgX = positions.reduce((s, p) => s + p.edfX, 0) / 4;
  const cgY = positions.reduce((s, p) => s + p.edfY, 0) / 4;
  const cgScr = toScreen(cgX, cgY);
  const baseScr = toScreen(0, 0);

  return (
    <svg width="100%" height="auto" viewBox={`0 0 ${size} ${size}`}
         preserveAspectRatio="xMidYMid meet" style={{ display: "block", maxWidth: size }}>
      <rect x={baseScr.sx - 16} y={baseScr.sy - 16} width={32} height={32} rx={3}
            fill="#374151" stroke="#6b7280" strokeWidth={1.5} />
      <text x={baseScr.sx} y={baseScr.sy + 4} textAnchor="middle"
            fontSize={9} fill="#d1d5db" fontWeight={600}>CG</text>
      {positions.map((p, i) => {
        const joint = toScreen(p.jointX, p.jointY);
        const edf = toScreen(p.edfX, p.edfY);
        const isMoving = config.moving.includes(i);
        const armColor = isMoving ? "#22c55e" : "#9ca3af";
        const edfColor = isMoving ? "#22c55e" : "#f59e0b";
        const jointColor = isMoving ? "#22c55e" : "#f59e0b";
        return (
          <g key={i}>
            <line x1={baseScr.sx} y1={baseScr.sy} x2={joint.sx} y2={joint.sy}
                  stroke="#9ca3af" strokeWidth={2} strokeLinecap="round" />
            <line x1={joint.sx} y1={joint.sy} x2={edf.sx} y2={edf.sy}
                  stroke={armColor} strokeWidth={2} strokeLinecap="round" />
            <circle cx={joint.sx} cy={joint.sy} r={3.5} fill={jointColor} />
            <circle cx={edf.sx} cy={edf.sy} r={8} fill="none" stroke={edfColor} strokeWidth={1.8} />
          </g>
        );
      })}
      {(Math.abs(cgX) > 2 || Math.abs(cgY) > 2) && (
        <circle cx={cgScr.sx} cy={cgScr.sy} r={3.5} fill="#ef4444"
                stroke="#fff" strokeWidth={0.5} />
      )}
    </svg>
  );
}

function ShapeCard({ configKey, config }) {
  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center",
                  padding: "12px 4px", minWidth: 0 }}>
      <div style={{ fontSize: 20, fontWeight: 700, color: "#e5e7eb",
                    letterSpacing: "0.05em", marginBottom: 2, fontFamily: "var(--font-mono)" }}>
        {configKey}-config
      </div>
      <div style={{ fontSize: 11, color: "#9ca3af", marginBottom: 6, fontFamily: "var(--font-mono)" }}>
        [{config.servos.map(s => s >= 0 ? `+${s}` : s).join(",")}]
      </div>
      <div style={{ fontSize: 11, color: "#9ca3af", marginBottom: 10,
                    textAlign: "center", height: 16 }}>
        {config.desc}
      </div>
      <ShapeDiagram config={config} />
    </div>
  );
}

/* ═══ SVG Chart ═══ */
function Chart({ data, xKey, yKeys, colors, yMax: ymO, yMin: ymiO, xUnit = "", fills = [], dashed = [], markers = [], xLabel, yFormatter, zones = [] }) {
  const W = 900, H = 380, pL = 70, pR = 20, pT = 20, pB = 44;
  const cw = W - pL - pR, ch = H - pT - pB;
  const xVals = data.map(d => d[xKey]);
  const xMin = Math.min(...xVals), xMax = Math.max(...xVals);
  const allY = yKeys.flatMap(k => data.map(d => d[k]));
  const yMin = ymiO != null ? ymiO : 0;
  const yMax = ymO != null ? ymO : Math.ceil(Math.max(...allY) * 1.1);
  const x = v => pL + ((v - xMin) / (xMax - xMin || 1)) * cw;
  const y = v => pT + ch - ((v - yMin) / (yMax - yMin || 1)) * ch;
  const fmt = yFormatter || (v => v >= 1000 ? (v / 1000).toFixed(1) + "k" : Number.isInteger(v) ? String(v) : v.toFixed(1));

  return (
    <svg viewBox={`0 0 ${W} ${H}`} style={{ width: "100%", height: "auto", display: "block" }}>
      <defs>
        {fills.map((f, i) => f ? (
          <linearGradient key={i} id={`gf${i}_${xKey}_${yKeys.join("_")}`} x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor={f} stopOpacity="0.25" />
            <stop offset="100%" stopColor={f} stopOpacity="0" />
          </linearGradient>
        ) : null)}
      </defs>
      {zones.map((z, i) => <rect key={i} x={pL} y={y(z.max)} width={cw} height={y(z.min) - y(z.max)} fill={z.color} opacity="0.08" />)}
      {Array.from({ length: 6 }, (_, i) => {
        const val = yMin + (yMax - yMin) * i / 5;
        return (<g key={i}>
          <line x1={pL} x2={pL + cw} y1={y(val)} y2={y(val)} stroke="var(--color-border-tertiary)" strokeWidth="0.8" strokeDasharray="4,4" />
          <text x={pL - 8} y={y(val) + 5} textAnchor="end" fontSize="14" fill="var(--color-text-tertiary)">{fmt(val)}</text>
        </g>);
      })}
      {Array.from({ length: 6 }, (_, i) => xMin + (xMax - xMin) * i / 5).map(t => (
        <text key={t} x={x(t)} y={H - 16} textAnchor="middle" fontSize="14" fill="var(--color-text-tertiary)">{Number.isInteger(t) ? t : t.toFixed(1)}{xUnit}</text>
      ))}
      {xLabel && <text x={pL + cw / 2} y={H - 2} textAnchor="middle" fontSize="13" fill="var(--color-text-tertiary)">{xLabel}</text>}
      {fills.map((f, fi) => {
        if (!f) return null;
        const k = yKeys[fi];
        const pts = data.map(d => x(d[xKey]) + "," + y(d[k])).join(" ");
        return <polygon key={fi} points={`${x(data[0][xKey])},${y(yMin)} ${pts} ${x(data[data.length - 1][xKey])},${y(yMin)}`} fill={`url(#gf${fi}_${xKey}_${yKeys.join("_")})`} />;
      })}
      {yKeys.map((k, ki) => (
        <path key={k} d={data.map((d, i) => `${i === 0 ? "M" : "L"}${x(d[xKey])},${y(d[k])}`).join(" ")}
          fill="none" stroke={colors[ki]} strokeWidth={dashed[ki] ? 2.5 : 3.5} strokeLinejoin="round"
          strokeDasharray={dashed[ki] ? "8,5" : "none"} />
      ))}
      {markers.map((m, i) => m.x >= xMin && m.x <= xMax ? (
        <g key={i}>
          <line x1={x(m.x)} x2={x(m.x)} y1={pT} y2={pT + ch} stroke={m.color} strokeWidth="2" strokeDasharray="6,3" />
          <text x={x(m.x)} y={pT - 5} textAnchor="middle" fontSize="14" fill={m.color} fontWeight="700">{m.label}</text>
        </g>
      ) : null)}
      {yKeys.length > 1 && yKeys.map((k, ki) => (
        <g key={ki}>
          <line x1={pL + cw - 130} x2={pL + cw - 105} y1={pT + 15 + ki * 20} y2={pT + 15 + ki * 20}
            stroke={colors[ki]} strokeWidth={dashed[ki] ? 2.5 : 3.5} strokeDasharray={dashed[ki] ? "6,3" : "none"} />
          <text x={pL + cw - 100} y={pT + 20 + ki * 20} fontSize="14" fill={colors[ki]} fontWeight="700">{k.replace(/_/g, " ")}</text>
        </g>
      ))}
    </svg>
  );
}

/* ═══ UI Primitives ═══ */
function Sl({ label, value, onChange, min, max, step, unit, desc }) {
  return (<div style={{ marginBottom: 12 }}>
    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline", marginBottom: 1 }}>
      <label style={{ fontSize: 10, fontWeight: 600, letterSpacing: "0.06em", textTransform: "uppercase", color: "var(--color-text-secondary)" }}>{label}</label>
      <span style={{ fontSize: 14, fontWeight: 600, fontFamily: "var(--font-mono)", color: "var(--color-text-primary)" }}>{typeof value === "number" ? (step >= 1 ? value : value.toFixed(1)) : value}{unit && <span style={{ fontSize: 9, color: "var(--color-text-tertiary)", marginLeft: 2 }}>{unit}</span>}</span>
    </div>
    {desc && <div style={{ fontSize: 9, color: "var(--color-text-tertiary)", marginBottom: 2 }}>{desc}</div>}
    <input type="range" min={min} max={max} step={step} value={value} onChange={e => onChange(Number(e.target.value))} style={{ width: "100%", cursor: "pointer" }} />
    <div style={{ display: "flex", justifyContent: "space-between", fontSize: 8, color: "var(--color-text-tertiary)" }}><span>{min}{unit}</span><span>{max}{unit}</span></div>
  </div>);
}
function Stat({ label, value, unit, status, detail }) {
  const c = { good: "#22c55e", warn: "#f59e0b", danger: "#ef4444", info: "#3b82f6" };
  const col = c[status] || "var(--color-text-secondary)";
  return (<div style={{ background: "var(--color-background-secondary)", borderRadius: 10, padding: "9px 11px", border: `1px solid ${col}33` }}>
    <div style={{ fontSize: 9, fontWeight: 600, textTransform: "uppercase", letterSpacing: "0.06em", color: col, marginBottom: 2 }}>{label}</div>
    <div style={{ fontSize: 18, fontWeight: 700, fontFamily: "var(--font-mono)", color: col, lineHeight: 1.1 }}>{value}<span style={{ fontSize: 10, fontWeight: 400, marginLeft: 2, color: "var(--color-text-tertiary)" }}>{unit}</span></div>
    {detail && <div style={{ fontSize: 9, color: "var(--color-text-tertiary)", marginTop: 2 }}>{detail}</div>}
  </div>);
}
function Row({ label, value, highlight }) {
  return (<div style={{ display: "flex", justifyContent: "space-between", padding: "3px 0", borderBottom: "1px solid var(--color-border-tertiary)", fontSize: 11 }}>
    <span style={{ color: "var(--color-text-secondary)" }}>{label}</span>
    <span style={{ fontFamily: "var(--font-mono)", fontWeight: 600, color: highlight || "var(--color-text-primary)" }}>{value}</span>
  </div>);
}
function Card({ title, children, accent }) {
  return (<div style={{ background: "var(--color-background-secondary)", borderRadius: 12, padding: 14, border: `1px solid ${accent || "var(--color-border-tertiary)"}`, marginBottom: 12 }}>
    {title && <h3 style={{ fontSize: 10, fontWeight: 700, textTransform: "uppercase", letterSpacing: "0.08em", color: accent || "var(--color-text-secondary)", margin: "0 0 8px" }}>{title}</h3>}
    {children}
  </div>);
}
function Note({ children }) { return <div style={{ fontSize: 9, color: "var(--color-text-tertiary)", marginBottom: 5 }}>{children}</div>; }
function Sel({ label, value, onChange, options }) {
  return (<div style={{ marginBottom: 12 }}>
    <label style={{ fontSize: 10, fontWeight: 600, letterSpacing: "0.06em", textTransform: "uppercase", color: "var(--color-text-secondary)", display: "block", marginBottom: 2 }}>{label}</label>
    <select value={value} onChange={e => onChange(e.target.value)} style={{ width: "100%", padding: "5px 7px", background: "var(--color-background-primary)", color: "var(--color-text-primary)", border: "1px solid var(--color-border-secondary)", borderRadius: 6, fontSize: 11, fontFamily: "var(--font-mono)", cursor: "pointer" }}>
      {options.map(([k, l]) => <option key={k} value={k}>{l}</option>)}
    </select>
  </div>);
}

/* ═══ Config Shape SVG ═══ */
function ConfigShape({ servoDeg, color, size = 90 }) {
  const S = size * 0.22;
  const positions = servoDeg.map((sd, i) => {
    const bh = BASE_HEADINGS_DEG[i] * Math.PI / 180;
    const sx = S * Math.cos(bh), sy = -S * Math.sin(bh);
    const oh = bh + sd * Math.PI / 180;
    const ex = sx + S * Math.cos(oh), ey = sy - S * Math.sin(oh);
    return { sx, sy, ex, ey };
  });
  const cx = size / 2, cy = size / 2;
  return (
    <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`}>
      <rect x={cx - 5} y={cy - 5} width={10} height={10} rx={2} fill="var(--color-text-tertiary)" opacity="0.3" />
      {positions.map((p, i) => (
        <g key={i}>
          <line x1={cx + p.sx * 0.35} y1={cy + p.sy * 0.35} x2={cx + p.sx} y2={cy + p.sy} stroke="var(--color-text-secondary)" strokeWidth="2" strokeLinecap="round" />
          <line x1={cx + p.sx} y1={cy + p.sy} x2={cx + p.ex} y2={cy + p.ey} stroke="var(--color-text-secondary)" strokeWidth="2" strokeLinecap="round" />
          <circle cx={cx + p.sx} cy={cy + p.sy} r="2" fill="#f59e0b" />
          <circle cx={cx + p.ex} cy={cy + p.ey} r="5" fill="none" stroke={color} strokeWidth="1.5" />
        </g>
      ))}
    </svg>
  );
}

/* ═══════════════════════════════════════════════════════════════
   MAIN APP
   ═══════════════════════════════════════════════════════════════ */
export default function App() {
  const [motorKey, setMotorKey] = useState("D2842_2800KV_4S");
  const [battKey, setBattKey] = useState("4S_5000");
  const [nBatt, setNBatt] = useState(1);
  const [frameW, setFrameW] = useState(0.4);
  const [payload, setPayload] = useState(0.0);
  const [servoW, setServoW] = useState(0.32);
  const [tab, setTab] = useState("overview");

  // Upload bridge state
  const [wsUrl, setWsUrl] = useState("ws://localhost:8765");
  const [wsConnected, setWsConnected] = useState(false);
  const [wsRef, setWsRef] = useState(null);
  const [uploadStatus, setUploadStatus] = useState("");
  const [uploadLog, setUploadLog] = useState([]); // { timestamp, param, oldValue, newValue, status }
  const [connectionLog, setConnectionLog] = useState([]);
  const [selectedForUpload, setSelectedForUpload] = useState({});

  // Load persisted change log from storage
  useEffect(() => {
    if (typeof window !== 'undefined' && window.storage) {
      window.storage.get('upload_log').then(r => {
        if (r && r.value) try { setUploadLog(JSON.parse(r.value)); } catch(e){}
      }).catch(() => {});
    }
  }, []);

  const motor = MOTORS[motorKey];
  const batt = BATTERIES[battKey];
  const sweep = useMemo(() => makeSweep(motor.table), [motorKey]);
  const nMotors = 4;

  const calc = useMemo(() => {
    const motorW = (motor.weight_g * nMotors) / 1000;
    const battW = (batt.weight_g * nBatt) / 1000;
    const auw = frameW + motorW + battW + servoW + payload;
    const auwG = auw * 1000;
    const needed = auwG / nMotors;
    const tbl = motor.table;
    const minT = tbl[0].throttle, maxT_ = tbl[tbl.length - 1].throttle;

    let hoverT = maxT_;
    for (let t = minT; t <= maxT_; t += 0.1) { if (interp(tbl, t).thrust_g >= needed) { hoverT = t; break; } }
    const canHover = hoverT < maxT_ - 0.5;
    const hp = interp(tbl, hoverT);
    const maxPt = tbl[tbl.length - 1];
    const maxThrust = maxPt.thrust_g * nMotors;
    const tw = maxThrust / auwG;
    const hoverI = hp.current_a * nMotors;
    const hoverP = hp.power_w * nMotors;
    const usable = batt.capacity_mah * nBatt * 0.8;
    const flightMin = canHover ? (usable / hoverI) * 60 / 1000 : 0;

    const heatLoss = hp.current_a ** 2 * (motor.resistance_mohm / 1000);
    const mechPower = hp.torque_nm * (hp.rpm * 2 * Math.PI / 60);
    const motorEff = hp.power_w > 0 ? mechPower / hp.power_w * 100 : 0;
    const excessN = (maxThrust / 1000 - auw) * G;
    const maxClimb = excessN > 0 ? excessN / (auw * G) * 10 : 0;

    // Per-config analysis
    const configData = {};
    for (const [cfgKey, cfg] of Object.entries(CONFIGS)) {
      const pos = computeEdfPositions(cfg.servo);
      const inertia = computeInertia(auw, pos);
      const arms = momentArms(pos);

      // Each config may have different effective control authority
      // Smaller moment arms → less roll/pitch authority → needs higher gains
      const rollAuthority = arms.rollArm / 0.225; // normalized to X-config nominal
      const pitchAuthority = arms.pitchArm / 0.225;

      // Gain scaling: inversely proportional to moment arm (smaller arm = higher gain needed)
      const rollGainScale = rollAuthority > 0.1 ? 1 / rollAuthority : 5;
      const pitchGainScale = pitchAuthority > 0.1 ? 1 / pitchAuthority : 5;

      // ArduPilot params for this config
      const pG = Math.min(0.45, Math.max(0.05, (1 / Math.max(tw, 1)) * 0.35));
      const ap = {};
      ap["FRAME_CLASS"] = 1; // Quad
      ap["FRAME_TYPE"] = 1;  // X
      ap["MOT_THST_EXPO"] = 0.65; // EDF thrust curve is more linear than props
      ap["MOT_SPIN_MIN"] = 0.12;
      ap["MOT_SPIN_ARM"] = 0.08;
      ap["MOT_BAT_VOLT_MAX"] = (batt.cells * 4.2).toFixed(1);
      ap["MOT_BAT_VOLT_MIN"] = (batt.cells * 3.3).toFixed(1);
      ap["MOT_PWM_MIN"] = 1000;
      ap["MOT_PWM_MAX"] = 2000;
      ap["BATT_CAPACITY"] = batt.capacity_mah * nBatt;
      ap["BATT_MONITOR"] = 4;
      ap["BATT_ARM_VOLT"] = (batt.cells * 3.9).toFixed(1);
      ap["BATT_CRT_VOLT"] = (batt.cells * 3.4).toFixed(1);
      ap["BATT_LOW_VOLT"] = (batt.cells * 3.5).toFixed(1);

      const rP = (pG * rollGainScale).toFixed(4);
      const rI = (pG * rollGainScale * 0.5).toFixed(4);
      const rD = (pG * rollGainScale * 0.012).toFixed(5);
      ap["ATC_RAT_RLL_P"] = rP;
      ap["ATC_RAT_RLL_I"] = rI;
      ap["ATC_RAT_RLL_D"] = rD;
      const pP = (pG * pitchGainScale).toFixed(4);
      const pI = (pG * pitchGainScale * 0.5).toFixed(4);
      const pD = (pG * pitchGainScale * 0.012).toFixed(5);
      ap["ATC_RAT_PIT_P"] = pP;
      ap["ATC_RAT_PIT_I"] = pI;
      ap["ATC_RAT_PIT_D"] = pD;
      ap["ATC_RAT_YAW_P"] = (pG * 1.2).toFixed(4);
      ap["ATC_RAT_YAW_I"] = (pG * 0.12).toFixed(4);
      ap["ATC_ACCEL_R_MAX"] = Math.round(110000 / (auw * 2));
      ap["ATC_ACCEL_P_MAX"] = Math.round(110000 / (auw * 2));
      ap["ATC_ACCEL_Y_MAX"] = Math.round(50000 / (auw * 2));
      ap["INS_GYRO_FILTER"] = auw > 5 ? 15 : 20;
      ap["INS_ACCEL_FILTER"] = auw > 5 ? 15 : 20;
      ap["PSC_POSXY_P"] = 1.0;
      ap["PSC_VELXY_P"] = 2.0;
      ap["PSC_POSZ_P"] = 1.0;
      ap["PSC_VELZ_P"] = 5.0;
      ap["PSC_ACCZ_P"] = tw > 0 ? (1 / (tw * G) * 2).toFixed(3) : "0.200";
      ap["PSC_ACCZ_I"] = (parseFloat(ap["PSC_ACCZ_P"]) * 2).toFixed(3);
      ap["ANGLE_MAX"] = 3000; // 30 degrees

      configData[cfgKey] = {
        positions: pos, inertia, arms, ap,
        rollAuthority: +(rollAuthority * 100).toFixed(1),
        pitchAuthority: +(pitchAuthority * 100).toFixed(1),
        rollGainScale: +rollGainScale.toFixed(2),
        pitchGainScale: +pitchGainScale.toFixed(2),
      };
    }

    // ── Universal ArduPilot params (safe across ALL configs) ──
    // Strategy: use the worst-case (lowest authority) config to set gains,
    // and most conservative filters/limits across all shapes.
    const allRollScales = Object.values(configData).map(cd => cd.rollGainScale);
    const allPitchScales = Object.values(configData).map(cd => cd.pitchGainScale);
    const worstRollScale = Math.max(...allRollScales);
    const worstPitchScale = Math.max(...allPitchScales);
    const minRollAuth = Math.min(...Object.values(configData).map(cd => cd.rollAuthority));
    const minPitchAuth = Math.min(...Object.values(configData).map(cd => cd.pitchAuthority));

    const uPG = Math.min(0.45, Math.max(0.05, (1 / Math.max(tw, 1)) * 0.35));
    const universalAP = {};
    universalAP["FRAME_CLASS"] = 1;
    universalAP["FRAME_TYPE"] = 1;
    universalAP["MOT_THST_EXPO"] = 0.65;
    universalAP["MOT_SPIN_MIN"] = 0.12;
    universalAP["MOT_SPIN_ARM"] = 0.08;
    universalAP["MOT_BAT_VOLT_MAX"] = (batt.cells * 4.2).toFixed(1);
    universalAP["MOT_BAT_VOLT_MIN"] = (batt.cells * 3.3).toFixed(1);
    universalAP["MOT_PWM_MIN"] = 1000;
    universalAP["MOT_PWM_MAX"] = 2000;
    universalAP["BATT_CAPACITY"] = batt.capacity_mah * nBatt;
    universalAP["BATT_MONITOR"] = 4;
    universalAP["BATT_ARM_VOLT"] = (batt.cells * 3.9).toFixed(1);
    universalAP["BATT_CRT_VOLT"] = (batt.cells * 3.4).toFixed(1);
    universalAP["BATT_LOW_VOLT"] = (batt.cells * 3.5).toFixed(1);
    // Use worst-case gain scaling so it's safe in every shape
    universalAP["ATC_RAT_RLL_P"] = (uPG * worstRollScale).toFixed(4);
    universalAP["ATC_RAT_RLL_I"] = (uPG * worstRollScale * 0.5).toFixed(4);
    universalAP["ATC_RAT_RLL_D"] = (uPG * worstRollScale * 0.012).toFixed(5);
    universalAP["ATC_RAT_PIT_P"] = (uPG * worstPitchScale).toFixed(4);
    universalAP["ATC_RAT_PIT_I"] = (uPG * worstPitchScale * 0.5).toFixed(4);
    universalAP["ATC_RAT_PIT_D"] = (uPG * worstPitchScale * 0.012).toFixed(5);
    universalAP["ATC_RAT_YAW_P"] = (uPG * 1.2).toFixed(4);
    universalAP["ATC_RAT_YAW_I"] = (uPG * 0.12).toFixed(4);
    // Use most conservative accel limits (lowest across configs)
    const minAccelR = Math.min(...Object.values(configData).map(cd => cd.ap["ATC_ACCEL_R_MAX"]));
    const minAccelP = Math.min(...Object.values(configData).map(cd => cd.ap["ATC_ACCEL_P_MAX"]));
    universalAP["ATC_ACCEL_R_MAX"] = minAccelR;
    universalAP["ATC_ACCEL_P_MAX"] = minAccelP;
    universalAP["ATC_ACCEL_Y_MAX"] = Math.round(50000 / (auw * 2));
    // Conservative filters
    universalAP["INS_GYRO_FILTER"] = auw > 5 ? 12 : 18;
    universalAP["INS_ACCEL_FILTER"] = auw > 5 ? 12 : 18;
    universalAP["PSC_POSXY_P"] = 1.0;
    universalAP["PSC_VELXY_P"] = 1.5;
    universalAP["PSC_POSZ_P"] = 1.0;
    universalAP["PSC_VELZ_P"] = 5.0;
    universalAP["PSC_ACCZ_P"] = tw > 0 ? (1 / (tw * G) * 2).toFixed(3) : "0.200";
    universalAP["PSC_ACCZ_I"] = (parseFloat(universalAP["PSC_ACCZ_P"]) * 2).toFixed(3);
    universalAP["ANGLE_MAX"] = 2500; // More conservative for morphing platform

    // ── Morph Transition Analysis (ALL directional pairs) ──
    const servoSpeed = 300;
    const configKeys = Object.keys(CONFIGS);
    const morphTransitions = [];
    const transitionMap = {};

    // Compute transitions for the 4 main configs (X, O, T, H)
    for (let i = 0; i < configKeys.length; i++) {
      for (let j = 0; j < configKeys.length; j++) {
        if (i === j) continue;
        const fromKey = configKeys[i], toKey = configKeys[j];
        const fromCfg = CONFIGS[fromKey], toCfg = CONFIGS[toKey];
        const fromData = configData[fromKey], toData = configData[toKey];

        const travels = fromCfg.servo.map((a, idx) => Math.abs(toCfg.servo[idx] - a));
        const maxTravel = Math.max(...travels);
        const avgTravel = travels.reduce((a, b) => a + b, 0) / 4;
        const servoTime = maxTravel / servoSpeed;
        const servosMoving = travels.filter(t => t > 0).length;

        const ixxChange = Math.abs(toData.inertia.ixx - fromData.inertia.ixx);
        const iyyChange = Math.abs(toData.inertia.iyy - fromData.inertia.iyy);
        const maxInertiaChange = Math.max(ixxChange, iyyChange);
        const avgInertia = (fromData.inertia.ixx + fromData.inertia.iyy + toData.inertia.ixx + toData.inertia.iyy) / 4;
        const inertiaChangeRatio = avgInertia > 0 ? maxInertiaChange / avgInertia : 0;

        const fromCentroid = { x: fromData.positions.reduce((s, p) => s + p.x, 0) / 4, y: fromData.positions.reduce((s, p) => s + p.y, 0) / 4 };
        const toCentroid = { x: toData.positions.reduce((s, p) => s + p.x, 0) / 4, y: toData.positions.reduce((s, p) => s + p.y, 0) / 4 };
        const cgShift = Math.sqrt((toCentroid.x - fromCentroid.x) ** 2 + (toCentroid.y - fromCentroid.y) ** 2) * 1000;

        const rollAuthDelta = Math.abs(toData.rollAuthority - fromData.rollAuthority);
        const pitchAuthDelta = Math.abs(toData.pitchAuthority - fromData.pitchAuthority);
        const worstAuthDuring = Math.min(fromData.rollAuthority, toData.rollAuthority, fromData.pitchAuthority, toData.pitchAuthority);

        const fcResponseTime = 0.05;
        const settlingFactor = 1 + inertiaChangeRatio * 3;
        const disturbTime = servoTime + fcResponseTime * settlingFactor;
        const thrustAsymmetry = 0.15 + 0.15 * (maxTravel / 90);
        const altDrop = 0.5 * G * thrustAsymmetry * disturbTime ** 2;

        const complexityFactor = 1.5 + (maxTravel / 90) * 0.5 + inertiaChangeRatio * 2;
        const safeTransitionTime = servoTime * complexityFactor + 0.5;
        const recommendedTime = Math.max(1.0, Math.ceil(safeTransitionTime * 2) / 2);

        let risk = "low";
        if (maxTravel > 60 || worstAuthDuring < 50 || cgShift > 5) risk = "medium";
        if (maxTravel > 120 || worstAuthDuring < 30 || cgShift > 15) risk = "high";

        const transitionCost = recommendedTime * 2 + (100 - worstAuthDuring) * 0.05 + cgShift * 0.1 + (risk === "high" ? 10 : risk === "medium" ? 3 : 0);

        const entry = {
          from: fromKey, to: toKey, fromColor: fromCfg.color, toColor: toCfg.color,
          maxTravel, avgTravel: +avgTravel.toFixed(1), servosMoving,
          servoTime: +servoTime.toFixed(2), inertiaChangeRatio: +(inertiaChangeRatio * 100).toFixed(1),
          cgShift: +cgShift.toFixed(1), altDrop: +(altDrop * 100).toFixed(1),
          rollAuthDelta: +rollAuthDelta.toFixed(1), pitchAuthDelta: +pitchAuthDelta.toFixed(1),
          worstAuthDuring: +worstAuthDuring.toFixed(1), safeTransitionTime: +safeTransitionTime.toFixed(2),
          recommendedTime, risk, disturbTime: +disturbTime.toFixed(3), transitionCost: +transitionCost.toFixed(2),
        };
        morphTransitions.push(entry);
        transitionMap[`${fromKey}_${toKey}`] = entry;
      }
    }
    morphTransitions.sort((a, b) => a.transitionCost - b.transitionCost);

    // ── T-Variant Analysis ──
    // Compute each T variant's properties and transitions to/from X, O, H
    const tVariantData = {};
    const tVariantTransitions = {};
    for (const [tvKey, tv] of Object.entries(T_VARIANTS)) {
      const pos = computeEdfPositions(tv.servo);
      const inertia = computeInertia(auw, pos);
      const arms = momentArms(pos);
      const rollAuth = +(arms.rollArm / 0.225 * 100).toFixed(1);
      const pitchAuth = +(arms.pitchArm / 0.225 * 100).toFixed(1);
      const cx = pos.reduce((s, p) => s + p.x, 0) / 4;
      const cy = pos.reduce((s, p) => s + p.y, 0) / 4;
      const cgOffset = +(Math.sqrt(cx * cx + cy * cy) * 1000).toFixed(1);
      const minAuth = Math.min(rollAuth, pitchAuth);
      const maxDiffThrust = 17.86 * 0.5;
      const cgTorque = auw * G * cgOffset / 1000;
      const corrTorque = maxDiffThrust * Math.min(arms.rollArm, arms.pitchArm) * 2;
      const trimRatio = cgTorque > 0.01 ? +(corrTorque / cgTorque).toFixed(1) : 999;
      const servosMoving = tv.servo.filter((s, i) => s !== 0).length;

      tVariantData[tvKey] = { ...tv, positions: pos, inertia, arms, rollAuth, pitchAuth, cgOffset, minAuth, trimRatio, servosMoving };

      // Transitions from each main config to this T variant and back
      tVariantTransitions[tvKey] = {};
      for (const ck of ['X', 'O', 'Y', 'H']) {
        const fromCfg = CONFIGS[ck];
        const fromData2 = configData[ck];
        for (const dir of ['to', 'from']) {
          const srcServo = dir === 'to' ? fromCfg.servo : tv.servo;
          const dstServo = dir === 'to' ? tv.servo : fromCfg.servo;
          const srcData = dir === 'to' ? fromData2 : { positions: pos, inertia, rollAuthority: rollAuth, pitchAuthority: pitchAuth };
          const dstData = dir === 'to' ? { positions: pos, inertia, rollAuthority: rollAuth, pitchAuthority: pitchAuth } : fromData2;

          const travels2 = srcServo.map((a, idx) => Math.abs(dstServo[idx] - a));
          const maxT2 = Math.max(...travels2);
          const sTime2 = maxT2 / servoSpeed;
          const sm2 = travels2.filter(t => t > 0).length;

          const srcCent = { x: srcData.positions.reduce((s2, p) => s2 + p.x, 0) / 4, y: srcData.positions.reduce((s2, p) => s2 + p.y, 0) / 4 };
          const dstCent = { x: dstData.positions.reduce((s2, p) => s2 + p.x, 0) / 4, y: dstData.positions.reduce((s2, p) => s2 + p.y, 0) / 4 };
          const cg2 = +(Math.sqrt((dstCent.x - srcCent.x) ** 2 + (dstCent.y - srcCent.y) ** 2) * 1000).toFixed(1);
          const wa2 = +Math.min(srcData.rollAuthority || 100, dstData.rollAuthority || 100, srcData.pitchAuthority || 100, dstData.pitchAuthority || 100).toFixed(1);
          const cf2 = 1.5 + (maxT2 / 90) * 0.5;
          const st2 = sTime2 * cf2 + 0.5;
          const rt2 = Math.max(1.0, Math.ceil(st2 * 2) / 2);
          let rk2 = "low";
          if (maxT2 > 60 || wa2 < 50 || cg2 > 5) rk2 = "medium";
          if (maxT2 > 120 || wa2 < 30 || cg2 > 15) rk2 = "high";
          const cost2 = +(rt2 * 2 + (100 - wa2) * 0.05 + cg2 * 0.1 + (rk2 === "high" ? 10 : rk2 === "medium" ? 3 : 0)).toFixed(2);

          const key2 = dir === 'to' ? `${ck}_${tvKey}` : `${tvKey}_${ck}`;
          const label2 = dir === 'to' ? `${ck}→${tvKey}` : `${tvKey}→${ck}`;
          tVariantTransitions[tvKey][key2] = {
            label: label2, from: dir === 'to' ? ck : tvKey, to: dir === 'to' ? tvKey : ck,
            maxTravel: maxT2, servosMoving: sm2, servoTime: +sTime2.toFixed(2),
            cgShift: cg2, worstAuthDuring: wa2, recommendedTime: rt2, risk: rk2, transitionCost: +cost2,
          };
        }
      }
    }

    // ── T-Variant Overall Score (same as our earlier analysis) ──
    const tVariantScores = {};
    for (const [tvKey, tvd] of Object.entries(tVariantData)) {
      const hoverScore = Math.max(0, 100 - tvd.cgOffset * 0.4 - (100 - tvd.minAuth) * 0.3);
      // Transition score from X
      const xToT = tVariantTransitions[tvKey][`X_${tvKey}`];
      const transScore = xToT ? Math.max(0, 100 - xToT.cgShift * 0.3 - (100 - xToT.worstAuthDuring) * 0.3 - xToT.servosMoving * 2) : 50;
      const mechScore = 100 - tvd.servosMoving * 15;
      const overall = +(hoverScore * 0.4 + transScore * 0.4 + mechScore * 0.2).toFixed(1);
      tVariantScores[tvKey] = { hoverScore: +hoverScore.toFixed(1), transScore: +transScore.toFixed(1), mechScore: +mechScore.toFixed(1), overall };
    }

    // ── Optimal Cycle Finder (always starts from X) ──
    // Cycle visits X → {O, Y, T, H in some order} → back to X
    // For the T slot, we try all 3 T variants and pick the best
    // Y is a distinct config (rear-folding variant)
    const otherGroups = ['O', 'Y', 'H']; // T is handled specially via T_SLOT
    // All 6 permutations of [O, Y, H]
    const groupPerms = [
      ['O','Y','H'], ['O','H','Y'],
      ['Y','O','H'], ['Y','H','O'],
      ['H','O','Y'], ['H','Y','O'],
    ];
    const tSlotOptions = Object.keys(T_VARIANTS); // T1, T2, T3

    const cycleResults = [];
    for (const perm of groupPerms) {
      // Insert T at each possible position (4 possible positions in a 3-element array)
      for (let tPos = 0; tPos <= perm.length; tPos++) {
        const orderWithoutT = [...perm];
        orderWithoutT.splice(tPos, 0, 'T_SLOT');

        for (const tVar of tSlotOptions) {
          // Build the actual config sequence: X → ... → X
          const sequence = ['X'];
          for (const item of orderWithoutT) {
            if (item === 'T_SLOT') {
              sequence.push(tVar); // Use T variant key
            } else {
              sequence.push(item);
            }
          }
          sequence.push('X'); // Return to X

          // Compute cycle cost using transitions
          let totalCost = 0, totalTime = 0, worstRisk = "low", totalCgShift = 0;
          let worstMinAuth = 100, valid = true;
          const steps = [];

          for (let s = 0; s < sequence.length - 1; s++) {
            const from = sequence[s], to = sequence[s + 1];
            // Look up transition — could be main configs or T variant
            let t = null;
            const fromServo = T_VARIANTS[from] ? T_VARIANTS[from].servo : (CONFIGS[from] ? CONFIGS[from].servo : null);
            const toServo = T_VARIANTS[to] ? T_VARIANTS[to].servo : (CONFIGS[to] ? CONFIGS[to].servo : null);
            const fromColor = T_VARIANTS[from] ? T_VARIANTS[from].color : (CONFIGS[from] ? CONFIGS[from].color : "#888");
            const toColor = T_VARIANTS[to] ? T_VARIANTS[to].color : (CONFIGS[to] ? CONFIGS[to].color : "#888");

            if (!fromServo || !toServo) { valid = false; break; }

            const travels3 = fromServo.map((a, idx) => Math.abs(toServo[idx] - a));
            const maxT3 = Math.max(...travels3);
            const sm3 = travels3.filter(tt => tt > 0).length;
            const st3 = maxT3 / servoSpeed;
            const cf3 = 1.5 + (maxT3 / 90) * 0.5;
            const safe3 = st3 * cf3 + 0.5;
            const rec3 = Math.max(1.0, Math.ceil(safe3 * 2) / 2);

            // Compute CG shift
            const fromPos = computeEdfPositions(fromServo);
            const toPos = computeEdfPositions(toServo);
            const fc3 = { x: fromPos.reduce((s2, p) => s2 + p.x, 0) / 4, y: fromPos.reduce((s2, p) => s2 + p.y, 0) / 4 };
            const tc3 = { x: toPos.reduce((s2, p) => s2 + p.x, 0) / 4, y: toPos.reduce((s2, p) => s2 + p.y, 0) / 4 };
            const cg3 = +(Math.sqrt((tc3.x - fc3.x) ** 2 + (tc3.y - fc3.y) ** 2) * 1000).toFixed(1);

            const fromArms = momentArms(fromPos);
            const toArms = momentArms(toPos);
            const wa3 = +Math.min(fromArms.rollArm / 0.225 * 100, toArms.rollArm / 0.225 * 100, fromArms.pitchArm / 0.225 * 100, toArms.pitchArm / 0.225 * 100).toFixed(1);

            let rk3 = "low";
            if (maxT3 > 60 || wa3 < 50 || cg3 > 5) rk3 = "medium";
            if (maxT3 > 120 || wa3 < 30 || cg3 > 15) rk3 = "high";

            const cost3 = rec3 * 2 + (100 - wa3) * 0.05 + cg3 * 0.1 + (rk3 === "high" ? 10 : rk3 === "medium" ? 3 : 0);

            t = { from, to, fromColor, toColor, maxTravel: maxT3, servosMoving: sm3, recommendedTime: rec3, cgShift: cg3, worstAuthDuring: wa3, risk: rk3, transitionCost: +cost3.toFixed(2) };

            totalCost += cost3;
            totalTime += rec3;
            totalCgShift += cg3;
            if (wa3 < worstMinAuth) worstMinAuth = wa3;
            if (rk3 === "high") worstRisk = "high";
            else if (rk3 === "medium" && worstRisk !== "high") worstRisk = "medium";
            steps.push(t);
          }

          if (!valid) continue;

          const tVarLabel = T_VARIANTS[tVar].label.split(":")[0];
          const cycleDisplay = sequence.map(s => T_VARIANTS[s] ? tVarLabel : s).join(" → ");

          cycleResults.push({
            cycle: sequence,
            cycleLabel: cycleDisplay,
            tVariant: tVar,
            tVariantLabel: T_VARIANTS[tVar].label,
            totalCost: +totalCost.toFixed(2),
            totalTime: +totalTime.toFixed(1),
            totalCgShift: +totalCgShift.toFixed(1),
            worstMinAuth: +worstMinAuth.toFixed(1),
            worstRisk, steps,
          });
        }
      }
    }

    cycleResults.sort((a, b) => a.totalCost - b.totalCost);
    const optimalCycle = cycleResults[0];

    // Best cycle per T variant
    const bestCyclePerTVar = {};
    for (const tv of tSlotOptions) {
      const filtered = cycleResults.filter(c => c.tVariant === tv);
      if (filtered.length > 0) bestCyclePerTVar[tv] = filtered[0];
    }

    // Morph speed sweep for worst transition in optimal cycle
    const worstTransition = optimalCycle ? optimalCycle.steps.reduce((worst, s) => s.transitionCost > (worst ? worst.transitionCost : 0) ? s : worst, null) : (morphTransitions[morphTransitions.length - 1] || null);
    const morphSpeedSweep = [];
    if (worstTransition) {
      for (let dur = 0.5; dur <= 8; dur += 0.25) {
        const servoRate2 = worstTransition.maxTravel / dur;
        const feasible = servoRate2 <= servoSpeed;
        const margin = feasible ? (1 - servoRate2 / servoSpeed) * 100 : 0;
        const asymmetry = 0.15 + 0.15 * Math.min(1, servoRate2 / servoSpeed);
        const drop = 0.5 * G * asymmetry * (dur * 0.15) ** 2 * 100;
        const stabilityScore = feasible ? Math.max(0, 100 - (servoRate2 / servoSpeed) * 60) : 0;
        morphSpeedSweep.push({ duration_s: +dur.toFixed(2), servo_rate: +servoRate2.toFixed(0), alt_drop_cm: +drop.toFixed(1), stability: +stabilityScore.toFixed(0), margin_pct: +margin.toFixed(0) });
      }
    }

    return {
      motorW, battW, auw, auwG, hoverT, hp, maxThrust, tw, hoverI, hoverP, flightMin,
      canHover, heatLoss, mechPower, motorEff, maxClimb, configData,
      maxI: maxPt.current_a * nMotors, battMaxI: batt.max_a * nBatt,
      universalAP, worstRollScale, worstPitchScale, minRollAuth, minPitchAuth,
      morphTransitions, morphSpeedSweep, worstTransition, transitionMap,
      cycleResults, optimalCycle, bestCyclePerTVar,
      tVariantData, tVariantTransitions, tVariantScores,
    };
  }, [motorKey, battKey, nBatt, frameW, payload, servoW]);

  const twSt = calc.tw >= 2.5 ? "good" : calc.tw >= 1.8 ? "warn" : "danger";
  const hSt = !calc.canHover ? "danger" : calc.hoverT <= 55 ? "good" : calc.hoverT <= 70 ? "warn" : "danger";
  const fSt = calc.flightMin >= 8 ? "good" : calc.flightMin >= 4 ? "warn" : "danger";

  const tabs = [["overview", "Overview"], ["shapes", "Shapes"], ["configs", "Configs"], ["morph", "Morphing"], ["params", "ArduPilot"], ["upload", "Upload"], ["graphs", "Flight Data"], ["motor", "Hardware"]];
  const tBtnStyle = t => ({
    padding: "5px 10px", fontSize: 10, fontWeight: 700, letterSpacing: "0.04em", textTransform: "uppercase",
    background: tab === t ? "var(--color-text-info)" : "transparent",
    color: tab === t ? "#fff" : "var(--color-text-secondary)",
    border: "none", borderRadius: 6, cursor: "pointer",
  });

  return (
    <>
      <style>{`
        :root {
          --color-text-primary: #e0e0e0;
          --color-text-secondary: #a0a0a0;
          --color-text-tertiary: #707070;
          --color-text-info: #3b82f6;
          --color-text-danger: #ef4444;
          --color-text-success: #22c55e;
          --color-text-warning: #f59e0b;
          --color-background-primary: #1a1a2e;
          --color-background-secondary: #16213e;
          --color-background-tertiary: #0f172a;
          --color-border-primary: #334155;
          --color-border-secondary: #2d3748;
          --color-border-tertiary: #1e293b;
          --font-sans: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
          --font-mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
          --font-serif: Georgia, 'Times New Roman', serif;
          --border-radius-md: 8px;
          --border-radius-lg: 12px;
          --border-radius-xl: 16px;
        }
      `}</style>
    <div style={{ color: "var(--color-text-primary)", fontFamily: "var(--font-sans)", padding: "16px 12px" }}>
      <div style={{ maxWidth: 960, margin: "0 auto" }}>
        <div style={{ marginBottom: 14 }}>
          <div style={{ display: "flex", gap: 6, alignItems: "center", flexWrap: "wrap", marginBottom: 4 }}>
            <span style={{ fontSize: 9, background: "#22c55e22", color: "#22c55e", padding: "2px 7px", borderRadius: 16, fontWeight: 700 }}>SIMULATION</span>
            <span style={{ fontSize: 9, background: "#3b82f622", color: "#3b82f6", padding: "2px 7px", borderRadius: 16, fontWeight: 700 }}>EDF QUADCOPTER</span>
            <span style={{ fontSize: 9, background: "#a78bfa22", color: "#a78bfa", padding: "2px 7px", borderRadius: 16, fontWeight: 700 }}>MORPHING</span>
          </div>
          <h1 style={{ fontSize: 19, fontWeight: 800, margin: "2px 0", letterSpacing: "-0.02em" }}>Morphing EDF Quad — ArduPilot Tuning Simulator</h1>
          <p style={{ fontSize: 11, color: "var(--color-text-secondary)", margin: 0 }}>D2842-2800KV 70mm EDF · 4S LiPo · 35KG Servos · X/O/T/H Configurations</p>
        </div>

        <div style={{ display: "flex", gap: 3, marginBottom: 12, background: "var(--color-background-secondary)", padding: 3, borderRadius: 8, width: "fit-content", flexWrap: "wrap" }}>
          {tabs.map(([k, l]) => <button key={k} onClick={() => setTab(k)} style={tBtnStyle(k)}>{l}</button>)}
        </div>

        <div style={{ display: "grid", gridTemplateColumns: "240px 1fr", gap: 14, alignItems: "start" }}>
          {/* ── SIDEBAR ── */}
          <div style={{ background: "var(--color-background-secondary)", borderRadius: 12, padding: 12, border: "1px solid var(--color-border-tertiary)" }}>
            <h3 style={{ fontSize: 10, fontWeight: 700, textTransform: "uppercase", letterSpacing: "0.08em", color: "var(--color-text-secondary)", margin: "0 0 8px" }}>Configuration</h3>
            <Sel label="EDF Motor" value={motorKey} onChange={setMotorKey} options={Object.entries(MOTORS).map(([k, m]) => [k, m.label])} />
            <Sel label="Battery" value={battKey} onChange={setBattKey} options={Object.entries(BATTERIES).map(([k, b]) => [k, b.label])} />
            <Sl label="Battery Packs" value={nBatt} onChange={setNBatt} min={1} max={3} step={1} unit="x" desc="Parallel packs" />
            <Sl label="Frame Weight" value={frameW} onChange={setFrameW} min={0.2} max={1.5} step={0.05} unit="kg" desc="Carbon fiber frame + wiring" />
            <Sl label="Servo Weight" value={servoW} onChange={setServoW} min={0.1} max={0.6} step={0.02} unit="kg" desc="4x 35KG servos total" />
            <Sl label="Payload" value={payload} onChange={setPayload} min={0} max={3} step={0.1} unit="kg" desc="Camera, sensors, etc." />
            {!calc.canHover && <div style={{ background: "#ef444422", border: "1px solid #ef4444", borderRadius: 8, padding: 8, fontSize: 11, color: "#ef4444", marginTop: 6, fontWeight: 600 }}>
              CANNOT HOVER — Reduce weight or choose a more powerful motor.
            </div>}

            {/* Config shapes */}
            <div style={{ marginTop: 10 }}>
              <div style={{ fontSize: 9, fontWeight: 600, textTransform: "uppercase", letterSpacing: "0.06em", color: "var(--color-text-secondary)", marginBottom: 4 }}>Shapes</div>
              <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 4 }}>
                {Object.entries(CONFIGS).map(([k, cfg]) => (
                  <div key={k} style={{ textAlign: "center" }}>
                    <ConfigShape servoDeg={cfg.servo} color={cfg.color} size={80} />
                    <div style={{ fontSize: 8, fontWeight: 600, color: cfg.color }}>{cfg.label}</div>
                  </div>
                ))}
              </div>
            </div>
          </div>

          {/* ── CONTENT ── */}
          <div style={{ minWidth: 0 }}>
            {/* ═══ OVERVIEW ═══ */}
            {tab === "overview" && <div>
              <div style={{ display: "grid", gridTemplateColumns: "repeat(3,1fr)", gap: 8, marginBottom: 10 }}>
                <Stat label="T:W Ratio" value={calc.tw.toFixed(2)} unit=":1" status={twSt} detail={calc.tw >= 2.5 ? "Excellent for agility" : calc.tw >= 1.8 ? "Adequate" : "UNSAFE"} />
                <Stat label="Hover Throttle" value={calc.canHover ? calc.hoverT.toFixed(1) : "N/A"} unit="%" status={hSt} detail={calc.canHover ? `${(calc.hoverP).toFixed(0)}W total` : "Cannot hover"} />
                <Stat label="Flight Time" value={calc.flightMin.toFixed(1)} unit="min" status={fSt} detail={`${calc.hoverI.toFixed(1)}A draw`} />
                <Stat label="Max Climb" value={calc.maxClimb.toFixed(1)} unit="m/s" status={calc.maxClimb > 5 ? "good" : calc.maxClimb > 2 ? "warn" : "danger"} />
                <Stat label="Max Thrust" value={(calc.maxThrust / 1000).toFixed(2)} unit="kg" status="info" detail={`4× ${(motor.table[motor.table.length - 1].thrust_g / 1000).toFixed(2)} kg`} />
                <Stat label="AUW" value={calc.auw.toFixed(2)} unit="kg" status={calc.canHover ? "info" : "danger"} detail={`${(calc.auw * 2.205).toFixed(1)} lbs`} />
              </div>
              <Card title="Weight Breakdown">
                {[["Frame", `${frameW.toFixed(2)} kg`], [`Motors (x4)`, `${calc.motorW.toFixed(2)} kg`],
                [`Battery (x${nBatt})`, `${calc.battW.toFixed(2)} kg`], ["Servos (x4)", `${servoW.toFixed(2)} kg`],
                ["Payload", `${payload.toFixed(2)} kg`]].map(([l, v]) => <Row key={l} label={l} value={v} />)}
                <div style={{ display: "flex", justifyContent: "space-between", padding: "5px 0 0", fontSize: 13, fontWeight: 700 }}>
                  <span>All-Up Weight</span>
                  <span style={{ fontFamily: "var(--font-mono)", color: calc.canHover ? "#3b82f6" : "#ef4444" }}>{calc.auw.toFixed(2)} kg</span>
                </div>
              </Card>
              <Card title="Thrust vs Throttle (per EDF)">
                <Chart data={sweep} xKey="throttle" yKeys={["thrust_g"]} colors={["#3b82f6"]}
                  fills={["#3b82f6"]} xUnit="%" yMax={Math.ceil(motor.table[motor.table.length - 1].thrust_g * 1.1)}
                  markers={calc.canHover ? [{ x: calc.hoverT, label: "Hover", color: "#f59e0b" }] : []}
                  yFormatter={v => v.toFixed(0) + "g"} />
              </Card>
              <Card title="Power Budget at Hover">
                <div style={{ display: "grid", gridTemplateColumns: "repeat(3,1fr)", gap: 8, marginBottom: 8 }}>
                  <Stat label="Electrical In" value={calc.hp.power_w.toFixed(0)} unit="W/motor" status="info" />
                  <Stat label="I²R Loss" value={calc.heatLoss.toFixed(1)} unit="W/motor" status={calc.heatLoss > 20 ? "warn" : "info"} />
                  <Stat label="Motor Eff." value={calc.motorEff.toFixed(1)} unit="%" status={calc.motorEff > 60 ? "good" : "warn"} />
                </div>
                {[["Hover RPM", calc.hp.rpm.toFixed(0)], ["Hover Current/EDF", calc.hp.current_a.toFixed(1) + " A"],
                ["Total Hover Current", calc.hoverI.toFixed(1) + " A"], ["Total Hover Power", calc.hoverP.toFixed(0) + " W"],
                ["Thrust Headroom", ((calc.tw - 1) * 100).toFixed(0) + "%"],
                ].map(([l, v]) => <Row key={l} label={l} value={v} />)}
              </Card>
            </div>}

            {/* ═══ SHAPES VISUAL ═══ */}
            {tab === "shapes" && <div>
              <Card title="All Five Morphing Configurations" accent="#22c55e">
                <Note>Visual reference for every shape the drone can take. Orange joints = fixed arms, green = moving arms during morph, red dot = thrust centroid (shown when offset from geometric center).</Note>
                <div style={{
                  display: "grid",
                  gridTemplateColumns: "repeat(5, minmax(0, 1fr))",
                  gap: 4,
                  background: "#1f2937",
                  borderRadius: 10,
                  padding: 12,
                }}>
                  {["X", "O", "Y", "T", "H"].map(k => (
                    <ShapeCard key={k} configKey={k} config={SHAPE_VISUAL_DATA[k]} />
                  ))}
                </div>
              </Card>

              <Card title="Configuration Metrics Comparison" accent="#3b82f6">
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 12 }}>
                    <thead>
                      <tr style={{ borderBottom: "1px solid var(--color-border-secondary)" }}>
                        <th style={{ textAlign: "left", padding: "8px 10px", color: "var(--color-text-secondary)", fontWeight: 500 }}>Metric</th>
                        {["X", "O", "Y", "T", "H"].map(k => (
                          <th key={k} style={{ textAlign: "center", padding: "8px 10px", color: "var(--color-text-secondary)", fontWeight: 500, fontFamily: "var(--font-mono)" }}>{k}-config</th>
                        ))}
                      </tr>
                    </thead>
                    <tbody>
                      {[
                        { label: "CG offset",            get: c => `${c.cgOffset.toFixed(1)} mm` },
                        { label: "Symmetry",             get: c => c.cgOffset < 0.5 ? "Symmetric" : (c.label === "Y" ? "Fore-aft" : "Diagonal") },
                        { label: "Roll authority",       get: c => `${c.rollAuth.toFixed(1)} mm` },
                        { label: "Pitch authority",      get: c => `${c.pitchAuth.toFixed(1)} mm` },
                        { label: "Ixx (roll inertia)",   get: c => c.Ixx.toFixed(4) },
                        { label: "Iyy (pitch inertia)",  get: c => c.Iyy.toFixed(4) },
                        { label: "Servos moving",        get: c => c.servosMove },
                        { label: "Thrust differential",  get: c => c.cgOffset < 0.5 ? "0%" : (c.label === "Y" ? "±9.4%" : "±41.4%") },
                        { label: "Best for",             get: c => c.bestFor },
                      ].map((row, i, arr) => (
                        <tr key={i} style={{ borderBottom: i < arr.length - 1 ? "1px solid #2d3642" : "none" }}>
                          <td style={{ padding: "8px 10px", color: "var(--color-text-secondary)" }}>{row.label}</td>
                          {["X", "O", "Y", "T", "H"].map(k => (
                            <td key={k} style={{
                              padding: "8px 10px",
                              textAlign: "center",
                              color: "var(--color-text-primary)",
                              fontFamily: row.label === "Best for" ? "inherit" : "var(--font-mono)",
                            }}>
                              {row.get(SHAPE_VISUAL_DATA[k])}
                            </td>
                          ))}
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>

                {/* ── Metric definitions ── */}
                <div style={{ marginTop: 16, paddingTop: 14, borderTop: "1px solid var(--color-border-tertiary)" }}>
                  <div style={{ fontSize: 11, fontWeight: 700, color: "var(--color-text-secondary)", textTransform: "uppercase", letterSpacing: "0.06em", marginBottom: 10 }}>
                    What each metric means
                  </div>
                  <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8, fontSize: 11, color: "var(--color-text-secondary)", lineHeight: 1.6 }}>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #3b82f6" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>CG Offset (mm)</div>
                      Distance between the geometric center of the drone and the thrust centroid (average motor position).
                      When non-zero, equal motor thrust creates a tipping moment that must be corrected by differential thrust.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>0 mm = no correction needed. Higher = harder to stabilize.</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #3b82f6" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Symmetry</div>
                      Whether the thrust centroid sits at the geometric center. Symmetric configs (X, O, H) have equal thrust on all motors.
                      Asymmetric configs (Y, T) require one side to push harder than the other to prevent the drone from flipping.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>Symmetric = simplest to control.</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #f59e0b" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Roll Authority (mm)</div>
                      Average lateral distance of motors from the roll axis (body X-axis). This is the effective lever arm for
                      creating roll torque. Larger = more torque per unit of thrust differential = better roll control.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>X-config = 225 mm (baseline). O = 112 mm (half the authority).</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #f59e0b" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Pitch Authority (mm)</div>
                      Average forward distance of motors from the pitch axis (body Y-axis). Same concept as roll authority but
                      for the nose-up/nose-down axis. When this drops, the drone needs higher PID gains to feel responsive in pitch.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>H-config = 112 mm (half of X). Must increase pitch P gain to compensate.</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #a78bfa" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Ixx / Iyy — Moments of Inertia (kg·m²)</div>
                      Resistance to angular acceleration about the roll (Ixx) and pitch (Iyy) axes. Computed as Σ mᵢ·dᵢ² where dᵢ
                      is each motor's distance from the axis. Higher I = the drone responds more sluggishly to torque inputs but is
                      also harder to disturb.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>H has highest Ixx (0.059) — most roll-resistant. O has lowest (0.010) — fastest but least stable.</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #a78bfa" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Servos Moving</div>
                      How many of the four servos change position when transitioning from X-config to this shape. More servos
                      moving = more mechanical complexity during the morph and a longer transition window where the drone is
                      between stable configurations.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>O moves all 4 servos 90° each — the most dramatic morph.</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #ef4444" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Thrust Differential (%)</div>
                      The percentage difference between the highest and lowest per-motor thrust needed for level hover. This is
                      the direct consequence of CG offset — the flight controller must command unequal thrust to cancel the
                      tipping moment. This number determines how much motor headroom is consumed just to stay level.
                      <div style={{ marginTop: 4, color: "#ef4444", fontWeight: 600 }}>T-config: ±41.4% — uses 48.5% of motor capacity just for hover. Only 51.5% left for maneuvering.</div>
                    </div>
                    <div style={{ padding: 10, background: "var(--color-background-tertiary)", borderRadius: 8, borderLeft: "3px solid #22c55e" }}>
                      <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 2 }}>Best For</div>
                      The operational scenario where this configuration provides a clear advantage over X-config. Each shape
                      exists because it solves a specific problem that a fixed-geometry drone cannot — fitting through tight spaces,
                      reaching a surface for inspection, or providing a wider baseline for precision landing.
                      <div style={{ marginTop: 4, color: "#22c55e", fontWeight: 600 }}>If no scenario requires a specific advantage, X-config is always the safest choice.</div>
                    </div>
                  </div>
                </div>
              </Card>

              <Card title="Flight Sequence" accent="#f59e0b">
                <div style={{ fontSize: 13, color: "var(--color-text-secondary)", lineHeight: 1.7 }}>
                  <div style={{ marginBottom: 8 }}>
                    <b style={{ color: "var(--color-text-primary)", fontSize: 15 }}>X → O → Y → T → H → X</b>
                  </div>
                  <div>10 seconds per configuration, 5-second morph transitions between each shape.</div>
                  <div style={{ marginTop: 8 }}>
                    Only Y and T require differential thrust to maintain level hover; symmetric configs (X, O, H)
                    hover with equal thrust on all four motors. Starting and ending in X ensures stable takeoff and landing.
                  </div>
                </div>
              </Card>
            </div>}

            {/* ═══ CONFIG COMPARISON ═══ */}
            {tab === "configs" && <div>
              <Card title="How These Numbers Are Calculated" accent="#a78bfa">
                <Note>Every metric in the tables below comes from the formulas here. Coordinates use the standard ROS REP-103 convention: X is forward, Y is left, Z is up.</Note>
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 12, fontSize: 11, color: "var(--color-text-secondary)", lineHeight: 1.6 }}>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>1. EDF Position (per motor)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      x_i = L_in · cos(θ_base,i) + L_out · cos(θ_base,i + θ_servo,i)<br/>
                      y_i = L_in · sin(θ_base,i) + L_out · sin(θ_base,i + θ_servo,i)
                    </code>
                    <div>L_in = L_out = 159 mm (arm segment lengths). θ_base = [45°, 135°, 225°, 315°] for the four arms (FR/FL/RL/RR).</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>2. Roll Arm (lever arm for roll torque)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      RollArm = mean(|y_i|) for i = 1..4
                    </code>
                    <div>Average distance of motors from the roll axis (X-axis). Larger = more torque per unit thrust differential = better roll control.</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>3. Pitch Arm (lever arm for pitch torque)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      PitchArm = mean(|x_i|) for i = 1..4
                    </code>
                    <div>Average distance of motors from the pitch axis (Y-axis). Larger = better pitch control.</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>4. Roll &amp; Pitch Authority (% of X-config baseline)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      RollAuth%  = (RollArm  / RollArm_X)  · 100<br/>
                      PitchAuth% = (PitchArm / PitchArm_X) · 100
                    </code>
                    <div style={{ marginBottom: 4 }}>
                      <b style={{ color: "var(--color-text-primary)" }}>Roll Authority</b> measures how much torque the drone can produce around its forward (X) axis — i.e., how strongly it can tilt left/right — relative to the standard X-config. It scales with the average lateral distance of the motors from the roll axis: motors farther out give bigger lever arms.
                    </div>
                    <div style={{ marginBottom: 4 }}>
                      <b style={{ color: "var(--color-text-primary)" }}>Pitch Authority</b> measures torque around the lateral (Y) axis — how strongly the drone can tilt nose-up/nose-down. It scales with the average forward distance of the motors from the pitch axis.
                    </div>
                    <div>X-config gets 100% by definition. &gt; 100% = stronger control (e.g., H-config has 121% roll authority because front/rear pairs are pushed wider apart). &lt; 100% = weaker control (e.g., O-config has 50% on both axes because all arms are pulled inward, halving the lever arm).</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>5. Roll P Gain Scale (PID adjustment factor)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      RollPScale = 1 / RollAuth_normalized
                    </code>
                    <div>How much to multiply X-config's tuned roll P gain to compensate for changed control authority. A value of 2.0× means doubling the gain. Pitch follows the same logic.</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>6. Moments of Inertia (kg·m²)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      Ixx (roll)  = Σ m_i · y_i²<br/>
                      Iyy (pitch) = Σ m_i · x_i²
                    </code>
                    <div>Resistance to angular acceleration about each axis. Higher I = slower angular response; the controller needs higher gains or faster updates to feel "snappy".</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>7. Thrust Centroid Offset (CG Offset)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      cx = mean(x_i),  cy = mean(y_i)<br/>
                      CG_offset = √(cx² + cy²)
                    </code>
                    <div>Distance between the center of total thrust and the geometric center of mass. Non-zero offset means equal motor thrust will create a constant tipping moment that must be canceled by differential thrust.</div>
                  </div>

                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>8. Trim Ratio (asymmetric configs only)</div>
                    <code style={{ display: "block", background: "var(--color-background-secondary)", padding: "4px 8px", borderRadius: 4, fontSize: 10, marginBottom: 4 }}>
                      TrimRatio = HoverThrust / (4 · CG_offset / RollArm · HoverThrust)
                    </code>
                    <div>Ratio of available thrust margin to thrust required for trim. Higher is better; &gt; 4:1 means plenty of headroom for maneuvering on top of trim.</div>
                  </div>
                </div>
              </Card>

              <Card title="Stability Rating Definitions" accent="#22c55e">
                <Note>Stability ratings (high / medium-high / medium / low) are derived from a combination of CG offset, control authority loss, and thrust differential required.</Note>
                <div style={{ display: "grid", gridTemplateColumns: "repeat(4, 1fr)", gap: 8 }}>
                  <div style={{ background: "#22c55e15", border: "1px solid #22c55e44", borderRadius: 8, padding: 10 }}>
                    <div style={{ fontWeight: 700, color: "#22c55e", fontSize: 13, marginBottom: 4 }}>HIGH</div>
                    <div style={{ fontSize: 10, color: "var(--color-text-secondary)", lineHeight: 1.5 }}>
                      <div>• CG offset = 0 mm</div>
                      <div>• Roll &amp; pitch authority ≥ 100%</div>
                      <div>• Thrust differential = 0%</div>
                      <div>• PID gains unchanged from X baseline</div>
                      <div style={{ marginTop: 4, color: "var(--color-text-tertiary)" }}>Examples: X-config</div>
                    </div>
                  </div>
                  <div style={{ background: "#84cc1615", border: "1px solid #84cc1644", borderRadius: 8, padding: 10 }}>
                    <div style={{ fontWeight: 700, color: "#84cc16", fontSize: 13, marginBottom: 4 }}>MEDIUM-HIGH</div>
                    <div style={{ fontSize: 10, color: "var(--color-text-secondary)", lineHeight: 1.5 }}>
                      <div>• CG offset = 0 mm</div>
                      <div>• Authority asymmetric (roll ≠ pitch)</div>
                      <div>• Thrust differential = 0%</div>
                      <div>• Per-axis gain scaling needed</div>
                      <div style={{ marginTop: 4, color: "var(--color-text-tertiary)" }}>Examples: H-config</div>
                    </div>
                  </div>
                  <div style={{ background: "#f59e0b15", border: "1px solid #f59e0b44", borderRadius: 8, padding: 10 }}>
                    <div style={{ fontWeight: 700, color: "#f59e0b", fontSize: 13, marginBottom: 4 }}>MEDIUM</div>
                    <div style={{ fontSize: 10, color: "var(--color-text-secondary)", lineHeight: 1.5 }}>
                      <div>• CG offset 0 – 80 mm</div>
                      <div>• Authority 50–110% baseline</div>
                      <div>• Thrust differential ≤ ±42%</div>
                      <div>• Differential thrust + PID retune</div>
                      <div style={{ marginTop: 4, color: "var(--color-text-tertiary)" }}>Examples: O, Y, T</div>
                    </div>
                  </div>
                  <div style={{ background: "#ef444415", border: "1px solid #ef444444", borderRadius: 8, padding: 10 }}>
                    <div style={{ fontWeight: 700, color: "#ef4444", fontSize: 13, marginBottom: 4 }}>LOW</div>
                    <div style={{ fontSize: 10, color: "var(--color-text-secondary)", lineHeight: 1.5 }}>
                      <div>• CG offset &gt; 80 mm</div>
                      <div>• Authority &lt; 50% baseline</div>
                      <div>• Thrust differential &gt; ±42%</div>
                      <div>• Likely uncontrollable without active mixing</div>
                      <div style={{ marginTop: 4, color: "var(--color-text-tertiary)" }}>Examples: (none in this design)</div>
                    </div>
                  </div>
                </div>
                <div style={{ marginTop: 12, padding: 10, background: "var(--color-background-secondary)", borderRadius: 6, fontSize: 11, color: "var(--color-text-secondary)", lineHeight: 1.6 }}>
                  <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Color coding in the table below:</div>
                  <div>
                    <span style={{ color: "#22c55e", fontWeight: 700 }}>Green</span> = good (Authority &gt; 80% or CG offset &lt; 30 mm or gain scale ≤ 2×) ·{" "}
                    <span style={{ color: "#f59e0b", fontWeight: 700 }}>Amber</span> = caution (50–80% authority, 30–60 mm CG offset) ·{" "}
                    <span style={{ color: "#ef4444", fontWeight: 700 }}>Red</span> = poor (Authority &lt; 50% or CG offset &gt; 60 mm or gain scale &gt; 2×)
                  </div>
                </div>
              </Card>

              <Card title="Morphing Configuration Comparison" accent="#a78bfa">
                <Note>How arm geometry affects control authority and required PID gains. Values relative to X-config baseline.</Note>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["Config", "Servo Angles", "Roll Arm", "Pitch Arm", "Roll Auth", "Pitch Auth", "Roll P Scale", "Ixx", "Iyy"].map(h =>
                        <th key={h} style={{ padding: "5px 5px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 9 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{Object.entries(CONFIGS).map(([k, cfg]) => {
                      const cd = calc.configData[k];
                      return (
                        <tr key={k} style={{ borderBottom: "1px solid var(--color-border-tertiary)" }}>
                          <td style={{ padding: "4px 5px", textAlign: "left", fontWeight: 700, color: cfg.color }}>{cfg.label}</td>
                          <td style={{ padding: "4px 5px", textAlign: "right", fontSize: 9 }}>[{cfg.servo.join(", ")}]°</td>
                          <td style={{ padding: "4px 5px", textAlign: "right" }}>{(cd.arms.rollArm * 1000).toFixed(0)} mm</td>
                          <td style={{ padding: "4px 5px", textAlign: "right" }}>{(cd.arms.pitchArm * 1000).toFixed(0)} mm</td>
                          <td style={{ padding: "4px 5px", textAlign: "right", color: cd.rollAuthority > 80 ? "#22c55e" : cd.rollAuthority > 50 ? "#f59e0b" : "#ef4444" }}>{cd.rollAuthority}%</td>
                          <td style={{ padding: "4px 5px", textAlign: "right", color: cd.pitchAuthority > 80 ? "#22c55e" : cd.pitchAuthority > 50 ? "#f59e0b" : "#ef4444" }}>{cd.pitchAuthority}%</td>
                          <td style={{ padding: "4px 5px", textAlign: "right", color: cd.rollGainScale > 2 ? "#ef4444" : "#22c55e" }}>{cd.rollGainScale}x</td>
                          <td style={{ padding: "4px 5px", textAlign: "right" }}>{cd.inertia.ixx.toFixed(4)}</td>
                          <td style={{ padding: "4px 5px", textAlign: "right" }}>{cd.inertia.iyy.toFixed(4)}</td>
                        </tr>
                      );
                    })}</tbody>
                  </table>
                </div>
              </Card>
              {Object.entries(CONFIGS).map(([k, cfg]) => {
                const cd = calc.configData[k];
                return (
                  <Card key={k} title={`${cfg.label} — ${cfg.desc}`} accent={cfg.color}>
                    <div style={{ display: "flex", gap: 12, alignItems: "start" }}>
                      <ConfigShape servoDeg={cfg.servo} color={cfg.color} size={100} />
                      <div style={{ flex: 1 }}>
                        <div style={{ display: "grid", gridTemplateColumns: "repeat(3,1fr)", gap: 6, marginBottom: 6 }}>
                          <Stat label="Roll Authority" value={cd.rollAuthority} unit="%" status={cd.rollAuthority > 80 ? "good" : cd.rollAuthority > 50 ? "warn" : "danger"} />
                          <Stat label="Pitch Authority" value={cd.pitchAuthority} unit="%" status={cd.pitchAuthority > 80 ? "good" : cd.pitchAuthority > 50 ? "warn" : "danger"} />
                          <Stat label="Stability" value={cfg.stability} unit="" status={cfg.stability === "high" ? "good" : cfg.stability.includes("medium") ? "warn" : "danger"} />
                        </div>
                        <div style={{ fontSize: 9, color: "var(--color-text-tertiary)", lineHeight: 1.5 }}>
                          Avg moment arm: {(cd.arms.avgArm * 1000).toFixed(0)} mm.
                          {cd.rollGainScale > 1.5 ? " ⚠ Reduced control authority — gains must be increased significantly." : " Control authority is adequate."}
                          {" "}Servo angles: [{cfg.servo.join("°, ")}°]
                        </div>
                      </div>
                    </div>
                  </Card>
                );
              })}
            </div>}

            {/* ═══ MORPHING TRANSITIONS ═══ */}
            {tab === "morph" && <div>
              {/* T-VARIANT COMPARISON */}
              <Card title="T-Shape Variant Comparison" accent="#f59e0b">
                <Note>Three ways to form the T-shape. Each uses different servo pairs. Scores combine hover stability (40%), transition safety from X (40%), and mechanical simplicity (20%).</Note>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["Variant", "Servos", "Angles", "CG Offset", "Roll Auth", "Pitch Auth", "Trim Ratio", "Hover", "Trans", "Mech", "Overall"].map(h =>
                        <th key={h} style={{ padding: "5px 3px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 8 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{Object.entries(calc.tVariantData).map(([tvk, tvd]) => {
                      const sc = calc.tVariantScores[tvk];
                      const best = Object.values(calc.tVariantScores).every(s => s.overall <= sc.overall);
                      return (
                        <tr key={tvk} style={{ borderBottom: "1px solid var(--color-border-tertiary)", background: best ? "#22c55e08" : "transparent" }}>
                          <td style={{ padding: "4px 3px", textAlign: "left", fontWeight: 700, color: tvd.color }}>{tvd.label}{best ? " ★" : ""}</td>
                          <td style={{ padding: "4px 3px", textAlign: "right" }}>{tvd.servosMoving}/4</td>
                          <td style={{ padding: "4px 3px", textAlign: "right", fontSize: 8 }}>[{tvd.servo.join(",")}]</td>
                          <td style={{ padding: "4px 3px", textAlign: "right", color: tvd.cgOffset < 30 ? "#22c55e" : tvd.cgOffset < 60 ? "#f59e0b" : "#ef4444" }}>{tvd.cgOffset}mm</td>
                          <td style={{ padding: "4px 3px", textAlign: "right" }}>{tvd.rollAuth}%</td>
                          <td style={{ padding: "4px 3px", textAlign: "right" }}>{tvd.pitchAuth}%</td>
                          <td style={{ padding: "4px 3px", textAlign: "right", color: tvd.trimRatio > 4 ? "#22c55e" : tvd.trimRatio > 2 ? "#f59e0b" : "#ef4444" }}>{tvd.trimRatio}:1</td>
                          <td style={{ padding: "4px 3px", textAlign: "right" }}>{sc.hoverScore}</td>
                          <td style={{ padding: "4px 3px", textAlign: "right" }}>{sc.transScore}</td>
                          <td style={{ padding: "4px 3px", textAlign: "right" }}>{sc.mechScore}</td>
                          <td style={{ padding: "4px 3px", textAlign: "right", fontWeight: 700, color: best ? "#22c55e" : "var(--color-text-primary)" }}>{sc.overall}</td>
                        </tr>
                      );
                    })}</tbody>
                  </table>
                </div>
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 6, marginTop: 8 }}>
                  {Object.entries(calc.tVariantData).map(([tvk, tvd]) => (
                    <div key={tvk} style={{ textAlign: "center" }}>
                      <ConfigShape servoDeg={tvd.servo} color={tvd.color} size={80} />
                      <div style={{ fontSize: 9, fontWeight: 600, color: tvd.color }}>{tvd.label}</div>
                      <div style={{ fontSize: 8, color: "var(--color-text-tertiary)" }}>{tvd.desc}</div>
                    </div>
                  ))}
                </div>
              </Card>

              {/* OPTIMAL CYCLE */}
              {calc.optimalCycle && <Card title="Optimal Morph Cycle (starting from X)" accent="#22c55e">
                <Note>Best order: X → visit O, T, H → return to X. Tries all 3 T-variants in all positions to find the lowest-cost cycle.</Note>
                <div style={{ display: "grid", gridTemplateColumns: "repeat(3,1fr)", gap: 8, marginBottom: 10 }}>
                  <Stat label="Best Cycle" value={calc.optimalCycle.cycleLabel} unit="" status="good" />
                  <Stat label="Total Time" value={calc.optimalCycle.totalTime} unit="s" status="good" detail={`${calc.optimalCycle.steps.length} transitions`} />
                  <Stat label="T-Variant Used" value={calc.optimalCycle.tVariantLabel} unit="" status="info" detail={`Worst risk: ${calc.optimalCycle.worstRisk}`} />
                </div>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["Step", "Transition", "Travel", "Servos", "Time", "CG Shift", "Min Auth", "Risk"].map(h =>
                        <th key={h} style={{ padding: "5px 4px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 9 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{calc.optimalCycle.steps.map((st, i) => (
                      <tr key={i} style={{ borderBottom: "1px solid var(--color-border-tertiary)" }}>
                        <td style={{ padding: "4px 4px", textAlign: "right", fontWeight: 600 }}>{i + 1}</td>
                        <td style={{ padding: "4px 4px", textAlign: "left", fontSize: 9 }}>
                          <span style={{ color: st.fromColor, fontWeight: 700 }}>{st.from}</span>
                          <span style={{ color: "var(--color-text-tertiary)" }}> → </span>
                          <span style={{ color: st.toColor, fontWeight: 700 }}>{st.to}</span>
                        </td>
                        <td style={{ padding: "4px 4px", textAlign: "right" }}>{st.maxTravel}°</td>
                        <td style={{ padding: "4px 4px", textAlign: "right" }}>{st.servosMoving}/4</td>
                        <td style={{ padding: "4px 4px", textAlign: "right", fontWeight: 700, color: "#3b82f6" }}>{st.recommendedTime}s</td>
                        <td style={{ padding: "4px 4px", textAlign: "right" }}>{st.cgShift}mm</td>
                        <td style={{ padding: "4px 4px", textAlign: "right", color: st.worstAuthDuring < 50 ? "#ef4444" : st.worstAuthDuring < 80 ? "#f59e0b" : "#22c55e" }}>{st.worstAuthDuring}%</td>
                        <td style={{ padding: "4px 4px", textAlign: "right", fontWeight: 600, color: st.risk === "high" ? "#ef4444" : st.risk === "medium" ? "#f59e0b" : "#22c55e" }}>{st.risk.toUpperCase()}</td>
                      </tr>
                    ))}</tbody>
                  </table>
                </div>
              </Card>}

              {/* BEST CYCLE PER T-VARIANT */}
              <Card title="Best Cycle by T-Variant" accent="#a78bfa">
                <Note>Each row shows the optimal X-start cycle when using that specific T-shape variant.</Note>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["T-Variant", "Cycle", "Time", "CG Shift", "Min Auth", "Risk", "Cost"].map(h =>
                        <th key={h} style={{ padding: "5px 4px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 9 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{Object.entries(calc.bestCyclePerTVar).map(([tv, c]) => {
                      const isBest = calc.optimalCycle && c.totalCost === calc.optimalCycle.totalCost;
                      return (
                        <tr key={tv} style={{ borderBottom: "1px solid var(--color-border-tertiary)", background: isBest ? "#22c55e08" : "transparent" }}>
                          <td style={{ padding: "4px 4px", textAlign: "left", fontWeight: 700, color: T_VARIANTS[tv].color }}>{T_VARIANTS[tv].label}{isBest ? " ★" : ""}</td>
                          <td style={{ padding: "4px 4px", textAlign: "left", fontSize: 9 }}>{c.cycleLabel}</td>
                          <td style={{ padding: "4px 4px", textAlign: "right", fontWeight: 700, color: "#3b82f6" }}>{c.totalTime}s</td>
                          <td style={{ padding: "4px 4px", textAlign: "right" }}>{c.totalCgShift}mm</td>
                          <td style={{ padding: "4px 4px", textAlign: "right", color: c.worstMinAuth < 50 ? "#ef4444" : "#22c55e" }}>{c.worstMinAuth}%</td>
                          <td style={{ padding: "4px 4px", textAlign: "right", color: c.worstRisk === "high" ? "#ef4444" : c.worstRisk === "medium" ? "#f59e0b" : "#22c55e" }}>{c.worstRisk.toUpperCase()}</td>
                          <td style={{ padding: "4px 4px", textAlign: "right", color: isBest ? "#22c55e" : "var(--color-text-primary)" }}>{c.totalCost}</td>
                        </tr>
                      );
                    })}</tbody>
                  </table>
                </div>
              </Card>

              {/* TOP CYCLES */}
              <Card title="Top 5 Cycles Overall">
                <Note>All cycle permutations × T-variants, ranked by cost. All start from X.</Note>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["#", "Cycle", "T-Var", "Time", "CG", "Auth", "Risk", "Cost"].map(h =>
                        <th key={h} style={{ padding: "5px 3px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 8 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{calc.cycleResults.slice(0, 5).map((c, i) => (
                      <tr key={i} style={{ borderBottom: "1px solid var(--color-border-tertiary)", background: i === 0 ? "#22c55e08" : "transparent" }}>
                        <td style={{ padding: "4px 3px", textAlign: "right", fontWeight: i === 0 ? 700 : 400 }}>{i + 1}</td>
                        <td style={{ padding: "4px 3px", textAlign: "left", fontSize: 8 }}>{c.cycleLabel}</td>
                        <td style={{ padding: "4px 3px", textAlign: "left", fontSize: 8, color: T_VARIANTS[c.tVariant]?.color }}>{c.tVariantLabel.split(":")[0]}</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", color: "#3b82f6" }}>{c.totalTime}s</td>
                        <td style={{ padding: "4px 3px", textAlign: "right" }}>{c.totalCgShift}mm</td>
                        <td style={{ padding: "4px 3px", textAlign: "right" }}>{c.worstMinAuth}%</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", color: c.worstRisk === "high" ? "#ef4444" : c.worstRisk === "medium" ? "#f59e0b" : "#22c55e" }}>{c.worstRisk.toUpperCase()}</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", fontWeight: i === 0 ? 700 : 400, color: i === 0 ? "#22c55e" : "var(--color-text-primary)" }}>{c.totalCost}</td>
                      </tr>
                    ))}</tbody>
                  </table>
                </div>
              </Card>

              {/* ALL 12 DIRECTIONAL TRANSITIONS */}
              <Card title="All 12 Directional Transitions (X, O, T, H)">
                <Note>Main config transitions using the currently selected T-shape [-45,+45,0,0]. Cost sorted (easiest first).</Note>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["Transition", "Travel", "Servos", "Time", "CG Shift", "Min Auth", "Safe Time", "Risk", "Cost"].map(h =>
                        <th key={h} style={{ padding: "5px 3px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 8 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{calc.morphTransitions.map((mt, i) => (
                      <tr key={i} style={{ borderBottom: "1px solid var(--color-border-tertiary)" }}>
                        <td style={{ padding: "4px 3px", textAlign: "left", fontSize: 9 }}>
                          <span style={{ color: mt.fromColor, fontWeight: 700 }}>{mt.from}</span>
                          <span style={{ color: "var(--color-text-tertiary)" }}>→</span>
                          <span style={{ color: mt.toColor, fontWeight: 700 }}>{mt.to}</span>
                        </td>
                        <td style={{ padding: "4px 3px", textAlign: "right" }}>{mt.maxTravel}°</td>
                        <td style={{ padding: "4px 3px", textAlign: "right" }}>{mt.servosMoving}/4</td>
                        <td style={{ padding: "4px 3px", textAlign: "right" }}>{mt.servoTime}s</td>
                        <td style={{ padding: "4px 3px", textAlign: "right" }}>{mt.cgShift}mm</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", color: mt.worstAuthDuring < 50 ? "#ef4444" : mt.worstAuthDuring < 80 ? "#f59e0b" : "#22c55e" }}>{mt.worstAuthDuring}%</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", fontWeight: 700, color: "#3b82f6" }}>{mt.recommendedTime}s</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", fontWeight: 600, color: mt.risk === "high" ? "#ef4444" : mt.risk === "medium" ? "#f59e0b" : "#22c55e" }}>{mt.risk.toUpperCase()}</td>
                        <td style={{ padding: "4px 3px", textAlign: "right", fontSize: 9 }}>{mt.transitionCost}</td>
                      </tr>
                    ))}</tbody>
                  </table>
                </div>
              </Card>

              {calc.morphSpeedSweep.length > 0 && calc.worstTransition && <Card title={`Speed Analysis — ${calc.worstTransition.from} → ${calc.worstTransition.to} (hardest in optimal cycle)`}>
                <Chart data={calc.morphSpeedSweep} xKey="duration_s" yKeys={["stability", "alt_drop_cm"]}
                  colors={["#22c55e", "#ef4444"]} xUnit="s" yMin={0} yMax={110}
                  dashed={[false, true]} fills={["#22c55e"]}
                  markers={calc.worstTransition ? [{ x: calc.worstTransition.recommendedTime, label: "Recommended", color: "#3b82f6" }] : []}
                  xLabel="Transition Duration (seconds)"
                  zones={[{ min: 60, max: 110, color: "#22c55e" }]} />
              </Card>}

              <div style={{ background: "#22c55e15", borderRadius: 10, padding: 10, border: "1px solid #22c55e33", fontSize: 11, color: "var(--color-text-success)", marginBottom: 10 }}>
                <strong>Recommended sequence:</strong> {calc.optimalCycle ? calc.optimalCycle.cycleLabel : "N/A"} — total {calc.optimalCycle ? calc.optimalCycle.totalTime + "s" : "N/A"} using {calc.optimalCycle ? calc.optimalCycle.tVariantLabel : "N/A"}.
              </div>
              <div style={{ background: "#f59e0b15", borderRadius: 10, padding: 10, border: "1px solid #f59e0b33", fontSize: 11, color: "var(--color-text-warning)" }}>
                <strong>Flight rules:</strong> (1) Always start and end in X-config. (2) Maintain 3m+ altitude during morphing. (3) Use recommended times as minimums. (4) Follow the optimal cycle order. (5) 50% throttle headroom before morphing.
              </div>
            </div>}

            {/* ═══ ARDUPILOT PARAMS ═══ */}
            {tab === "params" && <div>
              {/* Universal params first */}
              <Card title="Universal Parameters (safe across all configurations)" accent="#22c55e">
                <Note>
                  These gains are computed from the worst-case config (lowest control authority) so the drone remains stable
                  in <strong>every</strong> shape without switching parameter sets mid-flight. Trade-off: X-config will be slightly
                  less responsive than its optimal gains.
                  Worst-case scaling: roll {calc.worstRollScale.toFixed(2)}x, pitch {calc.worstPitchScale.toFixed(2)}x
                  (min authority: roll {calc.minRollAuth.toFixed(0)}%, pitch {calc.minPitchAuth.toFixed(0)}%).
                </Note>
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10, marginBottom: 8 }}>
                  <div>
                    <div style={{ fontSize: 9, fontWeight: 700, textTransform: "uppercase", letterSpacing: "0.06em", color: "#22c55e", marginBottom: 4 }}>Frame & Motor</div>
                    {Object.entries(calc.universalAP).filter(([k]) => k.startsWith("FRAME") || k.startsWith("MOT")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                  </div>
                  <div>
                    <div style={{ fontSize: 9, fontWeight: 700, textTransform: "uppercase", letterSpacing: "0.06em", color: "#22c55e", marginBottom: 4 }}>Battery</div>
                    {Object.entries(calc.universalAP).filter(([k]) => k.startsWith("BATT")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                  </div>
                </div>
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10 }}>
                  <div>
                    <div style={{ fontSize: 9, fontWeight: 700, textTransform: "uppercase", letterSpacing: "0.06em", color: "#22c55e", marginBottom: 4 }}>Attitude Control (worst-case gains)</div>
                    {Object.entries(calc.universalAP).filter(([k]) => k.startsWith("ATC")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                  </div>
                  <div>
                    <div style={{ fontSize: 9, fontWeight: 700, textTransform: "uppercase", letterSpacing: "0.06em", color: "#22c55e", marginBottom: 4 }}>Position, Filters & Limits</div>
                    {Object.entries(calc.universalAP).filter(([k]) => k.startsWith("PSC") || k.startsWith("INS") || k.startsWith("ANGLE")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                  </div>
                </div>
              </Card>

              <div style={{ background: "#22c55e15", borderRadius: 10, padding: 10, border: "1px solid #22c55e33", fontSize: 11, color: "var(--color-text-success)", marginBottom: 14 }}>
                <strong>When to use universal params:</strong> Use these if you need to morph mid-flight without switching parameter sets.
                The gains are tuned for the worst-case geometry, so the drone will fly safely in all shapes.
                Filters are set conservatively (gyro/accel: {calc.universalAP["INS_GYRO_FILTER"]}Hz) and ANGLE_MAX is limited to {calc.universalAP["ANGLE_MAX"] / 100}° for safety during morphing.
              </div>

              <h2 style={{ fontSize: 13, fontWeight: 700, margin: "16px 0 8px", color: "var(--color-text-secondary)" }}>Per-Configuration Parameters (optimal for each shape)</h2>
              {Object.entries(CONFIGS).map(([k, cfg]) => {
                const cd = calc.configData[k];
                const ap = cd.ap;
                return (
                  <div key={k} style={{ marginBottom: 16 }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 6 }}>
                      <ConfigShape servoDeg={cfg.servo} color={cfg.color} size={50} />
                      <div>
                        <h2 style={{ fontSize: 14, fontWeight: 700, margin: 0, color: cfg.color }}>{cfg.label} Parameters</h2>
                        <div style={{ fontSize: 10, color: "var(--color-text-secondary)" }}>{cfg.desc} · Gain scale: roll {cd.rollGainScale}x, pitch {cd.pitchGainScale}x</div>
                      </div>
                    </div>
                    <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10 }}>
                      <Card title="Frame & Motor">
                        {Object.entries(ap).filter(([k]) => k.startsWith("FRAME") || k.startsWith("MOT")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                      </Card>
                      <Card title={`Battery (${batt.label} x${nBatt})`}>
                        {Object.entries(ap).filter(([k]) => k.startsWith("BATT")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                      </Card>
                      <Card title="Attitude Control" accent={cfg.color}>
                        {Object.entries(ap).filter(([k]) => k.startsWith("ATC")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                      </Card>
                      <Card title="Position & Filters">
                        {Object.entries(ap).filter(([k]) => k.startsWith("PSC") || k.startsWith("INS") || k.startsWith("ANGLE")).map(([k, v]) => <Row key={k} label={k} value={v} />)}
                      </Card>
                    </div>
                  </div>
                );
              })}
              <div style={{ background: "#3b82f615", borderRadius: 10, padding: 10, border: "1px solid #3b82f633", fontSize: 11, color: "var(--color-text-info)" }}>
                <strong>Per-config params are optimal</strong> for each individual shape but require switching parameter sets when morphing.
                Use ArduPilot Lua scripting to automate: detect servo positions → load matching param set.
                ALWAYS run <strong>AUTOTUNE</strong> in each configuration separately to refine these starting values.
              </div>
            </div>}

            {/* ═══ FLIGHT DATA / GRAPHS ═══ */}
            {tab === "upload" && <div>
              <Card title="Connect to ArduPilot via MAVLink Bridge" accent="#22c55e">
                <Note>
                  This uploads parameters wirelessly to ArduPilot via a Python bridge script that relays WebSocket messages to MAVLink. Run <code style={{ background: "#0a0a0f", padding: "1px 4px", borderRadius: 3 }}>python3 mavlink_bridge.py</code> on a computer connected to your telemetry radio, WiFi telemetry module, or companion computer. The bridge forwards all parameter changes to ArduPilot's standard UDP port 14550.
                </Note>
                <div style={{ display: "grid", gridTemplateColumns: "1fr auto auto", gap: 8, alignItems: "center", marginBottom: 10 }}>
                  <input
                    type="text"
                    value={wsUrl}
                    onChange={e => setWsUrl(e.target.value)}
                    disabled={wsConnected}
                    style={{ padding: "8px 10px", fontSize: 12, background: "#0a0a0f", color: "#e5e7eb", border: "1px solid #333", borderRadius: 6, fontFamily: "var(--font-mono)" }}
                    placeholder="ws://localhost:8765"
                  />
                  <button
                    onClick={() => {
                      if (wsConnected) {
                        if (wsRef) wsRef.close();
                        return;
                      }
                      try {
                        const ws = new WebSocket(wsUrl);
                        ws.onopen = () => {
                          setWsConnected(true);
                          setWsRef(ws);
                          setConnectionLog(l => [...l, { t: new Date().toLocaleTimeString(), msg: "Connected to bridge" }]);
                          ws.send(JSON.stringify({ cmd: "ping" }));
                        };
                        ws.onclose = () => {
                          setWsConnected(false);
                          setWsRef(null);
                          setConnectionLog(l => [...l, { t: new Date().toLocaleTimeString(), msg: "Disconnected from bridge" }]);
                        };
                        ws.onerror = () => {
                          setConnectionLog(l => [...l, { t: new Date().toLocaleTimeString(), msg: "Connection error — is the bridge running?", err: true }]);
                        };
                        ws.onmessage = (ev) => {
                          try {
                            const msg = JSON.parse(ev.data);
                            if (msg.type === "param_ack") {
                              const entry = { timestamp: new Date().toISOString(), param: msg.param, oldValue: msg.oldValue, newValue: msg.newValue, status: msg.status || "ok" };
                              setUploadLog(l => {
                                const next = [entry, ...l].slice(0, 500);
                                if (typeof window !== 'undefined' && window.storage) {
                                  window.storage.set('upload_log', JSON.stringify(next)).catch(() => {});
                                }
                                return next;
                              });
                              setUploadStatus(`${msg.param}: ${msg.oldValue} → ${msg.newValue}`);
                            } else if (msg.type === "error") {
                              setConnectionLog(l => [...l, { t: new Date().toLocaleTimeString(), msg: msg.error, err: true }]);
                            } else if (msg.type === "status") {
                              setConnectionLog(l => [...l, { t: new Date().toLocaleTimeString(), msg: msg.message }]);
                            }
                          } catch (e) { /* ignore */ }
                        };
                      } catch (e) {
                        setConnectionLog(l => [...l, { t: new Date().toLocaleTimeString(), msg: `Failed: ${e.message}`, err: true }]);
                      }
                    }}
                    style={{ padding: "8px 14px", fontSize: 11, fontWeight: 700, background: wsConnected ? "#ef4444" : "#22c55e", color: "#fff", border: "none", borderRadius: 6, cursor: "pointer" }}
                  >
                    {wsConnected ? "Disconnect" : "Connect"}
                  </button>
                  <div style={{ padding: "6px 12px", background: wsConnected ? "#22c55e22" : "#ef444422", color: wsConnected ? "#22c55e" : "#ef4444", fontSize: 11, fontWeight: 700, borderRadius: 6 }}>
                    {wsConnected ? "● CONNECTED" : "○ OFFLINE"}
                  </div>
                </div>

                <div style={{ display: "flex", gap: 8, marginBottom: 10, flexWrap: "wrap" }}>
                  <button
                    disabled={!wsConnected}
                    onClick={() => {
                      if (!wsRef) return;
                      wsRef.send(JSON.stringify({ cmd: "test_connection" }));
                      setUploadStatus("Testing connection to ArduPilot...");
                    }}
                    style={{ padding: "6px 12px", fontSize: 11, background: wsConnected ? "#3b82f6" : "#334155", color: "#fff", border: "none", borderRadius: 6, cursor: wsConnected ? "pointer" : "not-allowed", opacity: wsConnected ? 1 : 0.5 }}
                  >Test Link</button>
                  <button
                    disabled={!wsConnected}
                    onClick={() => {
                      if (!wsRef) return;
                      const params = Object.keys(calc.universalAP);
                      wsRef.send(JSON.stringify({ cmd: "backup_params", params }));
                      setUploadStatus("Backing up current values from ArduPilot...");
                    }}
                    style={{ padding: "6px 12px", fontSize: 11, background: wsConnected ? "#f59e0b" : "#334155", color: "#fff", border: "none", borderRadius: 6, cursor: wsConnected ? "pointer" : "not-allowed", opacity: wsConnected ? 1 : 0.5 }}
                  >Backup Current Values</button>
                  <button
                    disabled={!wsConnected}
                    onClick={() => {
                      if (!wsRef) return;
                      const paramsToSend = Object.keys(selectedForUpload).filter(k => selectedForUpload[k]);
                      const toUpload = paramsToSend.length > 0 ? paramsToSend : Object.keys(calc.universalAP);
                      if (!confirm(`Upload ${toUpload.length} parameters to ArduPilot?`)) return;
                      toUpload.forEach(p => {
                        wsRef.send(JSON.stringify({ cmd: "set_param", param: p, value: Number(calc.universalAP[p]) }));
                      });
                      setUploadStatus(`Uploading ${toUpload.length} parameters...`);
                    }}
                    style={{ padding: "6px 12px", fontSize: 11, fontWeight: 700, background: wsConnected ? "#22c55e" : "#334155", color: "#fff", border: "none", borderRadius: 6, cursor: wsConnected ? "pointer" : "not-allowed", opacity: wsConnected ? 1 : 0.5 }}
                  >Upload Parameters →</button>
                  <button
                    onClick={() => {
                      const lines = Object.entries(calc.universalAP).map(([k, v]) => `${k},${v}`).join("\n");
                      const blob = new Blob([lines], { type: "text/plain" });
                      const url = URL.createObjectURL(blob);
                      const a = document.createElement("a");
                      a.href = url;
                      a.download = "morphing_drone.parm";
                      a.click();
                    }}
                    style={{ padding: "6px 12px", fontSize: 11, background: "#6b7280", color: "#fff", border: "none", borderRadius: 6, cursor: "pointer" }}
                  >Download .parm File</button>
                </div>

                {uploadStatus && <div style={{ padding: 8, background: "#0a0a0f", borderRadius: 6, fontSize: 11, fontFamily: "var(--font-mono)", color: "#94a3b8" }}>{uploadStatus}</div>}
              </Card>

              <Card title="Select Parameters to Upload" accent="#3b82f6">
                <Note>Check the parameters you want to upload. If none selected, ALL parameters below will be uploaded.</Note>
                <div style={{ display: "flex", gap: 6, marginBottom: 8 }}>
                  <button onClick={() => {
                    const all = {}; Object.keys(calc.universalAP).forEach(k => all[k] = true); setSelectedForUpload(all);
                  }} style={{ padding: "4px 10px", fontSize: 10, background: "#1f2937", color: "#e5e7eb", border: "1px solid #374151", borderRadius: 4, cursor: "pointer" }}>Select All</button>
                  <button onClick={() => setSelectedForUpload({})} style={{ padding: "4px 10px", fontSize: 10, background: "#1f2937", color: "#e5e7eb", border: "1px solid #374151", borderRadius: 4, cursor: "pointer" }}>Clear</button>
                </div>
                <div style={{ display: "grid", gridTemplateColumns: "repeat(2, 1fr)", gap: 4, maxHeight: 280, overflowY: "auto", padding: 6, background: "#0a0a0f", borderRadius: 6 }}>
                  {Object.entries(calc.universalAP).map(([p, v]) => (
                    <label key={p} style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 10, fontFamily: "var(--font-mono)", padding: "3px 6px", cursor: "pointer" }}>
                      <input type="checkbox" checked={!!selectedForUpload[p]} onChange={e => setSelectedForUpload(s => ({ ...s, [p]: e.target.checked }))} />
                      <span style={{ color: "#94a3b8", minWidth: 130 }}>{p}</span>
                      <span style={{ color: "#22c55e", fontWeight: 700 }}>{v}</span>
                    </label>
                  ))}
                </div>
              </Card>

              <Card title="Change Log — Parameter History" accent="#a78bfa">
                <Note>Every parameter change made through this interface. Previous values are recorded so you can revert. Log persists across sessions.</Note>
                <div style={{ display: "flex", gap: 6, marginBottom: 8 }}>
                  <button
                    onClick={() => {
                      const csv = "Timestamp,Parameter,Old Value,New Value,Status\n" +
                        uploadLog.map(e => `${e.timestamp},${e.param},${e.oldValue},${e.newValue},${e.status}`).join("\n");
                      const blob = new Blob([csv], { type: "text/csv" });
                      const url = URL.createObjectURL(blob);
                      const a = document.createElement("a");
                      a.href = url; a.download = "param_change_log.csv"; a.click();
                    }}
                    style={{ padding: "4px 10px", fontSize: 10, background: "#1f2937", color: "#e5e7eb", border: "1px solid #374151", borderRadius: 4, cursor: "pointer" }}
                  >Export CSV</button>
                  <button
                    disabled={!wsConnected || uploadLog.length === 0}
                    onClick={() => {
                      if (!wsRef || !confirm("Revert ALL logged changes back to their previous values?")) return;
                      const seen = new Set();
                      const toRevert = [];
                      for (const e of uploadLog) {
                        if (!seen.has(e.param)) { seen.add(e.param); toRevert.push(e); }
                      }
                      toRevert.forEach(e => {
                        wsRef.send(JSON.stringify({ cmd: "set_param", param: e.param, value: Number(e.oldValue) }));
                      });
                      setUploadStatus(`Reverting ${toRevert.length} parameters to previous values...`);
                    }}
                    style={{ padding: "4px 10px", fontSize: 10, background: wsConnected && uploadLog.length > 0 ? "#ef4444" : "#334155", color: "#fff", border: "none", borderRadius: 4, cursor: wsConnected && uploadLog.length > 0 ? "pointer" : "not-allowed", opacity: wsConnected && uploadLog.length > 0 ? 1 : 0.5 }}
                  >Revert All</button>
                  <button
                    onClick={() => {
                      if (!confirm("Clear the change log? This cannot be undone.")) return;
                      setUploadLog([]);
                      if (typeof window !== 'undefined' && window.storage) window.storage.delete('upload_log').catch(() => {});
                    }}
                    style={{ padding: "4px 10px", fontSize: 10, background: "#1f2937", color: "#e5e7eb", border: "1px solid #374151", borderRadius: 4, cursor: "pointer" }}
                  >Clear Log</button>
                </div>
                <div style={{ maxHeight: 340, overflowY: "auto", background: "#0a0a0f", borderRadius: 6 }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead style={{ position: "sticky", top: 0, background: "#1a1a2e", zIndex: 1 }}><tr>
                      {["Time", "Parameter", "Old Value", "New Value", "Status", "Revert"].map(h => <th key={h} style={{ padding: "6px 8px", textAlign: "left", color: "#94a3b8", fontWeight: 700, fontSize: 9, borderBottom: "1px solid #333" }}>{h}</th>)}
                    </tr></thead>
                    <tbody>
                      {uploadLog.length === 0 && <tr><td colSpan="6" style={{ padding: 20, textAlign: "center", color: "#64748b" }}>No parameter changes logged yet. Changes will appear here after upload.</td></tr>}
                      {uploadLog.map((e, i) => (
                        <tr key={i} style={{ borderBottom: "1px solid #1a1a2e" }}>
                          <td style={{ padding: "4px 8px", color: "#64748b", fontSize: 9 }}>{new Date(e.timestamp).toLocaleTimeString()}</td>
                          <td style={{ padding: "4px 8px", color: "#e5e7eb" }}>{e.param}</td>
                          <td style={{ padding: "4px 8px", color: "#f59e0b" }}>{e.oldValue}</td>
                          <td style={{ padding: "4px 8px", color: "#22c55e" }}>{e.newValue}</td>
                          <td style={{ padding: "4px 8px", color: e.status === "ok" ? "#22c55e" : "#ef4444" }}>{e.status}</td>
                          <td style={{ padding: "4px 8px" }}>
                            <button disabled={!wsConnected} onClick={() => {
                              if (!wsRef) return;
                              wsRef.send(JSON.stringify({ cmd: "set_param", param: e.param, value: Number(e.oldValue) }));
                              setUploadStatus(`Reverting ${e.param} to ${e.oldValue}...`);
                            }} style={{ padding: "2px 6px", fontSize: 9, background: wsConnected ? "#ef4444" : "#334155", color: "#fff", border: "none", borderRadius: 3, cursor: wsConnected ? "pointer" : "not-allowed", opacity: wsConnected ? 1 : 0.5 }}>↶</button>
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </Card>

              <Card title="Connection Log" accent="#64748b">
                <div style={{ maxHeight: 150, overflowY: "auto", background: "#0a0a0f", borderRadius: 6, padding: 8, fontSize: 10, fontFamily: "var(--font-mono)" }}>
                  {connectionLog.length === 0 && <div style={{ color: "#64748b" }}>No messages yet.</div>}
                  {connectionLog.slice(-50).map((l, i) => (
                    <div key={i} style={{ color: l.err ? "#ef4444" : "#94a3b8", marginBottom: 2 }}>
                      <span style={{ color: "#64748b" }}>[{l.t}]</span> {l.msg}
                    </div>
                  ))}
                </div>
              </Card>
            </div>}

            {tab === "graphs" && <div>
              <Card title="Flight Data — How Each Metric is Calculated" accent="#a78bfa">
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 12, fontSize: 11, color: "var(--color-text-secondary)", lineHeight: 1.6 }}>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Thrust from RPM (per EDF)</div>
                    <code style={{ background: "var(--color-background-secondary)", padding: "2px 6px", borderRadius: 4, fontSize: 11 }}>T = k_t · ω² · ρ · D⁴</code>
                    <div style={{ marginTop: 2 }}>From bench test table, interpolated linearly between measured points at 0/10/20…100% throttle.</div>
                  </div>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Electrical Power (per EDF)</div>
                    <code style={{ background: "var(--color-background-secondary)", padding: "2px 6px", borderRadius: 4, fontSize: 11 }}>P = V · I</code>
                    <div style={{ marginTop: 2 }}>Battery voltage × motor current drawn. Measured directly on the test stand.</div>
                  </div>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Current Draw (per EDF)</div>
                    <code style={{ background: "var(--color-background-secondary)", padding: "2px 6px", borderRadius: 4, fontSize: 11 }}>I = P / V</code>
                    <div style={{ marginTop: 2 }}>Derived from measured power and nominal battery voltage at 4S (14.8 V).</div>
                  </div>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Motor RPM</div>
                    <code style={{ background: "var(--color-background-secondary)", padding: "2px 6px", borderRadius: 4, fontSize: 11 }}>RPM = Kv · V · throttle%</code>
                    <div style={{ marginTop: 2 }}>Ideal RPM from motor Kv rating, derated for load per bench table.</div>
                  </div>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Efficiency (g/W)</div>
                    <code style={{ background: "var(--color-background-secondary)", padding: "2px 6px", borderRadius: 4, fontSize: 11 }}>η = thrust_grams / power_watts</code>
                    <div style={{ marginTop: 2 }}>Higher is better. Typical peak is 50-70% throttle for EDFs.</div>
                  </div>
                  <div>
                    <div style={{ fontWeight: 700, color: "var(--color-text-primary)", marginBottom: 4 }}>Hover Throttle</div>
                    <code style={{ background: "var(--color-background-secondary)", padding: "2px 6px", borderRadius: 4, fontSize: 11 }}>throttle where 4·T(throttle) = AUW·g</code>
                    <div style={{ marginTop: 2 }}>Solved by interpolating the thrust curve until 4 motors equal the vehicle weight.</div>
                  </div>
                </div>
              </Card>

              <Card title="Power vs Throttle (per EDF)">
                <Note>Shows electrical power consumed by a single EDF motor as throttle increases. The hover marker shows where a single motor sits when the drone is holding altitude. Formula: <code>P = V · I</code> where V = battery voltage and I is the measured current from the bench test.</Note>
                <Chart data={sweep} xKey="throttle" yKeys={["power_w"]} colors={["#f59e0b"]} fills={["#f59e0b"]} xUnit="%"
                  yMax={Math.ceil(motor.max_power * 1.1)} markers={calc.canHover ? [{ x: calc.hoverT, label: "Hover", color: "#3b82f6" }] : []}
                  xLabel="Throttle (%)" />
              </Card>

              <Card title="Current vs Throttle (per EDF)">
                <Note>Battery current drawn by a single EDF. Critical for sizing the battery's discharge rating (C-rating) and ESC amperage. Total drone current = 4 × this value. Formula: <code>I = P / V</code>.</Note>
                <Chart data={sweep} xKey="throttle" yKeys={["current_a"]} colors={["#ef4444"]} fills={["#ef4444"]} xUnit="%"
                  yMax={Math.ceil(motor.peak_a * 1.1)} markers={calc.canHover ? [{ x: calc.hoverT, label: "Hover", color: "#3b82f6" }] : []}
                  xLabel="Throttle (%)" />
              </Card>

              <Card title="RPM vs Throttle">
                <Note>Motor rotational speed vs throttle setting. Formula: <code>RPM ≈ Kv · V · throttle%</code> derated for propeller load. Important for matching propeller Reynolds number and ensuring the motor stays within its design envelope.</Note>
                <Chart data={sweep} xKey="throttle" yKeys={["rpm"]} colors={["#a78bfa"]} fills={["#a78bfa"]} xUnit="%"
                  yMax={45000} markers={calc.canHover ? [{ x: calc.hoverT, label: "Hover", color: "#f59e0b" }] : []}
                  yFormatter={v => (v / 1000).toFixed(0) + "k"}
                  xLabel="Throttle (%)" />
              </Card>

              <Card title="Efficiency (g/W per EDF)">
                <Note>Thrust produced per watt of electrical power consumed. Formula: <code>η = thrust_grams / power_watts</code>. Higher is better — you want to hover near the peak of this curve to maximize flight time. EDFs typically peak at 50-70% throttle.</Note>
                <Chart data={sweep} xKey="throttle" yKeys={["eff"]} colors={["#22c55e"]} fills={["#22c55e"]} xUnit="%"
                  yMax={4} markers={calc.canHover ? [{ x: calc.hoverT, label: "Hover", color: "#f59e0b" }] : []}
                  yFormatter={v => v.toFixed(1)}
                  xLabel="Throttle (%)" />
              </Card>

              <Card title="Thrust vs Throttle (per EDF)">
                <Note>Static thrust output vs throttle setting from bench test data. Hover marker shows the throttle needed to sustain <code>{calc.auw.toFixed(2)} kg</code> AUW (4 motors × hover thrust = total weight × g). Green zone = efficient operating range. Hover requirement: <code>T_hover = AUW · g / 4 = {(calc.auw * 9.81 / 4).toFixed(2)} N per motor</code>.</Note>
                <Chart data={sweep} xKey="throttle" yKeys={["thrust_g"]} colors={["#3b82f6"]} fills={["#3b82f6"]} xUnit="%"
                  yMax={Math.ceil(motor.table[motor.table.length - 1].thrust_g * 1.15)}
                  markers={calc.canHover ? [{ x: calc.hoverT, label: `Hover ${calc.hoverT.toFixed(0)}%`, color: "#f59e0b" }] : []}
                  yFormatter={v => v >= 1000 ? (v / 1000).toFixed(1) + "kg" : v.toFixed(0) + "g"}
                  zones={[{ min: 0, max: motor.table[motor.table.length - 1].thrust_g * 0.6, color: "#22c55e" }]}
                  xLabel="Throttle (%)" />
              </Card>
            </div>}

            {/* ═══ HARDWARE ═══ */}
            {tab === "motor" && <div>
              <Card title={`${motor.label} — Bench Test Data`}>
                <div style={{ overflowX: "auto" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 10, fontFamily: "var(--font-mono)" }}>
                    <thead><tr style={{ borderBottom: "2px solid var(--color-border-secondary)" }}>
                      {["Thr%", "Thrust", "Current", "RPM", "Power", "Torque", "Eff"].map(h =>
                        <th key={h} style={{ padding: "5px 5px", textAlign: "right", color: "var(--color-text-secondary)", fontWeight: 600, fontSize: 9 }}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{motor.table.map((r, i) =>
                      <tr key={i} style={{ borderBottom: "1px solid var(--color-border-tertiary)" }}>
                        <td style={{ padding: "3px 5px", textAlign: "right" }}>{r.throttle}%</td>
                        <td style={{ padding: "3px 5px", textAlign: "right", color: "#3b82f6" }}>{r.thrust_g}g</td>
                        <td style={{ padding: "3px 5px", textAlign: "right" }}>{r.current_a}A</td>
                        <td style={{ padding: "3px 5px", textAlign: "right" }}>{r.rpm}</td>
                        <td style={{ padding: "3px 5px", textAlign: "right" }}>{r.power_w}W</td>
                        <td style={{ padding: "3px 5px", textAlign: "right", color: "#06b6d4" }}>{r.torque_nm}Nm</td>
                        <td style={{ padding: "3px 5px", textAlign: "right", color: "#22c55e" }}>{r.eff}g/W</td>
                      </tr>
                    )}</tbody>
                  </table>
                </div>
              </Card>
              <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10 }}>
                <Card title="EDF Motor Specs">
                  {[["KV", motor.kv], ["Weight", motor.weight_g + " g"], ["Resistance", motor.resistance_mohm + " mΩ"],
                  ["Peak Current", motor.peak_a + " A"], ["Max Power", motor.max_power + " W"],
                  ["Voltage", motor.voltage_s + "S LiPo"], ["EDF Size", "70mm ducted fan"],
                  ["Max Thrust", (motor.table[motor.table.length - 1].thrust_g / 1000).toFixed(2) + " kg"],
                  ].map(([l, v]) => <Row key={l} label={l} value={v} />)}
                </Card>
                <Card title="Battery Specs">
                  {[["Pack", batt.label], ["Capacity", batt.capacity_mah + " mAh"], ["Config", batt.cells + "S"],
                  ["Nominal V", batt.nominal_v + " V"], ["Full V", batt.full_v + " V"],
                  ["Energy", batt.energy_wh + " Wh"], ["Weight", batt.weight_g + " g"],
                  ["Discharge", batt.discharge_c + "C"], ["Max Current", batt.max_a + " A"],
                  ].map(([l, v]) => <Row key={l} label={l} value={v} />)}
                </Card>
              </div>
              <Card title="Frame Design (from design doc)">
                {[["Configuration", "Quad X — morphing arms"],
                ["Center Plate", "150 × 150 mm, 2-3mm carbon fiber"],
                ["Arm Length", "318mm (center to EDF)"],
                ["Arm Segments", "Inner 159mm + Outer 159mm"],
                ["Servo Position", "159mm from center (arm midpoint)"],
                ["Servo Type", "35KG coreless (x4)"],
                ["EDF Diameter", "70mm"],
                ["Joint Range", "±90° per servo"],
                ["FC Mount", "30.5 × 30.5mm standard"],
                ].map(([l, v]) => <Row key={l} label={l} value={v} />)}
              </Card>
            </div>}
          </div>
        </div>
      </div>
    </div>
    </>
  );
}