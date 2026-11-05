import { useState, useMemo } from "react";

/* ═══ MOTOR DATABASES ═══ */
const MOTORS = {
  "U13II_KV130_12S": {
    label: "U13II KV130 (12S, G32x11)",
    kv: 130, weight_g: 990, resistance_mohm: 18, voltage_s: 12, nominal_v: 44.4,
    full_v: 50.4, peak_a: 118, max_power: 5659,
    table: [
      { throttle:50,thrust_g:7700,torque_nm:3.1,current_a:20.4,rpm:2612,power_w:979,eff:7.86 },
      { throttle:55,thrust_g:10000,torque_nm:4.0,current_a:30.8,rpm:3000,power_w:1478,eff:6.76 },
      { throttle:60,thrust_g:11500,torque_nm:4.7,current_a:38.4,rpm:3215,power_w:1843,eff:6.24 },
      { throttle:65,thrust_g:13500,torque_nm:5.5,current_a:46.7,rpm:3428,power_w:2242,eff:6.02 },
      { throttle:75,thrust_g:16700,torque_nm:6.9,current_a:65.4,rpm:3832,power_w:3139,eff:5.32 },
      { throttle:85,thrust_g:19700,torque_nm:8.1,current_a:83.7,rpm:4164,power_w:4018,eff:4.90 },
      { throttle:100,thrust_g:24300,torque_nm:10.0,current_a:117.9,rpm:4609,power_w:5659,eff:4.29 },
    ],
  },
  "U13II_KV65_24S": {
    label: "U13II KV65 (24S, G32x11)",
    kv: 65, weight_g: 975, resistance_mohm: 73, voltage_s: 24, nominal_v: 88.8,
    full_v: 100.8, peak_a: 68, max_power: 6727,
    table: [
      { throttle:40,thrust_g:8585,torque_nm:3.5,current_a:11.7,rpm:2485,power_w:1161,eff:7.4 },
      { throttle:45,thrust_g:9671,torque_nm:3.9,current_a:13.8,rpm:2639,power_w:1374,eff:7.0 },
      { throttle:50,thrust_g:10714,torque_nm:4.3,current_a:16.0,rpm:2777,power_w:1589,eff:6.7 },
      { throttle:55,thrust_g:11871,torque_nm:4.7,current_a:18.7,rpm:2922,power_w:1854,eff:6.4 },
      { throttle:60,thrust_g:13128,torque_nm:5.2,current_a:21.4,rpm:3055,power_w:2125,eff:6.2 },
      { throttle:65,thrust_g:14798,torque_nm:5.9,current_a:25.3,rpm:3235,power_w:2512,eff:5.9 },
      { throttle:70,thrust_g:16439,torque_nm:6.6,current_a:29.8,rpm:3414,power_w:2955,eff:5.6 },
      { throttle:75,thrust_g:18380,torque_nm:7.4,current_a:34.9,rpm:3596,power_w:3450,eff:5.3 },
      { throttle:80,thrust_g:20889,torque_nm:8.3,current_a:42.1,rpm:3803,power_w:4156,eff:5.0 },
      { throttle:90,thrust_g:24982,torque_nm:9.9,current_a:54.7,rpm:4125,power_w:5392,eff:4.6 },
      { throttle:100,thrust_g:28711,torque_nm:11.4,current_a:68.5,rpm:4397,power_w:6727,eff:4.3 },
    ],
  },
};

const BATTERIES = {
  "TATTU_22AH_12S": { label:"Tattu Plus 22Ah 12S", capacity_mah:22000, cells:12, nominal_v:44.4, full_v:50.4, weight_g:5800, discharge_c:25, max_a:550, energy_wh:976.8 },
  "TATTU_22AH_24S": { label:"Tattu Plus 22Ah (2x12S series)", capacity_mah:22000, cells:24, nominal_v:88.8, full_v:100.8, weight_g:11600, discharge_c:25, max_a:550, energy_wh:1953.6 },
};

function interp(table, throttle) {
  if(throttle<=table[0].throttle) return {...table[0]};
  if(throttle>=table[table.length-1].throttle) return {...table[table.length-1]};
  for(let i=0;i<table.length-1;i++){
    if(throttle>=table[i].throttle&&throttle<=table[i+1].throttle){
      const t=(throttle-table[i].throttle)/(table[i+1].throttle-table[i].throttle);
      const r={};
      for(const k of Object.keys(table[i])) r[k]=table[i][k]+t*(table[i+1][k]-table[i][k]);
      return r;
    }
  }
  return {...table[table.length-1]};
}

function makeSweep(table,steps=80){
  const mn=table[0].throttle, mx=table[table.length-1].throttle, out=[];
  for(let i=0;i<=steps;i++){
    const t=mn+(mx-mn)*i/steps, d=interp(table,t);
    out.push({throttle:+t.toFixed(1),thrust_kg:+(d.thrust_g/1000).toFixed(2),current_a:+d.current_a.toFixed(1),
      power_w:+d.power_w.toFixed(0),rpm:+d.rpm.toFixed(0),torque_nm:+d.torque_nm.toFixed(2),eff:+d.eff.toFixed(2)});
  }
  return out;
}

const G=9.80665;

/* ═══ SVG CHART ═══ */
function Chart(props) {
  const width = props.width || 540;
  const height = props.height || 210;
  const padL = props.padL || 52;
  const padR = props.padR || 14;
  const padT = props.padT || 16;
  const padB = props.padB || 30;
  const data = props.data;
  const xKey = props.xKey;
  const yKeys = props.yKeys;
  const colors = props.colors;
  const ymO = props.yMax;
  const ymiO = props.yMin;
  const xLabel = props.xLabel;
  const yUnit = props.yUnit || "";
  const xUnit = props.xUnit || "";
  const markers = props.markers || [];
  const fills = props.fills || [];
  const dashed = props.dashed || [];
  const yFormatter = props.yFormatter;
  const zones = props.zones || [];

  const cw = width - padL - padR;
  const ch = height - padT - padB;
  const xVals = data.map(function(d) { return d[xKey]; });
  const xMin = Math.min.apply(null, xVals);
  const xMax = Math.max.apply(null, xVals);
  const allY = yKeys.flatMap(function(k) { return data.map(function(d) { return d[k]; }); });
  const yMin = ymiO != null ? ymiO : 0;
  const yMax = ymO != null ? ymO : Math.ceil(Math.max.apply(null, allY) * 1.1);
  const x = function(v) { return padL + ((v - xMin) / (xMax - xMin || 1)) * cw; };
  const y = function(v) { return padT + ch - ((v - yMin) / (yMax - yMin || 1)) * ch; };
  const fmt = yFormatter || function(v) {
    if (v >= 10000) return (v / 1000).toFixed(0) + "k";
    if (v >= 1000) return (v / 1000).toFixed(1) + "k";
    if (Number.isInteger(v)) return String(v);
    return v.toFixed(1);
  };
  const xTicks = Array.from({ length: 6 }, function(_, i) { return xMin + (xMax - xMin) * i / 5; });

  var gridLines = [];
  for (var gi = 0; gi <= 5; gi++) {
    var val = yMin + (yMax - yMin) * gi / 5;
    gridLines.push(
      <g key={"grid" + gi}>
        <line x1={padL} x2={padL + cw} y1={y(val)} y2={y(val)} stroke="#2a2d38" strokeWidth="0.5" strokeDasharray="3,3" />
        <text x={padL - 5} y={y(val) + 3} textAnchor="end" fontSize="8" fill="#555870">{fmt(val)}</text>
      </g>
    );
  }

  var fillPolygons = fills.map(function(f, fi) {
    if (!f) return null;
    var k = yKeys[fi];
    if (!k) return null;
    var pts = data.map(function(d) { return x(d[xKey]) + "," + y(d[k]); }).join(" ");
    var startX = x(data[0][xKey]);
    var endX = x(data[data.length - 1][xKey]);
    var baseY = y(yMin);
    return (
      <polygon key={"fill" + fi}
        points={startX + "," + baseY + " " + pts + " " + endX + "," + baseY}
        fill={"url(#gf" + fi + "_" + xKey + "_" + yKeys.join("_") + ")"}
      />
    );
  });

  var linePaths = yKeys.map(function(k, ki) {
    var pathStr = data.map(function(d, i) {
      return (i === 0 ? "M" : "L") + x(d[xKey]) + "," + y(d[k]);
    }).join(" ");
    return (
      <path key={"line" + k}
        d={pathStr} fill="none" stroke={colors[ki]}
        strokeWidth={dashed[ki] ? 1.5 : 2}
        strokeLinejoin="round"
        strokeDasharray={dashed[ki] ? "5,3" : "none"}
      />
    );
  });

  var markerEls = markers.map(function(m, i) {
    if (m.x < xMin || m.x > xMax) return null;
    return (
      <g key={"marker" + i}>
        <line x1={x(m.x)} x2={x(m.x)} y1={padT} y2={padT + ch}
          stroke={m.color || "#f59e0b"} strokeWidth="1.2" strokeDasharray="4,2" />
        <text x={x(m.x)} y={padT - 3} textAnchor="middle" fontSize="8"
          fill={m.color || "#f59e0b"} fontWeight="600">{m.label}</text>
      </g>
    );
  });

  var zoneRects = zones.map(function(z, i) {
    return (
      <rect key={"zone" + i} x={padL} y={y(z.max)} width={cw}
        height={y(z.min) - y(z.max)} fill={z.color} opacity="0.08" />
    );
  });

  var legendEls = yKeys.length > 1 ? yKeys.map(function(k, ki) {
    return (
      <g key={"leg" + ki}>
        <line x1={padL + cw - 65} x2={padL + cw - 50}
          y1={padT + 8 + ki * 11} y2={padT + 8 + ki * 11}
          stroke={colors[ki]} strokeWidth={dashed[ki] ? 1.5 : 2}
          strokeDasharray={dashed[ki] ? "4,2" : "none"} />
        <text x={padL + cw - 47} y={padT + 11 + ki * 11}
          fontSize="8" fill={colors[ki]} fontWeight="600">
          {k.replace(/_/g, " ")}
        </text>
      </g>
    );
  }) : null;

  var gradientDefs = fills.map(function(f, i) {
    if (!f) return null;
    return (
      <linearGradient key={"grad" + i} id={"gf" + i + "_" + xKey + "_" + yKeys.join("_")}
        x1="0" y1="0" x2="0" y2="1">
        <stop offset="0%" stopColor={f} stopOpacity="0.2" />
        <stop offset="100%" stopColor={f} stopOpacity="0" />
      </linearGradient>
    );
  });

  return (
    <svg viewBox={"0 0 " + width + " " + height} style={{ width: "100%", height: "auto", display: "block" }}>
      <defs>{gradientDefs}</defs>
      {zoneRects}
      {gridLines}
      {xTicks.map(function(t) {
        return (
          <text key={t} x={x(t)} y={height - 5} textAnchor="middle" fontSize="8" fill="#555870">
            {typeof t === "number" ? (Number.isInteger(t) ? t : t.toFixed(1)) : t}{xUnit}
          </text>
        );
      })}
      {xLabel && <text x={padL + cw / 2} y={height} textAnchor="middle" fontSize="7" fill="#444">{xLabel}</text>}
      {fillPolygons}
      {linePaths}
      {markerEls}
      {legendEls}
    </svg>
  );
}

/* ═══ UI PRIMITIVES ═══ */
function Slider({label,value,onChange,min,max,step,unit,desc}){
  return(<div style={{marginBottom:13}}>
    <div style={{display:"flex",justifyContent:"space-between",alignItems:"baseline",marginBottom:2}}>
      <label style={{fontSize:10,fontWeight:700,letterSpacing:"0.07em",textTransform:"uppercase",color:"#8b8fa3"}}>{label}</label>
      <span style={{fontSize:15,fontWeight:600,fontFamily:"monospace",color:"#e8eaed"}}>{typeof value==="number"?(step>=1?value:value.toFixed(1)):value}{unit&&<span style={{fontSize:9,color:"#8b8fa3",marginLeft:2}}>{unit}</span>}</span>
    </div>
    {desc&&<div style={{fontSize:9,color:"#555870",marginBottom:2}}>{desc}</div>}
    <input type="range" min={min} max={max} step={step} value={value} onChange={e=>onChange(Number(e.target.value))} style={{width:"100%",accentColor:"#3b82f6",cursor:"pointer"}}/>
    <div style={{display:"flex",justifyContent:"space-between",fontSize:8,color:"#555870"}}><span>{min}{unit}</span><span>{max}{unit}</span></div>
  </div>);
}
function Stat({label,value,unit,status,detail}){
  const c={good:"#22c55e",warn:"#f59e0b",danger:"#ef4444",info:"#3b82f6"};const col=c[status]||"#8b8fa3";
  return(<div style={{background:"#181b23",borderRadius:10,padding:"10px 12px",border:`1px solid ${col}33`}}>
    <div style={{fontSize:9,fontWeight:600,textTransform:"uppercase",letterSpacing:"0.07em",color:col,marginBottom:2}}>{label}</div>
    <div style={{fontSize:19,fontWeight:700,fontFamily:"monospace",color:col,lineHeight:1.1}}>{value}<span style={{fontSize:10,fontWeight:400,marginLeft:2,color:"#8b8fa3"}}>{unit}</span></div>
    {detail&&<div style={{fontSize:9,color:"#555870",marginTop:2}}>{detail}</div>}
  </div>);
}
function Row({label,value,highlight}){
  return(<div style={{display:"flex",justifyContent:"space-between",padding:"4px 0",borderBottom:"1px solid #2a2d38",fontSize:11}}>
    <span style={{color:"#8b8fa3"}}>{label}</span>
    <span style={{fontFamily:"monospace",fontWeight:600,color:highlight||"#e8eaed"}}>{value}</span>
  </div>);
}
function Card({title,children,accent}){
  return(<div style={{background:"#181b23",borderRadius:12,padding:16,border:`1px solid ${accent||"#2a2d38"}`,marginBottom:14}}>
    {title&&<h3 style={{fontSize:10,fontWeight:700,textTransform:"uppercase",letterSpacing:"0.1em",color:accent||"#8b8fa3",margin:"0 0 10px"}}>{title}</h3>}
    {children}
  </div>);
}
function Note({children}){return (<div style={{fontSize:9,color:"#555870",marginBottom:6}}>{children}</div>)}
function Select({label,value,onChange,options}){
  return(<div style={{marginBottom:13}}>
    <label style={{fontSize:10,fontWeight:700,letterSpacing:"0.07em",textTransform:"uppercase",color:"#8b8fa3",display:"block",marginBottom:3}}>{label}</label>
    <select value={value} onChange={e=>onChange(e.target.value)} style={{width:"100%",padding:"6px 8px",background:"#0f1117",color:"#e8eaed",border:"1px solid #2a2d38",borderRadius:6,fontSize:11,fontFamily:"monospace",cursor:"pointer"}}>
      {options.map(([k,l])=><option key={k} value={k}>{l}</option>)}
    </select>
  </div>);
}

/* ═══ MAIN ═══ */
export default function App(){
  const [motorKey,setMotorKey]=useState("U13II_KV65_24S");
  const [nMotors,setNMotors]=useState(4);
  const [fw,setFw]=useState(6.8);
  const [arm,setArm]=useState(889);
  const [nBatt,setNBatt]=useState(2);
  const [payload,setPayload]=useState(50);
  const [alt,setAlt]=useState(0);
  const [tab,setTab]=useState("overview");

  const motor=MOTORS[motorKey];
  const battKey=motor.voltage_s>=24?"TATTU_22AH_24S":"TATTU_22AH_12S";
  const batt=BATTERIES[battKey];
  const sweep=useMemo(()=>makeSweep(motor.table),[motorKey]);

  const calc=useMemo(()=>{
    const motorW=(motor.weight_g*nMotors)/1000;
    const battW=(batt.weight_g*nBatt)/1000;
    const auw=fw+battW+payload;
    const auwG=auw*1000;
    const rho=Math.exp(-alt/8500);
    const needed=auwG/nMotors/Math.sqrt(rho);
    const tbl=motor.table;
    const minT=tbl[0].throttle,maxT_=tbl[tbl.length-1].throttle;

    let hoverT=maxT_;
    for(let t=minT;t<=maxT_;t+=0.1){if(interp(tbl,t).thrust_g>=needed){hoverT=t;break;}}
    const canHover=hoverT<maxT_-0.5;
    const hp=interp(tbl,hoverT);
    const maxPt=tbl[tbl.length-1];
    const maxThrust=maxPt.thrust_g*nMotors*Math.sqrt(rho);
    const tw=maxThrust/auwG;
    const hoverI=hp.current_a*nMotors;
    const hoverP=hp.power_w*nMotors;
    const usable=batt.capacity_mah*nBatt*0.8;
    const flightMin=canHover?(usable/hoverI)*60/1000:0;
    const maxI=maxPt.current_a*nMotors;
    const battMaxI=batt.max_a*nBatt;

    // Discharge sim
    const dischargeSim=[];
    const intR=(motor.voltage_s>=24?0.012*24:0.012*12)/nBatt;
    for(let pct=100;pct>=15;pct-=1){
      const soc=pct/100;
      const ocv=batt.cells*(3.3+0.9*soc-0.15*soc*(1-soc));
      const vLoad=ocv-hoverI*intR;
      const elapsed=((100-pct)/100)*(batt.capacity_mah*nBatt/1000)/(hoverI/1000)*60;
      dischargeSim.push({soc:pct,ocv:+ocv.toFixed(1),v_load:+vLoad.toFixed(1),time_min:+elapsed.toFixed(1)});
    }

    // Payload sweep
    const payloadSweep=[];
    for(let p=0;p<=80;p+=1){
      const aw=fw+battW+p,awg=aw*1000,nd=awg/nMotors/Math.sqrt(rho);
      let ht=maxT_;
      for(let t=minT;t<=maxT_;t+=0.2){if(interp(tbl,t).thrust_g>=nd){ht=t;break;}}
      if(ht>=maxT_-0.5){payloadSweep.push({payload_kg:p,flight_min:0,hover_pct:100,tw:+(maxPt.thrust_g*nMotors*Math.sqrt(rho)/awg).toFixed(2)});continue;}
      const h=interp(tbl,ht);
      payloadSweep.push({payload_kg:p,flight_min:+((usable/(h.current_a*nMotors))*60/1000).toFixed(1),hover_pct:+ht.toFixed(1),tw:+(maxPt.thrust_g*nMotors*Math.sqrt(rho)/awg).toFixed(2)});
    }

    // Altitude sweep
    const altSweep=[];
    for(let a=0;a<=5000;a+=100){
      const r=Math.exp(-a/8500),nd=auwG/nMotors/Math.sqrt(r);
      let ht=maxT_;
      for(let t=minT;t<=maxT_;t+=0.2){if(interp(tbl,t).thrust_g>=nd){ht=t;break;}}
      let ft=0;
      if(ht<maxT_-0.5){const h=interp(tbl,ht);ft=+((usable/(h.current_a*nMotors))*60/1000).toFixed(1);}
      altSweep.push({alt_m:a,hover_pct:+ht.toFixed(1),tw:+(maxPt.thrust_g*nMotors*Math.sqrt(r)/auwG).toFixed(2),flight_min:ft});
    }

    const heatLoss=hp.current_a**2*(motor.resistance_mohm/1000);
    const mechPower=hp.torque_nm*(hp.rpm*2*Math.PI/60);
    const motorEff=hp.power_w>0?mechPower/hp.power_w*100:0;
    const excessN=(maxThrust/1000-auw)*G;
    const maxClimb=excessN>0?excessN/(auw*G)*10:0;
    const propArea=Math.PI*(32*0.0254/2)**2;
    const discLoading=(auw*G)/(nMotors*propArea);
    const moi=auw*(arm/1000)**2*0.25;
    const maxPayload_kg=maxThrust/1000*0.45-fw-battW;

    // ── Tilt Angle Analysis ──
    // When a multirotor tilts forward by angle θ:
    //   - Vertical thrust component: T × cos(θ) must equal weight (mg)
    //   - Horizontal thrust component: T × sin(θ) provides forward force
    //   - Required total thrust: T = mg / cos(θ)
    //   - So throttle increases as 1/cos(θ)
    // Max tilt is limited by: thrust at 100% throttle × cos(θ) >= weight
    //   → cos(θ_max) = weight / max_thrust
    //   → θ_max = arccos(AUW / max_thrust_kg)
    // Drag model: F_drag = 0.5 × ρ_air × Cd × A × v²
    // At steady forward flight: T×sin(θ) = F_drag
    // Terminal velocity at angle θ: v = sqrt(2 × m × g × tan(θ) / (ρ × Cd × A))

    const weight_N = auw * G;
    const maxThrust_N = (maxThrust / 1000) * G;
    const maxTiltRad = Math.acos(Math.min(1, weight_N / maxThrust_N));
    const maxTiltDeg = maxTiltRad * 180 / Math.PI;

    // Drag parameters (estimated for a 70x70" frame with payload underneath)
    const rhoAir = 1.225 * Math.exp(-alt / 8500); // kg/m³
    const frameSpan_m = arm * 2 / 1000;
    const Cd = 1.2; // bluff body drag coefficient for a flat frame + hanging payload
    const frontalArea = frameSpan_m * 0.15 + 0.3; // frame cross-section + payload estimate m²

    // Tilt sweep: for each angle, compute required throttle, forward speed, power, flight time
    const tiltSweep = [];
    for (var deg = 0; deg <= 45; deg += 0.5) {
      var rad = deg * Math.PI / 180;
      var cosT = Math.cos(rad);
      var sinT = Math.sin(rad);

      // Total thrust needed to maintain altitude at this tilt
      var thrustNeeded_g = auwG / cosT;
      var thrustPerMotor_g = thrustNeeded_g / nMotors;

      // Check if motors can provide this thrust
      var maxMotorThrust = maxPt.thrust_g * Math.sqrt(rho);
      if (thrustPerMotor_g > maxMotorThrust) break; // beyond capability

      // Find throttle for this thrust level
      var thr = maxT_;
      for (var t = minT; t <= maxT_; t += 0.2) {
        if (interp(tbl, t).thrust_g >= thrustPerMotor_g) { thr = t; break; }
      }
      if (thr >= maxT_ - 0.3) break;

      var pt = interp(tbl, thr);
      var totalPower = pt.power_w * nMotors;
      var totalCurrent = pt.current_a * nMotors;

      // Forward force = total_thrust × sin(θ)
      var forwardForce_N = (thrustNeeded_g / 1000) * G * sinT;

      // Equilibrium speed where drag = forward force
      // F_drag = 0.5 × ρ × Cd × A × v²  →  v = sqrt(2F / (ρ×Cd×A))
      var denom = rhoAir * Cd * frontalArea;
      var speed_ms = denom > 0 ? Math.sqrt(2 * forwardForce_N / denom) : 0;
      var speed_kmh = speed_ms * 3.6;
      var speed_mph = speed_ms * 2.237;

      // Flight time at this tilt angle
      var ft = canHover ? (usable / totalCurrent) * 60 / 1000 : 0;

      // Range = speed × flight time
      var range_km = speed_ms * ft * 60 / 1000;

      // Power efficiency in forward flight (distance per energy)
      var km_per_kwh = totalPower > 0 ? (speed_ms * 3600) / totalPower : 0;

      tiltSweep.push({
        angle_deg: deg,
        throttle_pct: +thr.toFixed(1),
        speed_kmh: +speed_kmh.toFixed(1),
        speed_mph: +speed_mph.toFixed(1),
        power_kw: +(totalPower / 1000).toFixed(2),
        current_a: +totalCurrent.toFixed(0),
        flight_min: +ft.toFixed(1),
        range_km: +range_km.toFixed(1),
        km_per_kwh: +km_per_kwh.toFixed(2),
      });
    }

    // Find optimal tilt = angle that maximizes range
    var optimalTilt = { angle_deg: 0, range_km: 0, speed_kmh: 0, flight_min: 0, throttle_pct: 0, power_kw: 0 };
    for (var i = 0; i < tiltSweep.length; i++) {
      if (tiltSweep[i].range_km > optimalTilt.range_km) {
        optimalTilt = tiltSweep[i];
      }
    }

    // Find max speed tilt (last point before motors max out)
    var maxSpeedTilt = tiltSweep.length > 0 ? tiltSweep[tiltSweep.length - 1] : optimalTilt;

    // Recommended ANGLE_MAX for ArduPilot (centidegrees)
    // Use the lesser of: optimal tilt + margin, or max tilt × 0.7 for safety
    var recommendedAngleMax = Math.min(
      Math.round(optimalTilt.angle_deg + 5),
      Math.round(maxTiltDeg * 0.7)
    );
    recommendedAngleMax = Math.max(15, Math.min(35, recommendedAngleMax)); // clamp 15-35 for heavy lift

    // ArduPilot
    const ap={};
    ap["FRAME_CLASS"]=nMotors===4?1:nMotors===6?2:3;
    ap["FRAME_TYPE"]=nMotors===4?1:nMotors===6?0:1;
    ap["MOT_THST_EXPO"]=0.55; ap["MOT_SPIN_MIN"]=0.15; ap["MOT_SPIN_ARM"]=0.10;
    ap["MOT_BAT_VOLT_MAX"]=(batt.cells*4.2).toFixed(1);
    ap["MOT_BAT_VOLT_MIN"]=(batt.cells*3.3).toFixed(1);
    ap["MOT_PWM_MIN"]=1000; ap["MOT_PWM_MAX"]=2000;
    ap["BATT_CAPACITY"]=batt.capacity_mah*nBatt; ap["BATT_MONITOR"]=4;
    ap["BATT_ARM_VOLT"]=(batt.cells*4.0).toFixed(1);
    ap["BATT_CRT_VOLT"]=(batt.cells*3.5).toFixed(1);
    ap["BATT_LOW_VOLT"]=(batt.cells*3.6).toFixed(1);
    const pG=Math.min(0.3,Math.max(0.03,(1/Math.max(tw,1))*0.45));
    ap["ATC_RAT_RLL_P"]=pG.toFixed(4); ap["ATC_RAT_RLL_I"]=(pG*0.5).toFixed(4); ap["ATC_RAT_RLL_D"]=(pG*0.015).toFixed(5);
    ap["ATC_RAT_PIT_P"]=pG.toFixed(4); ap["ATC_RAT_PIT_I"]=(pG*0.5).toFixed(4); ap["ATC_RAT_PIT_D"]=(pG*0.015).toFixed(5);
    ap["ATC_RAT_YAW_P"]=(pG*1.5).toFixed(4); ap["ATC_RAT_YAW_I"]=(pG*0.15).toFixed(4);
    ap["ATC_ACCEL_R_MAX"]=Math.round(80000/(auw*2)); ap["ATC_ACCEL_P_MAX"]=Math.round(80000/(auw*2));
    ap["ATC_ACCEL_Y_MAX"]=Math.round(40000/(auw*2));
    ap["INS_GYRO_FILTER"]=auw>20?8:auw>8?10:15; ap["INS_ACCEL_FILTER"]=auw>20?8:auw>8?10:15;
    ap["PSC_POSXY_P"]=1.0; ap["PSC_VELXY_P"]=auw>20?1.5:2.0; ap["PSC_POSZ_P"]=1.0; ap["PSC_VELZ_P"]=5.0;
    ap["PSC_ACCZ_P"]=tw>0?(1/(tw*G)*2).toFixed(3):"0.200"; ap["PSC_ACCZ_I"]=(parseFloat(ap["PSC_ACCZ_P"])*2).toFixed(3);
    ap["ANGLE_MAX"]=recommendedAngleMax*100; // centidegrees
    ap["LOIT_SPEED"]=Math.min(1000, Math.round(optimalTilt.speed_kmh / 3.6 * 100)); // cm/s
    ap["WPNAV_SPEED"]=ap["LOIT_SPEED"];
    ap["WPNAV_SPEED_DN"]=Math.min(150, Math.round(maxClimb*50)); // cm/s

    return {motorW,battW,auw,auwG,hoverT,hp,maxThrust,tw,hoverI,hoverP,flightMin,maxI,battMaxI,
      dischargeSim,payloadSweep,altSweep,heatLoss,mechPower,motorEff,maxClimb,discLoading,moi,
      canHover,maxPayload_kg,rho,ap,
      tiltSweep,optimalTilt,maxSpeedTilt,maxTiltDeg,recommendedAngleMax};
  },[motorKey,nMotors,fw,arm,nBatt,payload,alt]);

  // Config comparison
  const configs=useMemo(()=>{
    const cfgs=[];
    for(const[mk,m]of Object.entries(MOTORS)){
      const bk=m.voltage_s>=24?"TATTU_22AH_24S":"TATTU_22AH_12S";
      const b=BATTERIES[bk];
      for(const nm of[4,6,8]){
        for(const nb of[1,2,3]){
          const mw=(m.weight_g*nm)/1000, bw=(b.weight_g*nb)/1000;
          const aw=fw+bw+50;
          const maxT=m.table[m.table.length-1].thrust_g*nm/1000;
          const twR=maxT/(aw);
          if(twR<1.5)continue;
          let ht=m.table[m.table.length-1].throttle;
          const nd=aw*1000/nm;
          for(let t=m.table[0].throttle;t<=m.table[m.table.length-1].throttle;t+=0.2){
            if(interp(m.table,t).thrust_g>=nd){ht=t;break;}
          }
          if(ht>=m.table[m.table.length-1].throttle-0.5)continue;
          const hp=interp(m.table,ht);
          const hi=hp.current_a*nm;
          const ft=(b.capacity_mah*nb*0.8/hi)*60/1000;
          cfgs.push({motor:m.label,nMotors:nm,nBatt:nb,battLabel:b.label,auw:+aw.toFixed(1),tw:+twR.toFixed(2),
            hover_pct:+ht.toFixed(1),flight_min:+ft.toFixed(1),hover_power:+(hp.power_w*nm/1000).toFixed(1),
            total_current:+hi.toFixed(0),key:`${mk}_${nm}m_${nb}b`});
        }
      }
    }
    cfgs.sort((a,b)=>b.flight_min-a.flight_min);
    return cfgs;
  },[fw]);

  const twSt=calc.tw>=2.5?"good":calc.tw>=2?"warn":"danger";
  const hSt=!calc.canHover?"danger":calc.hoverT<=50?"good":calc.hoverT<=65?"warn":"danger";
  const fSt=calc.flightMin>=15?"good":calc.flightMin>=8?"warn":"danger";
  const iSt=calc.maxI<=calc.battMaxI?"good":"danger";

  const tabs=[["overview","Overview"],["compare","Config Advisor"],["graphs","Flight Data"],["params","ArduPilot"],["motor","Hardware"]];
  const tBtn=t=>({padding:"6px 12px",fontSize:10,fontWeight:700,letterSpacing:"0.05em",textTransform:"uppercase",
    background:tab===t?"#3b82f6":"transparent",color:tab===t?"#fff":"#8b8fa3",border:"none",borderRadius:6,cursor:"pointer"});

  return(
    <div style={{background:"#0f1117",color:"#e8eaed",fontFamily:"'Inter',system-ui,sans-serif",minHeight:"100vh",padding:"20px 16px"}}>
      <div style={{maxWidth:980,margin:"0 auto"}}>
        <div style={{marginBottom:16}}>
          <div style={{display:"flex",gap:8,alignItems:"center",flexWrap:"wrap"}}>
            <span style={{fontSize:9,background:"#22c55e22",color:"#22c55e",padding:"2px 8px",borderRadius:20,fontWeight:700}}>SIMULATION</span>
            <span style={{fontSize:9,background:"#f59e0b22",color:"#f59e0b",padding:"2px 8px",borderRadius:20,fontWeight:700}}>TARGET: 110 LBS PAYLOAD</span>
          </div>
          <h1 style={{fontSize:21,fontWeight:800,margin:"4px 0",letterSpacing:"-0.02em"}}>ArduPilot Heavy-Lift Quadrotor Estimator</h1>
          <p style={{fontSize:11,color:"#8b8fa3",margin:0}}>T-Motor U13II (KV65/KV130) &middot; Tattu Plus 22Ah &middot; 70x70in CF Truss X-Frame</p>
        </div>

        <div style={{display:"flex",gap:4,marginBottom:14,background:"#181b23",padding:3,borderRadius:8,width:"fit-content",flexWrap:"wrap"}}>
          {tabs.map(([k,l])=><button key={k} onClick={()=>setTab(k)} style={tBtn(k)}>{l}</button>)}
        </div>

        <div style={{display:"grid",gridTemplateColumns:"265px 1fr",gap:16,alignItems:"start"}}>
          {/* ── SIDEBAR ── */}
          <div style={{background:"#181b23",borderRadius:12,padding:14,border:"1px solid #2a2d38"}}>
            <h3 style={{fontSize:10,fontWeight:700,textTransform:"uppercase",letterSpacing:"0.1em",color:"#8b8fa3",margin:"0 0 10px"}}>Configuration</h3>
            <Select label="Motor" value={motorKey} onChange={setMotorKey} options={Object.entries(MOTORS).map(([k,m])=>[k,m.label])}/>
            <Select label="Motor Count" value={String(nMotors)} onChange={v=>setNMotors(Number(v))} options={[["4","4 (Quad X)"],["6","6 (Hex)"],["8","8 (Octo)"]]}/>
            <Slider label="Frame Weight" value={fw} onChange={setFw} min={3} max={20} step={0.1} unit="kg" desc="15 lbs = 6.8 kg without battery"/>
            <Slider label="Arm Length" value={arm} onChange={setArm} min={400} max={1200} step={10} unit="mm" desc="889mm for 70x70in frame"/>
            <Slider label="Battery Packs" value={nBatt} onChange={setNBatt} min={1} max={4} step={1} unit="x" desc={`Parallel ${batt.label}`}/>
            <Slider label="Payload" value={payload} onChange={setPayload} min={0} max={80} step={0.5} unit="kg" desc="110 lbs = 49.9 kg"/>
            <Slider label="Altitude" value={alt} onChange={setAlt} min={0} max={5000} step={100} unit="m"/>

            {!calc.canHover&&<div style={{background:"#ef444422",border:"1px solid #ef4444",borderRadius:8,padding:10,fontSize:11,color:"#ef4444",marginTop:8,fontWeight:600}}>
              CANNOT HOVER — AUW exceeds thrust capacity. Reduce payload or change config.
            </div>}
          </div>

          {/* ── CONTENT ── */}
          <div style={{minWidth:0}}>

            {/* ═══ OVERVIEW ═══ */}
            {tab==="overview"&&<div>
              <div style={{display:"grid",gridTemplateColumns:"repeat(3,1fr)",gap:8,marginBottom:12}}>
                <Stat label="T:W Ratio" value={calc.tw.toFixed(2)} unit=":1" status={twSt} detail={calc.tw>=2.5?"Good for heavy lift":calc.tw>=2?"Minimum safe":"UNSAFE"}/>
                <Stat label="Hover Throttle" value={calc.canHover?calc.hoverT.toFixed(1):"N/A"} unit="%" status={hSt} detail={calc.canHover?`${(calc.hoverP/1000).toFixed(1)}kW total`:"Cannot hover"}/>
                <Stat label="Flight Time" value={calc.flightMin.toFixed(1)} unit="min" status={fSt} detail={`${calc.hoverI.toFixed(0)}A draw`}/>
                <Stat label="Max Current" value={calc.maxI.toFixed(0)} unit="A" status={iSt} detail={`Batt: ${calc.battMaxI}A`}/>
                <Stat label="Max Climb" value={calc.maxClimb.toFixed(1)} unit="m/s" status={calc.maxClimb>3?"good":calc.maxClimb>1?"warn":"danger"}/>
                <Stat label="Disc Loading" value={calc.discLoading.toFixed(0)} unit="N/m2" status={calc.discLoading<300?"good":"warn"}/>
              </div>
              <Card title="Weight Breakdown">
                {[["Frame (no batt)",`${fw.toFixed(1)} kg (${(fw*2.205).toFixed(1)} lbs)`],
                  [`Motors (x${nMotors})`,`${calc.motorW.toFixed(2)} kg`],
                  [`Battery (x${nBatt})`,`${calc.battW.toFixed(2)} kg`],
                  ["Payload",`${payload.toFixed(1)} kg (${(payload*2.205).toFixed(0)} lbs)`],
                ].map(([l,v])=><Row key={l} label={l} value={v}/>)}
                <div style={{display:"flex",justifyContent:"space-between",padding:"6px 0 0",fontSize:13,fontWeight:700}}>
                  <span>All-Up Weight</span>
                  <span style={{fontFamily:"monospace",color:calc.canHover?"#3b82f6":"#ef4444"}}>{calc.auw.toFixed(1)} kg ({(calc.auw*2.205).toFixed(0)} lbs)</span>
                </div>
              </Card>
              <Card title="Thrust vs Throttle (per motor)">
                <Chart data={sweep} xKey="throttle" yKeys={["thrust_kg"]} colors={["#3b82f6"]}
                  fills={["#3b82f6"]} xUnit="%" yMax={Math.ceil(motor.table[motor.table.length-1].thrust_g/1000*1.1)}
                  markers={calc.canHover?[{x:calc.hoverT,label:"Hover",color:"#f59e0b"}]:[]}/>
              </Card>
              <Card title="Frame Geometry">
                {[["Type",`${nMotors===4?"Quad X":nMotors===6?"Hex":"Octo"} - CF Truss`],["Span",`${(arm*2/25.4).toFixed(0)}in / ${arm*2}mm`],
                  ["Arm",`${arm} mm`],["MOI",`${calc.moi.toFixed(3)} kg*m2`]
                ].map(([l,v])=><Row key={l} label={l} value={v}/>)}
              </Card>
            </div>}

            {/* ═══ CONFIG ADVISOR ═══ */}
            {tab==="compare"&&<div>
              <Card title="Configuration Comparison (50kg / 110lbs payload)" accent="#f59e0b">
                <Note>All viable configs that can hover with 50kg payload, sorted by flight time. Green = recommended range.</Note>
                <div style={{overflowX:"auto"}}>
                  <table style={{width:"100%",borderCollapse:"collapse",fontSize:10,fontFamily:"monospace"}}>
                    <thead><tr style={{borderBottom:"2px solid #2a2d38"}}>
                      {["Motor","Motors","Batt","AUW(kg)","T:W","Hover%","Time(min)","Power(kW)","Amps"].map(h=>
                        <th key={h} style={{padding:"5px 6px",textAlign:"right",color:"#8b8fa3",fontWeight:600,fontSize:9,whiteSpace:"nowrap"}}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{configs.map((c,i)=>{
                      const good=c.tw>=2&&c.tw<=3&&c.hover_pct<=60;
                      return(<tr key={c.key} style={{borderBottom:"1px solid #2a2d38",background:good?"#22c55e08":"transparent"}}>
                        <td style={{padding:"4px 6px",textAlign:"left",fontSize:9,whiteSpace:"nowrap"}}>{c.motor.replace("U13II ","")}</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{c.nMotors}</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{c.nBatt}x</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{c.auw}</td>
                        <td style={{padding:"4px 6px",textAlign:"right",color:c.tw>=2?"#22c55e":"#ef4444"}}>{c.tw}:1</td>
                        <td style={{padding:"4px 6px",textAlign:"right",color:c.hover_pct<=55?"#22c55e":"#f59e0b"}}>{c.hover_pct}%</td>
                        <td style={{padding:"4px 6px",textAlign:"right",color:c.flight_min>=10?"#22c55e":"#f59e0b",fontWeight:600}}>{c.flight_min}</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{c.hover_power}</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{c.total_current}A</td>
                      </tr>);
                    })}</tbody>
                  </table>
                </div>
              </Card>
              <Card title="Recommendation" accent="#22c55e">
                <div style={{fontSize:12,lineHeight:1.6,color:"#e8eaed"}}>
                  <p style={{margin:"0 0 8px"}}>For a <strong>110 lb (50 kg) payload</strong> target with a 2:1+ T:W ratio, the best options are:</p>
                  <p style={{margin:"0 0 6px"}}><span style={{color:"#22c55e",fontWeight:700}}>BEST:</span> <strong>U13II KV65 (24S), 4 motors, 2+ battery packs</strong> — This gives a 2:1 T:W ratio hovering around 55%, with ~10-13 min flight time. The KV65 on 24S produces 28.7kg/motor (114.8 kg total) and draws much less current per kg of thrust than the KV130.</p>
                  <p style={{margin:"0 0 6px"}}><span style={{color:"#3b82f6",fontWeight:700}}>SAFER:</span> <strong>U13II KV65, 6 motors (hex), 2 packs</strong> — If your frame supports it, 6 motors gives motor-out redundancy and a more comfortable hover throttle (~40-45%), with better T:W around 2.7:1.</p>
                  <p style={{margin:0,fontSize:10,color:"#8b8fa3"}}>The KV130 (12S) variant <strong>cannot safely lift 110 lbs</strong> on 4 motors — max total thrust of 97.2 kg doesn't leave enough margin above the ~63 kg AUW.</p>
                </div>
              </Card>
            </div>}

            {/* ═══ FLIGHT DATA ═══ */}
            {tab==="graphs"&&<div>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                <Card title="Power vs Throttle (per motor)">
                  <Chart data={sweep} xKey="throttle" yKeys={["power_w"]} colors={["#f59e0b"]} fills={["#f59e0b"]} xUnit="%" yMax={Math.ceil(motor.max_power*1.1)} markers={calc.canHover?[{x:calc.hoverT,label:"Hover",color:"#3b82f6"}]:[]}/>
                </Card>
                <Card title="Current vs Throttle (per motor)">
                  <Chart data={sweep} xKey="throttle" yKeys={["current_a"]} colors={["#ef4444"]} fills={["#ef4444"]} xUnit="%" yMax={Math.ceil(motor.peak_a*1.1)} markers={calc.canHover?[{x:calc.hoverT,label:"Hover",color:"#3b82f6"}]:[]}/>
                </Card>
              </div>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                <Card title="RPM vs Throttle">
                  <Chart data={sweep} xKey="throttle" yKeys={["rpm"]} colors={["#8b5cf6"]} fills={["#8b5cf6"]} xUnit="%" yMax={5000} markers={calc.canHover?[{x:calc.hoverT,label:"Hover",color:"#f59e0b"}]:[]} yFormatter={v=>v.toFixed(0)}/>
                </Card>
                <Card title="Torque vs Throttle (per motor)">
                  <Chart data={sweep} xKey="throttle" yKeys={["torque_nm"]} colors={["#06b6d4"]} fills={["#06b6d4"]} xUnit="%" yMax={14} markers={calc.canHover?[{x:calc.hoverT,label:"Hover",color:"#f59e0b"}]:[]} yFormatter={v=>v.toFixed(1)}/>
                </Card>
              </div>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                <Card title="Efficiency (g/W per motor)">
                  <Chart data={sweep} xKey="throttle" yKeys={["eff"]} colors={["#22c55e"]} fills={["#22c55e"]} xUnit="%" yMax={9} markers={calc.canHover?[{x:calc.hoverT,label:"Hover",color:"#f59e0b"}]:[]} yFormatter={v=>v.toFixed(1)}/>
                </Card>
                <Card title="Battery Discharge at Hover">
                  <Note>Voltage sag model at {calc.hoverI.toFixed(0)}A hover draw</Note>
                  <Chart data={calc.dischargeSim} xKey="time_min" yKeys={["ocv","v_load"]} colors={["#8b8fa3","#f59e0b"]} yMin={motor.voltage_s>=24?70:36} yMax={motor.voltage_s>=24?105:53} dashed={[true,false]} fills={[null,"#f59e0b"]} yFormatter={v=>v.toFixed(0)+"V"} xLabel="Minutes"/>
                </Card>
              </div>
              <Card title="Flight Time vs Payload">
                <Note>Green = flight time, dashed = hover throttle. Vertical marker = current payload.</Note>
                <Chart data={calc.payloadSweep.filter(d=>d.flight_min>0)} xKey="payload_kg" yKeys={["flight_min","hover_pct"]} colors={["#22c55e","#f59e0b"]} yMax={Math.max(50,calc.flightMin*1.4)} xUnit="kg" dashed={[false,true]} fills={["#22c55e"]} yFormatter={v=>v.toFixed(0)} markers={[{x:payload,label:`${payload}kg`,color:"#3b82f6"},{x:49.9,label:"110lbs",color:"#ef4444"}]} xLabel="Payload (kg)"/>
              </Card>
              <Card title="Altitude Envelope">
                <Note>Hover throttle (red) and flight time (green dashed) vs altitude at {calc.auw.toFixed(0)}kg AUW</Note>
                <Chart data={calc.altSweep.filter(d=>d.flight_min>0)} xKey="alt_m" yKeys={["hover_pct","flight_min"]} colors={["#ef4444","#22c55e"]} yMax={100} xUnit="m" dashed={[false,true]} fills={["#ef4444"]} yFormatter={v=>v.toFixed(0)} markers={alt>0?[{x:alt,label:`${alt}m`,color:"#3b82f6"}]:[]} xLabel="Altitude (m)" zones={[{min:0,max:50,color:"#22c55e"},{min:70,max:100,color:"#ef4444"}]}/>
              </Card>
              <Card title="T:W Ratio vs Payload">
                <Note>Safety margin. Below 2:1 (red zone) is dangerous for heavy-lift operations.</Note>
                <Chart data={calc.payloadSweep} xKey="payload_kg" yKeys={["tw"]} colors={["#3b82f6"]} fills={["#3b82f6"]} yMax={Math.ceil(calc.tw*1.3)} yMin={0} xUnit="kg" yFormatter={v=>v.toFixed(1)} markers={[{x:payload,label:`${payload}kg`,color:"#f59e0b"},{x:49.9,label:"110lbs",color:"#ef4444"}]} xLabel="Payload (kg)" zones={[{min:0,max:2,color:"#ef4444"},{min:2,max:2.5,color:"#f59e0b"}]}/>
              </Card>

              {/* ── TILT ANGLE ANALYSIS ── */}
              {calc.tiltSweep.length > 2 && <div>
                <Card title="Tilt Angle Analysis — Forward Flight Envelope" accent="#a78bfa">
                  <Note>
                    When tilted at angle theta, the drone needs thrust = weight / cos(theta) to maintain altitude.
                    The horizontal component drives forward speed. Optimal tilt maximizes range (speed x endurance).
                  </Note>
                  <div style={{display:"grid",gridTemplateColumns:"repeat(3,1fr)",gap:8,marginBottom:12}}>
                    <Stat label="Optimal Tilt" value={calc.optimalTilt.angle_deg.toFixed(1)} unit="deg" status="good"
                      detail={"Max range: " + calc.optimalTilt.range_km + " km"} />
                    <Stat label="Speed at Optimal" value={calc.optimalTilt.speed_mph.toFixed(0)} unit="mph" status="info"
                      detail={calc.optimalTilt.speed_kmh + " km/h"} />
                    <Stat label="Max Speed" value={calc.maxSpeedTilt.speed_mph.toFixed(0)} unit="mph" status="warn"
                      detail={"At " + calc.maxSpeedTilt.angle_deg + " deg tilt"} />
                    <Stat label="ANGLE_MAX" value={calc.recommendedAngleMax} unit="deg" status="info"
                      detail={calc.ap["ANGLE_MAX"] + " centideg"} />
                    <Stat label="Theoretical Max Tilt" value={calc.maxTiltDeg.toFixed(1)} unit="deg" status="danger"
                      detail="Motors saturate beyond this" />
                    <Stat label="Range at Optimal" value={calc.optimalTilt.range_km.toFixed(1)} unit="km" status="good"
                      detail={calc.optimalTilt.flight_min + " min endurance"} />
                  </div>
                </Card>

                <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                  <Card title="Forward Speed vs Tilt Angle">
                    <Note>Equilibrium speed accounting for aerodynamic drag.</Note>
                    <Chart data={calc.tiltSweep} xKey="angle_deg" yKeys={["speed_mph"]}
                      colors={["#a78bfa"]} fills={["#a78bfa"]} xUnit="deg" yMin={0}
                      yMax={Math.ceil(calc.maxSpeedTilt.speed_mph * 1.2)}
                      markers={[{x:calc.optimalTilt.angle_deg, label:"Optimal", color:"#22c55e"},
                                {x:calc.recommendedAngleMax, label:"ANGLE_MAX", color:"#f59e0b"}]}
                      yFormatter={function(v){return v.toFixed(0)+"mph";}}
                      xLabel="Tilt Angle (degrees)" />
                  </Card>
                  <Card title="Throttle Required vs Tilt Angle">
                    <Note>Motors must increase thrust to compensate for vertical component loss.</Note>
                    <Chart data={calc.tiltSweep} xKey="angle_deg" yKeys={["throttle_pct"]}
                      colors={["#ef4444"]} fills={["#ef4444"]} xUnit="deg" yMin={30}
                      yMax={100}
                      markers={[{x:calc.optimalTilt.angle_deg, label:"Optimal", color:"#22c55e"},
                                {x:calc.recommendedAngleMax, label:"ANGLE_MAX", color:"#f59e0b"}]}
                      yFormatter={function(v){return v.toFixed(0)+"%";}}
                      xLabel="Tilt Angle (degrees)"
                      zones={[{min:70,max:100,color:"#ef4444"}]} />
                  </Card>
                </div>

                <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                  <Card title="Range vs Tilt Angle">
                    <Note>Range = speed x endurance. Peak = optimal tilt angle for mission planning.</Note>
                    <Chart data={calc.tiltSweep} xKey="angle_deg" yKeys={["range_km"]}
                      colors={["#22c55e"]} fills={["#22c55e"]} xUnit="deg" yMin={0}
                      markers={[{x:calc.optimalTilt.angle_deg, label:"Optimal", color:"#f59e0b"}]}
                      yFormatter={function(v){return v.toFixed(1)+"km";}}
                      xLabel="Tilt Angle (degrees)" />
                  </Card>
                  <Card title="Flight Time vs Tilt Angle">
                    <Note>Endurance drops as tilt increases due to higher power demand.</Note>
                    <Chart data={calc.tiltSweep} xKey="angle_deg" yKeys={["flight_min"]}
                      colors={["#3b82f6"]} fills={["#3b82f6"]} xUnit="deg" yMin={0}
                      markers={[{x:calc.optimalTilt.angle_deg, label:"Optimal", color:"#22c55e"}]}
                      yFormatter={function(v){return v.toFixed(0)+"m";}}
                      xLabel="Tilt Angle (degrees)" />
                  </Card>
                </div>

                <Card title="Power vs Tilt Angle (total system)">
                  <Note>Total power consumption across all {nMotors} motors at each tilt angle. Dashed = current draw.</Note>
                  <Chart data={calc.tiltSweep} xKey="angle_deg" yKeys={["power_kw","current_a"]}
                    colors={["#f59e0b","#ef4444"]} xUnit="deg"
                    yMax={Math.ceil(calc.maxSpeedTilt.power_kw * 1.2)}
                    markers={[{x:calc.optimalTilt.angle_deg, label:"Optimal", color:"#22c55e"},
                              {x:calc.recommendedAngleMax, label:"ANGLE_MAX", color:"#3b82f6"}]}
                    dashed={[false,true]}
                    xLabel="Tilt Angle (degrees)" />
                </Card>

                <Card title="Tilt Angle Reference Table">
                  <div style={{overflowX:"auto"}}>
                    <table style={{width:"100%",borderCollapse:"collapse",fontSize:10,fontFamily:"monospace"}}>
                      <thead><tr style={{borderBottom:"2px solid #2a2d38"}}>
                        {["Tilt","Speed","Throttle","Power","Current","Time","Range"].map(function(h) {
                          return <th key={h} style={{padding:"5px 6px",textAlign:"right",color:"#8b8fa3",fontWeight:600,fontSize:9}}>{h}</th>;
                        })}
                      </tr></thead>
                      <tbody>{calc.tiltSweep.filter(function(r){return r.angle_deg % 5 === 0 || r.angle_deg === calc.optimalTilt.angle_deg;}).map(function(r,i) {
                        var isOpt = Math.abs(r.angle_deg - calc.optimalTilt.angle_deg) < 0.6;
                        return (
                          <tr key={i} style={{borderBottom:"1px solid #2a2d38", background:isOpt?"#22c55e11":"transparent"}}>
                            <td style={{padding:"4px 6px",textAlign:"right",color:isOpt?"#22c55e":"#e8eaed",fontWeight:isOpt?700:400}}>{r.angle_deg}°{isOpt?" ★":""}</td>
                            <td style={{padding:"4px 6px",textAlign:"right"}}>{r.speed_mph} mph</td>
                            <td style={{padding:"4px 6px",textAlign:"right",color:r.throttle_pct>80?"#ef4444":"#e8eaed"}}>{r.throttle_pct}%</td>
                            <td style={{padding:"4px 6px",textAlign:"right"}}>{r.power_kw} kW</td>
                            <td style={{padding:"4px 6px",textAlign:"right"}}>{r.current_a} A</td>
                            <td style={{padding:"4px 6px",textAlign:"right"}}>{r.flight_min} min</td>
                            <td style={{padding:"4px 6px",textAlign:"right",color:isOpt?"#22c55e":"#e8eaed",fontWeight:isOpt?700:400}}>{r.range_km} km</td>
                          </tr>
                        );
                      })}</tbody>
                    </table>
                  </div>
                </Card>
              </div>}

              <Card title="Power Budget at Hover (per motor)">
                <div style={{display:"grid",gridTemplateColumns:"repeat(3,1fr)",gap:8,marginBottom:10}}>
                  <Stat label="Electrical In" value={calc.hp.power_w.toFixed(0)} unit="W" status="info"/>
                  <Stat label="Mechanical Out" value={calc.mechPower.toFixed(0)} unit="W" status="good"/>
                  <Stat label="I2R Loss" value={calc.heatLoss.toFixed(0)} unit="W" status={calc.heatLoss>150?"warn":"info"}/>
                </div>
                {[["Hover RPM",calc.hp.rpm.toFixed(0)],["Hover Current/motor",calc.hp.current_a.toFixed(1)+" A"],
                  [`Hover Current total (x${nMotors})`,calc.hoverI.toFixed(0)+" A"],
                  ["Motor Efficiency",calc.motorEff.toFixed(1)+"%"],
                  ["Total Hover Power",(calc.hoverP/1000).toFixed(1)+" kW"],
                  ["Energy Used at Hover",(calc.hoverP/1000*calc.flightMin/60).toFixed(2)+" kWh"],
                  ["Disc Loading",calc.discLoading.toFixed(0)+" N/m2"],
                  ["Thrust Headroom",((calc.tw-1)*100).toFixed(0)+"%"],
                ].map(([l,v])=><Row key={l} label={l} value={v}/>)}
              </Card>
            </div>}

            {/* ═══ ARDUPILOT ═══ */}
            {tab==="params"&&<div>
              <Card title={`Frame & Motor (${nMotors===4?"Quad":nMotors===6?"Hex":"Octo"})`}>{Object.entries(calc.ap).filter(([k])=>k.startsWith("FRAME")||k.startsWith("MOT")).map(([k,v])=><Row key={k} label={k} value={v}/>)}</Card>
              <Card title={`Battery (${batt.label} x${nBatt})`}>{Object.entries(calc.ap).filter(([k])=>k.startsWith("BATT")).map(([k,v])=><Row key={k} label={k} value={v}/>)}</Card>
              <Card title="Attitude Control">{Object.entries(calc.ap).filter(([k])=>k.startsWith("ATC")).map(([k,v])=><Row key={k} label={k} value={v}/>)}</Card>
              <Card title="Position & Filters">{Object.entries(calc.ap).filter(([k])=>k.startsWith("PSC")||k.startsWith("INS")).map(([k,v])=><Row key={k} label={k} value={v}/>)}</Card>
              <Card title="Tilt Angle & Navigation">
                {[["ANGLE_MAX",calc.ap["ANGLE_MAX"]],["LOIT_SPEED",calc.ap["LOIT_SPEED"]],["WPNAV_SPEED",calc.ap["WPNAV_SPEED"]],["WPNAV_SPEED_DN",calc.ap["WPNAV_SPEED_DN"]]].map(([k,v])=><Row key={k} label={k} value={v}/>)}
                <div style={{marginTop:8,fontSize:10,color:"#8b8fa3",lineHeight:1.5}}>
                  <strong>ANGLE_MAX</strong> ({calc.recommendedAngleMax}°) is set to the optimal tilt for maximum range plus a small margin, clamped for heavy-lift safety.
                  At this angle the drone achieves ~{calc.optimalTilt.speed_mph.toFixed(0)} mph forward while keeping throttle at ~{calc.optimalTilt.throttle_pct}%.
                  Theoretical max tilt before motors saturate is {calc.maxTiltDeg.toFixed(1)}°, but you should never approach that in practice.
                </div>
              </Card>
              <div style={{background:"#3b82f615",borderRadius:10,padding:12,border:"1px solid #3b82f633",fontSize:11,color:"#93c5fd"}}>
                <strong>Critical for heavy lift:</strong> These gains are conservative starting points for a {calc.auw.toFixed(0)}kg platform. Gyro/accel filters are set low ({calc.ap["INS_GYRO_FILTER"]}Hz) to avoid oscillation on a high-inertia frame. ALWAYS run AUTOTUNE. Start in Stabilize with very gentle stick inputs.
              </div>
            </div>}

            {/* ═══ HARDWARE ═══ */}
            {tab==="motor"&&<div>
              <Card title={`${motor.label} - Bench Test Data`}>
                <div style={{overflowX:"auto"}}>
                  <table style={{width:"100%",borderCollapse:"collapse",fontSize:10,fontFamily:"monospace"}}>
                    <thead><tr style={{borderBottom:"2px solid #2a2d38"}}>
                      {["Thr%","Thrust","Current","RPM","Power","Torque","Eff"].map(h=>
                        <th key={h} style={{padding:"5px 6px",textAlign:"right",color:"#8b8fa3",fontWeight:600,fontSize:9}}>{h}</th>
                      )}
                    </tr></thead>
                    <tbody>{motor.table.map((r,i)=>
                      <tr key={i} style={{borderBottom:"1px solid #2a2d38"}}>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{r.throttle}%</td>
                        <td style={{padding:"4px 6px",textAlign:"right",color:"#3b82f6"}}>{(r.thrust_g/1000).toFixed(1)}kg</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{r.current_a}A</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{r.rpm}</td>
                        <td style={{padding:"4px 6px",textAlign:"right"}}>{r.power_w}W</td>
                        <td style={{padding:"4px 6px",textAlign:"right",color:"#06b6d4"}}>{r.torque_nm}Nm</td>
                        <td style={{padding:"4px 6px",textAlign:"right",color:"#22c55e"}}>{r.eff}g/W</td>
                      </tr>
                    )}</tbody>
                  </table>
                </div>
              </Card>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                <Card title="Motor Specs">
                  {[["KV",motor.kv],["Weight",motor.weight_g+" g"],["Resistance",motor.resistance_mohm+" mohm"],
                    ["Peak Current",motor.peak_a+" A"],["Max Power",motor.max_power+" W"],
                    ["Voltage",motor.voltage_s+"S LiPo"],["Prop",'G32x11" CF'],
                  ].map(([l,v])=><Row key={l} label={l} value={v}/>)}
                </Card>
                <Card title="Battery Specs">
                  {[["Pack",batt.label],["Capacity",batt.capacity_mah+" mAh"],["Config",batt.cells+"S"],
                    ["Nominal V",batt.nominal_v+" V"],["Full V",batt.full_v+" V"],
                    ["Energy",batt.energy_wh+" Wh"],["Weight",batt.weight_g+" g"],
                    ["Discharge",batt.discharge_c+"C"],["Max Current",batt.max_a+" A"],
                  ].map(([l,v])=><Row key={l} label={l} value={v}/>)}
                </Card>
              </div>
            </div>}

          </div>
        </div>
      </div>
    </div>
  );
}
