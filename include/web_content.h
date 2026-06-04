#pragma once

// -----------------------------------------------------------------
// Embedded web page served from ESP32 flash (no SPIFFS needed).
// Self-contained HTML + CSS + JS for the HeartBeat Sensor UI.
// -----------------------------------------------------------------

const char WEB_PAGE[] PROGMEM = R"===(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HeartBeat Sensor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111827;color:#d1d5db;font-family:'Courier New',monospace;padding:8px}
.hdr{display:flex;justify-content:space-between;align-items:center;padding:8px 12px;background:#1f2937;border-radius:6px;margin-bottom:8px}
.hdr h2{font-size:16px;color:#f87171}
.st{display:flex;align-items:center;gap:6px;font-size:13px}
.dot{width:10px;height:10px;border-radius:50%;background:#ef4444;transition:background .3s}
.dot.on{background:#34d399}
canvas{width:100%;height:280px;background:#0d1b2a;border-radius:6px;display:block}
.stats{display:flex;gap:8px;margin:8px 0}
.sb{background:#1f2937;padding:12px;border-radius:6px;text-align:center;flex:1}
.sv{font-size:28px;font-weight:bold;color:#34d399;transition:transform .15s}
.sv.pulse{transform:scale(1.15);color:#f87171}
.sl{font-size:11px;color:#6b7280;margin-top:4px}
.ctrls{display:flex;flex-wrap:wrap;gap:6px;margin:4px 0}
.cg{background:#1f2937;padding:8px 10px;border-radius:6px;flex:1;min-width:130px}
.cg label{display:block;font-size:11px;color:#818cf8;margin-bottom:4px}
select,input[type=number]{background:#111827;color:#d1d5db;border:1px solid #374151;padding:4px 6px;border-radius:4px;width:70px;font-family:inherit;font-size:13px}
select{width:auto}
button{background:#1e3a5f;color:#d1d5db;border:1px solid #374151;padding:4px 10px;border-radius:4px;cursor:pointer;font-family:inherit;font-size:13px}
button:hover{background:#374151}
button.rec{background:#991b1b;border-color:#f87171;color:#fca5a5}
small{color:#6b7280;font-size:10px}
.row{display:flex;gap:4px;align-items:center;margin-top:4px}
.chk{display:flex;align-items:center;gap:4px;font-size:12px;margin-top:4px}
.log{background:#0d1b2a;border-radius:6px;padding:6px 10px;margin-top:6px;font-size:11px;max-height:80px;overflow-y:auto;color:#6b7280}
</style>
</head><body>

<div class="hdr">
 <h2>&#9829; HeartBeat Sensor</h2>
 <div class="st"><div class="dot" id="dot"></div><span id="stxt">Disconnected</span></div>
</div>

<canvas id="cv"></canvas>

<div class="stats">
 <div class="sb"><div class="sv" id="vBpm">--</div><div class="sl">BPM</div></div>
 <div class="sb"><div class="sv" id="vHrv">--</div><div class="sl">HRV (ms)</div></div>
 <div class="sb"><div class="sv" id="vN">0</div><div class="sl">Samples</div></div>
</div>

<div class="ctrls">
 <div class="cg">
  <label>Signal</label>
  <select id="axis" onchange="onAxisChange()">
   <option value="m" selected>Magnitude</option>
   <option value="sf">Server Filtered (mag)</option>
   <option value="x">Accel X</option>
   <option value="y">Accel Y</option>
   <option value="z">Accel Z</option>
   <option value="all">All Axes</option>
  </select>
  <div class="chk"><input type="checkbox" id="showRaw"><label for="showRaw">Show raw</label></div>
 </div>
 <div class="cg">
  <label>Sample Rate (Hz)</label>
  <div class="row"><input type="number" id="inRate" value="100" min="10" max="500" step="10">
  <button onclick="setRate()">Set</button></div>
 </div>
 <div class="cg">
  <label>Duration (s) <small>0=&infin;</small></label>
  <input type="number" id="inDur" value="0" min="0" max="3600" step="5">
 </div>
 <div class="cg">
  <label>Filters (Hz)</label>
  <div class="row">HP <input type="number" id="inHP" value="0.5" min="0.1" max="5" step="0.1">
  LP <input type="number" id="inLP" value="10" min="1" max="50" step="1">
  <button onclick="setFilters()">Set</button></div>
 </div>
 <div class="cg">
  <label>Graph window (s)</label>
  <input type="number" id="inWin" value="10" min="2" max="60" step="1" onchange="onWinChange()">
 </div>
 <div class="cg">
  <label>Actions</label>
  <div class="row">
   <button id="btnRec" onclick="toggleRec()">&#9654; Start</button>
   <button onclick="saveCSV()">&#128190; CSV</button>
   <button onclick="clearAll()">Clear</button>
  </div>
 </div>
</div>

<div class="log" id="log"></div>

<script>
// ====================== CONFIG ======================
let CFG = {
  graphWin: 10,        // seconds visible in canvas
  maxCSV: 600000,      // max raw points kept for CSV (~10 min @ 1kHz)
  reconnMs: 1500,
};

// ====================== STATE =======================
let ws = null, connected = false, sampling = false;
let sRate = 100;  // current sample rate
let selAxis = 'm';
let showRaw = false;

// Raw data for CSV export: arrays of {t,x,y,z,m}
let csv_t=[], csv_x=[], csv_y=[], csv_z=[], csv_m=[];
let totalN = 0;

// Graph data: per-axis filtered + raw circular buffers
// Each entry: {t (us), rx, ry, rz, rm, fx, fy, fz, fm, sf (server-filtered mag)}
let gBuf = [];

// Peak markers from server: [{t (us), interval_ms}]
let peakMarkers = [];

// client-side filter state (4 axes)
let F = {};
function resetFilters(){
  F={x:{hi:0,ho:0,lo:0},y:{hi:0,ho:0,lo:0},z:{hi:0,ho:0,lo:0},m:{hi:0,ho:0,lo:0}};
}
resetFilters();
let hpA=0.95, lpA=0.3;

function cfgFilters(sr, hpHz, lpHz){
  let dt=1/sr;
  let rh=1/(2*Math.PI*hpHz), rl=1/(2*Math.PI*lpHz);
  hpA=rh/(rh+dt); lpA=dt/(rl+dt);
}

function filt(axis, v){
  let f=F[axis];
  let hp=hpA*(f.ho+v-f.hi); f.hi=v; f.ho=hp;
  let lp=lpA*hp+(1-lpA)*f.lo; f.lo=lp;
  return lp;
}

// ===================== CANVAS =======================
let cvEl, ctx, W, H, dpr;
function initCanvas(){
  cvEl=document.getElementById('cv');
  dpr=window.devicePixelRatio||1;
  sizeCanvas();
  window.addEventListener('resize', sizeCanvas);
}
function sizeCanvas(){
  let r=cvEl.getBoundingClientRect();
  W=r.width; H=r.height;
  cvEl.width=W*dpr; cvEl.height=H*dpr;
  ctx=cvEl.getContext('2d');
  ctx.scale(dpr,dpr);
}

function drawGraph(){
  if(!ctx) return;
  ctx.fillStyle='#0d1b2a'; ctx.fillRect(0,0,W,H);
  let n=gBuf.length;
  if(n<2) return;

  let winUs = CFG.graphWin*1e6;
  let tEnd = gBuf[n-1].t;
  let tStart = tEnd - winUs;

  // find start index via binary search
  let lo=0, hi2=n-1;
  while(lo<hi2){let mid=(lo+hi2)>>1; gBuf[mid].t<tStart?lo=mid+1:hi2=mid;}
  let si=lo;
  if(si>=n-1) return;

  let axes = selAxis==='all' ? ['x','y','z'] : selAxis==='sf' ? ['sf'] : [selAxis];
  let showR = document.getElementById('showRaw').checked;
  let prefix = showR ? 'r' : 'f';

  // auto-scale Y
  let yMin=1e9, yMax=-1e9;
  for(let i=si;i<n;i++){
    for(let a of axes){
      let key = a==='sf' ? 'sf' : prefix+a;
      let v = gBuf[i][key];
      if(v<yMin) yMin=v; if(v>yMax) yMax=v;
    }
    if(showR && selAxis!=='all' && selAxis!=='sf'){
      let vf=gBuf[i]['f'+axes[0]];
      if(vf<yMin) yMin=vf; if(vf>yMax) yMax=vf;
    }
  }
  let pad=(yMax-yMin)*0.12||0.005;
  yMin-=pad; yMax+=pad;

  let mapX=t=>(t-tStart)/winUs*W;
  let mapY=v=>H-(v-yMin)/(yMax-yMin)*H;

  // grid
  ctx.strokeStyle='#1a2744'; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){let y=H*i/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
  ctx.fillStyle='#4b5563'; ctx.font='10px monospace';
  for(let i=0;i<=4;i++){
    let v=yMax-(yMax-yMin)*i/4;
    ctx.fillText(v.toFixed(4),4,H*i/4+12);
  }
  // vertical 1-s marks
  let t0s=Math.ceil(tStart/1e6);
  for(let ts=t0s; ts*1e6<=tEnd; ts++){
    let x=mapX(ts*1e6);
    ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();
    ctx.fillText(ts+'s',x+3,H-4);
  }

  // colours per axis
  let axCol={x:'#f87171',y:'#34d399',z:'#60a5fa',m:'#a78bfa',sf:'#fbbf24'};
  let axColDim={x:'rgba(248,113,113,0.25)',y:'rgba(52,211,153,0.25)',z:'rgba(96,165,250,0.25)',m:'rgba(167,139,250,0.25)',sf:'rgba(251,191,36,0.25)'};

  // If single axis + showRaw: draw raw dim, filtered bright
  if(selAxis!=='all' && selAxis!=='sf' && showR){
    let a=axes[0];
    // raw (dim)
    ctx.strokeStyle=axColDim[a]; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(mapX(gBuf[si].t), mapY(gBuf[si]['r'+a]));
    for(let i=si+1;i<n;i++) ctx.lineTo(mapX(gBuf[i].t), mapY(gBuf[i]['r'+a]));
    ctx.stroke();
    // filtered (bright)
    ctx.strokeStyle=axCol[a]; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.moveTo(mapX(gBuf[si].t), mapY(gBuf[si]['f'+a]));
    for(let i=si+1;i<n;i++) ctx.lineTo(mapX(gBuf[i].t), mapY(gBuf[i]['f'+a]));
    ctx.stroke();
  } else {
    for(let a of axes){
      let key = a==='sf' ? 'sf' : prefix+a;
      ctx.strokeStyle=axCol[a]; ctx.lineWidth= axes.length>1?1:1.5;
      ctx.beginPath();
      ctx.moveTo(mapX(gBuf[si].t), mapY(gBuf[si][key]));
      for(let i=si+1;i<n;i++) ctx.lineTo(mapX(gBuf[i].t), mapY(gBuf[i][key]));
      ctx.stroke();
    }
  }

  // legend
  if(axes.length>1){
    let lx=W-80, ly=14;
    for(let a of axes){
      ctx.fillStyle=axCol[a]; ctx.fillText(a.toUpperCase(),lx,ly); ly+=14;
    }
  }

  // ========== PEAK MARKERS ==========
  // Draw vertical lines + dots + interval labels at server-detected peaks
  let visiblePeaks = peakMarkers.filter(p => p.t >= tStart && p.t <= tEnd);
  for(let pk of visiblePeaks){
    let px = mapX(pk.t);
    // vertical dashed line
    ctx.save();
    ctx.setLineDash([4,4]);
    ctx.strokeStyle='rgba(251,191,36,0.5)'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(px,0); ctx.lineTo(px,H); ctx.stroke();
    ctx.restore();

    // find the filtered value at this peak time (nearest sample)
    let nearIdx = -1, bestDist = Infinity;
    for(let i=si;i<n;i++){
      let d = Math.abs(gBuf[i].t - pk.t);
      if(d < bestDist){bestDist=d; nearIdx=i;}
      if(gBuf[i].t > pk.t + 20000) break; // early exit
    }

    if(nearIdx >= 0){
      // Determine which signal to use for the dot Y position
      let dotVal;
      if(selAxis==='sf'){
        dotVal = gBuf[nearIdx].sf;
      } else {
        let dotAxis = selAxis==='all' ? 'm' : selAxis;
        dotVal = showR ? gBuf[nearIdx]['r'+dotAxis] : gBuf[nearIdx]['f'+dotAxis];
      }
      let py = mapY(dotVal);

      // filled circle at peak
      ctx.beginPath();
      ctx.arc(px, py, 6, 0, 2*Math.PI);
      ctx.fillStyle='rgba(251,191,36,0.85)';
      ctx.fill();
      ctx.strokeStyle='#fbbf24'; ctx.lineWidth=2;
      ctx.stroke();

      // peak diamond marker
      ctx.beginPath();
      ctx.moveTo(px,py-4); ctx.lineTo(px+4,py); ctx.lineTo(px,py+4); ctx.lineTo(px-4,py);
      ctx.closePath(); ctx.fillStyle='#f59e0b'; ctx.fill();
    }

    // interval label between peaks
    if(pk.interval_ms > 0){
      let bpm = Math.round(60000 / pk.interval_ms);
      ctx.fillStyle='#fbbf24'; ctx.font='bold 11px monospace';
      ctx.fillText(Math.round(pk.interval_ms)+'ms', px+4, 14);
      ctx.fillStyle='#f87171';
      ctx.fillText(bpm+' bpm', px+4, 26);
    }
  }
}

// =================== WEBSOCKET ======================
function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81');
  ws.onopen=()=>{connected=true;updSt();logMsg('Connected');};
  ws.onclose=()=>{connected=false;updSt();setTimeout(connect,CFG.reconnMs);};
  ws.onerror=()=>{};
  ws.onmessage=e=>{
    let msg;
    try{msg=JSON.parse(e.data);}catch(_){return;}
    if(msg.type==='data') onData(msg);
    else if(msg.type==='config') onConfig(msg);
  };
}

function send(obj){ if(ws&&ws.readyState===1) ws.send(JSON.stringify(obj)); }

function updSt(){
  document.getElementById('dot').className='dot'+(connected?' on':'');
  document.getElementById('stxt').textContent=connected?'Connected':'Disconnected';
}

// =================== DATA HANDLING ==================
function onData(msg){
  let len=msg.t.length;
  for(let i=0;i<len;i++){
    let t=msg.t[i], x=+msg.x[i], y=+msg.y[i], z=+msg.z[i], m=+msg.m[i];
    let sf = msg.f ? +msg.f[i] : 0;  // server-filtered magnitude
    // store for CSV
    csv_t.push(t); csv_x.push(x); csv_y.push(y); csv_z.push(z); csv_m.push(m);
    totalN++;
    // client-side filter
    let fx=filt('x',x), fy=filt('y',y), fz=filt('z',z), fm=filt('m',m);
    gBuf.push({t,rx:x,ry:y,rz:z,rm:m,fx,fy,fz,fm,sf});
  }
  // Store server-detected peaks
  if(msg.peaks && msg.peaks.length > 0){
    for(let i=0;i<msg.peaks.length;i++){
      peakMarkers.push({t:msg.peaks[i], interval_ms:msg.intervals?+msg.intervals[i]:0});
    }
    logMsg('Peak detected! '+msg.peaks.length+' peaks, interval='+
           (msg.intervals?msg.intervals[msg.intervals.length-1]:'?')+'ms, BPM='+
           (msg.bpm>0?Math.round(msg.bpm):'--'));
    // trim old peaks (keep last 200)
    while(peakMarkers.length > 200) peakMarkers.shift();
  }
  // trim CSV buffer
  while(csv_t.length>CFG.maxCSV){csv_t.shift();csv_x.shift();csv_y.shift();csv_z.shift();csv_m.shift();}
  // trim graph buffer (keep 2x window)
  let maxG = sRate * CFG.graphWin * 2.5;
  while(gBuf.length>maxG) gBuf.shift();

  // update stats
  if(msg.bpm>0) {
    let el=document.getElementById('vBpm');
    el.textContent=Math.round(msg.bpm);
    el.classList.add('pulse'); setTimeout(()=>el.classList.remove('pulse'),150);
  }
  document.getElementById('vHrv').textContent = msg.hrv>0 ? Math.round(msg.hrv) : '--';
  document.getElementById('vN').textContent = totalN;
}

function onConfig(msg){
  sRate = msg.sampleRate||100;
  sampling = msg.sampling||false;
  document.getElementById('inRate').value = sRate;
  updRecBtn();
  cfgFilters(sRate, parseFloat(document.getElementById('inHP').value),
                    parseFloat(document.getElementById('inLP').value));
  logMsg('Config: '+sRate+'Hz, sampling='+sampling);
}

// ==================== CONTROLS ======================
function toggleRec(){
  if(!sampling){
    let dur=parseInt(document.getElementById('inDur').value)||0;
    send({cmd:'set_duration',value:dur});
    send({cmd:'start'});
    sampling=true;
  } else {
    send({cmd:'stop'});
    sampling=false;
  }
  updRecBtn();
}
function updRecBtn(){
  let b=document.getElementById('btnRec');
  if(sampling){b.textContent='\u23F9 Stop';b.classList.add('rec');}
  else{b.textContent='\u25B6 Start';b.classList.remove('rec');}
}
function setRate(){
  let v=parseInt(document.getElementById('inRate').value)||100;
  send({cmd:'set_rate',value:v}); sRate=v;
  cfgFilters(sRate, parseFloat(document.getElementById('inHP').value),
                    parseFloat(document.getElementById('inLP').value));
}
function setFilters(){
  let hp=parseFloat(document.getElementById('inHP').value)||0.5;
  let lp=parseFloat(document.getElementById('inLP').value)||10;
  send({cmd:'set_filters',hp:hp,lp:lp});
  cfgFilters(sRate, hp, lp);
  resetFilters();
  logMsg('Filters updated: HP='+hp+' LP='+lp);
}
function onAxisChange(){ selAxis=document.getElementById('axis').value; }
function onWinChange(){ CFG.graphWin=parseInt(document.getElementById('inWin').value)||10; }
function clearAll(){
  csv_t=[];csv_x=[];csv_y=[];csv_z=[];csv_m=[];gBuf=[];peakMarkers=[];totalN=0;
  resetFilters();
  document.getElementById('vBpm').textContent='--';
  document.getElementById('vHrv').textContent='--';
  document.getElementById('vN').textContent='0';
  logMsg('Data cleared');
}
function saveCSV(){
  if(!csv_t.length){logMsg('No data');return;}
  let lines=['timestamp_us,accel_x,accel_y,accel_z,magnitude'];
  for(let i=0;i<csv_t.length;i++){
    lines.push(csv_t[i]+','+csv_x[i]+','+csv_y[i]+','+csv_z[i]+','+csv_m[i]);
  }
  let blob=new Blob([lines.join('\n')],{type:'text/csv'});
  let a=document.createElement('a');
  a.href=URL.createObjectURL(blob);
  a.download='heartbeat_'+new Date().toISOString().replace(/[:.]/g,'-').slice(0,19)+'.csv';
  a.click(); URL.revokeObjectURL(a.href);
  logMsg('Saved '+csv_t.length+' samples');
}

function logMsg(s){
  let el=document.getElementById('log');
  let ts=new Date().toLocaleTimeString();
  el.innerHTML+='<div>['+ts+'] '+s+'</div>';
  el.scrollTop=el.scrollHeight;
}

// =================== RENDER LOOP ====================
let lastDraw=0;
function frame(now){
  if(now-lastDraw>33){drawGraph();lastDraw=now;}  // ~30 fps
  requestAnimationFrame(frame);
}

// =================== INIT ===========================
window.onload=()=>{
  initCanvas();
  cfgFilters(sRate, 0.5, 10);
  connect();
  requestAnimationFrame(frame);
  logMsg('UI ready — connect to HeartBeat_Sensor WiFi');
};
</script>
</body></html>)===";
