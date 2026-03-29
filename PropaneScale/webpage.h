#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <pgmspace.h>

// Stored in flash (PROGMEM) to avoid consuming ESP8266's limited RAM.
// Served via server.send_P(200, "text/html", WEBPAGE).

static const char WEBPAGE[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Propane Scale</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;
       min-height:100vh;display:flex;flex-direction:column;
       align-items:center;padding:16px 12px}
  h1{color:#ff6b35;font-size:1.8em;margin-bottom:16px;letter-spacing:1px}
  .card{background:#16213e;border-radius:12px;padding:20px;margin:8px 0;
        width:100%;max-width:420px;box-shadow:0 4px 24px rgba(0,0,0,.45)}
  /* ── Weight display ── */
  .weight-wrap{text-align:center;margin-bottom:8px}
  .weight-val{font-size:3.4em;font-weight:700;color:#ff6b35;line-height:1}
  .weight-unit{font-size:.45em;color:#aaa;margin-left:4px}
  /* ── Gauge bar ── */
  .gauge-legend{display:flex;justify-content:space-between;
                font-size:.8em;color:#888;margin-bottom:4px}
  .gauge-track{width:100%;height:28px;background:#0f3460;
               border-radius:14px;overflow:hidden}
  .gauge-fill{height:100%;border-radius:14px;width:0%;
              transition:width .6s ease,background-color .6s ease}
  /* ── Info grid ── */
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:14px}
  .cell{background:#0f3460;border-radius:8px;padding:12px;text-align:center}
  .cell-val{font-size:1.5em;font-weight:700;color:#4fc3f7}
  .cell-lbl{font-size:.75em;color:#888;margin-top:3px}
  /* ── Status line ── */
  .status{font-size:.8em;text-align:center;margin-top:10px;color:#888}
  .status.ok{color:#4caf50}.status.err{color:#f44336}
  /* ── Buttons ── */
  .btn{display:block;width:100%;padding:12px;border:none;border-radius:8px;
       font-size:1em;font-weight:700;cursor:pointer;margin:6px 0;
       transition:opacity .15s}
  .btn:hover{opacity:.82}
  .btn-tare{background:#ff6b35;color:#fff}
  .btn-cal {background:#4fc3f7;color:#1a1a2e}
  /* ── Calibration form ── */
  #calPanel{display:none;margin-top:6px}
  #calPanel input{display:block;width:100%;padding:10px;border:none;
    border-radius:8px;background:#0f3460;color:#eee;font-size:1em;
    text-align:center;margin-bottom:8px}
  #calPanel input::placeholder{color:#666}
  .guide-box{margin-top:10px;padding:10px;border-radius:8px;background:#0f3460}
  .guide-hint{font-size:.82em;color:#9db3c7;margin-bottom:8px;line-height:1.35}
  .guide-row{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .btn-guide{background:#ffd166;color:#1a1a2e}
  .btn-stop{background:#ef476f;color:#fff}
  .msg{font-size:.85em;text-align:center;min-height:1.2em;margin-top:4px}
  .msg.ok{color:#4caf50}.msg.err{color:#f44336}
</style>
</head>
<body>
<h1>&#x1F525; Propane Scale</h1>

<!-- Weight card -->
<div class="card">
  <div class="weight-wrap">
    <span class="weight-val" id="wVal">--</span><span class="weight-unit">lbs</span>
  </div>
  <div class="gauge-legend">
    <span>Empty</span>
    <span id="pctLbl">--%</span>
    <span>Full</span>
  </div>
  <div class="gauge-track">
    <div class="gauge-fill" id="gFill"></div>
  </div>
  <div class="grid">
    <div class="cell">
      <div class="cell-val" id="propLbs">--</div>
      <div class="cell-lbl">Propane (lbs)</div>
    </div>
    <div class="cell">
      <div class="cell-val" id="propPct">--%</div>
      <div class="cell-lbl">Tank Level</div>
    </div>
  </div>
  <div class="status" id="lastUpdate">Connecting&hellip;</div>
</div>

<!-- Controls card -->
<div class="card">
  <button class="btn btn-tare" onclick="doTare()">TARE &mdash; Zero Scale</button>
  <button class="btn btn-cal"  onclick="toggleCal()">CALIBRATE</button>
  <div id="calPanel">
    <input id="knownWt" type="number" step="0.1" min="0.01"
           placeholder="Known weight on scale (lbs)">
    <button class="btn btn-cal" onclick="doCalibrate()">Apply Calibration</button>
    <div class="guide-box">
      <div class="guide-hint" id="guidedHint">Guided calibration runs in two steps: empty platform, then known weight.</div>
      <div class="guide-row">
        <button class="btn btn-guide" onclick="guidedStart()">Start Guided</button>
        <button class="btn btn-guide" onclick="guidedNext()">Continue</button>
      </div>
      <button class="btn btn-stop" onclick="guidedCancel()">Cancel Guided</button>
    </div>
  </div>
  <div class="msg" id="ctrlMsg"></div>
</div>

<script>
// ── Helpers ──────────────────────────────────────────────────────────────
function levelColor(pct) {
  if (pct > 50) return '#4caf50';   // green
  if (pct > 20) return '#ff9800';   // orange
  return '#f44336';                 // red
}

function setText(id, val) { document.getElementById(id).textContent = val; }

function showMsg(id, text, isErr) {
  var el = document.getElementById(id);
  el.textContent = text;
  el.className = 'msg ' + (isErr ? 'err' : 'ok');
}

// ── Data polling ─────────────────────────────────────────────────────────
function updateUI(d) {
  setText('wVal', d.weight_lbs.toFixed(2));
  var pct = Math.min(100, Math.max(0, d.propane_pct));
  setText('pctLbl', pct.toFixed(1) + '%');
  var fill = document.getElementById('gFill');
  fill.style.width = pct + '%';
  fill.style.backgroundColor = levelColor(pct);
  setText('propLbs', d.propane_lbs.toFixed(2));
  setText('propPct', pct.toFixed(1) + '%');
  var el = document.getElementById('lastUpdate');
  el.textContent = 'Updated ' + new Date().toLocaleTimeString();
  el.className = 'status ok';
}

function fetchData() {
  fetch('/api/data')
    .then(function(r){ return r.json(); })
    .then(function(d){ updateUI(d); })
    .catch(function(){
      var el = document.getElementById('lastUpdate');
      el.textContent = 'Connection error';
      el.className = 'status err';
    });
}

// ── Tare ─────────────────────────────────────────────────────────────────
function doTare() {
  showMsg('ctrlMsg', 'Taring\u2026', false);
  fetch('/api/tare', { method: 'POST' })
    .then(function(r){ return r.json(); })
    .then(function(d){
      showMsg('ctrlMsg', d.message || 'Tare complete', !d.success);
    })
    .catch(function(){ showMsg('ctrlMsg', 'Tare request failed', true); });
}

// ── Calibration ─────────────────────────────────────────────────────────
function toggleCal() {
  var p = document.getElementById('calPanel');
  p.style.display = p.style.display === 'block' ? 'none' : 'block';
}

function doCalibrate() {
  var w = parseFloat(document.getElementById('knownWt').value);
  if (!w || w <= 0) { showMsg('ctrlMsg', 'Enter a valid weight', true); return; }
  showMsg('ctrlMsg', 'Calibrating\u2026', false);
  fetch('/api/calibrate?weight=' + w, { method: 'POST' })
    .then(function(r){ return r.json(); })
    .then(function(d){
      showMsg('ctrlMsg', d.message || 'Calibration saved', !d.success);
      if (d.success) {
        document.getElementById('calPanel').style.display = 'none';
        document.getElementById('knownWt').value = '';
      }
    })
    .catch(function(){ showMsg('ctrlMsg', 'Calibration request failed', true); });
}

function setGuidedHint(txt) {
  document.getElementById('guidedHint').textContent = txt;
}

function guidedStart() {
  var w = parseFloat(document.getElementById('knownWt').value);
  if (!w || w <= 0) { showMsg('ctrlMsg', 'Enter a valid weight', true); return; }
  showMsg('ctrlMsg', 'Starting guided calibration...', false);
  fetch('/api/guidedcal/start?weight=' + w, { method: 'POST' })
    .then(function(r){ return r.json(); })
    .then(function(d){
      showMsg('ctrlMsg', d.message || 'Guided calibration started', !d.success);
      if (d.success) setGuidedHint('Step ' + d.step + ': ' + (d.prompt || 'Continue guided calibration'));
    })
    .catch(function(){ showMsg('ctrlMsg', 'Guided start failed', true); });
}

function guidedNext() {
  showMsg('ctrlMsg', 'Running guided step...', false);
  fetch('/api/guidedcal/next', { method: 'POST' })
    .then(function(r){ return r.json(); })
    .then(function(d){
      showMsg('ctrlMsg', d.message || (d.success ? 'Step complete' : 'Step failed'), !d.success);
      if (!d.success) return;
      if (d.active) {
        setGuidedHint('Step ' + d.step + ': ' + (d.prompt || 'Continue guided calibration'));
      } else {
        setGuidedHint('Guided calibration completed. Verification: ' + (d.verify_lbs || 0).toFixed(3) + ' lbs');
      }
    })
    .catch(function(){ showMsg('ctrlMsg', 'Guided step failed', true); });
}

function guidedCancel() {
  fetch('/api/guidedcal/cancel', { method: 'POST' })
    .then(function(r){ return r.json(); })
    .then(function(d){
      showMsg('ctrlMsg', d.message || 'Guided calibration cancelled', !d.success);
      setGuidedHint('Guided calibration runs in two steps: empty platform, then known weight.');
    })
    .catch(function(){ showMsg('ctrlMsg', 'Cancel request failed', true); });
}

// ── Start ────────────────────────────────────────────────────────────────
fetchData();
setInterval(fetchData, 2000);
</script>
</body>
</html>
)HTMLEOF";

#endif // WEBPAGE_H
