/* ============================================================
   SimSync - Dashboard Core
   WebSocket client, telemetry gauges, safety controls, PID panel
   ============================================================ */
(function () {
  'use strict';

  /* ---- element refs ---- */
  var $ = function (id) { return document.getElementById(id); };
  var els = {
    pillSerial: $('pillSerial'), pillUDP: $('pillUDP'), pillArmed: $('pillArmed'),
    estopBtn: $('estopBtn'), estopOverlay: $('estopOverlay'),
    armBtn: $('armBtn'), disarmBtn: $('disarmBtn'),
    estopClearBtn: $('estopClearBtn'),

    // Gauges
    gSpeed: $('gSpeed'), gGLat: $('gGLat'), gGLong: $('gGLong'),
    gThrottle: $('gThrottle'), gBrake: $('gBrake'), gGear: $('gGear'), gRPM: $('gRPM'),
    throttleBar: $('throttleBar'), brakeBar: $('brakeBar'),
    steerFill: $('steerFill'),

    // Joint cards
    j1Actual: $('j1Actual'), j1Target: $('j1Target'), j1Err: $('j1Err'),
    j2Actual: $('j2Actual'), j2Target: $('j2Target'), j2Err: $('j2Err'),
    j3Actual: $('j3Actual'), j3Target: $('j3Target'), j3Err: $('j3Err'),
    j1DutyFwd: $('j1DutyFwd'), j1DutyRev: $('j1DutyRev'),
    j2DutyFwd: $('j2DutyFwd'), j2DutyRev: $('j2DutyRev'),
    j3DutyFwd: $('j3DutyFwd'), j3DutyRev: $('j3DutyRev'),

    // PID
    kpSlider: $('kpSlider'), kiSlider: $('kiSlider'), kdSlider: $('kdSlider'),
    kpNum: $('kpNum'), kiNum: $('kiNum'), kdNum: $('kdNum'),
    pidSendBtn: $('pidSendBtn'), pidSaveBtn: $('pidSaveBtn'),

    // Step test
    stepJoint: $('stepJoint'), stepSize: $('stepSize'), stepTestBtn: $('stepTestBtn'),
    sweepBtn: $('sweepBtn'),
    kpMin: $('kpMin'), kpMax: $('kpMax'), kdMin: $('kdMin'), kdMax: $('kdMax'),
    sweepKi: $('sweepKi'), sweepSteps: $('sweepSteps'),
    resultsBody: $('resultsBody'), testRunning: $('testRunning'),

    // Status bar
    sbSerial: $('sbSerial'), sbUDP: $('sbUDP'), sbSession: $('sbSession'),
    sbEStop: $('sbEStop'),

    // Export
    exportBtn: $('exportBtn'),

    // Motion config
    pitchGain: $('pitchGain'), rollGain: $('rollGain'), heaveGain: $('heaveGain'),
    filterHz: $('filterHz'),
    pitchGainVal: $('pitchGainVal'), rollGainVal: $('rollGainVal'),
    heaveGainVal: $('heaveGainVal'), filterHzVal: $('filterHzVal'),
    motionSendBtn: $('motionSendBtn'),

    // Direct position
    manJoint: $('manJoint'), manAngle: $('manAngle'), manGoBtn: $('manGoBtn'),
    stepPlus5: $('stepPlus5'), stepMinus5: $('stepMinus5'),
    stepPlus10: $('stepPlus10'), stepMinus10: $('stepMinus10'),
    centerBtn: $('centerBtn'),

    // Scope
    twBtns: document.querySelectorAll('.tw-btn'),
    jointTabs: document.querySelectorAll('.joint-tab'),
    traceBtns: document.querySelectorAll('[data-trace]'),
  };

  /* ---- state ---- */
  var ws;
  var latestFrame = null;
  var currentJoint = 1;
  var sampleCount = 0;
  var estopActive = false;

  /* ---- WebSocket ---- */
  function connect() {
    var proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(proto + '://' + location.host + '/ws');
    ws.onmessage = function (e) {
      try { onFrame(JSON.parse(e.data)); } catch (_) {}
    };
    ws.onclose = function () {
      setTimeout(connect, 1500);
    };
  }

  function onFrame(f) {
    latestFrame = f;
    sampleCount++;

    updatePills(f);
    updateGauges(f);
    updateJoints(f);
    updateStatusBar(f);
    updateEStop(f);

    // Push to oscilloscope
    if (f.joints && window.Scope) {
      f.joints.forEach(function (j) {
        if (j && j.j) {
          Scope.push(j.j, j.target, j.actual, j.error, j.duty);
        }
      });
    }
  }

  /* ---- pills ---- */
  function updatePills(f) {
    setPill(els.pillSerial, f.serial_ok, 'SERIAL', f.serial_ok ? 'OK' : 'DISC');
    setPill(els.pillUDP, f.udp_hz > 5, 'UDP', f.udp_hz > 5 ? f.udp_hz.toFixed(0) + 'Hz' : 'OFF');
    setPill(els.pillArmed, f.armed, f.armed ? 'OUTPUT ENABLED' : 'PAUSED', null, f.armed ? 'ok' : 'warn');
  }

  function setPill(el, active, label, suffix, forceClass) {
    if (!el) return;
    var text = label;
    if (suffix) text += ' ' + suffix;
    el.querySelector('.pill-text').textContent = text;
    el.className = 'pill ' + (forceClass || (active ? 'ok' : 'off'));
  }

  /* ---- gauges ---- */
  function updateGauges(f) {
    var t = f.telemetry || {};
    setText(els.gSpeed, (t.speed_kmh || 0).toFixed(0));
    setText(els.gGLat,  (t.g_lat  || 0).toFixed(2));
    setText(els.gGLong, (t.g_long || 0).toFixed(2));
    setText(els.gGear,  t.gear || 0);
    setText(els.gRPM,   ((t.rpm || 0) / 1000).toFixed(1) + 'k');

    var thr = (t.throttle || 0) * 100;
    var brk = (t.brake || 0) * 100;
    setText(els.gThrottle, thr.toFixed(0) + '%');
    setText(els.gBrake,    brk.toFixed(0) + '%');
    if (els.throttleBar) els.throttleBar.style.width = thr + '%';
    if (els.brakeBar)    els.brakeBar.style.width    = brk + '%';

    // Steer bar: -1..1 → left 50% = center
    if (els.steerFill) {
      var st = t.steer || 0;
      if (st >= 0) {
        els.steerFill.style.left  = '50%';
        els.steerFill.style.right = 'auto';
        els.steerFill.style.width = (st * 50) + '%';
      } else {
        els.steerFill.style.right = '50%';
        els.steerFill.style.left  = 'auto';
        els.steerFill.style.width = (-st * 50) + '%';
      }
    }
  }

  /* ---- joint cards ---- */
  var jEls = [
    { actual: els.j1Actual, target: els.j1Target, err: els.j1Err, fwd: els.j1DutyFwd, rev: els.j1DutyRev },
    { actual: els.j2Actual, target: els.j2Target, err: els.j2Err, fwd: els.j2DutyFwd, rev: els.j2DutyRev },
    { actual: els.j3Actual, target: els.j3Target, err: els.j3Err, fwd: els.j3DutyFwd, rev: els.j3DutyRev },
  ];

  function updateJoints(f) {
    if (!f.joints) return;
    f.joints.forEach(function (j, i) {
      if (!j) return;
      var e = jEls[i];
      if (!e) return;
      setText(e.actual, j.actual !== undefined ? j.actual.toFixed(1) + ' counts' : '--');
      setText(e.target, 'TGT ' + (j.target !== undefined ? j.target.toFixed(1) : '--') + ' counts');
      setText(e.err,    'ERR ' + (j.error !== undefined ? j.error.toFixed(2) : '--') + ' counts');
      setDutyBar(e.fwd, e.rev, j.duty || 0);
    });
  }

  function setDutyBar(fwd, rev, duty) {
    var MAX = 255;
    var pct = Math.min(Math.abs(duty) / MAX * 50, 50) + '%';
    if (duty >= 0) {
      if (fwd) { fwd.style.width = pct; fwd.style.display = 'block'; }
      if (rev) { rev.style.width = '0'; }
    } else {
      if (rev) { rev.style.width = pct; rev.style.display = 'block'; }
      if (fwd) { fwd.style.width = '0'; }
    }
  }

  /* ---- e-stop ---- */
  function updateEStop(f) {
    estopActive = f.estop;
    if (els.estopOverlay) {
      els.estopOverlay.classList.toggle('active', estopActive);
    }
    if (els.estopBtn) {
      els.estopBtn.classList.toggle('active', estopActive);
      els.estopBtn.textContent = estopActive ? 'CLEAR PAUSE' : 'LATCH PAUSE';
    }
    if (els.sbEStop) {
      els.sbEStop.textContent = estopActive
        ? 'HOST PAUSE: ' + (f.estop_reason || 'TRIPPED')
        : (f.armed ? 'Host output enabled' : 'Host motion paused');
      els.sbEStop.style.color = estopActive ? 'var(--red)' : 'var(--text-dim)';
    }
  }

  /* ---- status bar ---- */
  function updateStatusBar(f) {
    setText(els.sbSerial, f.serial_ok ? 'USB OK' : 'USB DISC');
    setText(els.sbUDP,    f.udp_hz > 5 ? 'UDP ' + f.udp_hz.toFixed(0) + 'Hz' : 'UDP --');
    setText(els.sbSession, 'SAMPLES ' + sampleCount);
  }

  /* ---- PID panel ---- */
  function syncPIDFromFrame(f) {
    if (!f || !f.pid) return;
    setSliderAndNum(els.kpSlider, els.kpNum, f.pid.kp);
    setSliderAndNum(els.kiSlider, els.kiNum, f.pid.ki);
    setSliderAndNum(els.kdSlider, els.kdNum, f.pid.kd);
  }

  function setSliderAndNum(slider, num, val) {
    if (slider) slider.value = val;
    if (num)    num.value = parseFloat(val).toFixed(2);
  }

  function setupPIDSync(slider, num) {
    if (slider) slider.addEventListener('input', function () {
      if (num) num.value = parseFloat(slider.value).toFixed(2);
    });
    if (num) num.addEventListener('change', function () {
      if (slider) slider.value = num.value;
    });
  }
  setupPIDSync(els.kpSlider, els.kpNum);
  setupPIDSync(els.kiSlider, els.kiNum);
  setupPIDSync(els.kdSlider, els.kdNum);

  if (els.pidSendBtn) els.pidSendBtn.addEventListener('click', function () {
    apiPost('/api/pid', null, {
      kp: parseFloat(els.kpNum.value),
      ki: parseFloat(els.kiNum.value),
      kd: parseFloat(els.kdNum.value),
    });
  });

  if (els.pidSaveBtn) els.pidSaveBtn.addEventListener('click', function () {
    apiGet('/api/pid/save');
  });

  /* ---- Safety controls ---- */
  if (els.estopBtn) els.estopBtn.addEventListener('click', function () {
    if (estopActive) {
      apiGet('/api/estop/clear');
    } else {
      apiGet('/api/estop?state=1');
    }
  });

  if (els.armBtn) els.armBtn.addEventListener('click', function () {
    apiGet('/api/arm?state=1');
  });
  if (els.disarmBtn) els.disarmBtn.addEventListener('click', function () {
    apiGet('/api/arm?state=0');
  });
  if (els.estopClearBtn) els.estopClearBtn.addEventListener('click', function () {
    apiGet('/api/estop/clear');
  });

  /* ---- Direct position commands ---- */
  if (els.manGoBtn) els.manGoBtn.addEventListener('click', function () {
    var j = parseInt(els.manJoint.value, 10) || 1;
    var a = parseFloat(els.manAngle.value);
    apiGet('/api/target?joint=' + j + '&angle=' + a.toFixed(1));
  });

  function jogJoint(delta) {
    var j = latestFrame && latestFrame.joints ? latestFrame.joints[currentJoint - 1] : null;
    var base = j && Number.isFinite(j.target) ? j.target : 512;
    apiGet('/api/target?joint=' + currentJoint + '&angle=' + (base + delta).toFixed(1));
  }

  if (els.stepPlus5)  els.stepPlus5.addEventListener('click',  function () { jogJoint(+5); });
  if (els.stepMinus5) els.stepMinus5.addEventListener('click', function () { jogJoint(-5); });
  if (els.stepPlus10) els.stepPlus10.addEventListener('click', function () { jogJoint(+10); });
  if (els.stepMinus10) els.stepMinus10.addEventListener('click', function () { jogJoint(-10); });
  if (els.centerBtn)  els.centerBtn.addEventListener('click', function () {
    [1, 2, 3].forEach(function (j) { apiGet('/api/target?joint=' + j + '&angle=512'); });
  });

  /* ---- Step Test ---- */
  if (els.stepTestBtn) els.stepTestBtn.addEventListener('click', function () {
    var j    = parseInt(els.stepJoint.value, 10) || 1;
    var step = parseFloat(els.stepSize.value) || 15;
    var kp   = parseFloat(els.kpNum.value);
    var ki   = parseFloat(els.kiNum.value);
    var kd   = parseFloat(els.kdNum.value);
    apiGet('/api/steptest?joint=' + j + '&kp=' + kp + '&ki=' + ki + '&kd=' + kd + '&step=' + step);
    if (els.testRunning) els.testRunning.style.display = 'inline-block';
  });

  if (els.sweepBtn) els.sweepBtn.addEventListener('click', function () {
    var j = parseInt(els.stepJoint.value, 10) || 1;
    var params = [
      'joint=' + j,
      'kp_min=' + (els.kpMin.value || 4),
      'kp_max=' + (els.kpMax.value || 14),
      'kd_min=' + (els.kdMin.value || 0.5),
      'kd_max=' + (els.kdMax.value || 3.0),
      'ki=' + (els.sweepKi.value || 0.2),
      'steps=' + (els.sweepSteps.value || 4),
      'step=' + (els.stepSize.value || 15),
    ].join('&');
    apiGet('/api/sweep?' + params);
    if (els.testRunning) els.testRunning.style.display = 'inline-block';
  });

  /* Poll step results every 2s during tests */
  setInterval(function () {
    fetch('/api/steptest/results')
      .then(function (r) { return r.json(); })
      .then(renderResults)
      .catch(function () {});
  }, 2000);

  function renderResults(results) {
    if (!els.resultsBody || !results || !results.length) return;
    if (els.testRunning) {
      els.testRunning.style.display = results.some(function (r) { return !r.Success; }) ? 'inline-block' : 'none';
    }

    var bestIdx = 0;
    var bestScore = Infinity;
    results.forEach(function (r, i) {
      var score = r.OvershootPct * 2 + r.SettleTimeMs / 100 + r.SteadyStateErr * 10;
      if (score < bestScore) { bestScore = score; bestIdx = i; }
    });

    var rows = results.slice(0, 20).map(function (r, i) {
      var cls = i === bestIdx ? 'best' : '';
      return '<tr class="' + cls + '">' +
        '<td>' + r.Kp.toFixed(2) + '</td>' +
        '<td>' + r.Kd.toFixed(2) + '</td>' +
        '<td>' + (r.RiseTimeMs || 0).toFixed(0) + 'ms</td>' +
        '<td>' + (r.OvershootPct || 0).toFixed(1) + '%</td>' +
        '<td>' + (r.SettleTimeMs || 0).toFixed(0) + 'ms</td>' +
        '<td>' + (r.SteadyStateErr || 0).toFixed(2) + ' counts</td>' +
        '</tr>';
    });
    els.resultsBody.innerHTML = rows.join('');
  }

  /* ---- Motion config sliders ---- */
  function setupMCFG(slider, valEl) {
    if (!slider || !valEl) return;
    slider.addEventListener('input', function () {
      valEl.textContent = parseFloat(slider.value).toFixed(1);
    });
  }
  setupMCFG(els.pitchGain, els.pitchGainVal);
  setupMCFG(els.rollGain, els.rollGainVal);
  setupMCFG(els.heaveGain, els.heaveGainVal);
  setupMCFG(els.filterHz, els.filterHzVal);

  function syncMotionFromFrame(f) {
    if (!f || !f.motion) return;
    var m = f.motion;
    if (els.pitchGain) { els.pitchGain.value = m.pitch_gain; els.pitchGainVal.textContent = m.pitch_gain.toFixed(1); }
    if (els.rollGain)  { els.rollGain.value  = m.roll_gain;  els.rollGainVal.textContent  = m.roll_gain.toFixed(1);  }
    if (els.heaveGain) { els.heaveGain.value = m.heave_gain; els.heaveGainVal.textContent = m.heave_gain.toFixed(1); }
    if (els.filterHz)  { els.filterHz.value  = m.filter_hz;  els.filterHzVal.textContent  = m.filter_hz.toFixed(1);  }
  }

  if (els.motionSendBtn) els.motionSendBtn.addEventListener('click', function () {
    var cfg = {
      PitchGain: parseFloat(els.pitchGain.value),
      RollGain:  parseFloat(els.rollGain.value),
      HeaveGain: parseFloat(els.heaveGain.value),
      FilterHz:  parseFloat(els.filterHz.value),
    };
    apiPost('/api/motion/config', null, cfg);
  });

  /* ---- Oscilloscope controls ---- */
  els.jointTabs.forEach(function (tab) {
    tab.addEventListener('click', function () {
      currentJoint = parseInt(tab.dataset.joint, 10);
      if (window.Scope) Scope.setJoint(currentJoint);
      els.jointTabs.forEach(function (t) { t.classList.toggle('active', t === tab); });
    });
  });

  els.twBtns.forEach(function (btn) {
    btn.addEventListener('click', function () {
      if (window.Scope) Scope.setWindow(parseInt(btn.dataset.s, 10));
      els.twBtns.forEach(function (b) { b.classList.toggle('active', b === btn); });
    });
  });

  els.traceBtns.forEach(function (btn) {
    btn.addEventListener('click', function () {
      var trace = btn.dataset.trace;
      if (window.Scope) Scope.toggleTrace(trace);
      btn.classList.toggle('active');
    });
  });

  /* ---- Export ---- */
  if (els.exportBtn) els.exportBtn.addEventListener('click', function () {
    window.open('/api/session/export', '_blank');
  });

  /* ---- Sync once on first frame ---- */
  var synced = false;
  var origOnFrame = onFrame;
  onFrame = function (f) {
    origOnFrame(f);
    if (!synced && f.serial_ok) {
      synced = true;
      syncPIDFromFrame(f);
      syncMotionFromFrame(f);
    }
  };

  /* ---- helpers ---- */
  function setText(el, val) {
    if (el) el.textContent = val;
  }

  function checkResponse(response) {
    return response.json().then(function (body) {
      if (!response.ok) throw new Error(body.error || 'Command failed');
      return body;
    });
  }
  function showError(error) {
    if (els.sbEStop) { els.sbEStop.textContent = error.message; els.sbEStop.style.color = 'var(--red)'; }
    window.alert(error.message);
  }
  function apiGet(path) {
    fetch(path).then(checkResponse).catch(showError);
  }

  function apiPost(path, query, body) {
    var url = path + (query ? '?' + query : '');
    fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then(checkResponse).catch(showError);
  }

  connect();
})();
