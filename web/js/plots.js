/* ============================================================
   SimSync - Live Oscilloscope
   Canvas-based, 60 FPS, multi-trace: target / actual / error / duty
   ============================================================ */
(function () {
  'use strict';

  var canvas = document.getElementById('scopeCanvas');
  var ctx = canvas.getContext('2d');

  var WINDOW_S = 5;   // seconds visible
  var MAX_POINTS = 6000;
  var activeJoint = 1;
  var limits=null, scale="auto", frozenAt=null, frozenBuffers=null;

  // Per-joint sample buffers
  var buffers = { 1: [], 2: [], 3: [] };

  var tokens=getComputedStyle(document.documentElement);
  function token(name){return tokens.getPropertyValue(name).trim();}
  var colors={target:token('--trace-target'),actual:token('--trace-actual'),error:token('--trace-error'),duty:token('--trace-pwm')};

  var showTraces = { target: true, actual: true, error: false, duty: false, limits:false, clip:false, host:false };

  /* Public API */
  window.Scope = {
    push: function (joint, target, actual, error, duty) {
      var buf = buffers[joint];
      buf.push({ t: Date.now(), target: target, actual: actual, error: error, duty: duty });
      if (buf.length > MAX_POINTS) buf.shift();
    },
    setJoint: function (j) { activeJoint = j; limits=null; },
    resize: resize,
    setLimits: function (s) { limits=s; },
    setScale: function (s) { scale=s; },
    freeze: function () {
      frozenAt=frozenAt===null?Date.now():null;
      frozenBuffers=frozenAt===null?null:{1:buffers[1].slice(),2:buffers[2].slice(),3:buffers[3].slice()};
      return frozenAt!==null;
    },
    saveImage: function () {
      var link=document.createElement('a'); link.download='simsync-m'+activeJoint+'-scope.png'; link.href=canvas.toDataURL('image/png'); link.click();
    },
    metrics: function (seconds) {
      var points=(frozenBuffers || buffers)[activeJoint].filter(function (s) { return s.t>=(frozenAt===null?Date.now():frozenAt)-seconds*1000 && (frozenAt===null || s.t<=frozenAt); });
      if(!points.length) return null;
      var sum=0,peak=0,pwm=0; points.forEach(function (s) { sum+=s.error*s.error; peak=Math.max(peak,Math.abs(s.error)); pwm=Math.max(pwm,Math.abs(s.duty)); });
      return {rms:Math.sqrt(sum/points.length),peak:peak,pwm:pwm};
    },
    setWindow: function (s) { WINDOW_S = s; },
    toggleTrace: function (name) { showTraces[name] = !showTraces[name]; },
    setTrace: function (name, v) { showTraces[name] = v; },
  };

  function resize() {
    var rect = canvas.getBoundingClientRect();
    var ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * ratio);
    canvas.height = Math.round(rect.height * ratio);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  }

  var lastDraw = 0;
  function draw(stamp) {
    requestAnimationFrame(draw);
    if (document.hidden || stamp-lastDraw < 33) return;
    lastDraw=stamp || 0;
    var W = canvas.clientWidth, H = canvas.clientHeight;
    if (W === 0) { resize(); return; }

    ctx.clearRect(0, 0, W, H);

    var plotText=token('--text-2');
    var gridColor=token('--border');
    // Background
    ctx.fillStyle = token('--bg');
    ctx.fillRect(0, 0, W, H);

    // Grid lines
    var gridLines = 6;
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 1;
    for (var i = 1; i < gridLines; i++) {
      var y = Math.round(H * i / gridLines) + 0.5;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }
    var timeGrids = 5;
    for (var i = 1; i < timeGrids; i++) {
      var x = Math.round(W * i / timeGrids) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }

    var now = frozenAt===null?Date.now():frozenAt;
    var buf = (frozenBuffers || buffers)[activeJoint];


    var windowMs = WINDOW_S * 1000;
    var cutoff = now - windowMs;
    var visible = buf.filter(function (s) { return s.t >= cutoff && s.t <= now; }).map(function (s) { return {t:s.t, target:Angles.value(activeJoint,s.target), actual:Angles.value(activeJoint,s.actual), error:Angles.value(activeJoint,s.error,true), duty:s.duty}; });


    // Determine Y range from active traces
    var allVals = [];
    visible.forEach(function (s) {
      if (showTraces.target) allVals.push(s.target);
      if (showTraces.actual) allVals.push(s.actual);
      if (showTraces.error)  allVals.push(s.error, -s.error);
    });
    var boundaries=[];
    if(limits) {
      if(showTraces.limits) boundaries.push({name:'CUTOFF',lo:limits.cutoff,hi:1023-limits.cutoff,color:token('--danger')});
      if(showTraces.clip) boundaries.push({name:'CLIP',lo:limits.clip,hi:1023-limits.clip,color:colors.error});
      if(showTraces.host && limits.configured) boundaries.push({name:'HOST',lo:limits.min,hi:limits.max,color:colors.target});
    }
    boundaries.forEach(function (b) { b.lo=Angles.value(activeJoint,b.lo); b.hi=Angles.value(activeJoint,b.hi); allVals.push(b.lo,b.hi); });
    if(scale==='full') allVals.push(Angles.value(activeJoint,0),Angles.value(activeJoint,1023));
    if(allVals.length===0) allVals=[Angles.value(activeJoint,0),Angles.value(activeJoint,1023)];

    var yMin = Math.min.apply(null, allVals);
    var yMax = Math.max.apply(null, allVals);
    var yPad = Math.max(1, (yMax - yMin) * 0.1);
    yMin -= yPad; yMax += yPad;
    if (yMax === yMin) { yMax += 1; yMin -= 1; }

    function xOf(sample) {
      return ((sample.t - cutoff) / windowMs) * W;
    }
    function yOf(val) {
      return H - ((val - yMin) / (yMax - yMin)) * H;
    }

    var boundaryLabels=[];
    boundaries.forEach(function (b) {
      [b.lo,b.hi].forEach(function (value) {
        var y=yOf(value); ctx.strokeStyle=b.color; ctx.lineWidth=1; ctx.setLineDash([6,4]);
        ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); ctx.setLineDash([]);
        boundaryLabels.push({y:Math.max(12,y-5),text:b.name+' '+value.toFixed(1)+Angles.unit(activeJoint),color:b.color});
      });
    });
    boundaryLabels.sort(function (a,b) {return a.y-b.y;});
    var labelY=-2;
    boundaryLabels.forEach(function (label) {
      labelY=Math.max(label.y,labelY+14); ctx.font='12px monospace'; ctx.fillStyle=label.color; ctx.textAlign='right';
      ctx.fillText(label.text, W-85,Math.min(H-8,labelY));
    });
    ctx.textAlign='left';
    if(!visible.length) { ctx.fillStyle=plotText; ctx.font='12px monospace'; ctx.textAlign='center'; ctx.fillText('No samples',W/2,H/2); ctx.textAlign='left'; }

    function drawTrace(key, colorHex, getValue, lineW) {
      if (!showTraces[key]) return;
      ctx.beginPath();
      ctx.strokeStyle = colorHex;
      ctx.lineWidth = lineW || 1.5;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      var first = true;
      visible.forEach(function (s) {
        var px = xOf(s), py = yOf(getValue(s));
        if (first) { ctx.moveTo(px, py); first = false; }
        else        { ctx.lineTo(px, py); }
      });
      ctx.stroke();
    }

    // Duty as shaded area (green, secondary)
    if (showTraces.duty && visible.length) {
      var dutyMax = 255;
      ctx.fillStyle = 'rgba(57,255,107,0.12)';
      ctx.beginPath();
      var first = true;
      visible.forEach(function (s) {
        var px = xOf(s);
        var py = H - (Math.abs(s.duty) / dutyMax) * H;
        if (first) { ctx.moveTo(px, H); ctx.lineTo(px, py); first = false; }
        else        { ctx.lineTo(px, py); }
      });
      ctx.lineTo(xOf(visible[visible.length - 1]), H);
      ctx.closePath();
      ctx.fill();
    }

    drawTrace('error',  colors.error,  function (s) { return s.error; },  1.2);
    drawTrace('target', colors.target, function (s) { return s.target; }, 1.5);
    drawTrace('actual', colors.actual, function (s) { return s.actual; }, 2);

    // Zero line for error
    if (showTraces.error) {
      var zy = yOf(0);
      ctx.strokeStyle = 'rgba(255,184,48,0.15)';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(0, zy); ctx.lineTo(W, zy); ctx.stroke();
      ctx.setLineDash([]);
    }

    // Numeric axes use the same per-motor angle calibration as the traces.
    ctx.font = '12px monospace';
    ctx.fillStyle = plotText;
    ctx.textAlign = 'right';
    for (var tick=1; tick<gridLines; tick++) {
      var value=yMax-(yMax-yMin)*tick/gridLines;
      ctx.fillText(value.toFixed(1)+Angles.unit(activeJoint),W-8,H*tick/gridLines-5);
    }
    ctx.textAlign = 'left';
    for (var tick=1; tick<timeGrids; tick++) {
      ctx.fillText('-'+(WINDOW_S*(1-tick/timeGrids)).toFixed(0)+'s',W*tick/timeGrids+4,H-6);
    }

    // Cursor readout
    var last = visible[visible.length - 1];
    if (last) {
      var labels = [];
      if (showTraces.actual) labels.push({ label: 'ACT', val: last.actual.toFixed(1) + Angles.unit(activeJoint), color: colors.actual });
      if (showTraces.target) labels.push({ label: 'TGT', val: last.target.toFixed(1) + Angles.unit(activeJoint), color: colors.target });
      if (showTraces.duty) labels.push({label:'PWM',val:Math.abs(last.duty)+'/255',color:colors.duty});
      if (frozenAt!==null) labels.push({label:'DISPLAY',val:'FROZEN',color:colors.error});
      if (showTraces.error)  labels.push({ label: 'ERR', val: last.error.toFixed(2) + Angles.unit(activeJoint),  color: colors.error });
      ctx.font = '12px JetBrains Mono, monospace';
      var ry = 14;
      labels.forEach(function (l) {
        ctx.fillStyle = l.color;
        ctx.fillText(l.label + ' ' + l.val, 8, ry);
        ry += 14;
      });
    }
  }

  if (window.ResizeObserver) new ResizeObserver(resize).observe(canvas);
  window.addEventListener('resize', resize);
  resize();
  draw();
})();
