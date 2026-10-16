/* Measured motor tuning with shared controller command gates. */
(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };
  var tabs = Array.from(document.querySelectorAll('[data-view]'));
  [['pidPanel','tuningPID'],['limitsPanel','tuningLimits'],['stepPanel','tuningStep'],['testResultsPanel','tuningResults'],['positionPanel','tuningPosition'],['jogPanel','tuningJog'],['mappingPanel','tuningMapping'],['displayPanel','tuningDisplay']].forEach(function (pair) {
    var node=$(pair[0]);
    $(pair[1]).append(node);
    if(node.tagName==='DETAILS') node.open=true;
  });
  var active=false, timingJoint=0, filterJoint=0, stats=null, pending=false, signalPending=false;
  var bestMeasured=null;
  [1,2,3].forEach(function (joint) {
    var button=document.createElement('button');
    button.className='tuning-motor';
    button.innerHTML='<strong>M'+joint+' · '+['Front left','Front right','Rear'][joint-1]+'</strong><span></span>';
    button.dataset.motor=joint;
    button.addEventListener('click',function () { SimSync.selectMotor(joint); });
    $('tuningMotors').append(button);
  });
  $('tuningEnable').addEventListener('click',function () { $('motorEnable').click(); });
  $('soloStop').addEventListener('click',function () { SimSync.post('/api/arm?state=0'); });
  function loadFilters() {
    var s=SimSync.state(), m=s.motors[s.joint-1];
    if(!s.fresh || !m || !m.filters_ready) return;
    $('filterLPF').value=m.lpf_hz;
    $('filterHPF').value=m.hpf_hz;
    filterJoint=s.joint;
  }
  $('filterRead').addEventListener('click',loadFilters);
  $('filterApply').addEventListener('click',function () {
    if($('filterLPF').value==='' || $('filterHPF').value==='') { SimSync.error(new Error('Enter both filter cutoffs; 0 disables a filter')); return; }
    SimSync.post('/api/motor/filters',{joint:SimSync.state().joint,lpf_hz:Number($('filterLPF').value),hpf_hz:Number($('filterHPF').value)});
  });
  ['Positive','Negative'].forEach(function (direction) {
    $('test'+direction).addEventListener('click',function () {
      $('stepSize').value=Math.abs(Number($('stepSize').value))*(direction==='Positive'?1:-1);
      $('stepTestBtn').click();
    });
  });
  $('stageMeasured').addEventListener('click',function () {
    if(!bestMeasured || bestMeasured.Joint!==SimSync.state().joint) return;
    [['kp',bestMeasured.Kp],['ki',bestMeasured.Ki],['kd',bestMeasured.Kd]].forEach(function (pair) {
      $(pair[0]+'Num').value=pair[1];
      $(pair[0]+'Num').dispatchEvent(new Event('change'));
    });
    $('commandStatus').textContent='Measured M'+bestMeasured.Joint+' gains staged · Apply Gains to send';
  });
  function updateResponse(s,m,live) {
    Array.from($('tuningMotors').children).forEach(function (button,index) {
      var motor=s.motors[index];
      button.setAttribute('aria-pressed',String(s.joint===index+1));
      button.querySelector('span').textContent=!s.fresh || !s.frame || !s.frame.serial_ok || !motor || !motor.ready?'Waiting for readback':!motor.active?'Excluded · select and Apply Limits':motor.disabled?'Included · firmware disabled':'Included · PID enabled';
    });
    $('tuningEnable').textContent='Run only M'+s.joint;
    $('tuningEnable').disabled=$('motorEnable').disabled;
    $('testPositive').disabled=$('stepTestBtn').disabled;
    $('testNegative').disabled=$('stepTestBtn').disabled;
    var results=(SimSync.results()||[]).filter(function (r) {return r.Joint===s.joint;});
    var latest=results[0];
    // Different amplitudes/directions are different experiments, not competing gains.
    var comparable=latest?results.filter(function (r) {return r.StepSize===latest.StepSize;}):[];
    bestMeasured=comparable.filter(function (r) {return r.Success;}).sort(function (a,b) {
      return (a.OvershootPct*2+a.SettleTimeMs/100+a.SteadyStateErr*10)-(b.OvershootPct*2+b.SettleTimeMs/100+b.SteadyStateErr*10);
    })[0]||null;
    $('stageMeasured').disabled=!bestMeasured || !live || s.testBusy;
    $('responseTitle').textContent='M'+s.joint+' · measured response';
    var advice=!live?'Waiting for live motor feedback.':!m.active?'Include M'+s.joint+' in output under Motor limits, then Apply Limits while paused.':m.disabled?'Use Run only M'+s.joint+' after verifying feedback direction and travel bounds.':!latest?'Run a small step in each direction to establish a baseline.':!latest.Success?'Last step did not reach or settle at the target. Inspect the trace for stalled motion or ringing before changing gains.':latest.OvershootPct>10?'Overshoot exceeds 10%. Try a small increase in Kd, or reduce Kp, and repeat the same step.':latest.SteadyStateErr>2?'Residual tracking error exceeds 2 counts. Check friction and PWM minimum; evaluate a small Ki only after the PD response is stable.':'Response reached and settled. Compare rise and settling time, then validate the opposite direction and a slow signal under load.';
    $('tuningAdvice').textContent=advice;
    function summary(r) {return 'P '+r.Kp+' / I '+r.Ki+' / D '+r.Kd+' · rise '+(r.RiseTimeMs<0?'unreached':Math.round(r.RiseTimeMs)+' ms')+' · overshoot '+r.OvershootPct.toFixed(1)+'% · settle '+(r.SettleTimeMs<0?'unsettled':Math.round(r.SettleTimeMs)+' ms')+' · error '+r.SteadyStateErr.toFixed(1)+' counts';}
    $('responseComparison').textContent=latest?'Last '+latest.StepSize+' count step: '+summary(latest)+(bestMeasured?' | Best of '+comparable.length+' matching steps: '+summary(bestMeasured):' | No successful matching step yet.'):'No recorded steps for M'+s.joint+'.';
  }
  function view(name) {
    if(active===(name==='tuning')) return;
    active=name==='tuning';
    document.body.classList.toggle('tuning',active);
    ['tuningPanel','tuningIntro','scopeTools','tuningMetrics'].forEach(function (id) { $(id).hidden=!active; });
    tabs.forEach(function (tab) { var selected=tab.dataset.view===name; tab.classList.toggle('active',selected); tab.setAttribute('aria-selected',String(selected)); tab.tabIndex=selected?0:-1; });
    if(window.Scope) Scope.resize();
  }
  tabs.forEach(function (tab,index) {
    tab.addEventListener('click',function () { view(tab.dataset.view); });
    tab.addEventListener('keydown',function (e) {
      var next;
      if(e.key==='ArrowRight' || e.key==='ArrowLeft') next=tabs[(index+1)%tabs.length];
      else if(e.key==='Home') next=tabs[0]; else if(e.key==='End') next=tabs[tabs.length-1];
      if(next) { e.preventDefault(); next.focus(); view(next.dataset.view); }
    });
  });
  $('scopeScale').addEventListener('change',function () { Scope.setScale(this.value); });
  // Tuning should start with the three traces that answer "is it following?".
  ['limits','clip','host'].forEach(function (name) { Scope.setTrace(name,true); var button=document.querySelector('[data-trace="'+name+'"]'); if(button) { button.classList.add('active'); button.setAttribute('aria-pressed','true'); } });
  $('scopeFreeze').addEventListener('click',function () { var frozen=Scope.freeze(); this.textContent=frozen?'Resume display':'Freeze display'; this.setAttribute('aria-pressed',String(frozen)); });
  $('scopeImage').addEventListener('click',function () { Scope.saveImage(); });
  $('tuningCSV').addEventListener('click',function () { window.open('/api/session/export','_blank'); });
  function loadTiming() {
    var s=SimSync.state(), m=s.motors[s.joint-1];
    if(!m || !m.timing_ready || !stats || !stats.timing_ready) return;
    $('tuningKs').value=m.ks;
    $('tuningDivider').value=stats.pid_divider;
    $('tuningPWM12').value=stats.pwm_12_khz;
    $('tuningPWM3').value=stats.pwm_3_khz>=15?'31':'4';
    timingJoint=s.joint;
  }
  $('timingRead').addEventListener('click',loadTiming);
  $('timingApply').addEventListener('click',function () {
    SimSync.post('/api/controller/tuning',{joint:SimSync.state().joint,ks:Number($('tuningKs').value),divider:Number($('tuningDivider').value),pwm_12_khz:Number($('tuningPWM12').value),pwm_3_khz:Number($('tuningPWM3').value)});
  });
  $('signalStart').addEventListener('click',function () {
    signalPending=true;
    $('signalStart').disabled=true;
    $('signalState').textContent='Signal running · Stop / Pause aborts movement';
    SimSync.post('/api/signal',{joint:SimSync.state().joint,mode:$('signalMode').value,amplitude:Number($('signalAmplitude').value),period:Number($('signalPeriod').value)})
      .finally(function () { signalPending=false; $('signalState').textContent='Monitor · signal finished or stopped; command status shows outcome'; });
  });
  $('signalStop').addEventListener('click',function () { SimSync.post('/api/arm?state=0'); });

  function updateClipReadback(m) {
    var summary=$('clipSummary'), health=$('clipHealth');
    if(!m || !m.ready) { summary.textContent='Waiting for motor readback'; health.textContent='Clipping guidance will appear when connected.'; health.className='clip-health'; return; }
    var cutoffLo=Number(m.cutoff), cutoffHi=1023-cutoffLo;
    var clipLo=Number(m.clip), clipHi=1023-clipLo;
    var hostLo=Number(m.min), hostHi=Number(m.max);
    summary.innerHTML='Cutoff <b>'+cutoffLo+'–'+cutoffHi+'</b><br>Input clip <b>'+clipLo+'–'+clipHi+'</b><br>Host target <b>'+hostLo+'–'+hostHi+'</b>';
    var issues=[];
    if(cutoffLo>=clipLo) issues.push('cutoff must be inside clip');
    if(hostLo<clipLo+4 || hostHi>clipHi-4) issues.push('host bounds need ≥4 counts inside clip');
    if(hostLo>=hostHi) issues.push('host range is inverted');
    if(Number(m.pwm_max)>=250) issues.push('PWM max is near full scale; check current draw');
    if(issues.length) { health.textContent='CHECK · '+issues.join(' · '); health.className='clip-health '+(issues.some(function (v) { return v.indexOf('cutoff')>=0 || v.indexOf('inverted')>=0; })?'bad':'warn'); }
    else { health.textContent='GOOD · target stays clear of the safety cutoff'; health.className='clip-health'; }
  }

  // Local profiles stage edits; loading never sends commands or enables a motor.
  var profileFields=['kpNum','kiNum','kdNum','motorMin','motorMax','motorCenter','pwmMin','pwmMax','pwmRev','motorCutoff','motorClip','motorDeadzone','filterLPF','filterHPF'];
  var row=document.createElement('div'); row.className='btn-row';
  ['Save profile','Load profile'].forEach(function (label,index) {
    var btn=document.createElement('button'); btn.className='btn'; btn.textContent=label;
    btn.addEventListener('click',function () {
      try {
        var key='simsync.tuning.profile.m'+SimSync.state().joint;
        if(index===0) {
          var values={}; profileFields.forEach(function (id) { values[id]=$(id).value; });
          localStorage.setItem(key,JSON.stringify(values));
          $('commandStatus').textContent='Profile saved in this browser for M'+SimSync.state().joint;
        } else {
          var raw=localStorage.getItem(key); if(!raw) throw new Error('No browser profile for this motor yet');
          var values=JSON.parse(raw);
          ['filterLPF','filterHPF'].forEach(function (id) {if(values[id]===undefined) values[id]=0;});
          if(profileFields.some(function (id) { return values[id]==='' || !Number.isFinite(Number(values[id])); })) throw new Error('Invalid saved profile');
          profileFields.forEach(function (id) { $(id).value=values[id]; $(id).dispatchEvent(new Event('change')); });
          $('commandStatus').textContent='Profile loaded into fields · Apply Gains / Limits / Filters to send';
        }
      } catch(e) { SimSync.error(e); }
    });
    row.append(btn);
  });
  $('pidPanel').append(row);
  var note=document.createElement('p'); note.className='control-note'; note.textContent='P20 / D0.4 loads your reported stable gains into the form. Apply Gains sends them. Apply saves tuning to the Markdown file for restart. Profiles save drafts in this browser; EEPROM saves firmware values.'; $('pidPanel').append(note);

  setInterval(function () {
    var s=SimSync.state(), f=s.frame, m=s.motors[s.joint-1];
    var live=s.fresh && f && f.serial_ok && m && m.ready;
    if(window.Scope) Scope.setLimits(live?m:null);
    updateClipReadback(live?m:null);
    var moving=s.testBusy || signalPending || s.motors.some(function (v) {return v.jogging;});
    var soloIndex=s.motors.findIndex(function (v) {return v.solo;});
    $('soloState').textContent=soloIndex>=0?'Running only M'+(soloIndex+1)+' · manual/test output · other motors stopped · Pause stops all; Resume enables all included motors.':'Solo mode stops the other motors and enables manual/test output for one motor. Pause stops it. Resume returns to all included motors.';
    $('soloStop').disabled=!s.fresh || !f || !f.serial_ok;
    $('filterTitle').textContent='M'+s.joint+' · feedback filters';
    $('filterApply').disabled=!live || !m.filters_ready || !m.disabled || f.armed || m.diagnostics || moving;
    $('filterRead').disabled=!live || !m.filters_ready;
    $('filterReadback').textContent=!live?'Waiting for live feedback':!m.filters_supported?'Upload updated SMC3 firmware for LPF / HPF':!m.filters_ready?'Waiting for filter readback':'Readback M'+s.joint+' · feedback LPF '+(m.lpf_hz?m.lpf_hz+' Hz':'off')+' · derivative HPF '+(m.hpf_hz?m.hpf_hz+' Hz':'off');
    if(!live) filterJoint=0;
    else if(filterJoint!==s.joint) loadFilters();
    $('signalStart').disabled=!live || !f.armed || f.estop || !m.configured || !m.active || m.disabled || m.diagnostics || moving;
    $('signalStop').disabled=!live;
    $('timingApply').disabled=!live || f.armed || moving || m.diagnostics || !m.timing_ready || !stats || !stats.timing_ready;
    if(!live) { timingJoint=0; $('controllerInfo').textContent='Controller disconnected · waiting for live feedback'; $('timingReadback').textContent='Waiting for timing readback'; }
    if(!active) return;
    updateResponse(s,m,live);
    if(live && stats) {
      $('controllerInfo').textContent=stats.controller+' · firmware '+stats.firmware+' · '+(stats.port||'USB')+' · '+stats.baud.toLocaleString()+' baud';
      $('timingReadback').textContent=stats.timing_ready && m.timing_ready ? 'Readback M'+s.joint+' · Ks '+m.ks+' · Fpid / '+stats.pid_divider+' · PWM M1/2 '+stats.pwm_12_khz+' kHz / M3 '+(stats.pwm_3_khz>=15?'31.25':'3.906')+' kHz' : 'Waiting for timing readback';
      if(timingJoint!==s.joint) loadTiming();
    }
    if(window.Scope) {
      var metric=Scope.metrics(5);
      $('trackingRMS').textContent='RMS error '+(metric?Angles.format(s.joint,metric.rms,true):'—');
      $('trackingPeak').textContent='Peak error '+(metric?Angles.format(s.joint,metric.peak,true):'—');
      $('trackingPWM').textContent='Peak PWM '+(metric?metric.pwm+'/255':'—');
    }
    if(pending) return;
    pending=true;
    fetch('/api/controller/stats',{signal:AbortSignal.timeout(2000)}).then(function (r) {if(!r.ok) throw new Error('Disconnected'); return r.json();}).then(function (v) {stats=v;}).catch(function () {stats=null; timingJoint=0;}).finally(function () {pending=false;});
  },500);
})();
