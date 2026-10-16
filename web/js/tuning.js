(function () {
 'use strict';
 var $=id=>document.getElementById(id), active='dashboard', stats=null, filterJoint=0,timingJoint=0,signalPending=false,best=null;
 var tabs=Array.from(document.querySelectorAll('[data-view]'));
 function view(name){
  active=name;
  tabs.forEach(tab=>{var selected=tab.dataset.view===name;tab.setAttribute('aria-selected',selected);tab.tabIndex=selected?0:-1;$(tab.dataset.view+'View').hidden=!selected;});
  var motorHost=name==='tuning'?$('motorToolbar'):name==='settings'?$('settingsMotor'):name==='setup'?$('setupMotor'): $('driveMotor');
  motorHost.append($('tuningMotors'));
  if(name==='dashboard' || name==='tuning') (name==='tuning'?$('tuneScope'):$('dashboardView')).prepend($('scopePanel'));
  ['scopeTools','tuningMetrics'].forEach(id=>$(id).hidden=name!=='tuning');
  Scope.resize();
 }
 window.Workspace={view};
 tabs.forEach((tab,index)=>{
  tab.addEventListener('click',()=>view(tab.dataset.view));
  tab.addEventListener('keydown',e=>{
   var i=e.key==='ArrowRight'?(index+1)%tabs.length:e.key==='ArrowLeft'?(index+tabs.length-1)%tabs.length:e.key==='Home'?0:e.key==='End'?tabs.length-1:-1;
   if(i>=0){e.preventDefault();tabs[i].focus();view(tabs[i].dataset.view);}
  });
 });
 // M3 remains supported by the controller, but is not exposed in the workspace.
 [1,2].forEach(j=>{
  var b=document.createElement('button');b.className='tuning-motor';b.innerHTML='<strong>M'+j+'</strong><span></span>';
  b.addEventListener('click',()=>{SimSync.selectMotor(j);filterJoint=timingJoint=0;});$('tuningMotors').append(b);
 });
 $('scopeScale').addEventListener('change',function(){Scope.setScale(this.value);});
 $('scopeFreeze').addEventListener('click',function(){var frozen=Scope.freeze();this.textContent=frozen?'Resume display':'Freeze display';this.setAttribute('aria-pressed',frozen);});
 $('scopeImage').addEventListener('click',()=>Scope.saveImage());
 $('editTravel').addEventListener('click',()=>{view('setup');Setup.step(2);});
 function loadFilters(){var s=SimSync.state(),m=s.motors[s.joint-1];if(!m?.filters_ready)return;$('filterLPF').value=m.lpf_hz;$('filterHPF').value=m.hpf_hz;filterJoint=s.joint;}
 $('filterRead').addEventListener('click',loadFilters);
 $('filterApply').addEventListener('click',()=>{
  if(['filterLPF','filterHPF'].some(id=>$(id).value==='')){SimSync.error(new Error('Enter both filter values'));return;}
  SimSync.post('/api/motor/filters',{joint:SimSync.state().joint,lpf_hz:Number($('filterLPF').value),hpf_hz:Number($('filterHPF').value)});
 });
 function loadTiming(){var s=SimSync.state(),m=s.motors[s.joint-1];if(!m?.timing_ready || !stats?.timing_ready)return;
  $('tuningKs').value=m.ks;$('tuningDivider').value=stats.pid_divider;$('tuningPWM12').value=stats.pwm_12_khz;$('tuningPWM3').value=stats.pwm_3_khz>=15?'31':'4';timingJoint=s.joint;}
 $('timingRead').addEventListener('click',loadTiming);
 $('timingApply').addEventListener('click',()=>SimSync.post('/api/controller/tuning',{joint:SimSync.state().joint,ks:Number($('tuningKs').value),divider:Number($('tuningDivider').value),pwm_12_khz:Number($('tuningPWM12').value),pwm_3_khz:Number($('tuningPWM3').value)}));
 $('signalStart').addEventListener('click',()=>{signalPending=true;SimSync.post('/api/signal',{joint:SimSync.state().joint,mode:$('signalMode').value,amplitude:Number($('signalAmplitude').value),period:Number($('signalPeriod').value)}).finally(()=>signalPending=false);});
 $('stageMeasured').addEventListener('click',()=>{
  if(!best || best.Joint!==SimSync.state().joint)return;
  ['kp','ki','kd'].forEach(k=>{$(k+'Num').value=best[k[0].toUpperCase()+k.slice(1)];$(k+'Num').dispatchEvent(new Event('change'));});
 });
 function gate(id,enabled,reason){$(id).disabled=!enabled;$(id).title=enabled?'':reason;}
 var fields=['kpNum','kiNum','kdNum','motorMin','motorMax','motorCenter','pwmMin','pwmMax','pwmRev','motorCutoff','motorClip','motorDeadzone','filterLPF','filterHPF'];
 var exportButton=document.createElement('button');exportButton.className='btn';exportButton.textContent='Export profile';
 exportButton.onclick=()=>{var values={};fields.forEach(id=>values[id]=$(id).value);var url=URL.createObjectURL(new Blob([JSON.stringify({version:1,motor:SimSync.state().joint,values},null,2)],{type:'application/json'}));var a=document.createElement('a');a.href=url;a.download='simsync-m'+SimSync.state().joint+'.json';a.click();URL.revokeObjectURL(url);};
 var importLabel=document.createElement('label');importLabel.textContent='Import profile ';var input=document.createElement('input');input.type='file';input.accept='.json';input.setAttribute('aria-label','Import motor profile');importLabel.append(input);
 input.onchange=async()=>{try{var data=JSON.parse(await input.files[0].text());if(data.version!==1 || fields.some(id=>data.values?.[id]==='' || !Number.isFinite(Number(data.values?.[id]))))throw new Error('Invalid profile');fields.forEach(id=>{$(id).value=data.values[id];$(id).dispatchEvent(new Event('change'));});$('commandStatus').textContent='Profile staged. Review before applying.';}catch(e){SimSync.error(e);}input.value='';};
 $('profileControls').append(exportButton,importLabel);
 async function readStats(){try{var r=await fetch('/api/controller/stats',{signal:AbortSignal.timeout(2000)});stats=r.ok?await r.json():null;}catch{stats=null;}}
 readStats();setInterval(readStats,2500);
 function update(){
  var s=SimSync.state(),f=s.frame,m=s.motors[s.joint-1],live=!!(s.fresh&&f?.serial_ok&&m?.ready),moving=s.testBusy||signalPending||s.motors.some(v=>v.jogging);
  Array.from($('tuningMotors').children).forEach((b,i)=>{b.setAttribute('aria-pressed',s.joint===i+1);var v=s.motors[i];b.querySelector('span').textContent=!live?'':!v?.active?'Excluded':v.disabled?'Paused':'Running';});
  $('motorEnable').setAttribute('aria-pressed',!!m?.solo);
  $('motorEnable').title=$('motorEnable').disabled?'Check travel, disable jog, and clear stop.':'Other motors are held while solo is on.';
  gate('testNegative',!$('stepTestBtn').disabled,'Enable Solo to test this motor.');
  $('stepTestBtn').title=$('stepTestBtn').disabled?'Enable Solo to test this motor.':'';
  gate('filterApply',live&&m.filters_ready&&m.disabled&&!f.armed&&!m.diagnostics&&!moving,'Pause motors and disable jog to apply.');
  gate('filterRead',live&&m.filters_ready,'Connect a controller with filter support.');
  gate('timingApply',live&&!f.armed&&!moving&&!m.diagnostics&&m.timing_ready&&stats?.timing_ready,'Pause motors and disable jog to apply.');
  gate('signalStart',live&&f.armed&&!f.estop&&m.configured&&m.active&&!m.disabled&&!m.diagnostics&&!moving,'Enable Solo and bench mode first.');
  $('filterTitle').textContent='M'+s.joint+' filters';$('filterReadback').textContent=live&&m.filters_ready?'Live: LPF '+m.lpf_hz+' Hz · HPF '+m.hpf_hz+' Hz':'';
  $('timingReadback').textContent=live&&stats?.timing_ready?'Live: Ks '+m.ks+' · Fpid '+stats.pid_divider:'';
  $('controllerInfo').textContent=live&&stats?'Firmware '+stats.firmware+' · '+stats.port:'';
  if(!live){filterJoint=timingJoint=0;}else{if(filterJoint!==s.joint)loadFilters();if(timingJoint!==s.joint)loadTiming();}
  $('signalState').textContent=signalPending?'Signal running':'Use bench mode for signals.';
  var diff=live?['kp','ki','kd'].filter(k=>Number($(k+'Num').value)!==m[k]).map(k=>k[0].toUpperCase()+k.slice(1)+' '+m[k].toFixed(2)+' → '+Number($(k+'Num').value).toFixed(2)):[];
  $('gainDiff').textContent=diff.join(' · ') || (live?'Matches live gains':'');
  $('pidSendBtn').title=$('pidSendBtn').disabled?'Connect a ready motor and finish any test.':'';
  var results=SimSync.results().filter(r=>r.Joint===s.joint),last=results[0],matching=last?results.filter(r=>r.StepSize===last.StepSize):[];
  best=matching.filter(r=>r.Success).sort((a,b)=>(a.OvershootPct*2+a.SettleTimeMs/100+a.SteadyStateErr*10)-(b.OvershootPct*2+b.SettleTimeMs/100+b.SteadyStateErr*10))[0];
  gate('stageMeasured',!!best&&live&&!moving,'Run a settled step first.');
  var verdict='Run a small step.',hint='',state='';
  if(last){
   if(!last.Success){verdict='✕ Did not settle';hint='Check direction and travel before changing gains.';state='bad';}
   else if(last.OvershootPct>10){verdict='⚠ Overshoot '+last.OvershootPct.toFixed(0)+'%';hint='Add a little Kd, then step again.';state='warn';}
   else if(last.RiseTimeMs>500){verdict='⚠ Slow response';hint='Raise Kp about 5%, then step again.';state='warn';}
   else{verdict='✓ Settled';state='ok';}
  }
  var metric=live?Scope.metrics(5):null;
  if(s.testBusy&&metric&&metric.pwm>=m.pwm_max){verdict='⚠ PWM saturated';hint='Lower Kp, or raise PWM max by 5 to 10.';state='warn';}
  $('tuningAdvice').textContent=last||s.testBusy?verdict:'';$('tuningAdvice').className=state;$('tuningAdvice').title=hint;
  $('responseComparison').textContent=last?'Rise '+Math.round(last.RiseTimeMs)+' ms · Overshoot '+last.OvershootPct.toFixed(1)+'% · Settle '+Math.round(last.SettleTimeMs)+' ms':'';
  gate('saveTune',live&&!!last?.Success&&!moving&&!diff.length&&['kp','ki','kd'].every(k=>m[k]===last[k[0].toUpperCase()+k.slice(1)]),'Run a settled step with the live gains first.');
  Scope.setLimits(live?m:null);
  $('trackingRMS').textContent='RMS '+(metric?metric.rms.toFixed(1):'-')+' counts';$('trackingPeak').textContent='Peak error '+(metric?metric.peak.toFixed(1):'-');$('trackingPWM').textContent='Peak PWM '+(metric?metric.pwm:'-');
 }
 setInterval(update,500);view('dashboard');
})();
