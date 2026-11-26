(function () {
 'use strict';
 var $=id=>document.getElementById(id), stats=null, filterJoint=0, timingJoint=0, signalPending=false, best=null;
 var tabs=Array.from(document.querySelectorAll('[data-view]'));

 function view(name){
  tabs.forEach(tab=>{
   var selected=tab.dataset.view===name;
   tab.setAttribute('aria-selected',selected);
   tab.tabIndex=selected?0:-1;
   $(tab.dataset.view+'View').hidden=!selected;
  });
  if(name==='dashboard' && window.Scope) Scope.resize();
 }
 window.Workspace={view};

 tabs.forEach((tab,index)=>{
  tab.addEventListener('click',()=>view(tab.dataset.view));
  tab.addEventListener('keydown',e=>{
   var i=e.key==='ArrowRight'?(index+1)%tabs.length:e.key==='ArrowLeft'?(index+tabs.length-1)%tabs.length:e.key==='Home'?0:e.key==='End'?tabs.length-1:-1;
   if(i>=0){e.preventDefault();tabs[i].focus();view(tabs[i].dataset.view);}
  });
 });

 [1,2].forEach(j=>{
  var b=document.createElement('button');
  b.className='tuning-motor';
  b.innerHTML='<strong>M'+j+'</strong><span></span>';
  b.addEventListener('click',()=>{SimSync.selectMotor(j);filterJoint=timingJoint=0;});
  $('tuningMotors').append(b);
 });

 $('scopeScale').addEventListener('change',function(){Scope.setScale(this.value);});
 $('scopeFreeze').addEventListener('click',function(){
  var frozen=Scope.freeze();
  this.textContent=frozen?'▶':'❄';
  this.title=frozen?'Resume scope':'Freeze scope';
  this.setAttribute('aria-pressed',frozen);
 });
 $('scopeImage').addEventListener('click',()=>Scope.saveImage());

 function loadFilters(){var s=SimSync.state(),m=s.motors[s.joint-1];if(!m?.filters_ready)return;$('filterLPF').value=m.lpf_hz;$('filterHPF').value=m.hpf_hz;filterJoint=s.joint;}
 $('filterRead').addEventListener('click',loadFilters);
 $('filterApply').addEventListener('click',()=>{
  if(['filterLPF','filterHPF'].some(id=>$(id).value==='')){SimSync.error(new Error('Enter both filter values'));return;}
  SimSync.post('/api/motor/filters',{joint:SimSync.state().joint,lpf_hz:Number($('filterLPF').value),hpf_hz:Number($('filterHPF').value)});
 });

 function loadTiming(){
  var s=SimSync.state(),m=s.motors[s.joint-1];
  if(!m?.timing_ready || !stats?.timing_ready)return;
  $('tuningKs').value=m.ks;$('tuningDivider').value=stats.pid_divider;$('tuningPWM12').value=stats.pwm_12_khz;$('tuningPWM3').value=stats.pwm_3_khz>=15?'31':'4';
  timingJoint=s.joint;
 }
 $('timingRead').addEventListener('click',loadTiming);
 $('timingApply').addEventListener('click',()=>SimSync.post('/api/controller/tuning',{joint:SimSync.state().joint,ks:Number($('tuningKs').value),divider:Number($('tuningDivider').value),pwm_12_khz:Number($('tuningPWM12').value),pwm_3_khz:Number($('tuningPWM3').value)}));

 $('signalStart').addEventListener('click',()=>{
  signalPending=true;
  SimSync.post('/api/signal',{joint:SimSync.state().joint,mode:$('signalMode').value,amplitude:Number($('signalAmplitude').value),period:Number($('signalPeriod').value)}).finally(()=>signalPending=false);
 });

 $('stageMeasured').addEventListener('click',()=>{
  if(!best || best.Joint!==SimSync.state().joint)return;
  ['kp','ki','kd'].forEach(k=>{$(k+'Num').value=best[k[0].toUpperCase()+k.slice(1)];$(k+'Num').dispatchEvent(new Event('change'));});
 });

 function gate(id,enabled,reason){var el=$(id);if(!el)return;el.disabled=!enabled;el.title=enabled?'':reason;}

 var fields=['kpNum','kiNum','kdNum','motorMin','motorMax','motorCenter','pwmMin','pwmMax','pwmRev','motorCutoff','motorClip','motorDeadzone','filterLPF','filterHPF'];
 var exportButton=document.createElement('button');exportButton.className='btn';exportButton.textContent='Export';
 exportButton.onclick=()=>{var values={};fields.forEach(id=>values[id]=$(id).value);var url=URL.createObjectURL(new Blob([JSON.stringify({version:1,motor:SimSync.state().joint,values},null,2)],{type:'application/json'}));var a=document.createElement('a');a.href=url;a.download='simsync-m'+SimSync.state().joint+'.json';a.click();URL.revokeObjectURL(url);};
 var importLabel=document.createElement('label');importLabel.className='btn';importLabel.textContent='Import ';var input=document.createElement('input');input.type='file';input.accept='.json';input.hidden=true;importLabel.append(input);
 input.onchange=async()=>{try{var data=JSON.parse(await input.files[0].text());if(data.version!==1 || fields.some(id=>data.values?.[id]==='' || !Number.isFinite(Number(data.values?.[id]))))throw new Error('Invalid profile');fields.forEach(id=>{$(id).value=data.values[id];$(id).dispatchEvent(new Event('change'));});$('commandStatus').textContent='Profile staged';}catch(e){SimSync.error(e);}input.value='';};
 $('profileControls').append(exportButton,importLabel);

 async function readStats(){try{var r=await fetch('/api/controller/stats',{signal:AbortSignal.timeout(2000)});stats=r.ok?await r.json():null;}catch{stats=null;}}
 readStats();setInterval(readStats,2500);

 function update(){
  var s=SimSync.state(),f=s.frame,m=s.motors[s.joint-1],live=!!(s.fresh&&f?.serial_ok&&m?.ready),moving=s.testBusy||signalPending||s.motors.some(v=>v.jogging);
  Array.from($('tuningMotors').children).forEach((b,i)=>{
   b.setAttribute('aria-pressed',s.joint===i+1);
   var v=s.motors[i];
   b.querySelector('span').textContent=!live?'':!v?.active?'Off':v.disabled?'⏸':'●';
  });
  $('motorEnable').setAttribute('aria-pressed',!!m?.solo);
  $('motorEnable').textContent=m?.solo?'Exit Solo':'Solo M'+s.joint;
  gate('testNegative',!$('stepTestBtn').disabled,'Resume or Solo M'+s.joint+' first');
  gate('filterApply',live&&m.filters_ready&&m.disabled&&!f.armed&&!moving,'Pause motors first');
  gate('filterRead',live&&m.filters_ready,'Connect controller');
  gate('timingApply',live&&!f.armed&&!moving&&m.timing_ready&&stats?.timing_ready,'Pause motors first');
  gate('signalStart',live&&f.armed&&!f.estop&&m.configured&&m.active&&!m.disabled&&!moving,'Enable bench mode & Solo');

  $('filterTitle').textContent='M'+s.joint+' Filters';
  $('filterReadback').textContent=live&&m.filters_ready?'LPF '+m.lpf_hz+' · HPF '+m.hpf_hz:'';
  $('timingReadback').textContent=live&&stats?.timing_ready?'Ks '+m.ks+' · div '+stats.pid_divider:'';
  $('controllerInfo').textContent=live&&stats?'v'+stats.firmware+' · '+stats.port:'';
  if(!live){filterJoint=timingJoint=0;}else{if(filterJoint!==s.joint)loadFilters();if(timingJoint!==s.joint)loadTiming();}
  $('signalState').textContent=signalPending?'Running…':'';

  var diff=live?['kp','ki','kd'].filter(k=>Number($(k+'Num').value)!==m[k]).map(k=>k[0].toUpperCase()+k.slice(1)+' '+m[k].toFixed(2)+'→'+Number($(k+'Num').value).toFixed(2)):[];
  $('gainDiff').textContent=diff.join(' ');

  var results=SimSync.results().filter(r=>r.Joint===s.joint),last=results[0],matching=last?results.filter(r=>r.StepSize===last.StepSize):[];
  best=matching.filter(r=>r.Success).sort((a,b)=>(a.OvershootPct*2+a.SettleTimeMs/100+a.SteadyStateErr*10)-(b.OvershootPct*2+b.SettleTimeMs/100+b.SteadyStateErr*10))[0];
  gate('stageMeasured',!!best&&live&&!moving,'No settled step yet');

  var verdict='',state='';
  if(last){
   if(!last.Success){verdict='✕ Unsettled';state='bad';}
   else if(last.OvershootPct>10){verdict='⚠ OS '+last.OvershootPct.toFixed(0)+'%';state='warn';}
   else if(last.RiseTimeMs>500){verdict='⚠ Slow';state='warn';}
   else{verdict='✓ Settled';state='ok';}
  }
  var metric=live?Scope.metrics(5):null;
  if(s.testBusy&&metric&&metric.pwm>=m.pwm_max){verdict='⚠ Sat';state='warn';}
  $('tuningAdvice').textContent=verdict;$('tuningAdvice').className=state;
  $('responseComparison').textContent=last?Math.round(last.RiseTimeMs)+'ms · OS '+last.OvershootPct.toFixed(1)+'% · '+Math.round(last.SettleTimeMs)+'ms':'';
  Scope.setLimits(live?m:null);
  $('trackingRMS').textContent='RMS '+(metric?metric.rms.toFixed(1):'-');
  $('trackingPeak').textContent='Peak '+(metric?metric.peak.toFixed(1):'-');
  $('trackingPWM').textContent='PWM '+(metric?metric.pwm:'-');
 }

 setInterval(update,500);
 view('dashboard');
 SimSync.selectMotor(1);
})();
