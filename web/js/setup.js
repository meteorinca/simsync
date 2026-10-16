(function(){
 'use strict';
 var $=id=>document.getElementById(id),current=1,progress=[],connection=null,previous=null,selected=0,directionObserved=false;
 var picker=document.createElement('div');picker.id='setupMotor';$('setupView').querySelector('h1').after(picker);
 async function read(path){var r=await fetch(path,{signal:AbortSignal.timeout(3000)});if(!r.ok)throw new Error('Could not read '+path);return r.json();}
 async function loadProgress(){try{progress=await read('/api/setup');}catch{} }
 async function ports(){try{
  connection=await read('/api/connection');var select=$('serialPort'),chosen=select.value||connection.port;
  select.replaceChildren(new Option('Auto-detect',''));
  (connection.ports||[]).forEach(port=>select.add(new Option(port,port)));
  if(chosen&&!Array.from(select.options).some(o=>o.value===chosen))select.add(new Option(chosen,chosen));
  select.value=chosen;$('benchMode').checked=connection.bench;$('benchMode').disabled=connection.bench_locked;
  $('benchMode').title=connection.bench_locked?'Started with --no-udp. Restart to enable games.':'';
 }catch(e){SimSync.error(e);}}
 $('refreshPorts').addEventListener('click',ports);
 async function configure(){var result=await SimSync.post('/api/connection',{port:$('serialPort').value,bench:$('benchMode').checked});if(result)await ports();}
 $('connectPort').addEventListener('click',configure);$('benchMode').addEventListener('change',configure);
 $('dismissSafety').addEventListener('click',()=>{$('safetyNotice').hidden=true;});
 function step(n){current=n===3?4:n;[1,2,4].forEach(i=>$('setupStep'+i).hidden=i!==current);document.querySelectorAll('[data-step]').forEach(b=>b.setAttribute('aria-current',Number(b.dataset.step)===current?'step':'false'));update();}
 document.querySelectorAll('[data-step]').forEach(b=>b.addEventListener('click',()=>step(Number(b.dataset.step))));
 $('setupNext').addEventListener('click',()=>step(current===1?2:4));
 $('firstTune').addEventListener('click',()=>Workspace.view('tuning'));
 async function save(tuned){var s=SimSync.state(),old=progress[s.joint-1]||{};var direction=tuned?!!old.direction:$('directionConfirmed').checked;
  var result=await SimSync.post('/api/setup',{joint:s.joint,direction,tuned:tuned?true:!!old.tuned});if(result)await loadProgress();}
 $('directionConfirmed').addEventListener('change',()=>save(false));$('saveTune').addEventListener('click',()=>save(true));
 function valid(m){return !!m&&Number.isFinite(m.min)&&m.cutoff>0&&m.cutoff<m.clip&&m.min>=m.clip+4&&m.max<=1019-m.clip&&m.min<m.center&&m.center<m.max;}
 function staged(){return {cutoff:Number($('motorCutoff').value),clip:Number($('motorClip').value),min:Number($('motorMin').value),max:Number($('motorMax').value),center:Number($('motorCenter').value)};}
 function range(m){
  var message=m.cutoff>=m.clip?'Cutoff must be smaller than clip.':m.min<m.clip+4?'Host min must be ≥ '+(m.clip+4)+'.':m.max>1019-m.clip?'Host max must be ≤ '+(1019-m.clip)+'.':m.min>=m.center||m.center>=m.max?'Center must be inside host bounds.':valid(m)?'Bounds valid.':'Enter valid travel bounds.';
  $('rangeValidation').textContent=message;$('rangeValidation').className=valid(m)?'ok':'bad';
  function rect(x,w,y,h,c){return '<rect x="'+Math.max(0,Math.min(1023,x))+'" width="'+Math.max(0,Math.min(1023,w))+'" y="'+y+'" height="'+h+'" class="'+c+'"/>';}
  $('rangeBar').innerHTML=rect(0,1023,35,30,'base')+rect(0,m.cutoff,35,30,'cutoff')+rect(1023-m.cutoff,m.cutoff,35,30,'cutoff')+rect(m.cutoff,m.clip-m.cutoff,35,30,'clip')+rect(1023-m.clip,m.clip-m.cutoff,35,30,'clip')+rect(m.min,m.max-m.min,30,40,valid(m)?'host':'invalid')+'<line x1="'+m.center+'" x2="'+m.center+'" y1="20" y2="80" class="center"/><text x="0" y="110">0</text><text x="380" y="110">Host '+m.min+' to '+m.max+'</text><text x="950" y="110">1023</text>';
 }
 function update(){
  var s=SimSync.state(),m=s.motors[s.joint-1],live=!!(s.fresh&&s.frame?.serial_ok&&m?.ready),j=s.frame?.joints?.[s.joint-1];
  if(selected!==s.joint){selected=s.joint;previous=null;directionObserved=false;}
  if(live&&j){if(previous!==null&&j.actual!==previous){$('feedbackDirection').textContent=(j.actual>previous?'↑ ':'↓ ')+j.actual+' counts';directionObserved=true;}previous=j.actual;}
  $('livePot').textContent=live&&j?'M'+s.joint+' · '+j.actual+' counts':'';
  $('directionConfirmed').checked=!!progress[s.joint-1]?.direction;$('directionConfirmed').disabled=!live||!directionObserved;
  $('directionConfirmed').title=directionObserved?'':'Jog this motor and observe feedback first.';
  var checks=[live&&!s.frame.armed,live&&valid(m),false,!!progress[s.joint-1]?.tuned];
  [1,2,4].forEach(i=>$('stepDone'+i).textContent=checks[i-1]?' ✓':'');
  $('setupPass').textContent=checks[current-1]?'Complete':'';
  $('setupNext').disabled=!checks[current-1];$('setupNext').hidden=current===4;
  $('connectPort').disabled=!!s.frame?.armed||s.testBusy;$('connectPort').title=$('connectPort').disabled?'Pause motors to connect.':'';
  $('limitsApply').title=$('limitsApply').disabled?'Pause motors to apply.':'';
  if(!valid(staged())){$('limitsApply').disabled=true;$('limitsApply').title='Correct the travel bounds first.';}
  range(staged());
 }
 ['motorMin','motorMax','motorCenter','motorCutoff','motorClip'].forEach(id=>$(id).addEventListener('input',()=>range(staged())));
 window.Setup={step,valid};loadProgress();ports();setInterval(update,500);step(1);
})();
