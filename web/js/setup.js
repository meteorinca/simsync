(function(){
 'use strict';
 var $=id=>document.getElementById(id);

 async function read(path){
  var r=await fetch(path,{signal:AbortSignal.timeout(3000)});
  if(!r.ok)throw new Error('Could not read '+path);
  return r.json();
 }

 async function ports(){
  try{
   var c=await read('/api/connection'),select=$('serialPort'),chosen=select.value||c.port;
   select.replaceChildren(new Option('Auto',''));
   (c.ports||[]).forEach(port=>select.add(new Option(port,port)));
   if(chosen&&!Array.from(select.options).some(o=>o.value===chosen))select.add(new Option(chosen,chosen));
   select.value=chosen;
   $('benchMode').checked=c.bench;
   $('benchMode').disabled=c.bench_locked;
  }catch(e){SimSync.error(e);}
 }

 async function configure(){
  var result=await SimSync.post('/api/connection',{port:$('serialPort').value,bench:$('benchMode').checked});
  if(result)await ports();
 }

 $('refreshPorts').addEventListener('click',ports);
 $('connectPort').addEventListener('click',configure);
 $('benchMode').addEventListener('change',configure);

 function valid(m){
  return !!m&&Number.isFinite(m.min)&&m.cutoff>0&&m.cutoff<m.clip&&m.min>=m.clip+4&&m.max<=1019-m.clip&&m.min<m.center&&m.center<m.max;
 }
 function staged(){
  return {cutoff:Number($('motorCutoff').value),clip:Number($('motorClip').value),min:Number($('motorMin').value),max:Number($('motorMax').value),center:Number($('motorCenter').value)};
 }
 function range(m){
  var msg=m.cutoff>=m.clip?'Cutoff < Clip':m.min<m.clip+4?'Min ≥ '+(m.clip+4):m.max>1019-m.clip?'Max ≤ '+(1019-m.clip):m.min>=m.center||m.center>=m.max?'Min < Ctr < Max':valid(m)?'✓':'Invalid';
  $('rangeValidation').textContent=msg;
  $('rangeValidation').className=valid(m)?'ok':'bad';
  function rect(x,w,y,h,c){return '<rect x="'+Math.max(0,Math.min(1023,x))+'" width="'+Math.max(0,Math.min(1023,w))+'" y="'+y+'" height="'+h+'" class="'+c+'"/>';}
  $('rangeBar').innerHTML=rect(0,1023,25,26,'base')+rect(0,m.cutoff,25,26,'cutoff')+rect(1023-m.cutoff,m.cutoff,25,26,'cutoff')+rect(m.cutoff,m.clip-m.cutoff,25,26,'clip')+rect(1023-m.clip,m.clip-m.cutoff,25,26,'clip')+rect(m.min,m.max-m.min,20,36,valid(m)?'host':'invalid')+'<line x1="'+m.center+'" x2="'+m.center+'" y1="12" y2="64" class="center"/><text x="0" y="96">0</text><text x="410" y="96">'+m.min+'–'+m.max+'</text><text x="945" y="96">1023</text>';
 }

 function update(){
  var s=SimSync.state();
  $('connectPort').disabled=!!s.frame?.armed||s.testBusy;
  var m=staged();
  if(!valid(m))$('limitsApply').disabled=true;
  range(m);
 }

 ['motorMin','motorMax','motorCenter','motorCutoff','motorClip'].forEach(id=>$(id).addEventListener('input',()=>range(staged())));
 window.Setup={valid};
 ports();
 setInterval(update,500);
})();
