import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

test('tuning selects M2 and compares only its matching experiments without applying gains', () => {
  function element() {
    return {children:[],value:'8',disabled:false,tagName:'DIV',classList:{toggle(){}},dataset:{},events:{},
      append(node){this.children.push(node);},before(){},after(){},setAttribute(k,v){this[k]=v;},
      addEventListener(k,fn){this.events[k]=fn;},querySelector(){return this.label ||= element();},
      dispatchEvent(){},click(){if(!this.disabled)this.events.click?.call(this);}};
  }
  const nodes=new Map();
  const get=id=>{if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);};
  const state={joint:1,fresh:true,frame:{serial_ok:true,armed:false},motors:[1,2,3].map(j=>({ready:true,configured:true,active:true,disabled:true,filters_supported:true,filters_ready:true,lpf_hz:j*30,hpf_hz:j})),testBusy:false};
  const result=(Joint,StepSize,Kp)=>({Joint,StepSize,Kp,Ki:0,Kd:.4,Success:true,RiseTimeMs:80,OvershootPct:2,SettleTimeMs:200,SteadyStateErr:1});
  const results=[result(1,8,99),result(2,8,21),result(2,-8,88)];
  const intervals=[],writes=[];
  const context={document:{getElementById:get,querySelectorAll:()=>[],querySelector:()=>null,createElement:element,createComment:element,body:{classList:{toggle(){}}}},
    window:{},Scope:{setTrace(){},setLimits(){},metrics(){return null;}},Angles:{format(){return '';}},
    SimSync:{state:()=>state,selectMotor:j=>{state.joint=j;},results:()=>results,post:(...args)=>writes.push(args),error(){}},
    setInterval:fn=>intervals.push(fn),fetch:()=>Promise.resolve({ok:true,json:()=>Promise.resolve({})}),AbortSignal,Event:class {},localStorage:{},};
  const tab=element();tab.dataset.view='tuning';
  context.document.querySelectorAll=selector=>selector==='[data-view]'?[tab]:[];
  vm.runInNewContext(readFileSync(new URL('web/js/tuning.js',import.meta.url),'utf8'),context);
  assert.ok(get('tuningPID').children.includes(get('pidPanel')));
  assert.ok(get('tuningJog').children.includes(get('jogPanel')));
  get('tuningMotors').children[1].click();tab.click();intervals[0]();
  assert.equal(state.joint,2);
  assert.match(get('tuningAdvice').textContent,/Run only M2/);
  assert.equal(get('filterLPF').value,60);
  assert.equal(get('filterHPF').value,2);
  assert.match(get('responseComparison').textContent,/Best of 1 matching steps/);
  get('stageMeasured').click();
  assert.equal(get('kpNum').value,21);
  assert.equal(writes.length,0);
  let enabled=false;get('motorEnable').addEventListener('click',()=>{enabled=true;});
  get('tuningEnable').click();assert.equal(enabled,true);
  get('filterLPF').value='80';get('filterHPF').value='0';
  intervals[0]();assert.equal(get('filterLPF').value,'80','readback must preserve edits');
  get('filterApply').click();
  assert.equal(writes[0][0],'/api/motor/filters');
  assert.equal(JSON.stringify(writes[0][1]),JSON.stringify({joint:2,lpf_hz:80,hpf_hz:0}));
  get('tuningMotors').children[0].click();intervals[0]();
  assert.equal(get('filterLPF').value,30);
  assert.equal(get('filterHPF').value,1);
  state.frame.armed=true;intervals[0]();assert.equal(get('filterApply').disabled,true);
  state.fresh=false;intervals[0]();
  assert.match(get('tuningAdvice').textContent,/Waiting/);
  assert.equal(get('stageMeasured').disabled,true);
});
