import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

const html = readFileSync(new URL('web/index.html', import.meta.url), 'utf8');
const script = readFileSync(new URL('web/js/app.js', import.meta.url), 'utf8');

function dashboard() {
  const elements = new Map();
  for (const match of html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)) {
    elements.set(match[1], {
      value: match[0].match(/\bvalue="([^"]*)"/)?.[1] ?? '',
      style: {}, dataset: {}, textContent: '', classList: { toggle() {} }, events: {},
      addEventListener(name, fn) { this.events[name] = fn; },
      querySelector() { return this; },
      click() { return this.events.click?.call(this); },
    });
  }
  const intervals = [], requests = [];
  const motors = [1, 2, 3].map(j => ({ kp: j, ki: 0, kd: 0.4, ready: true, configured: true, active: true, disabled: false, min: 480, max: 544, center: 508 + j * 4, pwm_min: 0, pwm_max: 40, pwm_rev: 0, cutoff: 30, clip: 50, deadzone: 0 }));
  let socket;
  const context = {
    document: { getElementById(id) { assert.ok(elements.has(id), `missing element ${id}`); return elements.get(id); }, querySelectorAll() { return []; } },
    location: { protocol: 'http:', host: 'localhost:7071' },
    window: {}, AbortSignal,
    WebSocket: function () { socket = this; },
    setInterval(fn) { intervals.push(fn); }, setTimeout() {},
    fetch: async (url, options) => {
      requests.push({url, options});
      return { ok: true, json: async () => url === '/api/motors' ? structuredClone(motors) : url === '/api/steptest/status' ? {running: false} : [] };
    },
  };
  vm.runInNewContext(readFileSync(new URL('web/js/angles.js', import.meta.url), 'utf8'), context);
  context.Angles=context.window.Angles;
  vm.runInNewContext(script, context);
  const frame = { serial_ok: true, armed: true, estop: false, udp_hz: 0, joints: motors.map((_,i) => ({j:i+1,actual:512,target:512,error:0,duty:0})) };
  return {elements, requests, motors, async poll() { socket.onmessage({data:JSON.stringify(frame)}); intervals.forEach(fn=>fn()); await new Promise(setImmediate); }};
}

test('startup fields are prefilled and refresh when automatic configuration completes', async () => {
  const d = dashboard();
  assert.equal(d.elements.get('motorMin').value, '50');
  assert.equal(d.elements.get('motorMax').value, '650');
  assert.equal(d.elements.get('motorCenter').value,'338');
  assert.equal(d.elements.get('kpNum').value,'20.00');
  assert.equal(d.elements.get('kiNum').value, '0.00');
  d.motors[0].configured = false;
  d.motors[0].ki = 0.4;
  d.motors[0].pwm_rev = 200;
  await d.poll();
  assert.equal(d.elements.get('motorMin').value, 50);
  assert.equal(d.elements.get('pwmRev').value, 50);
  assert.equal(d.elements.get('kiNum').value, '0.00');
  d.motors[0].configured = true;
  d.motors[0].ki = 0;
  await d.poll();
  assert.equal(d.elements.get('motorMin').value, 480);
});

test('selection, gain readback and commands use the same motor', async () => {
  const d = dashboard(); await d.poll();
  assert.equal(d.elements.get('kpNum').value, '1.00');
  const select = d.elements.get('manJoint'); select.value = '2'; select.events.change.call(select);
  assert.equal(d.elements.get('stepJoint').value, 2);
  assert.equal(d.elements.get('kpNum').value, '2.00');
  d.elements.get('centerBtn').click();
  assert.equal(d.requests.at(-1).url, '/api/target?joint=2&angle=516');
  assert.equal(d.requests.at(-1).options.method, 'POST');
  assert.equal(d.requests.at(-1).options.headers['X-SimSync'], '1');
  d.elements.get('manPosition').value = '520'; d.elements.get('manGoBtn').click();
  assert.equal(d.requests.at(-1).url, '/api/target?joint=2&angle=520.0');
  d.elements.get('pidSendBtn').click();
  assert.equal(d.requests.at(-1).url, '/api/pid?joint=2');
});

test('run-only action targets selected motor and solo mode gates other motor motion', async () => {
 const d=dashboard();d.motors[1].jog_supported=true;await d.poll();
 const select=d.elements.get('manJoint');select.value='2';select.events.change.call(select);
 d.elements.get('motorEnable').click();
 assert.equal(d.requests.at(-1).url,'/api/motor/solo?joint=2');
 assert.equal(d.requests.at(-1).options.method,'POST');
 d.motors[1].solo=true;await d.poll();
 assert.equal(d.elements.get('manGoBtn').disabled,false);
 select.value='1';select.events.change.call(select);await d.poll();
 assert.equal(d.elements.get('manGoBtn').disabled,true);
 assert.equal(d.elements.get('stepTestBtn').disabled,true);
});

test('readback does not overwrite edits and disabled motor cannot be jogged', async () => {
  const d = dashboard(); await d.poll();
  d.elements.get('kpNum').value = '3.5';
  d.motors[0].kp = 2; d.motors[0].disabled = true;
  await d.poll();
  assert.equal(d.elements.get('kpNum').value, '3.5');
  assert.match(d.elements.get('liveGains').textContent, /2.00/);
  assert.equal(d.elements.get('manGoBtn').disabled, true);
  d.elements.get('pidReadBtn').click();
  assert.equal(d.elements.get('kpNum').value, '2.00');
});

test('jog toggle gates pulses without polarity-check capability', async () => {
  const d=dashboard();
  d.motors[0].jog_supported=true;
  await d.poll();
  assert.equal(d.elements.get('jogPlus').disabled,true);
  const toggle=d.elements.get('diagnosticsToggle');
  toggle.checked=true; toggle.events.change.call(toggle);
  assert.equal(d.requests.at(-1).url,'/api/diagnostics');
  assert.deepEqual(JSON.parse(d.requests.at(-1).options.body),{enabled:true});
  d.motors[0].diagnostics=true; await d.poll();
  assert.equal(d.elements.get('jogPlus').disabled,false);
  assert.equal(d.elements.has('directionCheck'),false);
});

test('open-loop jog sends a signed timed pulse for the selected motor', async () => {
  const d=dashboard();d.motors[0].jog_supported=true;d.motors[0].disabled=true;d.motors[0].diagnostics=true;
  await d.poll();
  assert.equal(d.elements.get('jogMinus').disabled,false);
  d.elements.get('jogMinus').click();
  const request=d.requests.at(-1);
  assert.equal(request.url,'/api/jog');
  assert.deepEqual(JSON.parse(request.options.body),{joint:1,pwm:-40,duration_ms:150});
  await new Promise(setImmediate);
  d.motors[0].jogging=true;await d.poll();
  assert.equal(d.elements.get('jogPlus').disabled,true);
  d.elements.get('jogStop').click();
  assert.equal(d.requests.at(-1).url,'/api/jog/stop');
});

test('default display conversion switches without changing count commands or edits', async () => {
 const d=dashboard(); await d.poll();
 assert.equal(d.elements.get('angleMinCount').value,8);
 assert.equal(d.elements.get('angleMaxCount').value,668);
 assert.equal(d.elements.get('angleMaxDegrees').value,270);
 d.elements.get('manPosition').value='520';
 d.elements.get('stepSize').value='12';
 const before=d.requests.length;
 const display=d.elements.get('displayUnit');
 display.value='degrees'; display.events.change.call(display);
 assert.equal(d.elements.get('j1Actual').textContent,'≈206.2°');
 assert.equal(d.elements.get('j1Actual').title,'512 counts');
 assert.equal(d.elements.get('manPosition').value,'520');
 assert.equal(d.elements.get('stepSize').value,'12');
 assert.equal(d.elements.get('motorMin').value,480);
 assert.equal(d.requests.length,before);
 d.elements.get('manGoBtn').click();
 assert.equal(d.requests.at(-1).url,'/api/target?joint=1&angle=520.0');
 d.elements.get('stepPlus5').click();
 assert.equal(d.requests.at(-1).url,'/api/target?joint=1&angle=517.0');
 d.elements.get('stepTestBtn').click();
 assert.match(d.requests.at(-1).url,/step=12$/);
 display.value='percent'; display.events.change.call(display);
 assert.equal(d.elements.get('j1Actual').textContent,'76.4%');
 display.value='counts'; display.events.change.call(display);
 assert.equal(d.elements.get('j1Actual').textContent,'512.0 counts');
});

test('display range edits and reset never send motor writes', async () => {
 const d=dashboard(); await d.poll(); const before=d.requests.length;
 d.elements.get('angleMinCount').value='668';
 d.elements.get('angleMaxCount').value='8';
 d.elements.get('angleApply').click();
 assert.equal(d.requests.length,before);
 assert.match(d.elements.get('commandStatus').textContent,/min count/);
 d.elements.get('angleMinCount').value='0';
 d.elements.get('angleMaxCount').value='1000';
 d.elements.get('angleMaxDegrees').value='180';
 d.elements.get('angleApply').click();
 const display=d.elements.get('displayUnit');display.value='degrees';display.events.change.call(display);
 assert.equal(d.elements.get('j1Actual').textContent,'≈92.2°');
 d.elements.get('angleReset').click();
 assert.equal(d.elements.get('j1Actual').textContent,'≈206.2°');
 assert.equal(d.requests.length,before);
});
test('secondary panels are collapsed and polarity UI is absent', () => {
 assert.match(html,/<details class="sidebar-section tuning-only" id="jogPanel">\s*<summary>Open-loop Jog/);
 assert.match(html,/<details class="sidebar-section tuning-only" id="limitsPanel">\s*<summary>Motor limits/);
 assert.doesNotMatch(html,/directionCheck|directionStatus|Pause holds|Power cutoff/);
});


test('reported stable preset only stages gains for the selected motor', async () => {
 const d=dashboard(); await d.poll(); const before=d.requests.length;
 d.elements.get('stablePreset').click();
 assert.equal(d.elements.get('kpNum').value,'20.00');
 assert.equal(d.elements.get('kiNum').value,'0.00');
 assert.equal(d.elements.get('kdNum').value,'0.40');
 assert.equal(d.requests.length,before);
 d.elements.get('pidSendBtn').click();
 assert.deepEqual(JSON.parse(d.requests.at(-1).options.body),{kp:20,ki:0,kd:0.4});
});


test('endpoint math, error scaling, extrapolation and saved display preference', () => {
 const saved=new Map();
 function load() {
  const context={window:{},localStorage:{getItem:key=>saved.get(key)??null,setItem:(key,value)=>saved.set(key,value)}};
  vm.runInNewContext(readFileSync(new URL('web/js/angles.js',import.meta.url),'utf8'),context);
  return context.window.Angles;
 }
 let a=load();assert.equal(a.mode(),'counts');a.setMode('degrees');
 assert.equal(a.value(1,8),0);assert.equal(a.value(1,668),270);
 assert.equal(a.value(1,338),135);
 assert.ok(Math.abs(a.value(1,4,true)-4*270/660)<1e-9);
 assert.ok(a.value(1,700)>270);
 a=load();assert.equal(a.mode(),'degrees');
 a.setMode('percent');assert.equal(a.value(1,338),50);
 a.set(2,100,500,-90,90);a.setMode('degrees');
 assert.equal(a.value(2,300),0);assert.equal(a.value(1,338),135);
 a=load();assert.equal(a.value(2,500),90);
 saved.set('simsync.angles',JSON.stringify({1:{zero:512,span:270}}));
 a=load();assert.equal(a.value(1,512),0);
 assert.ok(Math.abs(a.value(1,1023)-(1023-512)*270/1023)<1e-9);
});
