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

test('calibrated degrees round-trip to counts without changing host bounds', async () => {
 const d=dashboard(); await d.poll();
 d.elements.get('angleZero').value='512'; d.elements.get('angleSpan').value='270';
 d.elements.get('angleApply').click();
 assert.equal(d.elements.get('j1Actual').textContent,'0.0°');
 assert.equal(d.elements.get('stepPlus5').textContent,'+5°');
 assert.equal(d.elements.get('motorMin').value,480);
 d.elements.get('manPosition').value='10'; d.elements.get('manGoBtn').click();
 assert.equal(d.requests.at(-1).url,'/api/target?joint=1&angle=549.9');
 d.elements.get('stepMinus5').click();
 assert.equal(d.requests.at(-1).url,'/api/target?joint=1&angle=493.1');
 const select=d.elements.get('manJoint'); select.value='2'; select.events.change.call(select);
 assert.equal(d.elements.get('stepPlus5').textContent,'+5 counts');
});

test('invalid calibration cannot change commands or send motor writes', async () => {
 const d=dashboard(); await d.poll(); const before=d.requests.length;
 d.elements.get('angleZero').value='512'; d.elements.get('angleSpan').value='0';
 d.elements.get('angleApply').click();
 assert.equal(d.requests.length,before);
 assert.match(d.elements.get('commandStatus').textContent,/full-scale/);
 assert.equal(d.elements.get('stepPlus5').textContent,'+5 counts');
});
test('secondary panels are collapsed and polarity UI is absent', () => {
 assert.match(html,/<details class="sidebar-section">\s*<summary>Open-loop Jog/);
 assert.match(html,/<details class="sidebar-section">\s*<summary>Motor limits/);
 assert.doesNotMatch(html,/directionCheck|directionStatus|Pause holds|Power cutoff/);
});
