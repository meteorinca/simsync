/* Display calibration only: serial/API positions remain ADC counts. */
(function () {
  'use strict';
  var calibration = {};
  try { calibration = JSON.parse(localStorage.getItem('simsync.angles') || '{}') || {}; } catch (_) {}
  function valid(c) { return c && Number.isFinite(c.zero) && c.zero >= 0 && c.zero <= 1023 && Number.isFinite(c.span) && c.span > 0 && c.span <= 3600; }
  function get(j) { return valid(calibration[j]) ? calibration[j] : null; }
  window.Angles = {
    get: get,
    set: function (j, zero, span) {
      var c = {zero: zero, span: span};
      if (!valid(c)) throw new Error('Enter zero count (0–1023) and full-scale angle (>0°).');
      calibration[j] = c;
      try { localStorage.setItem('simsync.angles', JSON.stringify(calibration)); } catch (_) {}
    },
    unit: function (j) { return get(j) ? '°' : ' counts'; },
    value: function (j, count, delta) { var c=get(j); return c ? (count-(delta ? 0 : c.zero))*c.span/1023 : count; },
    count: function (j, value, delta) { var c=get(j); return c ? value*1023/c.span+(delta ? 0 : c.zero) : value; },
    format: function (j, count, delta) { return Number.isFinite(count) ? this.value(j,count,delta).toFixed(delta ? 2 : 1)+this.unit(j) : '—'; }
  };
})();
