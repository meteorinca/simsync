/* Human-readable display only. All commands, bounds and CSV stay in counts. */
(function () {
  'use strict';
  var calibration={}, mode='counts';
  var defaults={min:8,max:668,angleMin:0,angleMax:270};
  function valid(c) {
    return c && Number.isFinite(c.min) && Number.isFinite(c.max) && c.min>=0 && c.max<=1023 && c.min<c.max && Number.isFinite(c.angleMin) && Number.isFinite(c.angleMax) && c.angleMax>c.angleMin && c.angleMax-c.angleMin<=3600;
  }
  try {
    var saved=JSON.parse(localStorage.getItem('simsync.angles') || '{}') || {};
    Object.keys(saved).forEach(function (j) {
      var c=saved[j];
      // Preserve the mapping of an explicitly saved legacy calibration.
      if(c && Number.isFinite(c.zero) && c.zero>=0 && c.zero<=1023 && Number.isFinite(c.span) && c.span>0 && c.span<=3600) {
        c={min:0,max:1023,angleMin:-c.zero*c.span/1023,angleMax:(1023-c.zero)*c.span/1023};
      }
      if(valid(c)) calibration[j]=c;
    });
    var preference=localStorage.getItem('simsync.displayUnit');
    if(['counts','degrees','percent'].includes(preference)) mode=preference;
  } catch (_) {}
  function get(j) { return Object.assign({},valid(calibration[j])?calibration[j]:defaults); }
  window.Angles={
    get:get,
    mode:function () {return mode;},
    setMode:function (value) {
      if(!['counts','degrees','percent'].includes(value)) throw new Error('Choose counts, approximate degrees or range percent.');
      mode=value;
      try {localStorage.setItem('simsync.displayUnit',mode);} catch (_) {}
    },
    set:function (j,min,max,angleMin,angleMax) {
      var c={min:min,max:max,angleMin:angleMin,angleMax:angleMax};
      if(!valid(c)) throw new Error('Require 0 ≤ min count < max count ≤ 1023 and an increasing angle range of up to 3600°.');
      calibration[j]=c;
      try {localStorage.setItem('simsync.angles',JSON.stringify(calibration));} catch (_) {}
    },
    reset:function (j) {delete calibration[j];try {localStorage.setItem('simsync.angles',JSON.stringify(calibration));} catch (_) {}},
    unit:function () {return mode==='degrees'?'°':mode==='percent'?'%':' counts';},
    value:function (j,count,delta) {
      if(mode==='counts') return count;
      var c=get(j), fraction=(count-(delta?0:c.min))/(c.max-c.min);
      return mode==='percent'?fraction*100:fraction*(c.angleMax-c.angleMin)+(delta?0:c.angleMin);
    },
    format:function (j,count,delta) {
      return Number.isFinite(count)?(mode==='degrees'?'≈':'')+this.value(j,count,delta).toFixed(delta?2:1)+this.unit():'—';
    }
  };
})();
