/* hud.js — приборный HUD НСУ с возможностью перетаскивания мышкой */
(function () {
  'use strict';
  var W = 360, H = 470;
  var box = document.createElement('div');
  box.style.cssText = 'position:fixed;right:10px;top:70px;z-index:1000;' +
    'background:rgba(10,14,18,.55);border:1px solid rgba(120,180,255,.35);' +
    'border-radius:10px;padding:6px;backdrop-filter:blur(4px);' +
    'font:12px/1.3 "Segoe UI",Roboto,sans-serif;color:#cfe8ff;user-select:none;cursor:move';
  var cv = document.createElement('canvas');
  cv.width = W; cv.height = H;
  box.appendChild(cv);
  document.body.appendChild(box);
  var ctx = cv.getContext('2d');

  var T = null, lastEv = 0;
  var es = new EventSource('api/stream');
  es.addEventListener('telemetry', function (e) { T = JSON.parse(e.data); lastEv = Date.now(); });

  var CARD = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  function cardName(d) { return CARD[Math.round(((d % 360) + 360) % 360 / 45) % 8]; }
  // число или null: поля телеметрии могут отсутствовать (undefined), если борт ещё не прислал данные
  function num(v) { return (typeof v === 'number' && isFinite(v)) ? v : null; }
  function fmt(v, d, u) { return num(v) === null ? '--' : v.toFixed(d) + u; }

  function drawHeading(yaw, track) {
    var y = 26, h = 34, pxd = 2.6, cx = W / 2;
    ctx.save();
    ctx.beginPath(); ctx.rect(8, y - h / 2, W - 16, h); ctx.clip();
    ctx.fillStyle = 'rgba(0,0,0,.45)'; ctx.fillRect(8, y - h / 2, W - 16, h);
    for (var d = Math.floor(yaw - 80); d <= yaw + 80; d += 5) {
      var dd = ((d % 360) + 360) % 360, x = cx + (d - yaw) * pxd;
      if (d % 30 === 0) {
        ctx.strokeStyle = '#9fd4ff'; ctx.beginPath();
        ctx.moveTo(x, y + h / 2); ctx.lineTo(x, y + h / 2 - 10); ctx.stroke();
        ctx.fillStyle = '#dff1ff'; ctx.textAlign = 'center'; ctx.font = '11px sans-serif';
        ctx.fillText(dd % 90 === 0 ? cardName(dd) : String(dd), x, y - 2);
      } else {
        ctx.strokeStyle = 'rgba(159,212,255,.6)'; ctx.beginPath();
        ctx.moveTo(x, y + h / 2); ctx.lineTo(x, y + h / 2 - 5); ctx.stroke();
      }
    }
    if (track !== null && track !== undefined) {
      var dt = track - yaw; if (dt > 180) dt -= 360; if (dt < -180) dt += 360;
      if (Math.abs(dt) < 80) {
        var tx = cx + dt * pxd;
        ctx.fillStyle = '#7CFC9A'; ctx.beginPath();
        ctx.moveTo(tx, y + h / 2 - 1); ctx.lineTo(tx - 5, y + h / 2 + 7);
        ctx.lineTo(tx + 5, y + h / 2 + 7); ctx.closePath(); ctx.fill();
      }
    }
    ctx.restore();
    ctx.fillStyle = '#ffcc33'; ctx.beginPath();
    ctx.moveTo(cx, y + h / 2 + 2); ctx.lineTo(cx - 6, y + h / 2 + 10);
    ctx.lineTo(cx + 6, y + h / 2 + 10); ctx.closePath(); ctx.fill();
    ctx.fillStyle = 'rgba(0,0,0,.6)'; ctx.fillRect(cx - 40, y + h / 2 + 12, 80, 18);
    ctx.strokeStyle = '#ffcc33'; ctx.strokeRect(cx - 40, y + h / 2 + 12, 80, 18);
    ctx.fillStyle = '#ffe9a8'; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText(yaw.toFixed(0) + '\u00B0 ' + cardName(yaw), cx, y + h / 2 + 25);
  }

  function drawADI(roll, pitch) {
    var cx = W / 2, cy = 205, R = 92, pxdeg = 2.0;
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.clip();
    ctx.translate(cx, cy); ctx.rotate(-roll * Math.PI / 180); ctx.translate(0, pitch * pxdeg);
    ctx.fillStyle = '#1b6ea8'; ctx.fillRect(-2 * R, -4 * R, 4 * R, 4 * R);
    ctx.fillStyle = '#7a4a21'; ctx.fillRect(-2 * R, 0, 4 * R, 4 * R);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(-2 * R, 0); ctx.lineTo(2 * R, 0); ctx.stroke();
    ctx.lineWidth = 1; ctx.font = '10px sans-serif'; ctx.fillStyle = '#fff'; ctx.textAlign = 'center';
    for (var p = -60; p <= 60; p += 10) {
      if (p === 0) continue;
      var yy = p * pxdeg, wdt = (p % 20 === 0) ? 46 : 26;
      ctx.beginPath(); ctx.moveTo(-wdt, yy); ctx.lineTo(wdt, yy); ctx.stroke();
      ctx.fillText(String(Math.abs(p)), wdt + 14, yy + 3);
    }
    ctx.restore();
    ctx.save(); ctx.translate(cx, cy);
    ctx.strokeStyle = 'rgba(255,255,255,.8)'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(0, 0, R + 6, Math.PI * 1.15, Math.PI * 1.85); ctx.stroke();
    for (var a = -60; a <= 60; a += 15) {
      var r1 = R + (a % 30 === 0 ? 14 : 10), ang = (-90 + a) * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo(Math.cos(ang) * (R + 6), Math.sin(ang) * (R + 6));
      ctx.lineTo(Math.cos(ang) * r1, Math.sin(ang) * r1); ctx.stroke();
    }
    ctx.rotate(-roll * Math.PI / 180);
    ctx.fillStyle = '#ffcc33'; ctx.beginPath();
    ctx.moveTo(0, -R - 4); ctx.lineTo(-6, -R - 14); ctx.lineTo(6, -R - 14);
    ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.strokeStyle = '#ffcc33'; ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx - 42, cy); ctx.lineTo(cx - 14, cy); ctx.lineTo(cx - 14, cy + 8);
    ctx.moveTo(cx + 42, cy); ctx.lineTo(cx + 14, cy); ctx.lineTo(cx + 14, cy + 8);
    ctx.stroke();
    ctx.fillStyle = '#ffcc33'; ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 7); ctx.fill();
    ctx.strokeStyle = 'rgba(120,180,255,.35)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();
  }

  function boxText(x, y, label, value) {
    ctx.fillStyle = 'rgba(0,0,0,.55)'; ctx.fillRect(x, y, 92, 34);
    ctx.strokeStyle = 'rgba(120,180,255,.5)'; ctx.strokeRect(x, y, 92, 34);
    ctx.fillStyle = '#8fb8dd'; ctx.font = '10px sans-serif'; ctx.textAlign = 'left';
    ctx.fillText(label, x + 6, y + 12);
    ctx.fillStyle = '#eaf6ff'; ctx.font = 'bold 13px sans-serif';
    ctx.fillText(value, x + 6, y + 28);
  }

  function draw() {
    try { drawFrame(); } catch (e) { console.error('HUD:', e); }
    requestAnimationFrame(draw);              // цикл продолжается даже после ошибки в кадре
  }

  function drawFrame() {
    ctx.clearRect(0, 0, W, H);
    var alive = !!(T && T.available !== false && (Date.now() - lastEv < 3000));
    var yaw = alive && num(T.yaw) !== null ? T.yaw : 0;
    var roll = alive && num(T.roll) !== null ? T.roll : 0;
    var pitch = alive && num(T.pitch) !== null ? T.pitch : 0;
    var track = alive ? num(T.track) : null;
    drawHeading(yaw, track);
    drawADI(roll, pitch);
    if (!alive) {
      ctx.fillStyle = 'rgba(255,80,80,.95)'; ctx.font = 'bold 14px sans-serif';
      ctx.textAlign = 'center'; ctx.fillText('НЕТ ТЕЛЕМЕТРИИ', W / 2, 205);
    }
    var gs = alive && num(T.gs) !== null ? T.gs * 3.6 : null;
    var vs = alive && num(T.vd) !== null ? -T.vd : null;
    boxText(10, 322, 'ПУТ. СКОРОСТЬ', gs === null ? '--' : gs.toFixed(1) + ' км/ч');
    boxText(10, 362, 'ВЕРТ. СКОРОСТЬ', vs === null ? '--' : (vs >= 0 ? '+' : '') + vs.toFixed(1) + ' м/с');
    boxText(W - 102, 322, 'ВЫСОТА AGL', fmt(alive ? T.agl : null, 1, ' м'));
    boxText(W - 102, 362, 'ВЫСОТА AMSL', fmt(alive ? T.amsl : null, 0, ' м'));
    ctx.fillStyle = 'rgba(0,0,0,.55)'; ctx.fillRect(10, 404, W - 20, 58);
    ctx.strokeStyle = 'rgba(120,180,255,.5)'; ctx.strokeRect(10, 404, W - 20, 58);
    ctx.fillStyle = alive && T.fix >= 3 ? '#7CFC9A' : '#ff8080';
    ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'left';
    ctx.fillText((alive ? (T.fix >= 3 ? 'GPS: 3D FIX' : 'GPS: НЕТ ФИКСА') : 'GPS: --') +
                 '   СПУТНИКОВ: ' + (alive && num(T.sats) !== null ? T.sats : '--'), 18, 421);
    ctx.fillStyle = '#cfe8ff'; ctx.font = '11px monospace';
    ctx.fillText('LAT ' + fmt(alive ? T.lat : null, 6, '\u00B0'), 18, 439);
    ctx.fillText('LON ' + fmt(alive ? T.lon : null, 6, '\u00B0'), 18, 455);
    if (track !== null) {
      var drift = track - yaw; if (drift > 180) drift -= 360; if (drift < -180) drift += 360;
      ctx.fillStyle = '#9fd4ff'; ctx.textAlign = 'right'; ctx.font = '11px sans-serif';
      ctx.fillText('СНОС ' + (drift >= 0 ? '+' : '') + drift.toFixed(0) + '\u00B0', W - 18, 421);
    }
  }
  draw();

  // === DRAG & DROP функциональность ===
  var isDragging = false, offsetX = 0, offsetY = 0;
  box.addEventListener('mousedown', function (e) {
    isDragging = true;
    offsetX = e.clientX - box.offsetLeft;
    offsetY = e.clientY - box.offsetTop;
    box.style.cursor = 'grabbing';
  });
  document.addEventListener('mousemove', function (e) {
    if (!isDragging) return;
    var newX = e.clientX - offsetX;
    var newY = e.clientY - offsetY;
    // Ограничение границами окна
    newX = Math.max(0, Math.min(window.innerWidth - box.offsetWidth, newX));
    newY = Math.max(0, Math.min(window.innerHeight - box.offsetHeight, newY));
    box.style.left = newX + 'px';
    box.style.top = newY + 'px';
    box.style.right = 'auto';  // Сбрасываем right, чтобы left работал
  });
  document.addEventListener('mouseup', function () {
    isDragging = false;
    box.style.cursor = 'move';
  });
})();
