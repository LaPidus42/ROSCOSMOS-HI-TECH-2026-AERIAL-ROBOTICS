/* Веб-интерфейс обнаружения: карта (GPS) или план площадки (ArUco), объекты, журнал, видео.
   Данные приходят потоком SSE /api/stream. */
'use strict';

const $ = (id) => document.getElementById(id);
const MAX_LOG_ROWS = 500;
const MAX_TRACK = 3000;
const MAX_DOTS = 1500;

let cfg = null;
let LOCAL = false;                 // режим плана площадки (координаты X/Y в метрах)
let map, droneMarker, trackLine, footprint, dotsLayer, objectsLayer;
let follow = true;
let paused = false;
let clockOffset = 0;               // серверное время - время браузера, с
let lastTel = null;
let logCount = 0;
const objects = new Map();         // id -> {data, marker}

// план площадки
let plan = null;                   // состояние с сервера
let planOverlay = null, arucoLayer = null, gridLayer = null;

// ---------- форматирование ----------
const pad = (n, w = 2) => String(n).padStart(w, '0');
function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}
function fmtDateTime(ts) {
  const d = new Date(ts * 1000);
  return `${pad(d.getDate())}.${pad(d.getMonth() + 1)}.${d.getFullYear()} ${fmtTime(ts)}.${pad(d.getMilliseconds(), 3)}`;
}
function fmtAgo(ts) {
  const s = Math.max(0, serverNow() - ts);
  if (s < 60) return `${Math.round(s)} с назад`;
  if (s < 3600) return `${Math.floor(s / 60)} мин назад`;
  return `${Math.floor(s / 3600)} ч ${Math.floor((s % 3600) / 60)} мин назад`;
}
const num = (v, d = 2) => Number(v).toFixed(d).replace('-0.00', '0.00');
/** Координаты объекта/детекции в виде текста для текущего режима. */
function fmtPos(o) {
  return LOCAL ? `X ${num(o.x)}, Y ${num(o.y)} м` : `${o.lat.toFixed(6)}, ${o.lon.toFixed(6)}`;
}
const copyText = (o) => (LOCAL ? `${num(o.x, 3)}, ${num(o.y, 3)}` : `${o.lat.toFixed(7)}, ${o.lon.toFixed(7)}`);
/** Точка режима -> LatLng Leaflet. На плане широта = Y, долгота = X. */
const ll = (o) => (LOCAL ? [o.y, o.x] : [o.lat, o.lon]);
const llPair = (p) => (LOCAL ? [p[1], p[0]] : p);   // footprint: [x,y] или [lat,lon]
const hasPos = (o) => (LOCAL ? o.x != null && o.y != null : o.lat != null && o.lon != null);

const serverNow = () => Date.now() / 1000 + clockOffset;
const label = (cls) => (cfg.class_labels && cfg.class_labels[cls]) || cls;
const color = (cls) => (cfg.class_colors && cfg.class_colors[cls]) || '#9aa7b8';
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const getCss = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

function confBar(conf) {
  const p = Math.max(0, Math.min(1, conf)) * 100;
  return `<span class="confbar"><span class="track"><span class="knob" style="left:${p}%"></span></span>${(conf * 100).toFixed(0)}%</span>`;
}

// ---------- карта ----------
function initMap() {
  if (LOCAL) {
    map = L.map('map', { crs: L.CRS.Simple, minZoom: -3, maxZoom: 9, zoomSnap: 0.25, zoomDelta: 0.5, attributionControl: false });
    map.createPane('planPane').style.zIndex = 250;
    map.createPane('gridPane').style.zIndex = 300;
    map.createPane('arucoPane').style.zIndex = 350;
    map.getPane('gridPane').style.pointerEvents = 'none';
    map.fitBounds([[-5, -5], [5, 5]]);
    gridLayer = L.layerGroup().addTo(map);
    arucoLayer = L.layerGroup().addTo(map);
    map.on('moveend zoomend', drawGrid);
    map.on('zoomend', updateArucoLabels);
  } else {
    map = L.map('map', { preferCanvas: true }).setView(cfg.center, cfg.zoom);
  }

  const base = {};
  if (!LOCAL) {
    if (cfg.local_tiles) {
      base['Локальная карта'] = L.tileLayer('tiles/{z}/{x}/{y}.png', { maxZoom: cfg.tile_max_zoom, attribution: cfg.tile_attribution });
    }
    base['Схема'] = L.tileLayer(cfg.tile_url, { maxZoom: cfg.tile_max_zoom, attribution: cfg.tile_attribution });
    if (cfg.satellite_url) base['Спутник'] = L.tileLayer(cfg.satellite_url, { maxZoom: 19, attribution: 'Esri World Imagery' });
    Object.values(base)[0].addTo(map);
  }

  trackLine = L.polyline([], { color: getCss('--drone'), weight: 2, opacity: 0.7 }).addTo(map);
  footprint = L.polygon([], { color: getCss('--drone'), weight: 1, dashArray: '4 4', fillOpacity: 0.08 }).addTo(map);
  dotsLayer = L.layerGroup();
  objectsLayer = L.layerGroup().addTo(map);

  const overlays = {
    'Объекты': objectsLayer,
    'Все детекции (точки)': dotsLayer,
    'Трек БПЛА': trackLine,
    'Поле зрения камеры': footprint,
  };
  if (LOCAL) Object.assign(overlays, { 'Сетка (м)': gridLayer, 'Маркеры ArUco': arucoLayer });
  L.control.layers(LOCAL ? {} : base, overlays, { collapsed: true }).addTo(map);
  L.control.scale({ imperial: false }).addTo(map);

  droneMarker = L.marker([0, 0], {
    icon: L.divIcon({
      className: 'drone-icon', iconSize: [34, 34], iconAnchor: [17, 17],
      html: `<svg width="34" height="34" viewBox="-17 -17 34 34"><path d="M0,-15 L10,12 L0,6 L-10,12 Z" fill="${getCss('--drone')}" stroke="#fff" stroke-width="2" stroke-linejoin="round"/></svg>`,
    }),
    zIndexOffset: 1000, keyboard: false, interactive: false,
  });

  map.on('dragstart', () => setFollow(false));
  if (LOCAL) drawGrid();
}

function setFollow(on) {
  follow = on;
  $('btn-follow').setAttribute('aria-pressed', String(on));
  if (on && lastTel && lastTel.available && hasPos(lastTel)) map.panTo(ll(lastTel));
}

function buildLegend() {
  let rows = Object.keys(cfg.class_colors).map((c) =>
    `<div class="row"><span class="sw" style="background:${color(c)}"></span>${esc(label(c))}</div>`).join('');
  if (LOCAL) rows += '<div class="row" id="legend-aruco" hidden><span class="sw aruco"></span>Маркер ArUco</div>';
  $('legend').innerHTML = rows +
    `<div class="row stack"><span class="cap">Уверенность модели</span>
       <span class="scale"></span><span class="scale-labels"><span>0%</span><span>100%</span></span></div>`;
  const sel = $('f-class');
  for (const c of Object.keys(cfg.class_colors)) {
    sel.insertAdjacentHTML('beforeend', `<option value="${esc(c)}">${esc(label(c))}</option>`);
  }
}

// ---------- сетка плана (метры) ----------
const GRID_STEPS = [0.05, 0.1, 0.25, 0.5, 1, 2, 5, 10, 25, 50, 100, 250];
function drawGrid() {
  if (!gridLayer) return;
  gridLayer.clearLayers();
  const pxPerM = Math.pow(2, map.getZoom());
  const step = GRID_STEPS.find((s) => s * pxPerM >= 45) || 250;
  const b = map.getBounds();
  const x0 = Math.floor(b.getWest() / step) * step, x1 = Math.ceil(b.getEast() / step) * step;
  const y0 = Math.floor(b.getSouth() / step) * step, y1 = Math.ceil(b.getNorth() / step) * step;
  if ((x1 - x0) / step > 200 || (y1 - y0) / step > 200) return;
  const line = getCss('--grid'), axis = getCss('--grid-axis');
  const dec = step < 0.1 ? 2 : step < 1 ? 2 : 0;
  const lab = (text, latlng, cls) => L.marker(latlng, {
    pane: 'gridPane', interactive: false, keyboard: false,
    icon: L.divIcon({ className: `grid-label ${cls}`, html: `<span>${text}</span>`, iconSize: null }),
  }).addTo(gridLayer);
  for (let x = x0; x <= x1 + 1e-9; x += step) {
    const isAxis = Math.abs(x) < 1e-9;
    L.polyline([[y0, x], [y1, x]], { pane: 'gridPane', color: isAxis ? axis : line, weight: isAxis ? 2 : 1, interactive: false }).addTo(gridLayer);
    lab(`${num(x, dec)}`, [b.getSouth(), x], 'bottom');
  }
  for (let y = y0; y <= y1 + 1e-9; y += step) {
    const isAxis = Math.abs(y) < 1e-9;
    L.polyline([[y, x0], [y, x1]], { pane: 'gridPane', color: isAxis ? axis : line, weight: isAxis ? 2 : 1, interactive: false }).addTo(gridLayer);
    lab(`${num(y, dec)}`, [y, b.getWest()], 'left');
  }
  if (b.contains([0, 0])) lab('0', [0, 0], 'origin');
}

// ---------- телеметрия ----------
function onTelemetry(t) {
  lastTel = t;
  if (!t.available) return;
  clockOffset = t.ts - Date.now() / 1000;

  if (LOCAL) {
    const fresh = t.x != null && t.pos_age != null && t.pos_age < 1.5;
    // позиция есть, но маркеры давно не видны: дрон держится по оптическому потоку, копится снос
    const blind = fresh && t.ref_age != null && t.ref_age > 3;
    setReadout('st-pos', t.x == null ? 'нет данных'
      : !fresh ? 'позиция устарела'
      : blind ? `X ${num(t.x)}  Y ${num(t.y)} (без маркеров ${Math.round(t.ref_age)} с)`
      : `X ${num(t.x)}  Y ${num(t.y)}`, !fresh ? 'bad' : blind ? 'warn' : 'ok');
    setReadout('st-agl', t.agl == null ? 'нет данных' : `${num(t.agl)} м`, t.agl == null ? 'bad' : '');
  } else {
    const fixOk = t.fix >= 3;
    setReadout('st-pos', fixOk ? `3D, ${t.sats ?? '?'} спутн.` : `нет фикса (${t.sats ?? 0})`, fixOk ? 'ok' : 'bad');
    setReadout('st-agl', t.agl == null ? 'нет данных' : `${t.agl.toFixed(1)} м`, t.agl == null ? 'bad' : '');
  }
  setReadout('st-yaw', `${Math.round(t.yaw)}°`);
  setReadout('st-gs', t.gs == null ? '—' : LOCAL ? `${t.gs.toFixed(1)} м/с` : `${(t.gs * 3.6).toFixed(0)} км/ч`);
  const geoOk = t.geo_status === 'ok';
  setReadout('st-geo', geoOk ? 'работает' : (cfg.geo_status_text[t.geo_status] || t.geo_status), geoOk ? 'ok' : 'warn');

  if (!hasPos(t) || (!LOCAL && t.fix < 3)) return;
  const p = ll(t);
  if (!map.hasLayer(droneMarker)) droneMarker.addTo(map);
  droneMarker.setLatLng(p);
  const svg = droneMarker.getElement() && droneMarker.getElement().querySelector('svg');
  if (svg) svg.style.transform = `rotate(${t.yaw}deg)`;

  const pts = trackLine.getLatLngs();
  const last = pts[pts.length - 1];
  if (!last || map.distance(last, p) > (LOCAL ? 0.05 : 1.5)) {
    trackLine.addLatLng(p);
    if (pts.length > MAX_TRACK) trackLine.setLatLngs(pts.slice(-MAX_TRACK));
  }
  footprint.setLatLngs((t.footprint || []).map(llPair));
  if (follow) map.panTo(p, { animate: false });
}

function setReadout(id, text, cls = '') {
  const el = $(id);
  el.textContent = text;
  el.className = cls;
}

// ---------- объекты ----------
function objIcon(o) {
  const stale = serverNow() - o.last_ts > cfg.stale_after_s;
  return L.divIcon({
    className: '', iconSize: [22, 22], iconAnchor: [11, 11],
    html: `<div class="obj-icon ${esc(o.cls)}${stale ? ' stale' : ''}" style="--cls:${color(o.cls)}"></div>`,
  });
}

function objPopup(o) {
  return `<div class="popup">
    <h3>${esc(label(o.cls))} №${o.id}</h3>
    <dl>
      <dt>Первая детекция</dt><dd>${fmtDateTime(o.first_ts)}</dd>
      <dt>Последняя</dt><dd>${fmtDateTime(o.last_ts)} (${fmtAgo(o.last_ts)})</dd>
      <dt>Координаты</dt><dd>${fmtPos(o)}</dd>
      <dt>Макс. уверенность</dt><dd>${confBar(o.max_conf)}</dd>
      <dt>Детекций</dt><dd>${o.hits}</dd>
    </dl>
    <button class="tool small" data-copy="${copyText(o)}">Копировать координаты</button>
  </div>`;
}

function upsertObject(o, announce) {
  if (!hasPos(o)) return;
  let rec = objects.get(o.id);
  if (!rec) {
    const marker = L.marker(ll(o), { icon: objIcon(o), title: `${label(o.cls)} №${o.id}` })
      .bindPopup('', { maxWidth: 320 });
    marker.on('popupopen', () => marker.setPopupContent(objPopup(objects.get(o.id).data)));
    marker.addTo(objectsLayer);
    rec = { data: o, marker };
    objects.set(o.id, rec);
  } else {
    rec.data = Object.assign(rec.data, o);
    rec.marker.setLatLng(ll(rec.data));
    rec.marker.setIcon(objIcon(rec.data));
    if (rec.marker.isPopupOpen()) rec.marker.setPopupContent(objPopup(rec.data));
  }
  renderObjects(announce ? o.id : null);
  if (announce) toast(rec.data);
}

function renderObjects(freshId) {
  const list = [...objects.values()].map((r) => r.data).sort((a, b) => b.last_ts - a.last_ts);
  $('n-objects').textContent = list.length;
  $('objects-empty').hidden = list.length > 0;
  const byCls = {};
  for (const o of list) byCls[o.cls] = (byCls[o.cls] || 0) + 1;
  $('st-objects').textContent = Object.keys(byCls).length
    ? Object.entries(byCls).map(([c, n]) => `${label(c).split(' ')[0]}: ${n}`).join(', ') : '0';

  $('objects').innerHTML = list.map((o) => {
    const stale = serverNow() - o.last_ts > cfg.stale_after_s;
    return `<li tabindex="0" data-id="${o.id}" style="--cls:${color(o.cls)}" class="${stale ? 'stale' : ''}${o.id === freshId ? ' fresh' : ''}">
      <span class="sw"></span>
      <span class="title">${esc(label(o.cls))} №${o.id}</span>
      <span class="when">${fmtTime(o.last_ts)}<br>${fmtAgo(o.last_ts)}</span>
      <span class="meta"><span>${fmtPos(o)}</span><span>обнаружен ${fmtTime(o.first_ts)}</span><span>${o.hits} дет.</span></span>
      <span class="conf">${confBar(o.max_conf)}</span>
    </li>`;
  }).join('');
}

function focusObject(id) {
  const rec = objects.get(Number(id));
  if (!rec) return;
  setFollow(false);
  map.flyTo(rec.marker.getLatLng(), Math.max(map.getZoom(), LOCAL ? 6 : 17), { duration: 0.6 });
  rec.marker.openPopup();
}

function toast(o) {
  const el = document.createElement('div');
  el.className = 'toast';
  el.style.setProperty('--cls', color(o.cls));
  el.innerHTML = `<strong>Новый объект: ${esc(label(o.cls))} №${o.id}</strong>
    <span>${fmtTime(o.first_ts)}, ${fmtPos(o)}, ${(o.max_conf * 100).toFixed(0)}%</span>`;
  el.onclick = () => { focusObject(o.id); el.remove(); };
  $('toasts').prepend(el);
  setTimeout(() => el.remove(), 8000);
}

function fitObjects() {
  const pts = [...objects.values()].map((r) => r.marker.getLatLng());
  if (!pts.length) return;
  setFollow(false);
  map.fitBounds(L.latLngBounds(pts).pad(0.3), { maxZoom: LOCAL ? 7 : 18 });
}

// ---------- журнал ----------
function logFilterPass(d) {
  const cls = $('f-class').value;
  if (cls && d.cls !== cls) return false;
  if ($('f-located').checked && d.geo_status !== 'ok') return false;
  return true;
}

function logRow(d) {
  const tr = document.createElement('tr');
  const ok = d.geo_status === 'ok' && hasPos(d);
  tr.className = ok ? 'located' : 'nogeo';
  tr.style.setProperty('--cls', color(d.cls));
  if (ok) { const p = ll(d); tr.dataset.lat = p[0]; tr.dataset.lon = p[1]; }
  const why = cfg.geo_status_text[d.geo_status] || d.geo_status;
  tr.title = `${fmtDateTime(d.ts)}${d.obj_id ? `, объект №${d.obj_id}` : ''}`;
  tr.innerHTML = `<td>${fmtTime(d.ts)}</td>
    <td><span class="cls" title="${esc(label(d.cls))}">${esc(label(d.cls).split(' ')[0])}</span></td>
    <td>${confBar(d.conf)}</td>
    <td class="coords">${ok ? fmtPos(d) : esc(why)}</td>
    <td>${d.ground_dist == null ? '' : LOCAL ? num(d.ground_dist, 1) + ' м' : Math.round(d.ground_dist) + ' м'}</td>`;
  return tr;
}

function addLogRows(rows, prepend) {
  const body = $('log');
  const frag = document.createDocumentFragment();
  for (const d of rows) if (logFilterPass(d)) frag.appendChild(logRow(d));
  if (prepend) body.prepend(frag); else body.appendChild(frag);
  while (body.rows.length > MAX_LOG_ROWS) body.deleteRow(-1);
}

function addDot(d) {
  if (d.geo_status !== 'ok' || !hasPos(d)) return;
  L.circleMarker(ll(d), {
    radius: 3, weight: 0, fillOpacity: 0.35 + 0.6 * d.conf, fillColor: color(d.cls), interactive: false,
  }).addTo(dotsLayer);
  const layers = dotsLayer.getLayers();
  if (layers.length > MAX_DOTS) dotsLayer.removeLayer(layers[0]);
}

function onDetection(d) {
  logCount += 1;
  $('n-log').textContent = logCount;
  addDot(d);
  if (!paused && $('f-session').value === 'current') addLogRows([d], true);
}

async function reloadLog() {
  const p = new URLSearchParams({ limit: MAX_LOG_ROWS, session: $('f-session').value });
  if ($('f-class').value) p.set('cls', $('f-class').value);
  if ($('f-located').checked) p.set('located', '1');
  const rows = await (await fetch(`api/events?${p}`)).json();
  $('log').innerHTML = '';
  addLogRows(rows, false);
  const csv = new URLSearchParams({ session: $('f-session').value });
  if ($('f-class').value) csv.set('cls', $('f-class').value);
  $('btn-csv').href = `api/export.csv?${csv}`;
}

// ---------- видео ----------
function buildVideos() {
  const box = $('videos');
  for (const s of cfg.video_streams || []) {
    const d = document.createElement('details');
    d.className = 'video';
    d.innerHTML = `<summary>${esc(s.label)}</summary>
      <div class="video-frame">
        <img alt="${esc(s.label)}">
        <p class="video-wait">Ожидание кадров…</p>
        <button class="tool small video-big" aria-pressed="false">Крупно</button>
      </div>`;
    const img = d.querySelector('img');
    const wait = d.querySelector('.video-wait');
    let poll = null;
    d.addEventListener('toggle', () => {
      clearInterval(poll);
      if (d.open) {
        // поток идёт, только пока панель открыта — экономим канал и CPU на борту
        img.src = `video/${encodeURIComponent(s.name)}.mjpg?t=${Date.now()}`;
        wait.hidden = false;
        wait.textContent = 'Ожидание кадров…';
        const started = Date.now();
        // MJPEG в <img> не сообщает о кадрах, поэтому спрашиваем у сервера, когда пришёл последний
        poll = setInterval(async () => {
          try {
            const st = (await (await fetch('api/video_status')).json())[s.name];
            const fresh = st && st.age != null && st.age < 3;
            wait.hidden = fresh;
            if (!fresh && Date.now() - started > 6000) {
              wait.textContent = st && st.age != null
                ? `Кадры не приходят ${Math.round(st.age)} с. Камера или её нода остановлены?`
                : 'Кадров нет. Проверьте, что камера публикует топик, указанный в настройках.';
            }
          } catch (_) { /* сервер недоступен — сообщит индикатор связи */ }
        }, 1000);
      } else {
        img.removeAttribute('src');
        d.classList.remove('big');
      }
    });
    const bigBtn = d.querySelector('.video-big');
    bigBtn.onclick = () => {
      const on = !d.classList.contains('big');
      document.querySelectorAll('.video.big').forEach((x) => { x.classList.remove('big'); x.querySelector('.video-big').setAttribute('aria-pressed', 'false'); });
      d.classList.toggle('big', on);
      bigBtn.setAttribute('aria-pressed', String(on));
      bigBtn.textContent = on ? 'Свернуть' : 'Крупно';
    };
    box.appendChild(d);
  }
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') document.querySelectorAll('.video.big .video-big').forEach((b) => b.click());
  });
}

// ---------- план площадки ----------
function planImageBounds(s) {
  if (!s.image || !s.image_w) return null;
  const h = s.width_m * s.image_h / s.image_w;
  return [[s.y0, s.x0], [s.y0 + h, s.x0 + s.width_m]];
}

function applyPlan(s, fromServer) {
  plan = s;
  // изображение
  const bounds = planImageBounds(s);
  if (bounds) {
    if (!planOverlay || planOverlay._url !== s.image_url) {
      if (planOverlay) map.removeLayer(planOverlay);
      planOverlay = L.imageOverlay(s.image_url, bounds, { pane: 'planPane', opacity: s.opacity, interactive: false }).addTo(map);
    } else {
      planOverlay.setBounds(L.latLngBounds(bounds));
      planOverlay.setOpacity(s.opacity);
    }
  } else if (planOverlay) {
    map.removeLayer(planOverlay);
    planOverlay = null;
  }
  // маркеры
  arucoLayer.clearLayers();
  for (const m of s.markers || []) {
    const h = m.length / 2, c = Math.cos(m.rot_z || 0), sn = Math.sin(m.rot_z || 0);
    const corners = [[-h, -h], [h, -h], [h, h], [-h, h]].map(([dx, dy]) => [m.y + sn * dx + c * dy, m.x + c * dx - sn * dy]);
    L.polygon(corners, { pane: 'arucoPane', color: '#fff', weight: 1, fillColor: '#000', fillOpacity: 0.8, interactive: true })
      .bindTooltip(`Маркер ${m.id}: X ${num(m.x)}, Y ${num(m.y)} м, размер ${num(m.length)} м`, { sticky: true })
      .addTo(arucoLayer);
    L.marker([m.y, m.x], {
      pane: 'arucoPane', interactive: false, keyboard: false,
      icon: L.divIcon({ className: 'aruco-id', html: `<span>${m.id}</span>`, iconSize: null }),
    }).addTo(arucoLayer);
  }
  updateArucoLabels();
  const legendAruco = $('legend-aruco');
  if (legendAruco) legendAruco.hidden = !(s.markers && s.markers.length);

  // поля формы (не трогаем поле, которое сейчас редактируют)
  const setField = (id, v) => { const el = $(id); if (document.activeElement !== el) el.value = v; };
  const r3 = (v) => String(Math.round(v * 1000) / 1000);
  setField('plan-width', r3(s.width_m));
  setField('plan-x0', r3(s.x0));
  setField('plan-y0', r3(s.y0));
  setField('plan-opacity', s.opacity);
  $('plan-fields').classList.toggle('disabled', !s.image);
  $('plan-clear').hidden = !s.image;
  $('plan-ros').hidden = !s.ros_available;
  $('plan-fit-row').hidden = !(s.image && s.markers && s.markers.length);
  if (s.ros_margin != null && document.activeElement !== $('plan-margin') && !$('plan-margin').dataset.touched) {
    $('plan-margin').value = s.ros_margin;
  }
  $('markers-clear').hidden = !(s.markers && s.markers.length);
  $('plan-info').textContent = s.image
    ? `${s.image_w}×${s.image_h} пикс., на плане ${num(s.width_m)} × ${num(s.width_m * s.image_h / s.image_w)} м (${num(s.image_w / s.width_m, 0)} пикс./м).`
    : 'Изображение не загружено: на плане только сетка и маркеры.';
  $('markers-info').textContent = s.markers && s.markers.length
    ? `${s.markers_name ? s.markers_name + ': ' : ''}${s.markers.length} маркеров, X от ${num(Math.min(...s.markers.map((m) => m.x)))} до ${num(Math.max(...s.markers.map((m) => m.x)))}, Y от ${num(Math.min(...s.markers.map((m) => m.y)))} до ${num(Math.max(...s.markers.map((m) => m.y)))} м.`
    : 'Карта маркеров не загружена.';
  if (!fromServer) return;
}

function updateArucoLabels() {
  if (!arucoLayer || !plan) return;
  const len = (plan.markers && plan.markers.length) ? plan.markers[0].length : 0;
  map.getContainer().classList.toggle('aruco-small', len * Math.pow(2, map.getZoom()) < 26);
}

function planExtent() {
  const b = L.latLngBounds([]);
  const ib = plan && planImageBounds(plan);
  if (ib) b.extend(ib);
  for (const m of (plan && plan.markers) || []) b.extend([[m.y - m.length, m.x - m.length], [m.y + m.length, m.x + m.length]]);
  return b;
}

let saveTimer = null;
function savePlanConfigSoon() {
  clearTimeout(saveTimer);
  $('plan-save').textContent = 'Изменения не сохранены';
  saveTimer = setTimeout(async () => {
    const body = {
      width_m: parseFloat($('plan-width').value), x0: parseFloat($('plan-x0').value),
      y0: parseFloat($('plan-y0').value), opacity: parseFloat($('plan-opacity').value),
    };
    if (!(body.width_m > 0) || Number.isNaN(body.x0) || Number.isNaN(body.y0)) {
      $('plan-save').textContent = 'Ширина должна быть больше 0, координаты — числами.';
      return;
    }
    const r = await postPlan('api/localmap/config', JSON.stringify(body), 'application/json');
    if (r) $('plan-save').textContent = 'Положение сохранено на борту';
  }, 600);
}

async function postPlan(url, body, ctype) {
  try {
    const resp = await fetch(url, { method: 'POST', headers: { 'Content-Type': ctype }, body });
    const data = await resp.json();
    if (!resp.ok) throw new Error(data.error || resp.statusText);
    if (data.state) applyPlan(data.state, true);
    return data;
  } catch (e) {
    $('plan-save').textContent = `Не сохранено: ${e.message}`;
    return null;
  }
}

function initPlanUi() {
  $('tab-plan').hidden = false;
  const live = () => {
    if (!plan) return;
    const s = Object.assign({}, plan, {
      width_m: parseFloat($('plan-width').value) || plan.width_m,
      x0: parseFloat($('plan-x0').value) || 0,
      y0: parseFloat($('plan-y0').value) || 0,
      opacity: parseFloat($('plan-opacity').value) || plan.opacity,
    });
    applyPlan(s, false);
    savePlanConfigSoon();
  };
  ['plan-width', 'plan-x0', 'plan-y0', 'plan-opacity'].forEach((id) => $(id).addEventListener('input', live));

  $('plan-file').addEventListener('change', async (e) => {
    const f = e.target.files[0];
    e.target.value = '';
    if (!f) return;
    $('plan-save').textContent = 'Загрузка изображения…';
    const first = !plan.image;
    const r = await postPlan('api/localmap/image', f, f.type || 'application/octet-stream');
    if (!r) return;
    $('plan-save').textContent = 'Изображение загружено';
    if (first) map.fitBounds(planExtent().pad(0.1));
  });
  $('plan-clear').onclick = async () => {
    if (confirm('Убрать изображение площадки?')) await postPlan('api/localmap/image/delete', '', 'text/plain');
  };
  $('markers-file').addEventListener('change', async (e) => {
    const f = e.target.files[0];
    e.target.value = '';
    if (!f) return;
    const r = await postPlan(`api/localmap/markers?name=${encodeURIComponent(f.name)}`, await f.text(), 'text/plain; charset=utf-8');
    if (!r) return;
    $('plan-save').textContent = `Загружено маркеров: ${r.count}` + (r.warnings.length ? `, пропущено строк: ${r.warnings.length} (${r.warnings[0]})` : '');
    map.fitBounds(planExtent().pad(0.1));
  });
  $('plan-from-ros').onclick = async (e) => {
    const b = e.currentTarget;
    b.disabled = true;
    $('plan-save').textContent = 'Получаю карту с дрона…';
    const r = await postPlan('api/localmap/from_ros', '', 'text/plain');
    b.disabled = false;
    if (!r) return;
    $('plan-save').textContent = `Карта с дрона загружена: ${r.state.markers.length} маркеров. Проверьте, что нарисованные маркеры легли на изображение.`;
    map.fitBounds(planExtent().pad(0.1));
  };
  $('plan-margin').addEventListener('input', (e) => { e.target.dataset.touched = '1'; });
  $('plan-fit').onclick = async () => {
    const m = parseFloat($('plan-margin').value) || 0;
    const r = await postPlan(`api/localmap/fit?margin=${m}`, '', 'text/plain');
    if (r) $('plan-save').textContent = 'Изображение выставлено по маркерам';
  };
  $('markers-clear').onclick = async () => {
    if (confirm('Убрать карту маркеров?')) await postPlan('api/localmap/markers/delete', '', 'text/plain');
  };
}

// ---------- поток событий ----------
function connect() {
  const es = new EventSource('api/stream');
  es.onopen = () => { $('st-link').className = 'dot warn'; };
  es.onerror = () => { $('st-link').className = 'dot'; $('st-link').title = 'Нет связи с сервером, переподключение…'; };
  es.addEventListener('telemetry', (e) => {
    const t = JSON.parse(e.data);
    const fresh = t.available && t.rx_age != null && t.rx_age < 2;
    $('st-link').className = fresh ? 'dot ok' : 'dot warn';
    $('st-link').title = fresh ? 'Телеметрия поступает' : 'Сервер на связи, телеметрии нет';
    onTelemetry(t);
  });
  es.addEventListener('detection', (e) => onDetection(JSON.parse(e.data)));
  es.addEventListener('object', (e) => {
    const o = JSON.parse(e.data);
    upsertObject(o, o.is_new);
  });
  es.addEventListener('localmap', (e) => { if (LOCAL) applyPlan(JSON.parse(e.data), true); });
}

// ---------- запуск ----------
async function main() {
  cfg = await (await fetch('api/config')).json();
  LOCAL = cfg.mode === 'local';
  document.body.classList.toggle('local', LOCAL);
  $('st-vehicle').textContent = cfg.vehicle || 'Тепловизионная разведка';
  $('st-mode').textContent = LOCAL ? 'Навигация по маркерам' : 'Навигация по GPS';
  $('st-pos-label').textContent = LOCAL ? 'Позиция, м' : 'GPS';
  $('st-agl-label').textContent = LOCAL ? 'Высота' : 'Высота AGL';
  document.title = `${cfg.vehicle || 'БПЛА'}: обнаружение возгораний`;

  initMap();
  buildLegend();
  buildVideos();

  if (LOCAL) {
    initPlanUi();
    const s = await (await fetch('api/localmap')).json();
    if (s) {
      applyPlan(s, true);
      const ext = planExtent();
      if (ext.isValid()) map.fitBounds(ext.pad(0.1));
    }
  }

  const state = await (await fetch('api/state')).json();
  if (state.telemetry) onTelemetry(state.telemetry);
  logCount = state.stats.detections;
  $('n-log').textContent = logCount;

  const objs = await (await fetch('api/objects')).json();
  objs.forEach((o) => upsertObject(o, false));
  if (objs.length && !(state.telemetry && hasPos(state.telemetry))) fitObjects();

  const recent = await (await fetch('api/events?located=1&limit=' + MAX_DOTS)).json();
  recent.reverse().forEach(addDot);
  await reloadLog();
  connect();

  // интерфейс
  $('btn-follow').onclick = () => setFollow(!follow);
  $('btn-fit').onclick = fitObjects;
  $('btn-pause').onclick = (e) => {
    paused = !paused;
    e.currentTarget.setAttribute('aria-pressed', String(paused));
    e.currentTarget.textContent = paused ? 'Продолжить' : 'Пауза';
    if (!paused) reloadLog();
  };
  ['f-class', 'f-session', 'f-located'].forEach((id) => $(id).addEventListener('change', reloadLog));

  $('objects').addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (li) focusObject(li.dataset.id);
  });
  $('objects').addEventListener('keydown', (e) => {
    const li = e.target.closest('li[data-id]');
    if (li && (e.key === 'Enter' || e.key === ' ')) { e.preventDefault(); focusObject(li.dataset.id); }
  });
  $('log').addEventListener('click', (e) => {
    const tr = e.target.closest('tr.located');
    if (!tr) return;
    setFollow(false);
    const p = [Number(tr.dataset.lat), Number(tr.dataset.lon)];
    map.flyTo(p, Math.max(map.getZoom(), LOCAL ? 6 : 17), { duration: 0.5 });
    const ping = L.circleMarker(p, { radius: 14, color: getCss('--ink'), weight: 2, fill: false }).addTo(map);
    setTimeout(() => map.removeLayer(ping), 2500);
  });
  document.addEventListener('click', (e) => {
    const b = e.target.closest('[data-copy]');
    if (!b) return;
    navigator.clipboard?.writeText(b.dataset.copy).then(() => { b.textContent = 'Скопировано'; });
  });

  document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
    document.querySelectorAll('.tabs button').forEach((x) => x.setAttribute('aria-selected', String(x === b)));
    for (const name of ['objects', 'log', 'plan']) $(`pane-${name}`).hidden = b.dataset.tab !== name;
  }));

  // «устаревание» объектов и подписи «N с назад»
  setInterval(() => {
    for (const r of objects.values()) r.marker.setIcon(objIcon(r.data));
    renderObjects(null);
  }, 5000);
}

main().catch((e) => {
  document.body.insertAdjacentHTML('afterbegin',
    `<p style="padding:12px;background:var(--bad);color:#fff;margin:0">Не удалось загрузить интерфейс: ${esc(e.message)}. Проверьте, что запущена нода geo_dashboard_node.</p>`);
});
