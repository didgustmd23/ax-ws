// 관제 화면 렌더러(시안 b). 입력은 서버 /api/state(판정 완료) 하나이며 여기서는 표시만 한다.
// 값 표시는 display_status(표시 여유 적용)를 따르고, 판정 status가 다르면 신선도 점에 표시 여유 테두리를 붙인다.
// 차트·아이콘은 SVG를 직접 그린다(외부 라이브러리 없음).
'use strict';

const FRESH = {ok: '정상', stale: '만료', missing: '미수집', invalid: '무효', no_source: '원천 없음'};
const ITEM = {actuation: '구동', safety: '안전', health: '위치 신뢰', executor: '로컬 작업', pose: '위치', battery: '배터리'};
// 상태값 → [상태색 클래스, 아이콘 모양]
const STATE_STYLE = {
  RUNNING: ['st-ok', 'play'], STOPPED: ['st-info', 'square'], STOPPING: ['st-warn', 'tri'], STOP_REQUESTED: ['st-warn', 'tri'],
  FAULT: ['st-bad', 'oct'], UNKNOWN: ['st-missing', 'ring'], OK: ['st-ok', 'check'], DEGRADED: ['st-warn', 'tri'], LOST: ['st-bad', 'oct'],
  CLEAR: ['st-ok', 'check'], SLOWDOWN_REQUIRED: ['st-warn', 'tri'], STOP_REQUIRED: ['st-bad', 'oct'],
  UNSPECIFIED: ['st-missing', 'ring'], QUEUED: ['st-info', 'ring'], ASSIGNED: ['st-info', 'square'], COMPLETED: ['st-ok', 'check'],
  FAILED: ['st-bad', 'oct'], CANCELED: ['st-missing', 'oct'], RETRY_PENDING: ['st-warn', 'tri'], LOCKED: ['st-warn', 'lock'],
  IN_PROGRESS: ['st-warn', 'play'], UNLOCKED: ['st-ok', 'check'], NONE: ['st-missing', 'ring'],
  SUCCEEDED: ['st-ok', 'check'], TIMED_OUT: ['st-bad', 'oct'], UNCERTAIN: ['st-warn', 'tri'],
};
const ICON = {
  play: '<path d="M3 2l7 4-7 4z" fill="currentColor"/>',
  square: '<rect x="2.5" y="2.5" width="7" height="7" fill="currentColor"/>',
  tri: '<path d="M6 1.5l5 9H1z" fill="currentColor"/>',
  oct: '<path d="M4 1h4l3 3v4l-3 3H4L1 8V4z" fill="currentColor"/>',
  check: '<circle cx="6" cy="6" r="5" fill="currentColor"/><path d="M3.5 6.2l1.8 1.8 3.4-3.6" stroke="#0008" stroke-width="1.6" fill="none"/>',
  ring: '<circle cx="6" cy="6" r="4.5" stroke="currentColor" stroke-width="1.5" fill="none" stroke-dasharray="2 1.5"/>',
  lock: '<rect x="2" y="5" width="8" height="6" rx="1" fill="currentColor"/><path d="M4 5V3.5a2 2 0 014 0V5" stroke="currentColor" fill="none" stroke-width="1.4"/>',
  clock: '<circle cx="6" cy="6" r="4.5" stroke="currentColor" fill="none" stroke-width="1.4"/><path d="M6 3.5V6l2 1.3" stroke="currentColor" fill="none" stroke-width="1.4"/>',
};
const ROLE_ICON = {
  MANIPULATOR: '<svg viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><rect x="3" y="15" width="8" height="3" rx="1" fill="currentColor"/><path d="M7 15V10l6-4 3 3"/><circle cx="7" cy="10" r="1.3" fill="currentColor"/><circle cx="13" cy="6" r="1.3" fill="currentColor"/></svg>',
  TRANSPORTER: '<svg viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="1.8"><rect x="2" y="6" width="13" height="8" rx="1.5"/><path d="M15 9h2.5l1 2.5V14H15"/><circle cx="6" cy="15.5" r="1.6" fill="currentColor"/><circle cx="14" cy="15.5" r="1.6" fill="currentColor"/></svg>',
};
const ROLE_LABEL = {MANIPULATOR: '조작', TRANSPORTER: '운송'};
const TERMINAL = ['COMPLETED', 'FAILED', 'CANCELED'];

const REGRESSED_LABEL = '시계 역행';

// 시계 역행 라벨(색만 쓰지 않고 글자·아이콘)
const regressedTag = () => `<span class="badge st-warn">${svgIcon('clock')}${REGRESSED_LABEL}</span>`;

const svgIcon = shape => `<svg viewBox="0 0 12 12">${ICON[shape]}</svg>`;
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
const shown = item => item.display_status || item.status;

// 상태 배지(색 + 모양 + 글자)
function badge(state) {
  const [cls, shape] = STATE_STYLE[state] || ['st-missing', 'ring'];
  return `<span class="badge ${cls}">${svgIcon(shape)}${esc(state)}</span>`;
}

// 값이 아님 표시(만료 빗금 / 미수집·원천 없음 점선 / 무효)
function notValue(status, reason) {
  const shape = status === 'stale' ? 'clock' : status === 'invalid' ? 'tri' : 'ring';
  const extra = status === 'invalid' && reason ? ` ${esc(reason)}` : '';
  return `<span class="nv ${status}">${svgIcon(shape)}${FRESH[status] || esc(status)}${extra}</span>`;
}

// 신선도 점(표시 여유 중이면 테두리 추가)
function dot(item) {
  const disp = shown(item), grace = item.status && item.status !== disp;
  const age = item.recv_age_s == null ? '' : ` · 수신 ${item.recv_age_s}s / 기준 ${item.limit_s}s`;
  const stamp = item.stamp_age_shown_s == null ? '' : ` · 표본 ${item.stamp_age_shown_s}s`;
  const regressed = item.clock_regressed ? ` · ${REGRESSED_LABEL}(원값 ${item.stamp_age_s}s)` : '';
  const title = `${FRESH[disp] || disp}${grace ? ` (판정: ${FRESH[item.status]}, 표시 여유)` : ''}${age}${stamp}${regressed}`;
  return `<span class="dot ${disp}${grace ? ' grace' : ''}" title="${esc(title)}"></span>`;
}

// 배터리 바(계약 B4 LOW 22% 이하 위험, 40% 이하 주의). 잔량이 없으면 값 아님
function batteryBar(v) {
  if (v.percentage === null) return notValue('invalid', 'NO_PERCENTAGE');
  const pct = Math.round(v.percentage * 100);
  const color = pct <= 22 ? 'var(--bad)' : pct <= 40 ? 'var(--warn)' : 'var(--ok)';
  return `<span class="bat"><span class="bar"><i style="width:${Math.max(0, Math.min(100, pct))}%;background:${color}"></i></span><span class="num">${pct}%</span>${v.present ? '' : '<span class="small st-warn">미장착</span>'}</span>`;
}

// 링 게이지(비율). ratio가 null이면 점선 링
function ring(ratio, size = 86) {
  const half = size / 2, r = half - 7, c = 2 * Math.PI * r;
  if (ratio === null) {
    return `<svg width="${size}" height="${size}" class="gauge"><circle cx="${half}" cy="${half}" r="${r}" fill="none" stroke="var(--missing)" stroke-width="6" stroke-dasharray="4 4"/>
      <text x="50%" y="54%" text-anchor="middle" fill="var(--muted)" font-size="11">미수집</text></svg>`;
  }
  return `<svg width="${size}" height="${size}" class="gauge" viewBox="0 0 ${size} ${size}">
    <circle cx="${half}" cy="${half}" r="${r}" fill="none" stroke="#34363c" stroke-width="7"/>
    <circle cx="${half}" cy="${half}" r="${r}" fill="none" stroke="var(--accent)" stroke-width="7" stroke-linecap="round"
      stroke-dasharray="${(c * ratio).toFixed(1)} ${c.toFixed(1)}" transform="rotate(-90 ${half} ${half})"/>
    <text x="50%" y="55%" text-anchor="middle" fill="var(--text)" font-size="15" font-weight="700" class="num">${Math.round(ratio * 100)}%</text></svg>`;
}

// ── 화면 기록(세션 내, 1 s 간격, 2분). 원장 이력이 아니며 새로고침하면 비워진다 ──
const HISTORY_LEN = 120;
const hist = {ratio: [], active: []};
let lastHistAt = 0;

// KPI 기록
function record(snap, now) {
  if (now - lastHistAt < 1000) return;
  lastHistAt = now;
  const k = snap.kpi;
  hist.ratio.push(k.running === null ? null : k.running / k.registered);
  hist.active.push(k.task_by_state === null ? null
    : Object.entries(k.task_by_state).filter(([s]) => !TERMINAL.includes(s)).reduce((a, [, n]) => a + n, 0));
  for (const key in hist) if (hist[key].length > HISTORY_LEN) hist[key].shift();
}

// 스파크라인(null은 끊김)
function spark(values, max, w = 220, h = 30) {
  const step = w / (HISTORY_LEN - 1), top = Math.max(1, max);
  let d = '', pen = false;
  values.forEach((v, i) => {
    if (v === null) { pen = false; return; }
    const x = (HISTORY_LEN - values.length + i) * step, y = h - 3 - (v / top) * (h - 6);
    d += `${pen ? 'L' : 'M'}${x.toFixed(1)} ${y.toFixed(1)}`; pen = true;
  });
  return `<svg class="spark" width="100%" height="${h}" viewBox="0 0 ${w} ${h}" preserveAspectRatio="none">
    <line x1="0" y1="${h - 3}" x2="${w}" y2="${h - 3}" stroke="var(--border)"/>
    <path d="${d}" fill="none" stroke="var(--accent2)" stroke-width="1.6" vector-effect="non-scaling-stroke"/></svg>`;
}

// 가로 막대(작업 상태별)
function hbars(entries) {
  const max = Math.max(1, ...entries.map(e => e[1]));
  const rowH = 22, w = 260, labelW = 104, numW = 28;
  const rows = entries.map(([s, n], i) => {
    const [cls] = STATE_STYLE[s] || ['st-missing'];
    const bw = (w - labelW - numW) * n / max;
    return `<g transform="translate(0 ${i * rowH})" class="${cls}">
      <text x="0" y="14" fill="var(--muted)" font-size="11">${esc(s)}</text>
      <rect x="${labelW}" y="5" width="${w - labelW - numW}" height="10" rx="2" fill="#2c2e33"/>
      <rect x="${labelW}" y="5" width="${bw.toFixed(1)}" height="10" rx="2" fill="currentColor"/>
      <text x="${w}" y="14" text-anchor="end" fill="var(--text)" font-size="12" class="num">${n}</text></g>`;
  }).join('');
  return `<svg width="100%" height="${Math.max(1, entries.length) * rowH}" viewBox="0 0 ${w} ${Math.max(1, entries.length) * rowH}">${rows}</svg>`;
}

// ── 골격 ──
let selected = null, lastSnap = null, mapView = null;
const $ = id => document.getElementById(id);

// 화면 틀 생성
function buildSkeleton(view) {
  mapView = view;
  document.body.innerHTML = `
  <header>
    <div class="title">AX 창고 통합 관제</div>
    <div class="left"><span>로봇 목록 출처 <b id="src">-</b></span><span>작업 Topic <b id="task-topic">-</b></span><span>시계 <b id="sim">-</b></span></div>
    <div class="right"><span id="conn" class="badge st-missing">${svgIcon('ring')}대기</span><span class="clock num" id="clock">--:--:--</span></div>
  </header>
  <main>
    <div class="col">
      <div class="panel kpi" id="kpi-run"></div>
      <div class="panel kpi tasks" id="kpi-task"></div>
    </div>
    <div class="panel center"><h3>현장 뷰 <span class="sub" id="map-name">지도 파일 기준 · 위치는 amcl_pose(표시 전용)</span></h3><div class="view" id="view"></div>
      <div class="legend" id="legend"></div></div>
    <div class="panel side"><h3>작업 목록 <span class="sub" id="task-fresh"></span></h3><div class="scroll" id="tasks"></div>
      <div class="foot" id="no-source"></div></div>
  </main>
  <section class="cards" id="cards"></section>
  <div class="panel detail" id="detail"><h3><span id="dtitle">상세</span><span class="close" id="dclose">닫기 ✕</span></h3><div class="body" id="dbody"></div></div>
  <div class="banner">연결 끊김 — 표시값은 마지막 응답이며 현재 상태가 아님</div>`;
  $('dclose').onclick = () => { selected = null; renderAll(lastSnap); };
  $('legend').innerHTML = legendHtml();
  view.init($('view'));
}

// 지도 범례
function legendHtml() {
  const robot = (shape, color, label, extra = '') => `<span><svg width="16" height="14" viewBox="-8 -7 16 14">${
    shape === 'm' ? `<circle r="5" fill="${color}" ${extra}/><path d="M3 0l5 0" stroke="${color}" stroke-width="2"/>`
                  : `<rect x="-6" y="-4" width="11" height="8" rx="2" fill="${color}" ${extra}/><path d="M5 0l3 0" stroke="${color}" stroke-width="2"/>`
  }</svg>${label}</span>`;
  return robot('m', 'var(--accent2)', '조작 로봇') + robot('t', 'var(--accent2)', '운송 로봇') +
    '<span><span class="dot ok"></span>RUNNING</span><span><span class="dot" style="background:var(--accent2)"></span>STOPPED</span>' +
    '<span><span class="dot" style="background:var(--warn)"></span>정지 중</span><span><span class="dot" style="background:var(--bad)"></span>FAULT</span>' +
    robot('t', 'var(--stale)', '만료·무효(마지막 위치, 반투명)', 'opacity=".45"') +
    '<span><span style="width:14px;height:8px;background:var(--map-wall);display:inline-block"></span>벽·장애물</span>';
}

// ── KPI ──
function renderKpi(snap) {
  const k = snap.kpi;
  const ratio = k.running === null ? null : k.running / k.registered;
  $('kpi-run').innerHTML = `
    <div class="label">가동 로봇 <span class="small">(구동 상태 신선·유효, FAULT·UNKNOWN 아님 / 등록)</span></div>
    <div class="value num">${k.running === null ? notValue('missing') : k.running}<span class="unit">/ ${k.registered} 대</span></div>
    ${ring(ratio)}
    <div class="note">가동률 · 판정 기준 ${k.running_strict}대 · 최근 2분(화면 기록)</div>
    ${spark(hist.ratio, 1)}`;

  const head = '<div class="label">작업 상태별 수</div>';
  if (k.task_by_state === null) { $('kpi-task').innerHTML = head + notValue(k.task_status); return; }
  const entries = Object.entries(k.task_by_state);
  const active = entries.filter(([s]) => !TERMINAL.includes(s)).reduce((a, [, n]) => a + n, 0);
  const known = hist.active.filter(v => v !== null);
  $('kpi-task').innerHTML = head + `<div class="value num">${active}<span class="unit">건 진행·대기</span></div>` +
    hbars(entries) + '<div class="note">진행·대기 작업 수 · 최근 2분</div>' + spark(hist.active, Math.max(1, ...known));
}

// ── 작업 표 ──
function renderTasks(snap) {
  const t = snap.tasks;
  $('task-fresh').innerHTML = `${dot(t)} ${FRESH[shown(t)]}`;
  $('no-source').textContent = `원천 없음(1단계): ${snap.no_source.join(', ')}`;
  if (shown(t) !== 'ok') { $('tasks').innerHTML = `<div style="padding:var(--pad)">${notValue(shown(t), t.reason)}</div>`; return; }
  if (!t.value.length) { $('tasks').innerHTML = '<div style="padding:var(--pad)" class="muted">작업 없음(빈 목록 수신)</div>'; return; }
  $('tasks').innerHTML = '<table class="grid"><tr><th>작업</th><th>상태</th><th>인계</th><th>조작/운송</th></tr>' + t.value.map(x =>
    `<tr><td class="num">${esc(x.task_id)}</td><td>${badge(x.state)}</td><td>${badge(x.transfer)}</td>
     <td class="small">${esc(x.manipulator || '—')} / ${esc(x.transporter || '—')}</td></tr>`).join('') + '</table>';
}

// 필드 값(표시 정상일 때만 값)
const fieldValue = (item, fn) => shown(item) === 'ok' ? fn(item.value) : notValue(shown(item), item.reason);
const permit = v => `주행 ${v.drive_allowed ? '<b class="st-ok">허가</b>' : '<b class="st-bad">차단</b>'} · 팔 ${v.arm_allowed ? '<b class="st-ok">허가</b>' : '<b class="st-bad">차단</b>'}` +
  (v.block_reasons.length ? ` <span class="small st-warn">${esc(v.block_reasons.join(','))}</span>` : '');
const assignText = asg => shown(asg) !== 'ok' ? notValue(shown(asg))
  : asg.tasks.length ? asg.tasks.map(t => `<span class="num">${esc(t.task_id)}</span> ${badge(t.transfer)}`).join(' ') : '<span class="muted">배정 없음</span>';

// ── 로봇 카드 ──
function renderCards(snap) {
  const box = $('cards');
  box.style.setProperty('--cols', String(Math.min(4, snap.robots.length)));
  box.innerHTML = snap.robots.map(r => {
    const it = r.items, asg = r.assignment;
    const f = (k, item, html) => `<div class="field"><span class="k">${k}</span><span class="v">${html}</span>${dot(item)}</div>`;
    return `<div class="panel card ${selected === r.robot_id ? 'sel' : ''}" data-id="${esc(r.robot_id)}">
      <div class="head">${ROLE_ICON[r.role] || ''}<span>${esc(r.robot_id)}</span>
        <span class="tag">${ROLE_LABEL[r.role] || esc(r.role)}</span>${r.spare ? '<span class="tag spare">예비</span>' : ''}</div>
      ${f('구동', it.actuation, fieldValue(it.actuation, v => badge(v.state)))}
      ${f('허가', it.actuation, fieldValue(it.actuation, permit))}
      ${f('안전', it.safety, fieldValue(it.safety, v => `${badge(v.level)} <span class="small muted">${esc(v.reason)}</span>`))}
      ${f('위치 신뢰', it.health, fieldValue(it.health, v => `${badge(v.state)} <span class="small muted num">σ ${v.position_std_m} m</span>`))}
      ${f('배터리', it.battery, fieldValue(it.battery, batteryBar))}
      ${f('배정·인계', asg, assignText(asg))}
    </div>`;
  }).join('');
  box.querySelectorAll('.card').forEach(el => { el.onclick = () => select(el.dataset.id); });
}

// 선택 전환
function select(id) { selected = selected === id ? null : id; renderAll(lastSnap); }

// 상세 값 한 칸
function detailValue(kind, it) {
  if (shown(it) !== 'ok') return notValue(shown(it), it.reason);
  const v = it.value;
  if (kind === 'actuation') return `${badge(v.state)} ${permit(v)}`;
  if (kind === 'safety') return `${badge(v.level)} ${esc(v.reason)} · 상한 ${v.max_linear_speed} m/s`;
  if (kind === 'health') return `${badge(v.state)} σ ${v.position_std_m} m`;
  if (kind === 'executor') return `${esc(v.task_id || '—')} · ${esc(v.local_phase || '—')}${v.paused ? ' · 일시 정지' : ''} · 마지막 ${badge(v.last_result)}`;
  if (kind === 'pose') return `<span class="num">x ${v.x.toFixed(2)} · y ${v.y.toFixed(2)} · yaw ${v.yaw.toFixed(2)}</span> ${esc(v.frame)}`;
  if (kind === 'battery') return batteryBar(v);
  return esc(JSON.stringify(v));
}

// ── 상세(항목별 표 + 원본 접힘) ──
function renderDetail() {
  const box = $('detail');
  const r = selected && lastSnap ? lastSnap.robots.find(x => x.robot_id === selected) : null;
  if (!r) { box.classList.remove('open'); return; }
  box.classList.add('open');
  $('dtitle').textContent = `${r.robot_id} · ${ROLE_LABEL[r.role] || r.role}${r.spare ? ' · 예비' : ''}`;
  const rows = Object.entries(r.items).map(([kind, it]) => {
    const m = it.meta;
    const judged = it.status === shown(it) ? FRESH[it.status] : `${FRESH[shown(it)]} (판정 ${FRESH[it.status]})`;
    return `<tr><td>${ITEM[kind] || esc(kind)}</td><td>${dot(it)} ${judged}</td><td>${detailValue(kind, it)}</td>
      <td class="num">${it.recv_age_s ?? '—'} / ${it.stamp_age_shown_s ?? '—'} / ${it.limit_s}${it.clock_regressed ? ` ${regressedTag()}` : ''}</td>
      <td class="num">${it.stamp_age_s ?? '—'}</td>
      <td class="small">${m ? esc(m.source_id) : '<span class="muted">meta 없음</span>'}</td>
      <td class="small num">${m ? esc(m.session) : '—'}</td><td class="num">${m ? m.revision : '—'}</td>
      <td>${m ? (m.valid ? '<span class="st-ok">true</span>' : `<span class="st-warn">false ${esc(m.reason)}</span>`) : '—'}</td></tr>`;
  }).join('');
  $('dbody').innerHTML =
    `<table class="grid"><tr><th>항목</th><th>상태</th><th>값</th><th>수신/표본 나이/기준(s)</th><th>표본 나이 원값(s)</th><th>source</th><th>session</th><th>rev</th><th>valid</th></tr>${rows}</table>
     <div class="small muted" style="margin-top:8px">배정·인계: ${assignText(r.assignment)}</div>
     <details><summary>원본 JSON</summary><pre>${esc(JSON.stringify(r, null, 1))}</pre></details>`;
}

// ── 전체 렌더 ──
function renderAll(snap) {
  if (!snap) return;
  lastSnap = snap;
  record(snap, performance.now());
  $('src').textContent = snap.robot_source === 'dev_config' ? '개발 설정' : snap.robot_source;
  $('task-topic').textContent = snap.topics ? `${snap.topics.tasks}${snap.topics.profile === 'dev' ? ' (개발)' : ''}` : '-';
  $('sim').innerHTML = snap.clock.sim_valid ? `<span class="num">${snap.clock.sim.toFixed(1)}</span> s` : notValue('invalid', 'CLOCK_INVALID');
  $('clock').textContent = new Date().toLocaleTimeString('ko-KR', {hour12: false});
  renderKpi(snap);
  renderTasks(snap);
  renderCards(snap);
  const open = document.querySelector('.detail details')?.open;
  renderDetail();
  if (open) document.querySelector('.detail details').open = true;
  mapView.update(snap.robots);
}

// 연결 상태
function setConn(up, text) {
  const el = $('conn');
  el.className = 'badge ' + (up ? 'st-ok' : 'st-bad');
  el.innerHTML = svgIcon(up ? 'check' : 'oct') + text;
  document.body.classList.toggle('conn-down', !up);
}

// CSS 변수 읽기
const cssVar = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// 지도 파일 읽기(P5 PGM)
async function loadMap() {
  const info = await (await fetch('/map/info.json')).json();
  const buf = new Uint8Array(await (await fetch('/map/map.pgm')).arrayBuffer());
  let pos = 0;
  const tokens = [];
  while (tokens.length < 4) {
    while (buf[pos] <= 32) pos++;
    if (buf[pos] === 35) { while (buf[pos] !== 10) pos++; continue; }
    const s = pos;
    while (buf[pos] > 32) pos++;
    tokens.push(String.fromCharCode(...buf.slice(s, pos)));
  }
  const w = +tokens[1], h = +tokens[2];
  return {info, w, h, pixels: buf.slice(pos + 1, pos + 1 + w * h)};
}

// 로봇 색: 구동·위치 표시가 모두 정상일 때만 상태색
function robotColor(r) {
  const a = r.items.actuation, p = r.items.pose;
  if (shown(a) !== 'ok' || shown(p) !== 'ok') return cssVar('--stale');
  const cls = (STATE_STYLE[a.value.state] || ['st-missing'])[0];
  return {'st-ok': cssVar('--ok'), 'st-info': cssVar('--accent2'), 'st-warn': cssVar('--warn'), 'st-bad': cssVar('--bad')}[cls] || cssVar('--missing');
}

// 마지막으로 받은 위치(서버는 정상일 때만 좌표를 보낸다. 한 번도 없으면 null → 그리지 않음)
const lastPose = {};
function poseOf(r) {
  const p = r.items.pose;
  if (shown(p) === 'ok') lastPose[r.robot_id] = p.value;
  return lastPose[r.robot_id] || null;
}

// ── 2D 지도 ──
const Map2D = {
  // canvas 생성과 지도 읽기
  async init(container) {
    this.canvas = document.createElement('canvas');
    container.appendChild(this.canvas);
    this.map = await loadMap();
    $('map-name').textContent = `${this.map.info.name} · 지도 파일 기준 · 위치는 amcl_pose(표시 전용)`;
    this.paintBase();
    window.addEventListener('resize', () => this.update(this.robots || []));
  },

  // 지도 바탕을 테마 색으로 칠하기
  paintBase() {
    const {w, h, pixels} = this.map;
    this.base = document.createElement('canvas');
    this.base.width = w;
    this.base.height = h;
    const ctx = this.base.getContext('2d'), img = ctx.createImageData(w, h);
    const hex = s => [1, 3, 5].map(i => parseInt(s.slice(i, i + 2), 16));
    const free = hex(cssVar('--map-free')), wall = hex(cssVar('--map-wall')), unknown = hex(cssVar('--map-unknown'));
    for (let i = 0; i < w * h; i++) {
      const v = pixels[i], c = v > 250 ? free : v < 50 ? wall : unknown;
      img.data[i * 4] = c[0]; img.data[i * 4 + 1] = c[1]; img.data[i * 4 + 2] = c[2]; img.data[i * 4 + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
  },

  // 로봇 그리기: 조작=원+방향, 운송=둥근 사각+방향, 만료·무효=반투명 회색+점선 원+라벨
  update(robots) {
    this.robots = robots;
    if (!this.map) return;
    const cv = this.canvas, box = cv.parentElement.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    cv.width = box.width * dpr;
    cv.height = box.height * dpr;
    const ctx = cv.getContext('2d');
    ctx.scale(dpr, dpr);
    const s = Math.min(box.width / this.map.w, box.height / this.map.h);
    const ox = (box.width - this.map.w * s) / 2, oy = (box.height - this.map.h * s) / 2;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.base, ox, oy, this.map.w * s, this.map.h * s);
    const res = this.map.info.resolution, [mx, my] = this.map.info.origin;

    for (const r of robots) {
      const pose = poseOf(r);
      if (!pose) continue;
      const x = ox + (pose.x - mx) / res * s, y = oy + (this.map.h - (pose.y - my) / res) * s;
      const fresh = shown(r.items.pose) === 'ok' && shown(r.items.actuation) === 'ok';
      const color = robotColor(r);
      ctx.save();
      ctx.globalAlpha = fresh ? 1 : 0.45;
      ctx.translate(x, y);
      ctx.rotate(-pose.yaw);
      ctx.fillStyle = color;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      if (r.role === 'MANIPULATOR') ctx.arc(0, 0, 8, 0, 2 * Math.PI);
      else if (ctx.roundRect) ctx.roundRect(-10, -7, 18, 14, 3);
      else ctx.rect(-10, -7, 18, 14);
      ctx.fill();
      ctx.beginPath(); ctx.moveTo(6, 0); ctx.lineTo(16, 0); ctx.stroke();
      if (!fresh) {
        ctx.setLineDash([3, 3]); ctx.strokeStyle = cssVar('--text'); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(0, 0, 13, 0, 2 * Math.PI); ctx.stroke();
      }
      ctx.restore();
      ctx.font = `600 11px ${cssVar('--font')}`;
      ctx.fillStyle = cssVar('--text');
      const why = shown(r.items.pose) !== 'ok' ? shown(r.items.pose) : shown(r.items.actuation);
      ctx.fillText(r.robot_id + (fresh ? '' : ` · ${FRESH[why] || why}`), x + 14, y - 12);
    }
  },
};
