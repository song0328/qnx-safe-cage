"use strict";
/* Safe Cage dashboard - vanilla JS, no build step. */

const $ = (id) => document.getElementById(id);
const ENV = { tmax: 4, vmax: 90, rom: [0, 90] };   // follows the patient envelope (care plan step 1)
const F = { NAN: 1, TORQUE: 2, SLEW: 4, SPEED: 8, ROM: 16, MISS: 32, FALLBACK: 64 };
const OVR_MASK = F.NAN | F.TORQUE | F.SPEED | F.ROM | F.SLEW;

const EVENT_TEXT = {
  connect: ["Controller connected", "info"],
  torque_limit: ["Torque clipped", "warn"],
  slew_limit: ["Rate of change limited", "warn"],
  speed_limit: ["Speed limit enforced", "warn"],
  rom_limit: ["Range-of-motion wall", "warn"],
  nan: ["Non-finite command replaced", "warn"],
  deadline_miss: ["Missed the 2 ms deadline", "warn"],
  revoked: ["Controller revoked", "bad"],
  controller_died: ["Controller died", "bad"],
  controller_exited: ["Controller exited early", "bad"],
  no_connect: ["Controller never connected", "bad"],
  cage_warning: ["Cage warning", "bad"],
  joint_finished: ["Session ended", "good"],
};

let S = null;            // current session data
let busy = false, target = false;
let model = "", speed = 1;
let aiIters = 0, logCount = 0;

/* ------------------------------------------------------------ theme */
(function theme() {
  const root = document.documentElement;
  try { const t = localStorage.getItem("sc-theme"); if (t) root.dataset.theme = t; } catch (e) { /* ignore */ }
  $("theme").onclick = () => {
    const dark = root.dataset.theme ? root.dataset.theme === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
    root.dataset.theme = dark ? "light" : "dark";
    try { localStorage.setItem("sc-theme", root.dataset.theme); } catch (e) { /* ignore */ }
    drawAll();
  };
})();

/* ------------------------------------------------------------ controls */
function seg(id, onPick) {
  $(id).querySelectorAll("button").forEach((b) => b.onclick = () => {
    $(id).querySelectorAll("button").forEach((x) => x.classList.toggle("on", x === b));
    onPick(b.dataset.v);
  });
}
seg("model", (v) => model = v);
/* one screen, one job: Session / Care plan / Evidence / Lab */
function showView(name) {
  document.querySelectorAll(".view").forEach((v) => v.hidden = v.dataset.view !== name);
  document.querySelectorAll(".dock button").forEach((b) => b.classList.toggle("on", b.dataset.go === name));
  if (name === "session") schedule();
  window.scrollTo({ top: 0 });
}
document.querySelectorAll("[data-go]").forEach((b) => b.onclick = () => showView(b.dataset.go));
function headline(text) { $("headline").textContent = text; }
seg("speed", (v) => speed = parseFloat(v));
document.querySelectorAll(".tab").forEach((t) => t.onclick = () => showTab(t.dataset.tab));
function showTab(name) {
  document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("on", t.dataset.tab === name));
  $("tab-ai").hidden = name !== "ai";
  $("tab-rec").hidden = name !== "rec";
}
function toast(text) {
  const el = document.createElement("div");
  el.className = "toast"; el.textContent = text; document.body.appendChild(el);
  setTimeout(() => el.remove(), 3500);
}
async function post(path, body) {
  const r = await fetch(path, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body) });
  if (!r.ok) { const j = await r.json().catch(() => ({})); toast(j.error === "busy" ? "A session is already running." : (j.error || "Request failed")); return false; }
  return true;
}
function setBusy(b) {
  busy = b;
  renderClinic();
  $("ai-go").disabled = b || !target;
  $("replay-go").disabled = b;
  document.querySelectorAll(".run").forEach((x) => x.disabled = b || !target);
}
$("ai-go").onclick = () => post("/api/ai", { prescription: $("rx").value, model, iters: 4 }).then((ok) => ok && setBusy(true));
$("replay-go").onclick = () => post("/api/replay", { id: $("replay-sel").value, speed }).then((ok) => ok && setBusy(true));

async function loadStatus() {
  const s = await (await fetch("/api/status")).json();
  target = s.target;
  const chip = $("chip-target");
  chip.classList.toggle("ok", target); chip.classList.toggle("off", !target);
  chip.querySelector("span").textContent = target ? `QNX target online · ${s.uname}` : "QNX target offline · replay only";
  const lib = $("lib"); lib.innerHTML = "";
  for (const c of s.controllers) {
    const li = document.createElement("li");
    li.innerHTML = `<div class="l-x"><div class="l-t ${c.id === "good_rom" ? "tag-good" : ""}">${c.title}</div><div class="l-d">${c.desc}</div></div>
      <button class="run" title="Run ${c.id} on the QNX target" aria-label="Run ${c.id}"><svg viewBox="0 0 12 12"><path d="M2 1.5v9l8-4.5z"/></svg></button>`;
    li.querySelector(".run").onclick = () => post("/api/run", { controller: c.id }).then((ok) => ok && setBusy(true));
    lib.appendChild(li);
  }
  const sel = $("replay-sel"); sel.innerHTML = "";
  const groups = {};
  for (const r of s.replays) (groups[r.group] ||= []).push(r);
  for (const [g, items] of Object.entries(groups)) {
    const og = document.createElement("optgroup"); og.label = g;
    for (const r of items) { const o = document.createElement("option"); o.value = r.id; o.textContent = r.label; og.appendChild(o); }
    sel.appendChild(og);
  }
  const pick = s.replays.find((r) => r.id.startsWith("loop/") && r.label.includes("Haiku")) || s.replays[0];
  if (pick) sel.value = pick.id;
  setBusy(s.busy);
}

/* ------------------------------------------------------------ session state */
function newSession(msg) {
  S = { T: msg.T || 8, t: [], a: [], v: [], rq: [], ap: [], f: [], o: [], revokeT: null, done: false, mode: msg.mode, iter: msg.iter, peakWin: 0 };
  setBar("bar-win", "b-win", 0, 25); setBar("bar-tot", "b-tot", 0, 100); $("j-state").textContent = "Controller in charge";
  const badge = $("mode");
  if (msg.envelope) useEnvelope(msg.envelope);
  badge.className = "badge " + (msg.mode === "live" ? "live" : msg.mode === "patient" ? "patient" : msg.mode === "ai" ? "ai" : "replay");
  badge.textContent = msg.mode === "live" ? "LIVE ON QNX" : msg.mode === "patient" ? "PATIENT SESSION" : msg.mode === "ai" ? (msg.replay ? "AI · RECORDED" : "AI TRIAL · LIVE ON QNX") : "REPLAY";
  if (msg.mode === "ai" && aiReplay) { badge.className = "badge ai"; badge.textContent = "AI · RECORDED RUN"; }
  $("title").textContent = msg.title;
  $("subtitle").textContent = msg.subtitle || "";
  $("verdict").hidden = true;
  $("cta").hidden = true;

  $("dock-live").classList.add("on");
  S.state = "";
  headline(msg.mode === "patient" ? "The approved controller is moving the arm"
    : msg.mode === "ai" ? `Trialling the AI's draft #${msg.iter} on a virtual patient`
    : msg.mode === "replay" ? "Replaying a recorded run"
    : "The cage is watching this controller on QNX");
  showView("session");
  if (msg.mode !== "ai" || msg.iter === 1) clearLog();
  if (msg.mode === "ai") addLog({ t: 0, kind: "_iter", detail: `Iteration ${msg.iter} starts in the cage` });
  setJoint(20, 0);
  drawAll();
}

function onTlm(m) {
  if (!S) return;
  S.t.push(m.t); S.a.push(m.a); S.v.push(m.v); S.rq.push(m.rq); S.ap.push(m.ap); S.f.push(m.f); S.o.push(m.o);
  $("clock").textContent = `${m.t.toFixed(1)} / ${S.T.toFixed(1)} s`;
  $("m-angle").textContent = m.a.toFixed(1);
  $("m-speed").textContent = Math.abs(m.v).toFixed(0);
  $("m-torque").textContent = m.ap.toFixed(2);
  $("m-req").textContent = m.rq === null ? "NaN" : (Math.abs(m.rq) > 99 ? m.rq.toFixed(0) : m.rq.toFixed(2));
  gauge("g-angle", (m.a - ENV.rom[0]) / Math.max(1, ENV.rom[1] - ENV.rom[0]), m.a < ENV.rom[0] || m.a > ENV.rom[1]);
  gauge("g-speed", Math.abs(m.v) / ENV.vmax);
  gauge("g-torque", Math.abs(m.ap) / ENV.tmax);
  // overrides budget
  let i0 = S.t.length - 1; while (i0 > 0 && S.t[i0 - 1] > m.t - 0.5) i0--;
  const win = m.o - (i0 > 0 ? S.o[i0 - 1] : 0);
  S.peakWin = Math.max(S.peakWin, win);
  setBar("bar-win", "b-win", win, 25, S.peakWin);
  setBar("bar-tot", "b-tot", m.o, 100);
  const st = S.revokeT !== null ? "revoked" : (m.f & F.FALLBACK) ? "fallback" : (m.f & OVR_MASK) ? "override" : "";
  setJoint(m.a, st);
  if (st === "revoked" && S.state !== "revoked") headline("Controller revoked. The trusted fallback holds the arm.");
  S.state = st === "revoked" ? "revoked" : S.state;
  schedule();
}
function gauge(id, frac, outside) {
  const el = $(id); if (!el) return;
  const f = Math.max(0, Math.min(1, frac));
  el.style.strokeDashoffset = (100 - f * 100).toFixed(1);
  el.classList.toggle("over", outside === true || frac > 1);
  el.classList.toggle("near", !(outside === true || frac > 1) && frac > 0.9);
}
function setBar(bar, lbl, n, max, peak) {
  const el = $(bar); const p = Math.min(1, (peak ?? n) / max);
  el.style.width = (p * 100).toFixed(1) + "%";
  el.classList.toggle("hot", p >= 0.5 && p < 1); el.classList.toggle("full", p >= 1);
  $(lbl).textContent = peak !== undefined ? `${n} now · peak ${peak} / ${max}` : `${n} / ${max}`;
}

/* ------------------------------------------------------------ joint drawing */
const CX = 170, CY = 190, R = 150;
function pt(deg, r = R) { const t = -deg * Math.PI / 180; return [CX + r * Math.cos(t), CY + r * Math.sin(t)]; }
function sector(a0, a1, r = R) {
  const [x0, y0] = pt(a0, r), [x1, y1] = pt(a1, r);
  return `M${CX},${CY} L${x0.toFixed(1)},${y0.toFixed(1)} A${r},${r} 0 0 0 ${x1.toFixed(1)},${y1.toFixed(1)} Z`;
}
function drawTicks() {
  let h = "";
  for (let d = 0; d <= 150; d += 10) {
    const major = d % 30 === 0;
    const [x0, y0] = pt(d, R + 2), [x1, y1] = pt(d, R + (major ? 11 : 7));
    h += `<line class="tick${major ? " major" : ""}" x1="${x0.toFixed(1)}" y1="${y0.toFixed(1)}" x2="${x1.toFixed(1)}" y2="${y1.toFixed(1)}"/>`;
  }
  $("ticks").innerHTML = h;
}
drawTicks();
function drawEnvelope() {
  $("env-sector").setAttribute("d", sector(Math.max(0, ENV.rom[0]), ENV.rom[1], R - 4));
  $("out-sector").setAttribute("d", sector(ENV.rom[1], 150, R - 4));
  const [lx, ly] = pt(ENV.rom[1], R + 14); $("lbl-90").setAttribute("x", lx + 5); $("lbl-90").setAttribute("y", ly + 4);
  $("lbl-90").textContent = `${ENV.rom[1]}°`;
  document.querySelectorAll(".m-s")[0].textContent = `safe range ${ENV.rom[0]}–${ENV.rom[1]}°`;
  document.querySelectorAll(".m-s")[1].textContent = `limit ${ENV.vmax} °/s`;
  const lim = document.querySelectorAll(".m-s")[2]; if (lim) lim.lastChild.textContent = ` · limit ${ENV.tmax}`;
}
function useEnvelope(e) {
  if (!e) return;
  ENV.tmax = e.tmax; ENV.vmax = e.vmax; ENV.rom = [e.rom_min, e.rom_max];
  drawEnvelope(); schedule();
}
drawEnvelope();
function setJoint(angle, state) {
  $("forearm").setAttribute("transform", `rotate(${(-angle).toFixed(2)} ${CX} ${CY})`);
  const ar = 44, a0 = Math.min(0, angle), a1 = Math.max(0, angle);
  const [ax0, ay0] = pt(a0, ar), [ax1, ay1] = pt(a1, ar);
  $("ang-arc").setAttribute("d", Math.abs(angle) < 0.5 ? "" : `M${ax0.toFixed(1)},${ay0.toFixed(1)} A${ar},${ar} 0 0 0 ${ax1.toFixed(1)},${ay1.toFixed(1)}`);
  const [tx, ty] = pt(angle / 2, ar + 18);
  $("ang-txt").setAttribute("x", (tx - 10).toFixed(1)); $("ang-txt").setAttribute("y", (ty + 4).toFixed(1));
  $("ang-txt").textContent = `${angle.toFixed(0)}°`;
  const j = $("joint");
  j.classList.toggle("override", state === "override");
  j.classList.toggle("fallback", state === "fallback");
  j.classList.toggle("revoked", state === "revoked");
  $("j-state").textContent = state === "idle" ? "Idle" : state === "revoked" ? "Revoked · fallback in charge"
    : state === "fallback" ? "Fallback in charge" : state === "override" ? "Cage overriding" : "Controller in charge";
  $("j-state").className = "j-state " + (state === "idle" ? "fallback" : state || "ok");
}

/* ------------------------------------------------------------ charts */
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
let pending = false;
function schedule() { if (!pending) { pending = true; requestAnimationFrame(() => { pending = false; drawAll(); }); } }

function prep(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  }
  const g = canvas.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
  return { g, w, h };
}
function chart(id, ymin, ymax, ticks, draw) {
  const { g, w, h } = prep($(id));
  const L = 38, Rr = 8, T = 6, B = 18;
  const T_END = S ? S.T : 8;
  const X = (t) => L + (t / T_END) * (w - L - Rr);
  const Y = (v) => T + (1 - (v - ymin) / (ymax - ymin)) * (h - T - B);
  g.font = `11px ${css("--mono") || "monospace"}`; g.fillStyle = css("--ink-3"); g.strokeStyle = css("--line"); g.lineWidth = 1;
  for (const v of ticks) { g.beginPath(); g.moveTo(L, Y(v) + .5); g.lineTo(w - Rr, Y(v) + .5); g.stroke(); g.textAlign = "right"; g.fillText(String(v), L - 6, Y(v) + 4); }
  g.textAlign = "center";
  for (let s = 0; s <= T_END; s += 1) g.fillText(s + "s", X(s), h - 4);
  if (S) {
    // fallback spans
    g.fillStyle = css("--fallback");
    let start = null;
    for (let i = 0; i <= S.t.length; i++) {
      const on = i < S.t.length && (S.f[i] & F.FALLBACK);
      if (on && start === null) start = i ? S.t[i - 1] : 0;
      if (!on && start !== null) { g.fillRect(X(start), T, Math.max(1, X(S.t[i - 1]) - X(start)), h - T - B); start = null; }
    }
  }
  draw(g, X, Y, w, h, L, Rr, T, B);
  if (S && S.revokeT !== null) { g.strokeStyle = css("--bad"); g.lineWidth = 2; g.beginPath(); g.moveTo(X(S.revokeT), T); g.lineTo(X(S.revokeT), h - B); g.stroke(); }
}
function line(g, X, Y, xs, ys, color, width, clip, area) {
  g.strokeStyle = color; g.lineWidth = width; g.lineJoin = "round"; g.lineCap = "round"; g.beginPath();
  let on = false, last = null, first = null;
  for (let i = 0; i < xs.length; i++) {
    let v = ys[i]; if (v === null || Number.isNaN(v)) { on = false; continue; }
    if (clip) v = Math.max(clip[0], Math.min(clip[1], v));
    if (!on) { g.moveTo(X(xs[i]), Y(v)); on = true; } else g.lineTo(X(xs[i]), Y(v));
    last = [X(xs[i]), Y(v)]; if (!first) first = last;
  }
  const glow = parseFloat(css("--glow")) || 0;
  if (glow) { g.shadowColor = color; g.shadowBlur = 8; }
  g.stroke();
  g.shadowBlur = 0;
  if (area && first && last) {          // soft fill down to the baseline
    g.lineTo(last[0], area.y); g.lineTo(first[0], area.y); g.closePath();
    const gr = g.createLinearGradient(0, area.top, 0, area.y);
    gr.addColorStop(0, hexA(color, glow ? 0.28 : 0.18)); gr.addColorStop(1, hexA(color, 0));
    g.fillStyle = gr; g.fill();
  }
  if (last && S && !S.done) {           // live cursor at the newest sample
    g.fillStyle = color; g.beginPath(); g.arc(last[0], last[1], 3.5, 0, Math.PI * 2); g.fill();
    g.strokeStyle = hexA(color, .35); g.lineWidth = 5; g.beginPath(); g.arc(last[0], last[1], 6, 0, Math.PI * 2); g.stroke();
  }
}
function hexA(c, a) {
  const m = /^#([0-9a-f]{6})$/i.exec(c); if (!m) return c;
  const n = parseInt(m[1], 16); return `rgba(${n >> 16},${(n >> 8) & 255},${n & 255},${a})`;
}
function drawAll() {
  chart("c-angle", -10, 110, [0, 30, 60, 90], (g, X, Y, w, h, L, Rr) => {
    g.fillStyle = css("--accent-soft"); g.fillRect(L, Y(ENV.rom[1]), w - L - Rr, Y(ENV.rom[0]) - Y(ENV.rom[1]));
    // speed-limit style: area beyond the envelope is faintly red
    g.fillStyle = hexA(css("--bad"), .05); g.fillRect(L, 6, w - L - Rr, Y(ENV.rom[1]) - 6); g.fillRect(L, Y(ENV.rom[0]), w - L - Rr, h - 18 - Y(ENV.rom[0]));
    g.setLineDash([4, 4]); g.strokeStyle = css("--accent"); g.globalAlpha = .45; g.lineWidth = 1;
    for (const v of ENV.rom) { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(w - Rr, Y(v)); g.stroke(); }
    g.setLineDash([]); g.globalAlpha = 1;
    if (S) line(g, X, Y, S.t, S.a, css("--accent"), 2, null, { y: h - 18, top: 6 });
  });
  chart("c-torque", -6, 6, [-4, 0, 4], (g, X, Y, w, h, L, Rr, T, B) => {
    g.setLineDash([4, 4]); g.strokeStyle = css("--bad"); g.globalAlpha = .55; g.lineWidth = 1;
    for (const v of [-ENV.tmax, ENV.tmax]) { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(w - Rr, Y(v)); g.stroke(); }
    g.setLineDash([]); g.globalAlpha = 1;
    if (!S) return;
    // override ticks along the bottom
    g.fillStyle = css("--requested");
    for (let i = 0; i < S.t.length; i++) if ((S.f[i] & OVR_MASK) && !(S.f[i] & F.FALLBACK)) g.fillRect(X(S.t[i]) - 1, h - B - 6, 2, 6);
    line(g, X, Y, S.t, S.rq, css("--requested"), 1.4, [-6, 6]);
    line(g, X, Y, S.t, S.ap, css("--accent"), 2);
  });
  chart("c-speed", 0, 180, [0, 90, 180], (g, X, Y, w, h, L, Rr) => {
    g.fillStyle = hexA(css("--bad"), .06); g.fillRect(L, 6, w - L - Rr, Y(ENV.vmax) - 6);
    g.strokeStyle = css("--bad"); g.lineWidth = 1.2; g.setLineDash([4, 4]);
    g.beginPath(); g.moveTo(L, Y(ENV.vmax)); g.lineTo(w - Rr, Y(ENV.vmax)); g.stroke(); g.setLineDash([]);
    if (S) line(g, X, Y, S.t, S.v.map(Math.abs), css("--accent"), 2, [0, 180], { y: h - 18, top: 6 });
  });
}
window.addEventListener("resize", schedule);

/* ------------------------------------------------------------ log */
function clearLog() { $("log").innerHTML = ""; logCount = 0; $("log-count").textContent = "0"; }
function addLog(e) {
  const [label, sev] = e.kind === "_iter" ? ["AI iteration", "info"] : (EVENT_TEXT[e.kind] || [e.kind, "info"]);
  const li = document.createElement("li");
  li.className = `ev ${sev}`;
  li.innerHTML = `<span class="ev-t">${e.t.toFixed(3)}</span><span class="ev-dot"></span><div><div class="ev-k">${label}</div><div class="ev-d"></div></div>`;
  li.querySelector(".ev-d").textContent = e.detail;
  $("log").appendChild(li);
  logCount++; $("log-count").textContent = String(logCount);
  li.scrollIntoView({ block: "nearest" });
}
function onEvent(m) {
  if (S && ["revoked", "controller_died", "controller_exited"].includes(m.kind) && S.revokeT === null) S.revokeT = m.t;
  addLog(m);
  schedule();
}

/* ------------------------------------------------------------ verdict */
function onSummary(m) {
  const s = m.summary; if (!S) return;
  S.done = true;
  const v = $("verdict");
  const cls = s.verdict === "PASSED" ? "pass" : s.verdict === "CORRECTED" ? "corr" : "rej";
  v.className = "verdict " + cls; v.hidden = false;
  $("v-icon").textContent = cls === "pass" ? "✓" : cls === "corr" ? "!" : "✕";
  const estopped = /emergency stop/.test(s.reason || "");
  $("v-title").textContent = estopped ? "Stopped by the operator" : s.verdict === "PASSED" ? "Passed"
    : s.verdict === "CORRECTED" ? "Passed with corrections" : "Rejected";
  headline(estopped ? "Emergency stop. The fallback brought the arm to rest."
    : s.verdict === "PASSED" ? "Passed. The arm stayed in its safe range."
    : s.verdict === "CORRECTED" ? "Survived, but the cage had to correct it."
    : "Rejected. The cage took the arm away from this code.");
  $("dock-live").classList.remove("on");
  $("cta").hidden = false;
  $("v-reason").textContent = s.reason ? s.reason.replace(/^revoked by governor: /, "Revoked: ").replace(/^controller died: /, "Died: ")
    : (cls === "pass" ? "Accepted. The cage barely had to touch it." : "Survived, but the cage had to correct it often.");
  const held = s.envelope_held ? "held" : "BREACHED";
  $("v-stats").innerHTML = `<span>Corrections<b>${s.overrides ?? "–"}</b></span><span>Peak speed<b>${Math.round(s.max_speed_dps)}°/s</b></span><span>Max torque<b>${s.max_abs_torque_applied.toFixed(2)} N·m</b></span>`;
  void held;
  if (s.verdict === "PASSED") setJoint(S.a.at(-1) ?? 20, "");
}

function onTruth(m) {
  const t = m.truth; if (!t) return;
  const box = $("v-stats"); if (!box) return;
  const el = document.createElement("span");
  el.innerHTML = `Independent judge<b class="${t.envelope_held ? "ok" : "bad"}">${t.envelope_held ? "in range" : "BREACHED"}</b>`;
  if (!t.envelope_held) headline("The independent judge saw the arm leave its safe range.");
  el.title = `Joint process: angle ${t.angle_range_deg[0]}–${t.angle_range_deg[1]}°, peak ${t.max_speed_dps} °/s` + (t.drive_sto ? ", drive watchdog cut the torque" : "");
  box.prepend(el);
}

/* ------------------------------------------------------------ care plan */
let C = null;
const esc = (x) => String(x ?? "").replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
function stp(n, cls, status) {
  const li = $("stp" + n); li.classList.remove("done", "active", "blocked"); if (cls) li.classList.add(cls);
  $("s" + n).textContent = status;
}
function renderClinic() {
  if (!C) return;
  const e = C.envelope, t = C.trial, a = C.approval;
  if (C.clinician && !$("f-clin").value) $("f-clin").value = C.clinician;   // survive a page reload
  if (C.patient && e) { $("f-pt").value = C.patient; $("f-rom0").value = e.rom_min; $("f-rom1").value = e.rom_max; $("f-vmax").value = e.vmax; $("f-tmax").value = e.tmax; }
  stp(1, e ? "done" : "active", e ? `${C.patient} · ${e.rom_min}–${e.rom_max}° · ${e.vmax}°/s · ${e.tmax} N·m` : "not set");
  const ok = t && t.verdict !== "REJECTED" && t.envelope_held;
  stp(2, !e ? "" : t ? (ok ? "done" : "blocked") : "active",
      !e ? "set the safe range first" : t ? `iteration ${t.iteration}: ${t.verdict}` : "ready");
  stp(3, a ? "done" : ok ? "active" : "", a ? `approved by ${a.clinician}` : t ? (ok ? "ready to review" : "not approvable") : "no trial yet");
  stp(4, C.session ? "done" : a ? "active" : "", C.session ? `last: ${C.session.verdict}` : a ? "ready" : "locked");
  if (t) {
    const vc = t.verdict === "PASSED" ? "pass" : t.verdict === "CORRECTED" ? "corr" : "fail";
    $("evid").innerHTML = `
      <div class="ev-row"><span>Trial verdict</span><b class="${vc}">${esc(t.verdict)}</b></div>
      ${t.reason ? `<div class="ev-row"><span>Reason</span><b>${esc(t.reason)}</b></div>` : ""}
      <div class="ev-row"><span>Independent judge</span><b class="${t.envelope_held ? "pass" : "fail"}">${t.envelope_held ? "envelope held" : "envelope breached"}</b></div>
      <div class="ev-row"><span>Cage overrides</span><b>${t.overrides ?? "–"}</b></div>
      ${t.verdict === "CORRECTED" ? `<p class="note warn-note">The cage had to correct this controller ${t.overrides} times in the trial (with a simulated spasm). Approve only if that is acceptable for this patient.</p>` : ""}
      <div class="ev-row"><span>Delivered motion</span><b>${t.angle_range_deg ? `${t.angle_range_deg[0].toFixed(0)}–${t.angle_range_deg[1].toFixed(0)}°, peak ${Math.round(t.max_speed_dps)} °/s` : "–"}</b></div>
      <div class="ev-row"><span>Binary SHA-256</span><b class="hash">${esc(t.binary_sha256.slice(0, 16))}…</b></div>
      <div class="ev-row"><span>Source</span><b class="hash">${esc(t.source)}</b></div>`;
  }
  $("approve-go").disabled = busy || !ok || !!a;
  $("session-go").disabled = busy || !a || !target;
  if (a) $("sess-note").innerHTML = `Approved binary <span class="hash">${esc(a.binary_sha256.slice(0, 12))}</span> by ${esc(a.clinician)} at ${esc(a.approved_at)}. The server re-checks the hash before every session.`;
  // open the step that needs attention, and say it in one sentence
  const focus = !e ? 1 : !ok ? 2 : !a ? 3 : 4;
  const nDone = [!!e, !!ok, !!a, !!C.session].filter(Boolean).length;
  $("plan-progress").textContent = `${nDone} of 4 done`;
  $("plan-next").textContent = !e ? "Set the patient's safe range"
    : !t ? "Let the AI draft the exercise and trial it in the cage"
    : !ok ? "The trial did not pass. Run the AI again."
    : !a ? "Review the evidence and approve this exact binary"
    : C.session ? `Last session: ${C.session.verdict.toLowerCase()}. You can run it again.`
    : "Ready. Start the patient session.";
  for (let n = 1; n <= 4; n++) if (!$("stp" + n).dataset.user) $("stp" + n).classList.toggle("open", n === focus);
  renderRecord(C.audit || []);
}
const REC_TEXT = {
  envelope_set: ["Envelope set", ""], trial: ["AI trial in the cage", ""], approved: ["Approved", "r-good"],
  session_start: ["Patient session started", ""], session_end: ["Patient session ended", ""],
  emergency_stop: ["Emergency stop", "r-bad"], session_refused: ["Session refused", "r-bad"],
};
function renderRecord(items) {
  const ol = $("rec"); if (!items.length) return;
  ol.innerHTML = "";
  for (const r of items.slice().reverse()) {
    const [label, cls] = REC_TEXT[r.kind] || [r.kind, ""];
    const d = Object.entries(r).filter(([k]) => !["time", "kind"].includes(k))
      .map(([k, v]) => `${k}: ${typeof v === "object" ? JSON.stringify(v) : v}`).join(" · ");
    const li = document.createElement("li"); li.className = cls;
    li.innerHTML = `<span class="r-t">${esc(r.time.slice(11))}</span><div><div class="r-k">${esc(label)}</div><div class="r-d">${esc(d)}</div></div>`;
    ol.appendChild(li);
  }
  $("rec-count").textContent = String(items.length);
}
document.querySelectorAll(".stp-h").forEach((h) => h.onclick = () => {
  const li = h.parentElement; li.dataset.user = "1"; li.classList.toggle("open");
});
$("env-go").onclick = () => post("/api/clinic/envelope", {
  patient: $("f-pt").value, clinician: $("f-clin").value, rom_min: $("f-rom0").value, rom_max: $("f-rom1").value,
  vmax: $("f-vmax").value, tmax: $("f-tmax").value,
}).then((ok) => ok && toast("Envelope written to the QNX target."));
$("approve-go").onclick = () => post("/api/clinic/approve", { clinician: $("f-clin").value, note: $("f-note").value });
$("session-go").onclick = () => post("/api/clinic/session", {}).then((ok) => ok && setBusy(true));
$("ai-go").addEventListener("click", () => showTab("ai"));
$("estop").onclick = () => post("/api/estop", { by: $("f-clin").value || "operator" })
  .then((ok) => ok && toast("Emergency stop sent to /dev/safecage/ctl"));
async function loadClinic() { C = await (await fetch("/api/clinic")).json(); if (C.envelope) useEnvelope(C.envelope); renderClinic(); }

/* ------------------------------------------------------------ AI loop */
let aiReplay = false;
function hl(code) {
  const esc = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  return code.split(/(\/\*[\s\S]*?\*\/|\/\/[^\n]*)/).map((part, i) => {
    if (i % 2) return `<span class="c">${esc(part)}</span>`;
    return esc(part)
      .replace(/\b(\d+(?:\.\d+)?(?:e-?\d+)?)\b/g, '<span class="num">$1</span>')
      .replace(/\b(static|const|double|float|int|void|return|if|else|for|while|bool|struct|#include|#define)\b/g, '<span class="kw">$1</span>');
  }).join("");
}
function aiCard(n) {
  let el = document.getElementById("it-" + n);
  if (el) return el;
  el = document.createElement("div"); el.className = "it"; el.id = "it-" + n;
  el.innerHTML = `<div class="it-h"><b>Iteration ${n}</b><div class="steps">
      <span class="st" data-s="writing">Code</span><span class="st" data-s="static">Static</span><span class="st" data-s="build">Build</span><span class="st" data-s="cage">Cage</span></div></div>
    <div class="it-body"><p class="it-notes"><span class="spinner"></span>Waiting for the AI…</p></div>`;
  $("ai").appendChild(el);
  aiIters = Math.max(aiIters, n); $("ai-count").textContent = String(aiIters);
  el.scrollIntoView({ block: "nearest" });
  return el;
}
function stepState(n, s, cls, title) {
  const st = aiCard(n).querySelector(`.st[data-s="${s}"]`);
  st.className = "st " + cls; if (title) st.title = title;
}
function onAi(m) {
  if (m.type === "ai_start") {
    aiReplay = !!m.replay; aiIters = 0;
    $("ai").innerHTML = `<div class="ai-head"><b>Prescription</b> · ${m.model === "recorded" ? "recorded run" : "model: " + m.model}<br></div>`;
    $("ai").firstChild.appendChild(document.createTextNode(m.prescription));
    showTab("ai");
    return;
  }
  if (m.type === "ai_step" && m.step === "writing") { aiCard(m.iter); stepState(m.iter, "writing", "run"); return; }
  if (m.type === "ai_code") {
    const el = aiCard(m.iter); stepState(m.iter, "writing", "ok");
    const body = el.querySelector(".it-body");
    body.innerHTML = `<pre class="code"></pre><button class="more">Show all ${m.code.split("\n").length} lines</button>`;
    body.querySelector(".code").innerHTML = hl(m.code);
    body.querySelector(".more").onclick = (e) => { body.querySelector(".code").classList.toggle("open"); e.target.remove(); };
    if (m.notes) { const p = document.createElement("p"); p.className = "it-notes"; p.textContent = m.notes; body.appendChild(p); }
    return;
  }
  if (m.type === "ai_step") {
    stepState(m.iter, m.step, m.ok ? "ok" : "fail", m.text);
    if (!m.ok) { const p = document.createElement("p"); p.className = "it-notes"; p.textContent = `${m.step === "static" ? "Static screen" : "Build"} failed: ${m.text}`; aiCard(m.iter).querySelector(".it-body").appendChild(p); }
    if (m.step === "build" && m.ok) stepState(m.iter, "cage", "run");
    return;
  }
  if (m.type === "ai_report") {
    const el = aiCard(m.iter);
    if (m.done) { el.classList.add("passed"); stepState(m.iter, "cage", "ok", "PASSED"); }
    else if (el.querySelector('.st[data-s="cage"]').classList.contains("run")) {
      const corr = /verdict: CORRECTED/.test(m.feedback);
      stepState(m.iter, "cage", corr ? "warn" : "fail");
    }
    if (m.feedback && !m.done) {
      const h = document.createElement("div"); h.className = "report-h"; h.textContent = "Report sent back to the AI";
      const r = document.createElement("pre"); r.className = "report"; r.textContent = m.feedback;
      el.appendChild(h); el.appendChild(r);
    }
    if (m.done) {
      const p = document.createElement("p"); p.className = "it-notes"; p.innerHTML = "<b>Accepted by the cage.</b> No further repair needed.";
      el.appendChild(p);
    }
    return;
  }
  if (m.type === "ai_end") { aiReplay = false; }
}

/* ------------------------------------------------------------ stream */
function connect() {
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    switch (m.type) {
      case "session": newSession(m); break;
      case "tlm": onTlm(m); break;
      case "event": onEvent(m); break;
      case "summary": onSummary(m); break;
      case "truth": onTruth(m); break;
      case "clinic": C = m.state; if (C.envelope) useEnvelope(C.envelope); renderClinic(); break;
      case "stderr": addLog({ t: S && S.t.length ? S.t.at(-1) : 0, kind: "_stderr", detail: m.text }); break;
      case "error": toast(m.text); break;
      case "idle": setBusy(false); $("dock-live").classList.remove("on"); break;
      default: if (m.type.startsWith("ai_")) onAi(m);
    }
  };
}
EVENT_TEXT._stderr = ["Controller said", "info"];

setJoint(20, "idle");
loadStatus().then(loadClinic).catch(() => toast("Could not reach the dashboard server."));
connect();
drawAll();
