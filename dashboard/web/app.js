"use strict";
/* Safe Cage dashboard - vanilla JS, no build step. */

const $ = (id) => document.getElementById(id);
const ENV = { tmax: 4, vmax: 90, rom: [0, 90] };
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
seg("speed", (v) => speed = parseFloat(v));
document.querySelectorAll(".tab").forEach((t) => t.onclick = () => showTab(t.dataset.tab));
function showTab(name) {
  document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("on", t.dataset.tab === name));
  $("tab-log").hidden = name !== "log";
  $("tab-ai").hidden = name !== "ai";
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
  badge.className = "badge " + (msg.mode === "live" ? "live" : msg.mode === "ai" ? "ai" : "replay");
  badge.textContent = msg.mode === "live" ? "LIVE ON QNX" : msg.mode === "ai" ? (msg.replay ? "AI · RECORDED" : "AI · LIVE ON QNX") : "REPLAY";
  if (msg.mode === "ai" && aiReplay) { badge.className = "badge ai"; badge.textContent = "AI · RECORDED RUN"; }
  $("title").textContent = msg.title;
  $("subtitle").textContent = msg.subtitle || "";
  $("verdict").hidden = true;
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
  // overrides budget
  let i0 = S.t.length - 1; while (i0 > 0 && S.t[i0 - 1] > m.t - 0.5) i0--;
  const win = m.o - (i0 > 0 ? S.o[i0 - 1] : 0);
  S.peakWin = Math.max(S.peakWin, win);
  setBar("bar-win", "b-win", win, 25, S.peakWin);
  setBar("bar-tot", "b-tot", m.o, 100);
  const st = S.revokeT !== null ? "revoked" : (m.f & F.FALLBACK) ? "fallback" : (m.f & OVR_MASK) ? "override" : "";
  setJoint(m.a, st);
  schedule();
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
$("env-sector").setAttribute("d", sector(0, 90, R - 4));
$("out-sector").setAttribute("d", sector(90, 150, R - 4));
const [lx, ly] = pt(90, R + 6); $("lbl-90").setAttribute("x", lx - 10); $("lbl-90").setAttribute("y", ly - 2);
function setJoint(angle, state) {
  $("forearm").setAttribute("transform", `rotate(${(-angle).toFixed(2)} ${CX} ${CY})`);
  const j = $("joint");
  j.classList.toggle("override", state === "override");
  j.classList.toggle("fallback", state === "fallback");
  j.classList.toggle("revoked", state === "revoked");
  $("j-state").textContent = state === "revoked" ? "Revoked · fallback in charge"
    : state === "fallback" ? "Fallback in charge" : state === "override" ? "Cage overriding" : "Controller in charge";
  $("j-state").className = "j-state " + (state || "ok");
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
function line(g, X, Y, xs, ys, color, width, clip) {
  g.strokeStyle = color; g.lineWidth = width; g.lineJoin = "round"; g.lineCap = "round"; g.beginPath();
  let on = false;
  for (let i = 0; i < xs.length; i++) {
    let v = ys[i]; if (v === null || Number.isNaN(v)) { on = false; continue; }
    if (clip) v = Math.max(clip[0], Math.min(clip[1], v));
    if (!on) { g.moveTo(X(xs[i]), Y(v)); on = true; } else g.lineTo(X(xs[i]), Y(v));
  }
  g.stroke();
}
function drawAll() {
  chart("c-angle", -10, 110, [0, 30, 60, 90], (g, X, Y, w, h, L, Rr) => {
    g.fillStyle = css("--accent-soft"); g.fillRect(L, Y(90), w - L - Rr, Y(0) - Y(90));
    g.setLineDash([4, 4]); g.strokeStyle = css("--accent"); g.globalAlpha = .45; g.lineWidth = 1;
    for (const v of ENV.rom) { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(w - Rr, Y(v)); g.stroke(); }
    g.setLineDash([]); g.globalAlpha = 1;
    if (S) line(g, X, Y, S.t, S.a, css("--accent"), 2);
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
    g.strokeStyle = css("--bad"); g.lineWidth = 1.2; g.setLineDash([4, 4]);
    g.beginPath(); g.moveTo(L, Y(90)); g.lineTo(w - Rr, Y(90)); g.stroke(); g.setLineDash([]);
    if (S) line(g, X, Y, S.t, S.v.map(Math.abs), css("--accent"), 2, [0, 180]);
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
  $("v-title").textContent = s.verdict;
  $("v-reason").textContent = s.reason ? s.reason.replace(/^revoked by governor: /, "Revoked: ").replace(/^controller died: /, "Died: ")
    : (cls === "pass" ? "Accepted. The cage barely had to touch it." : "Survived, but the cage had to correct it often.");
  const held = s.envelope_held ? "held" : "BREACHED";
  $("v-stats").innerHTML = `<span>Envelope<b>${held}</b></span><span>Overrides<b>${s.overrides ?? "–"}</b></span><span>Peak speed<b>${Math.round(s.max_speed_dps)}°/s</b></span><span>Max torque<b>${s.max_abs_torque_applied.toFixed(2)}</b></span>`;
  if (s.verdict === "PASSED") setJoint(S.a.at(-1) ?? 20, "");
}

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
      case "stderr": addLog({ t: S && S.t.length ? S.t.at(-1) : 0, kind: "_stderr", detail: m.text }); break;
      case "error": toast(m.text); break;
      case "idle": setBusy(false); break;
      default: if (m.type.startsWith("ai_")) onAi(m);
    }
  };
}
EVENT_TEXT._stderr = ["Controller said", "info"];

loadStatus().catch(() => toast("Could not reach the dashboard server."));
connect();
drawAll();
