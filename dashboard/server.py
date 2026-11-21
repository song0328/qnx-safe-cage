"""Safe Cage dashboard server (host side, Python standard library only).

  python dashboard/server.py [--host qnx] [--port 8765]     then open http://localhost:8765

Live mode runs the governor on the QNX target over SSH with -L and streams its telemetry to the
browser (Server-Sent Events). The AI mode runs the repair loop step by step. Replay mode streams
recorded runs from results/ at real speed, so the dashboard also works without a QNX target.

Care plan (the clinical workflow): the clinician sets the patient's envelope (written to the target
and enforced by the governor and judged by the joint) -> the AI drafts a controller and it is tried in
the cage against a virtual patient with a spasm -> the clinician approves a PASSED trial (the binary's
SHA-256 is pinned) -> only that exact binary may run a patient session -> emergency stop through the
QNX resource manager (/dev/safecage/ctl). Every step is appended to results/clinic/audit.jsonl.
"""
import hashlib
import argparse
import json
import pathlib
import queue
import re
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

ROOT = pathlib.Path(__file__).resolve().parents[1]
WEB = ROOT / "dashboard" / "web"
sys.path.insert(0, str(ROOT / "tools"))

REMOTE_DIR = "/data/sc"
ARGS = None

CONTROLLERS = [
    ("good_rom", "Reference", "Smooth 20-60° range-of-motion exercise"),
    ("bad_torque", "Too strong", "Huge gains, aims past the range of motion"),
    ("bad_nan", "Divide by zero", "Returns NaN torque after 2 s"),
    ("bad_loop", "Infinite loop", "Stops answering after 3 s"),
    ("bad_slow", "Too slow", "3 ms of work in a 2 ms tick"),
    ("bad_crash", "Bad pointer", "Dereferences an unset table after 3 s"),
    ("bad_mem", "Memory leak", "1 MB per step, never checks malloc"),
    ("bad_escape", "Escape attempt", "kill(governor), fork(), priority 60"),
    ("bad_hog", "CPU hog", "Raises itself to priority 63 and spins"),
]


# ---------------------------------------------------------------- event bus
class Bus:
    def __init__(self):
        self.subs = []
        self.lock = threading.Lock()
        self.busy = False

    def subscribe(self):
        q = queue.Queue(maxsize=5000)
        with self.lock:
            self.subs.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subs:
                self.subs.remove(q)

    def emit(self, _type, **data):
        msg = json.dumps({"type": _type, **data})
        with self.lock:
            for q in list(self.subs):
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    pass


BUS = Bus()

# ---------------------------------------------------------------- care plan state
CLINIC_DIR = ROOT / "results" / "clinic"
CLINIC = {"patient": None, "clinician": None, "envelope": None, "trial": None, "approval": None, "session": None}
ENV_LIMITS = {"tmax": (0.5, 6.0), "vmax": (10.0, 180.0), "rom_min": (-5.0, 60.0), "rom_max": (30.0, 140.0)}


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def audit(kind, **data):
    CLINIC_DIR.mkdir(parents=True, exist_ok=True)
    rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": kind, **data}
    with (CLINIC_DIR / "audit.jsonl").open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")
    BUS.emit("clinic", state=clinic_state())
    return rec


def audit_tail(n=40):
    p = CLINIC_DIR / "audit.jsonl"
    if not p.exists():
        return []
    return [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()][-n:]


def clinic_state():
    return {**CLINIC, "audit": audit_tail()}


def env_file():
    return f"{REMOTE_DIR}/env_{CLINIC['patient']}.txt" if CLINIC["envelope"] else None


def parse_line(line, extra=None):
    """Turn one governor stdout line into a bus event."""
    extra = extra or {}
    if line.startswith("TLM "):
        BUS.emit("tlm", **json.loads(line[4:]), **extra)
    elif line.startswith("EVENT "):
        m = re.match(r"EVENT t=([\d.]+) (\w+): (.*)", line)
        if m:
            BUS.emit("event", t=float(m.group(1)), kind=m.group(2), detail=m.group(3), **extra)
    elif line.startswith("SUMMARY "):
        s = json.loads(line[8:])
        BUS.emit("summary", summary=s, **extra)
        return s
    elif line.startswith("[escape]") or line.startswith("cage:") or line.startswith("controller:"):
        BUS.emit("stderr", text=line, **extra)
    return None


# ---------------------------------------------------------------- live runs on the target
def ssh_cmd(cmd):
    return ["ssh", "-o", "BatchMode=yes", "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=5", ARGS.host, cmd]


def target_ok():
    try:
        r = subprocess.run(ssh_cmd("uname -sr"), capture_output=True, text=True, timeout=10)
        return r.returncode == 0, r.stdout.strip()
    except Exception as e:  # noqa: BLE001
        return False, str(e)


def run_live(ctrl, T=8.0, spasm=False, extra=None, env=None):
    e = f" -e {env}" if env else ""
    jflags = f"-T {T}" + (" -s" if spasm else "") + " -w 20" + e
    # the joint process (drive + forearm + patient, with a 20 ms drive watchdog) starts first and judges
    # the run; the governor streams telemetry; the joint's verdict is printed last as a TRUTH line
    cmd = (f"ulimit -c 0; cd {REMOTE_DIR}; chmod 755 {REMOTE_DIR}/*; slay -f joint >/dev/null 2>&1; rm -f /dev/shmem/sc_drive; mkdir -p /dev/shmem/sc_dash; "
           f"rm -f /dev/shmem/sc_dash/truth.json; ./joint {jflags} -o /dev/shmem/sc_dash > /dev/shmem/sc_dash.joint 2>&1 & JP=$!; "
           f"./governor -c {REMOTE_DIR}/{ctrl} -T {T} -L{e} -o /dev/shmem/sc_dash 2>&1; wait $JP; "
           f"echo \"TRUTH $(cat /dev/shmem/sc_dash/truth.json)\"")   # quoted: ksh would brace-expand {a,b}
    p = subprocess.Popen(ssh_cmd(cmd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    summary = None
    for line in p.stdout:
        line = line.rstrip("\n")
        if line.startswith("TRUTH {"):
            truth = json.loads(line[6:])
            if summary is not None:
                summary["truth"] = truth
            BUS.emit("truth", truth=truth, **(extra or {}))
            continue
        s = parse_line(line, extra)
        summary = s or summary
    p.wait()
    return summary


def job_run(ctrl, spasm):
    try:
        meta = next((c for c in CONTROLLERS if c[0] == ctrl), (ctrl, ctrl, ""))
        BUS.emit("session", mode="live", controller=ctrl, title=meta[1], subtitle=meta[2], T=8.0)
        s = run_live(ctrl, spasm=spasm)
        if not s:
            BUS.emit("error", text="The run did not produce a summary. Is the QNX target reachable?")
    finally:
        BUS.emit("idle")
        BUS.busy = False


def job_ai(rx, model, iters):
    import llm_loop as L
    try:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        out = ROOT / "results" / "llm" / f"{stamp}-dash"
        out.mkdir(parents=True)
        BUS.emit("ai_start", prescription=rx, model=model or "default", iters=iters)
        prompt = L.BRIEF.format(api=L.API_H.strip(), rx=rx)
        code = ""
        for n in range(1, iters + 1):
            d = out / f"iter{n}"
            d.mkdir()
            BUS.emit("ai_step", iter=n, step="writing", text="The AI is writing the controller")
            (d / "prompt.txt").write_text(prompt, encoding="utf-8")
            resp = L.llm(prompt, model)
            (d / "response.txt").write_text(resp, encoding="utf-8")
            code = L.extract_c(resp)
            (d / "controller.c").write_text(code, encoding="utf-8")
            notes = resp.split("```")[-1].strip()[:400]
            BUS.emit("ai_code", iter=n, code=code, notes=notes)
            probs = L.static_screen(code)
            BUS.emit("ai_step", iter=n, step="static", ok=not probs, text="; ".join(probs) or "no forbidden calls")
            if probs:
                fb = L.feedback_text(None, None, static_problems=probs)
            else:
                name = f"dash_{stamp[-6:]}_i{n}"
                ok, blog = L.build(d / "controller.c", name)
                BUS.emit("ai_step", iter=n, step="build", ok=ok, text="qcc -Vgcc_ntox86_64" if ok else blog[-600:])
                if not ok:
                    fb = L.feedback_text(None, None, build_log=blog)
                else:
                    subprocess.run(["scp", "-q", "-o", "BatchMode=yes", str(ROOT / "build" / "x86_64" / name),
                                    f"{ARGS.host}:{REMOTE_DIR}/"], timeout=120)
                    BUS.emit("session", mode="ai", controller=name, title=f"AI iteration {n}",
                             subtitle=rx[:90], T=8.0, iter=n, envelope=CLINIC["envelope"])
                    # the trial runs against a virtual patient with a spasm at 5 s, inside the patient's envelope
                    s = run_live(name, extra={"iter": n}, env=env_file(), spasm=True)
                    rank = {"PASSED": 2, "CORRECTED": 1}
                    old = CLINIC["trial"] if CLINIC["trial"] and CLINIC["trial"].get("prescription") == rx else None
                    better = s and (old is None or rank.get(s["verdict"], 0) >= rank.get(old["verdict"], 0))
                    if s and better:   # keep the best iteration of this prescription as the approval candidate
                        CLINIC["trial"] = {"controller": name, "iteration": n, "prescription": rx,
                                           "verdict": s["verdict"], "reason": s.get("reason"),
                                           "overrides": s.get("overrides"),
                                           "envelope_held": (s.get("truth") or {}).get("envelope_held"),
                                           "max_speed_dps": (s.get("truth") or {}).get("max_speed_dps"),
                                           "angle_range_deg": (s.get("truth") or {}).get("angle_range_deg"),
                                           "binary_sha256": sha256(ROOT / "build" / "x86_64" / name),
                                           "source_sha256": sha256(d / "controller.c"),
                                           "source": str((d / "controller.c").relative_to(ROOT)),
                                           "envelope": CLINIC["envelope"], "patient": CLINIC["patient"]}
                        CLINIC["approval"] = None
                    if s:
                        audit("trial", patient=CLINIC["patient"], controller=name, iteration=n, verdict=s["verdict"],
                              reason=s.get("reason") or "", envelope_held=(s.get("truth") or {}).get("envelope_held"),
                              binary_sha256=sha256(ROOT / "build" / "x86_64" / name)[:16])
                    if not s:
                        fb = "The run produced no result."
                    else:
                        fb = L.feedback_text(s, [], rx=None)
                        (d / "summary.json").write_text(json.dumps(s), encoding="utf-8")
                        if s["verdict"] == "PASSED":
                            BUS.emit("ai_report", iter=n, feedback=fb, done=True)
                            break
            (d / "feedback.txt").write_text(fb, encoding="utf-8")
            BUS.emit("ai_report", iter=n, feedback=fb, done=False)
            prompt = L.BRIEF.format(api=L.API_H.strip(), rx=rx) + "\n\n" + L.REPAIR.format(feedback=fb, code=code.strip())
        BUS.emit("ai_end")
    except Exception as e:  # noqa: BLE001
        BUS.emit("error", text=f"AI loop failed: {e}")
    finally:
        BUS.emit("idle")
        BUS.busy = False


def job_session():
    """A patient session with the approved controller only."""
    try:
        a = CLINIC["approval"]
        BUS.emit("session", mode="patient", controller=a["controller"], title=f"Patient session · {CLINIC['patient']}",
                 subtitle=f"Approved by {a['clinician']} · binary {a['binary_sha256'][:12]}", T=8.0,
                 envelope=CLINIC["envelope"])
        audit("session_start", patient=CLINIC["patient"], controller=a["controller"], clinician=a["clinician"],
              binary_sha256=a["binary_sha256"][:16])
        s = run_live(a["controller"], env=env_file())
        if not s:
            BUS.emit("error", text="The session did not produce a summary. Is the QNX target reachable?")
            audit("session_end", patient=CLINIC["patient"], result="no result")
            return
        tr = s.get("truth") or {}
        CLINIC["session"] = {"verdict": s["verdict"], "reason": s.get("reason"), "overrides": s.get("overrides"),
                             "envelope_held": tr.get("envelope_held"), "angle_range_deg": tr.get("angle_range_deg"),
                             "max_speed_dps": tr.get("max_speed_dps"), "estop": "emergency stop" in (s.get("reason") or "")}
        audit("session_end", patient=CLINIC["patient"], verdict=s["verdict"], reason=s.get("reason") or "",
              envelope_held=tr.get("envelope_held"), overrides=s.get("overrides"))
    finally:
        BUS.emit("idle")
        BUS.busy = False


def estop(who):
    r = subprocess.run(ssh_cmd("echo stop > /dev/safecage/ctl"), capture_output=True, text=True, timeout=10)
    ok = r.returncode == 0
    audit("emergency_stop", by=who or "operator", ok=ok, detail=(r.stderr or "").strip()[:200])
    return ok


# ---------------------------------------------------------------- replay of recorded runs
def replay_dir(run_dir, extra=None, speed=1.0):
    run_dir = pathlib.Path(run_dir)
    rows = [l.split(",") for l in (run_dir / "run.csv").read_text().splitlines() if l.strip()]
    events = [json.loads(l) for l in (run_dir / "events.jsonl").read_text().splitlines() if l.strip()] \
        if (run_dir / "events.jsonl").exists() else []
    summary = json.loads((run_dir / "summary.json").read_text())
    extra = extra or {}
    t0 = time.time()
    ei, agg, late, ovr = 0, 0, 0.0, 0
    win = []
    for i, r in enumerate(rows):
        t = float(r[0])
        while ei < len(events) and events[ei]["t"] <= t:
            e = events[ei]
            BUS.emit("event", t=e["t"], kind=e["kind"], detail=e["detail"], **extra)
            ei += 1
        f = int(r[5])
        agg |= f
        late = max(late, float(r[6]))
        # recompute the override count the governor streams live
        ovr += 1 if (f & (1 | 2 | 8 | 16)) and not (f & 64) else 0
        if (i + 1) % 10 == 0:
            rq = float(r[3])
            BUS.emit("tlm", t=t, a=float(r[1]), v=float(r[2]), rq=rq if rq == rq else None, ap=float(r[4]),
                     f=agg, o=ovr, l=late, **extra)
            agg, late = 0, 0.0
            wait = t0 + t / speed - time.time()
            if wait > 0:
                time.sleep(wait)
    while ei < len(events):
        e = events[ei]
        BUS.emit("event", t=e["t"], kind=e["kind"], detail=e["detail"], **extra)
        ei += 1
    BUS.emit("summary", summary=summary, **extra)
    return summary


def replays():
    items = []
    names = {c[0]: c[1] for c in CONTROLLERS}
    for c in CONTROLLERS:
        if (ROOT / "results" / "v1-suite" / c[0] / "run.csv").exists():
            items.append({"id": f"suite/{c[0]}", "group": "Fault suite", "label": f"{names[c[0]]} ({c[0]})"})
    for loop in sorted((ROOT / "results" / "llm").glob("*/log.md")):
        meta = {}
        if (loop.parent / "meta.json").exists():
            meta = json.loads((loop.parent / "meta.json").read_text())
        model = {"default": "Claude", "haiku": "Claude Haiku"}.get(meta.get("model", ""), meta.get("model", "AI"))
        kind = "timing locked" if "strict" in meta.get("rx_file", "") else "fast prescription"
        items.append({"id": f"loop/{loop.parent.name}", "group": "AI repair loops", "label": f"{model} · {kind}"})
    for i, rt in enumerate(sorted((ROOT / "results" / "redteam").glob("*/log.md")), 1):
        for a in sorted(rt.parent.glob("attempt*/run/*/run.csv")):
            items.append({"id": f"redteam/{rt.parent.name}/{a.parents[2].name}", "group": "Red team",
                          "label": f"Round {i} · attempt {a.parents[2].name[7:]}"})
    return items


def job_replay(rid, speed):
    try:
        kind, rest = rid.split("/", 1)
        if kind == "suite":
            meta = next((c for c in CONTROLLERS if c[0] == rest), (rest, rest, ""))
            BUS.emit("session", mode="replay", controller=rest, title=meta[1], subtitle=meta[2], T=8.0)
            replay_dir(ROOT / "results" / "v1-suite" / rest, speed=speed)
        elif kind == "redteam":
            stamp, att = rest.split("/")
            base = ROOT / "results" / "redteam" / stamp / att
            idea = (base / "response.txt").read_text(encoding="utf-8").split("```")[-1].strip()[:300]
            BUS.emit("session", mode="replay", controller=att, title=f"Red team {att}", subtitle=idea, T=8.0)
            replay_dir(next((base / "run").glob("*/")), speed=speed)
        elif kind == "loop":
            base = ROOT / "results" / "llm" / rest
            rx = re.search(r"Prescription: (.*)", (base / "log.md").read_text(encoding="utf-8"))
            rx = rx.group(1) if rx else ""
            BUS.emit("ai_start", prescription=rx, model="recorded", iters=0, replay=True)
            for it in sorted(base.glob("iter*"), key=lambda p: int(p.name[4:])):
                n = int(it.name[4:])
                code = (it / "controller.c").read_text(encoding="utf-8")
                notes = (it / "response.txt").read_text(encoding="utf-8").split("```")[-1].strip()[:400]
                BUS.emit("ai_code", iter=n, code=code, notes=notes)
                time.sleep(1.0 / speed)
                st = (it / "static.txt").read_text(encoding="utf-8").strip() if (it / "static.txt").exists() else "ok"
                BUS.emit("ai_step", iter=n, step="static", ok=st == "ok", text="no forbidden calls" if st == "ok" else st)
                if (it / "build.txt").exists():
                    blog = (it / "build.txt").read_text(encoding="utf-8")
                    ok = "error" not in blog.lower()
                    BUS.emit("ai_step", iter=n, step="build", ok=ok, text="qcc -Vgcc_ntox86_64" if ok else blog[-600:])
                runs = list((it / "run").glob("*/summary.json")) if (it / "run").exists() else []
                if runs and runs[0].stat().st_size > 0:
                    BUS.emit("session", mode="ai", controller=runs[0].parent.name, title=f"AI iteration {n}",
                             subtitle=rx[:90], T=8.0, iter=n)
                    s = replay_dir(runs[0].parent, extra={"iter": n}, speed=speed)
                else:
                    s = None
                fb = (it / "feedback.txt").read_text(encoding="utf-8") if (it / "feedback.txt").exists() else ""
                BUS.emit("ai_report", iter=n, feedback=fb, done=bool(s and s["verdict"] == "PASSED"))
                time.sleep(0.8 / speed)
            BUS.emit("ai_end")
    except Exception as e:  # noqa: BLE001
        BUS.emit("error", text=f"Replay failed: {e}")
    finally:
        BUS.emit("idle")
        BUS.busy = False


# ---------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = BUS.subscribe()
            try:
                self.wfile.write(b"retry: 2000\n\n")
                while True:
                    try:
                        msg = q.get(timeout=15)
                        self.wfile.write(f"data: {msg}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                BUS.unsubscribe(q)
            return
        if self.path == "/api/clinic":
            return self._json(clinic_state())
        if self.path == "/api/status":
            ok, uname = target_ok()
            return self._json({"target": ok, "uname": uname, "busy": BUS.busy,
                               "controllers": [{"id": c[0], "title": c[1], "desc": c[2]} for c in CONTROLLERS],
                               "replays": replays()})
        path = "index.html" if self.path in ("/", "") else self.path.lstrip("/").split("?")[0]
        f = (WEB / path).resolve()
        if WEB.resolve() not in f.parents or not f.is_file():
            self.send_error(404)
            return
        ctype = {".html": "text/html; charset=utf-8", ".css": "text/css", ".js": "text/javascript",
                 ".svg": "image/svg+xml"}.get(f.suffix, "application/octet-stream")
        body = f.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        req = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/api/estop":                       # always allowed, even (especially) while busy
            return self._json({"ok": estop(req.get("by"))})
        if self.path == "/api/clinic/envelope":
            return self.set_envelope(req)
        if self.path == "/api/clinic/approve":
            return self.approve(req)
        if BUS.busy:
            return self._json({"error": "busy"}, 409)
        if self.path == "/api/run":
            ctrl = req.get("controller", "")
            if ctrl not in {c[0] for c in CONTROLLERS}:
                return self._json({"error": "unknown controller"}, 400)
            BUS.busy = True
            threading.Thread(target=job_run, args=(ctrl, bool(req.get("spasm"))), daemon=True).start()
        elif self.path == "/api/ai":
            rx = (req.get("prescription") or "").strip()
            if not rx:
                return self._json({"error": "empty prescription"}, 400)
            BUS.busy = True
            threading.Thread(target=job_ai, args=(rx, req.get("model") or None, int(req.get("iters", 4))),
                             daemon=True).start()
        elif self.path == "/api/clinic/session":
            a = CLINIC["approval"]
            if not a:
                return self._json({"error": "No approved controller. Approve a PASSED trial first."}, 400)
            binp = ROOT / "build" / "x86_64" / a["controller"]
            if not binp.exists() or sha256(binp) != a["binary_sha256"]:
                audit("session_refused", reason="binary does not match the approved SHA-256")
                return self._json({"error": "The controller binary changed after approval. Refused."}, 409)
            if a["envelope"] != CLINIC["envelope"]:
                return self._json({"error": "The envelope changed after approval. Re-run the trial."}, 409)
            BUS.busy = True
            threading.Thread(target=job_session, daemon=True).start()
        elif self.path == "/api/replay":
            BUS.busy = True
            threading.Thread(target=job_replay, args=(req.get("id", ""), float(req.get("speed", 1.0))),
                             daemon=True).start()
        else:
            return self._json({"error": "not found"}, 404)
        return self._json({"ok": True})


def _set_envelope(self, req):
    try:
        patient = re.sub(r"[^A-Za-z0-9_-]", "", str(req.get("patient", "")))[:24]
        clinician = str(req.get("clinician", "")).strip()[:60]
        e = {"tmax": float(req["tmax"]), "slew": 60.0, "vmax": float(req["vmax"]),
             "rom_min": float(req["rom_min"]), "rom_max": float(req["rom_max"])}
    except (KeyError, TypeError, ValueError):
        return self._json({"error": "Envelope values must be numbers."}, 400)
    if not patient or not clinician:
        return self._json({"error": "Patient ID and clinician name are required."}, 400)
    for k, (lo, hi) in ENV_LIMITS.items():
        if not lo <= e[k] <= hi:
            return self._json({"error": f"{k} must be between {lo} and {hi}."}, 400)
    if e["rom_max"] - e["rom_min"] < 20:
        return self._json({"error": "Range of motion must span at least 20 degrees."}, 400)
    if BUS.busy:
        return self._json({"error": "busy"}, 409)
    line = f"{e['tmax']} {e['slew']} {e['vmax']} {e['rom_min']} {e['rom_max']}"
    r = subprocess.run(ssh_cmd(f"echo '{line}' > {REMOTE_DIR}/env_{patient}.txt && cat {REMOTE_DIR}/env_{patient}.txt"),
                       capture_output=True, text=True, timeout=15)
    if r.returncode != 0 or line not in r.stdout:
        return self._json({"error": "Could not write the envelope to the QNX target."}, 502)
    CLINIC.update(patient=patient, clinician=clinician, envelope=e, trial=None, approval=None, session=None)
    audit("envelope_set", patient=patient, clinician=clinician, envelope=e)
    return self._json({"ok": True})


def _approve(self, req):
    t = CLINIC["trial"]
    who = str(req.get("clinician", "")).strip()[:60]
    if not who:
        return self._json({"error": "Clinician name is required to approve."}, 400)
    if not CLINIC["envelope"]:
        return self._json({"error": "Set the patient's envelope first (step 1)."}, 400)
    if not t:
        return self._json({"error": "No trial to approve."}, 400)
    if t["verdict"] not in ("PASSED", "CORRECTED") or not t.get("envelope_held"):
        return self._json({"error": f"Only a trial that survived with the envelope held (independent judge) can be approved (this one: {t['verdict']})."}, 400)
    if t["envelope"] != CLINIC["envelope"]:
        return self._json({"error": "The envelope changed after this trial. Re-run the trial."}, 409)
    CLINIC["approval"] = {**t, "clinician": who, "note": str(req.get("note", ""))[:300],
                          "approved_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    audit("approved", patient=CLINIC["patient"], clinician=who, controller=t["controller"],
          binary_sha256=t["binary_sha256"][:16], note=CLINIC["approval"]["note"])
    return self._json({"ok": True})


Handler.set_envelope = _set_envelope
Handler.approve = _approve


def main():
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="qnx")
    ap.add_argument("--port", type=int, default=8765)
    ARGS = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", ARGS.port), Handler)
    print(f"Safe Cage dashboard: http://localhost:{ARGS.port}")
    srv.serve_forever()


if __name__ == "__main__":
    main()
