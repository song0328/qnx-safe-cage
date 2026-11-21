"""Run one or more controllers through the governor on the QNX target and collect results.

Usage:
  python tools/run_case.py good_rom bad_loop ... [-T 8] [-p 2000] [--spasm] [--uncaged] [--host qnx] [--out results/<name>]
         [--mono] [--gov-prio 70] [--no-fixed-prio] [--no-abilities] [--watchdog MS] [--load SPEC] [--repeat N]

Each run starts the joint process (drive + forearm + patient, the independent judge) first, then the
governor (or, with --mono, the single-process build mono_<controller>), optionally with background load.
--load SPEC is passed to tools/load.c on the target, e.g. "-k cpu -p 30" or "-k cpu -p 20 -b 1000 -i 4000".

Needs: an SSH host alias for the QNX target (default "qnx"), binaries uploaded to /data/sc.
Writes results/<run>/<controller>[_rN]/{summary.json, truth.json, truth.csv, events.jsonl, run.csv, stdout.txt, joint.txt}.
"""
import argparse
import json
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
REMOTE_DIR = "/data/sc"


def ssh(host, cmd, timeout):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "LogLevel=ERROR", host, cmd],
                          capture_output=True, text=True, timeout=timeout)


def fetch(host, remote, local):
    subprocess.run(["scp", "-q", "-o", "BatchMode=yes", "-o", "LogLevel=ERROR", f"{host}:{remote}", str(local)],
                   capture_output=True, timeout=120)


def run_one(host, ctrl, T, period, spasm, uncaged, outdir, mono=False, gov_prio=None, no_fixed=False,
            no_abilities=False, watchdog=0, load=None, tag=None, kill_gov_after=None, trace=False):
    name = tag or ctrl
    rdir = f"/dev/shmem/sc_{name}"
    gflags = f"-T {T} -p {period}" + (" -U" if uncaged else "") + (f" -g {gov_prio}" if gov_prio else "") \
        + (" -P" if no_fixed else "") + (" -A" if no_abilities else "")
    jflags = f"-T {T} -p {period}" + (" -s" if spasm else "") + (f" -w {watchdog}" if watchdog else "")
    binary = f"./mono_{ctrl}" if mono else "./governor"
    # The load must start AFTER the joint and governor are running: a load above the launching shell's
    # priority (10) would otherwise starve the shell, and the session would only start once the load ended.
    # A launcher at 71 waits 1 s into the session, then becomes the load (which sets its own priority).
    load_cmd = (f"on -p 71 sh -c \"sleep 1; exec ./load {load} -t {T}\" > {rdir}.load 2>&1 & ") if load else ""
    # run detached inside the target so a stalled SSH session cannot kill the run
    # each step separately: removing a /dev/shmem directory can fail, and that must not skip the run
    # no core dumps in any condition: a dump of a large crashed process fills the small /data disk
    cmd = (f"ulimit -c 0; cd {REMOTE_DIR}; chmod 755 {REMOTE_DIR}/*; slay -f joint >/dev/null 2>&1; slay -f load >/dev/null 2>&1; "
           f"rm -f {rdir}.out {rdir}.joint {rdir}.load {rdir}/*; mkdir -p {rdir}; "
           f"{load_cmd}./joint {jflags} -o {rdir} > {rdir}.joint 2>&1 & JP=$!; "
           # reaper above every governor priority: a hung single-process governor (prio 70) would otherwise
           # starve sshd and lock up the target (killed by PID; cancelled when the governor exits by itself)
           f"{binary} -c {REMOTE_DIR}/{ctrl} {gflags} -o {rdir} > {rdir}.out 2>&1 & GP=$!; "
           # the reaper fires only after the joint (the session clock) has finished: in the VM a spinning
           # thread stretches wall time, so a fixed wall-clock deadline would kill healthy governors
           # (the joint's truth.json marks the end; a finished joint stays a zombie until `wait`, so kill -0 can't be used)
           f"on -p 71 sh -c \"i=0; while [ ! -s {rdir}/truth.json ] && [ \\$i -lt {int(T * 6 + 30)} ]; do sleep 1; i=\\$((i+1)); done; "
           f"sleep 3; kill -9 $GP\" >/dev/null 2>&1 & RP=$!; "
           + (f"(sleep {kill_gov_after}; kill -9 $GP) >/dev/null 2>&1 & " if kill_gov_after else "")
           + (f"tracelogger -n 0 -s 3 -f {rdir}/trace.kev >/dev/null 2>&1 & " if trace else "")
           + f"wait $GP; echo rc=$?; kill $RP >/dev/null 2>&1; wait $JP")
    t0 = time.time()
    try:
        r = ssh(host, cmd, timeout=T * 30 + 60)
        ok = "rc=" in r.stdout
    except subprocess.TimeoutExpired:
        ok = False
    wall = time.time() - t0
    d = outdir / name
    d.mkdir(parents=True, exist_ok=True)
    for old in d.iterdir():                 # never mix in files from an earlier run with the same name
        if old.is_file():
            old.unlink()
    for f in ("summary.json", "events.jsonl", "run.csv", "truth.json", "truth.csv") + (("trace.kev",) if trace else ()):
        fetch(host, f"{rdir}/{f}", d / f)
    fetch(host, f"{rdir}.out", d / "stdout.txt")
    fetch(host, f"{rdir}.joint", d / "joint.txt")
    if load:
        fetch(host, f"{rdir}.load", d / "load.txt")      # "load ... done after X s": proof the load ran
    s = None
    if (d / "summary.json").exists() and (d / "summary.json").stat().st_size > 0:
        s = json.loads((d / "summary.json").read_text())
    tr = None
    if (d / "truth.json").exists() and (d / "truth.json").stat().st_size > 0:
        tr = json.loads((d / "truth.json").read_text())
    if s is not None and tr is not None:
        s["truth"] = tr
    elif s is None and tr is not None:
        s = {"verdict": "GOVERNOR_DIED", "truth": tr}
    return ok, wall, s, d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("controllers", nargs="+")
    ap.add_argument("-T", type=float, default=8.0)
    ap.add_argument("-p", type=int, default=2000)
    ap.add_argument("--spasm", action="store_true")
    ap.add_argument("--uncaged", action="store_true")
    ap.add_argument("--host", default="qnx")
    ap.add_argument("--out", default=None)
    ap.add_argument("--mono", action="store_true", help="single-process build: controller linked into the governor")
    ap.add_argument("--gov-prio", type=int, default=None)
    ap.add_argument("--no-fixed-prio", action="store_true")
    ap.add_argument("--no-abilities", action="store_true")
    ap.add_argument("--watchdog", type=int, default=0, help="drive watchdog in ms (0 = drive holds the last command)")
    ap.add_argument("--load", default=None)
    ap.add_argument("--repeat", type=int, default=1)
    a = ap.parse_args()
    outdir = pathlib.Path(a.out) if a.out else ROOT / "results" / time.strftime("%Y%m%d-%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)
    rows = []
    jobs = [(c, r) for c in a.controllers for r in range(a.repeat)]
    for c, rep in jobs:
        tag = c if a.repeat == 1 else f"{c}_r{rep}"
        ok, wall, s, d = run_one(a.host, c, a.T, a.p, a.spasm, a.uncaged, outdir, a.mono, a.gov_prio, a.no_fixed_prio,
                                 a.no_abilities, a.watchdog, a.load, tag)
        if s and s.get("verdict") == "GOVERNOR_DIED":
            tr = s["truth"]
            print(f"{tag:12s} GOVERNOR DIED  truth: held={tr['envelope_held']} angle={tr['angle_range_deg']} "
                  f"|tau|max={tr['max_abs_torque']:.2f} stop={tr['hit_anatomical_stop']} sto={tr['drive_sto']}")
            rows.append({"controller": c, "tag": tag, **s})
            continue
        if not s:
            print(f"{c:12s} NO RESULT (finished={ok}, {wall:.0f}s) - see {d / 'stdout.txt'}")
            rows.append({"controller": c, "verdict": "NO_RESULT"})
            continue
        i = {k: v for k, v in s["interventions"].items() if v}
        t = s["timing"]
        tr = s.get("truth") or {}
        print(f"{tag:12s} {s['verdict']:9s} TRUTH held={tr.get('envelope_held')!s:5s} stop={tr.get('hit_anatomical_stop')!s:5s}  ovr={s.get('overrides', '?')!s:4s} held={s['envelope_held']!s:5s} |tau|max={s['max_abs_torque_applied']:.2f} "
              f"angle={s['angle_range_deg']} wall={t['wall_s']}s p99late={t['lateness_us_p99']:.0f}us "
              f"maxlate={t['lateness_us_max']:.0f}us :: {s['reason'] or '-'} :: {i}")
        rows.append({"controller": c, "tag": tag, **s})
    (outdir / "index.json").write_text(json.dumps(rows, indent=1))
    print(f"results: {outdir}")
    return 0 if all(r.get("verdict") != "NO_RESULT" for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
