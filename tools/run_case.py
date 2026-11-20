"""Run one or more controllers through the governor on the QNX target and collect results.

Usage:
  python tools/run_case.py good_rom bad_loop ... [-T 8] [-p 2000] [--spasm] [--uncaged] [--host qnx] [--out results/<name>]

Needs: an SSH host alias for the QNX target (default "qnx"), binaries uploaded to /data/sc.
Writes results/<run>/<controller>/{summary.json, events.jsonl, run.csv, stdout.txt}.
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


def run_one(host, ctrl, T, period, spasm, uncaged, outdir):
    rdir = f"/dev/shmem/sc_{ctrl}"
    flags = f"-T {T} -p {period}" + (" -s" if spasm else "") + (" -U" if uncaged else "")
    # run detached inside the target so a stalled SSH session cannot kill the run
    # each step separately: removing a /dev/shmem directory can fail, and that must not skip the run
    cmd = (f"cd {REMOTE_DIR}; chmod 755 {REMOTE_DIR}/*; rm -f {rdir}.out {rdir}/*; "
           f"./governor -c {REMOTE_DIR}/{ctrl} {flags} -o {rdir} > {rdir}.out 2>&1; echo rc=$?")
    t0 = time.time()
    try:
        r = ssh(host, cmd, timeout=T * 30 + 60)
        ok = "rc=0" in r.stdout
    except subprocess.TimeoutExpired:
        ok = False
    wall = time.time() - t0
    d = outdir / ctrl
    d.mkdir(parents=True, exist_ok=True)
    for f in ("summary.json", "events.jsonl", "run.csv"):
        fetch(host, f"{rdir}/{f}", d / f)
    fetch(host, f"{rdir}.out", d / "stdout.txt")
    s = None
    if (d / "summary.json").exists() and (d / "summary.json").stat().st_size > 0:
        s = json.loads((d / "summary.json").read_text())
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
    a = ap.parse_args()
    outdir = pathlib.Path(a.out) if a.out else ROOT / "results" / time.strftime("%Y%m%d-%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)
    rows = []
    for c in a.controllers:
        ok, wall, s, d = run_one(a.host, c, a.T, a.p, a.spasm, a.uncaged, outdir)
        if not s:
            print(f"{c:12s} NO RESULT (finished={ok}, {wall:.0f}s) - see {d / 'stdout.txt'}")
            rows.append({"controller": c, "verdict": "NO_RESULT"})
            continue
        i = {k: v for k, v in s["interventions"].items() if v}
        t = s["timing"]
        print(f"{c:12s} {s['verdict']:9s} ovr={s.get('overrides', '?')!s:4s} held={s['envelope_held']!s:5s} |tau|max={s['max_abs_torque_applied']:.2f} "
              f"angle={s['angle_range_deg']} wall={t['wall_s']}s p99late={t['lateness_us_p99']:.0f}us "
              f"maxlate={t['lateness_us_max']:.0f}us :: {s['reason'] or '-'} :: {i}")
        rows.append({"controller": c, **s})
    (outdir / "index.json").write_text(json.dumps(rows, indent=1))
    print(f"results: {outdir}")
    return 0 if all(r.get("verdict") != "NO_RESULT" for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
