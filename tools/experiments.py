"""Comparison experiments: turn ONE QNX mechanism off at a time and measure what changes.

Same VM, same controllers, same joint (the independent judge). Every run is appended to
results/exp/<experiment>/runs.jsonl; re-running skips runs that are already there (resumable).

  python tools/experiments.py isolation      # cage (separate process) vs single process, drive hold vs watchdog
  python tools/experiments.py inheritance    # fixed-priority channel on vs off, with periodic mid-priority load
  python tools/experiments.py priority       # governor at 70 vs 10 under different background loads
  python tools/experiments.py layers         # escape attempts with abilities off / without the cage
  python tools/experiments.py failover       # the governor itself is killed: drive hold vs drive watchdog
  python tools/experiments.py disturbance    # a patient spasm on top of good and bad controllers
  python tools/experiments.py all
Options: --repeat N (override), --T seconds (default 8), --host qnx
"""
import argparse
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from run_case import run_one, ROOT  # noqa: E402

FAULTS = ["good_rom", "bad_torque", "bad_nan", "bad_loop", "bad_slow", "bad_crash", "bad_mem", "bad_hog",
          "bad_tamper", "bad_drive"]

LOADS = {
    "idle": None,
    "cpu30": "-k cpu -p 30",                      # another app spinning at priority 30 (e.g. a vision process)
    "cpu63": "-k cpu -p 63",                      # the highest priority a non-root process can take
    "cpu20duty": "-k cpu -p 20 -b 1000 -i 1000",  # 50 % duty-cycle load at priority 20
    "mem30": "-k mem -p 30 -m 128",               # memory traffic at priority 30
    "disk30": "-k disk -p 30",                    # file writes + fsync at priority 30
}


def conditions(name):
    """Yield (condition_id, condition_dict, default_repeat)."""
    if name == "isolation":
        for mode in ("cage", "mono"):
            for wd in (0, 20):
                for c in FAULTS:
                    yield f"{mode}_wd{wd}_{c}", {"ctrl": c, "mono": mode == "mono", "watchdog": wd}, 3
    elif name == "inheritance":
        for load_id, load in (("p20_1ms_of_2ms", "-k cpu -p 20 -b 1000 -i 1000"),
                              ("p20_300us_of_1ms", "-k cpu -p 20 -b 300 -i 700")):
            for fixed in (True, False):
                yield f"{'fixed' if fixed else 'inherit'}_{load_id}", \
                    {"ctrl": "good_rom", "no_fixed": not fixed, "load": load}, 10
    elif name == "priority":
        for gp in (70, 10):
            for lid, load in LOADS.items():
                yield f"gov{gp}_{lid}", {"ctrl": "good_rom", "gov_prio": gp, "load": load}, 5
    elif name == "layers":
        yield "escape_full_cage", {"ctrl": "bad_escape"}, 3
        yield "escape_no_abilities", {"ctrl": "bad_escape", "no_abilities": True}, 3
        yield "drive_full_cage", {"ctrl": "bad_drive"}, 3
        yield "drive_uncaged_root", {"ctrl": "bad_drive", "uncaged": True}, 3
    elif name == "disturbance":
        # a patient spasm (1.2 N*m for 0.4 s at 5 s) on top of the controller: does the envelope hold?
        for c in ("good_rom", "bad_torque", "bad_tamper"):
            yield f"spasm_{c}", {"ctrl": c, "spasm": True}, 5
    elif name == "failover":
        # the governor itself is killed 4 s into the session: what does the joint do?
        for wd in (0, 20):
            yield f"govkill_wd{wd}", {"ctrl": "good_rom", "watchdog": wd, "kill_gov_after": 4}, 3
    else:
        raise SystemExit(f"unknown experiment {name}")


def flatten(cid, cond, rep, ok, wall, s):
    r = {"condition": cid, "rep": rep, **{k: v for k, v in cond.items()}, "finished": ok, "wall_s": round(wall, 1)}
    if not s:
        r["verdict"] = "NO_RESULT"
        return r
    r["verdict"] = s.get("verdict")
    r["reason"] = s.get("reason")
    r["overrides"] = s.get("overrides")
    r["interventions"] = s.get("interventions")
    r["timing"] = s.get("timing")
    r["truth"] = s.get("truth")
    return r


def run_experiment(name, host, T, repeat_override):
    out = ROOT / "results" / "exp" / name
    out.mkdir(parents=True, exist_ok=True)
    log = out / "runs.jsonl"
    done = set()
    if log.exists():
        for line in log.read_text(encoding="utf-8").splitlines():
            if line.strip():
                j = json.loads(line)
                done.add((j["condition"], j["rep"]))
    for cid, cond, rep_n in conditions(name):
        for rep in range(repeat_override or rep_n):
            if (cid, rep) in done:
                continue
            tag = f"{cid}_r{rep}"
            ok, wall, s, d = run_one(host, cond["ctrl"], T, 2000, cond.get("spasm", False), cond.get("uncaged", False), out / "runs",
                                     mono=cond.get("mono", False), gov_prio=cond.get("gov_prio"),
                                     no_fixed=cond.get("no_fixed", False), no_abilities=cond.get("no_abilities", False),
                                     watchdog=cond.get("watchdog", 0), load=cond.get("load"), tag=tag,
                                     kill_gov_after=cond.get("kill_gov_after"))
            r = flatten(cid, cond, rep, ok, wall, s)
            with log.open("a", encoding="utf-8") as f:
                f.write(json.dumps(r) + "\n")
            tr = r.get("truth") or {}
            tm = r.get("timing") or {}
            print(f"[{time.strftime('%H:%M:%S')}] {name} {tag:34s} {r['verdict']!s:13s} held={tr.get('envelope_held')!s:5s} "
                  f"maxv={tr.get('max_speed_dps', '-')!s:6s} stale={tr.get('stale_command_ticks', '-')!s:5s} "
                  f"late_max={tm.get('lateness_us_max', '-')!s:8s} :: {r.get('reason') or ''}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--repeat", type=int, default=0)
    ap.add_argument("--T", type=float, default=8.0)
    ap.add_argument("--host", default="qnx")
    a = ap.parse_args()
    names = ["isolation", "layers", "failover", "disturbance", "priority", "inheritance"] if a.experiment == "all" else [a.experiment]
    for n in names:
        run_experiment(n, a.host, a.T, a.repeat)


if __name__ == "__main__":
    main()
