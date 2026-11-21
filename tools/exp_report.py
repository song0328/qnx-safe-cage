"""Summarise results/exp/*/runs.jsonl into results/exp/REPORT.md (+ figures).

  D:\\eegenv\\Scripts\\python.exe tools/exp_report.py      (matplotlib needed for the figures)
"""
import collections
import json
import pathlib
import statistics as st

ROOT = pathlib.Path(__file__).resolve().parents[1]
EXP = ROOT / "results" / "exp"
DT = 0.002


def load(name):
    p = EXP / name / "runs.jsonl"
    if not p.exists():
        return []
    return [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]


def by_cond(rows):
    g = collections.OrderedDict()
    for r in rows:
        g.setdefault(r["condition"], []).append(r)
    return g


def truth(r, k, default=None):
    return (r.get("truth") or {}).get(k, default)


def med(xs):
    xs = [x for x in xs if x is not None]
    return st.median(xs) if xs else None


def fmt(x, nd=0, unit=""):
    if x is None:
        return "-"
    return f"{x:.{nd}f}{unit}"


def unsupervised_s(r):
    """Time the drive ran on a stale command (no fresh command from the safety layer)."""
    s = truth(r, "stale_command_ticks")
    return None if s is None else s * DT


def governor_alive(r):
    return r.get("verdict") not in ("GOVERNOR_DIED", "NO_RESULT")


# ---------------------------------------------------------------- isolation
def isolation_section(rows):
    if not rows:
        return ""
    g = by_cond(rows)
    ctrls, cells = [], {}
    for cid, rs in g.items():
        mode, wd, ctrl = cid.split("_", 2)
        if ctrl not in ctrls:
            ctrls.append(ctrl)
        n = len(rs)
        held = sum(1 for r in rs if truth(r, "envelope_held"))
        alive = sum(1 for r in rs if governor_alive(r))
        unsup = med([unsupervised_s(r) for r in rs])
        vmax = max((truth(r, "max_speed_dps") or 0) for r in rs)
        verdicts = collections.Counter(r.get("verdict") for r in rs)
        sto = sum(1 for r in rs if truth(r, "drive_sto"))
        cells[(mode, wd, ctrl)] = dict(n=n, held=held, alive=alive, unsup=unsup, vmax=vmax, verdicts=verdicts, sto=sto)

    def cell(c):
        if not c:
            return "-"
        v = ", ".join(f"{k} {n}" if c["n"] > 1 else k for k, n in c["verdicts"].most_common())
        gov = "alive" if c["alive"] == c["n"] else "died {}/{}".format(c["n"] - c["alive"], c["n"])
        s = f"envelope held {c['held']}/{c['n']}; governor {gov}; "
        s += f"unsupervised {fmt(c['unsup'], 2)} s; peak {fmt(c['vmax'])} deg/s; {v}"
        if c["sto"]:
            s += f"; drive STO {c['sto']}/{c['n']}"
        return s

    out = ["## E1 - Isolation: the same controller in the cage vs linked into the governor\n",
           "Judge: the joint process (independent of the governor). *Unsupervised* = time the drive ran on a stale "
           "command because no fresh command came from the safety layer. Drive without watchdog holds the last "
           "command; with a 20 ms watchdog it cuts the torque (safe torque off).\n"]
    for wd in ("wd0", "wd20"):
        out.append(f"\n### Drive {'holds the last command' if wd == 'wd0' else 'with 20 ms watchdog'}\n")
        out.append("| Controller | Cage (separate process) | Single process |")
        out.append("|---|---|---|")
        for c in ctrls:
            out.append(f"| {c} | {cell(cells.get(('cage', wd, c)))} | {cell(cells.get(('mono', wd, c)))} |")
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------- timing experiments
def timing_rows(rows):
    g = by_cond(rows)
    out = []
    for cid, rs in g.items():
        tm = [r.get("timing") or {} for r in rs]
        out.append(dict(
            cond=cid, n=len(rs),
            held=sum(1 for r in rs if truth(r, "envelope_held")),
            p99=med([t.get("lateness_us_p99") for t in tm]),
            lmax_med=med([t.get("lateness_us_max") for t in tm]),
            lmax_max=max([t.get("lateness_us_max") or 0 for t in tm]) if tm else None,
            govlate=med([t.get("governor_late_ticks") for t in tm]),
            miss=med([(r.get("interventions") or {}).get("deadline_miss") for r in rs]),
            fb=med([(r.get("interventions") or {}).get("fallback_ticks") for r in rs]),
            unsup=med([unsupervised_s(r) for r in rs]),
            verdicts=collections.Counter(r.get("verdict") for r in rs),
        ))
    return out


def timing_table(rows, title, intro):
    if not rows:
        return ""
    out = [f"## {title}\n", intro + "\n",
           "| Condition | n | Envelope held | Governor tick lateness p99 (median) | Worst tick lateness (median / max) | Governor late ticks (median) | Controller deadline misses (median) | Drive on stale command (median) | Verdicts |",
           "|---|---|---|---|---|---|---|---|---|"]
    for t in timing_rows(rows):
        v = ", ".join(f"{k} {n}" for k, n in t["verdicts"].most_common())
        out.append(f"| {t['cond']} | {t['n']} | {t['held']}/{t['n']} | {fmt(t['p99'] and t['p99'] / 1000, 2, ' ms')} | "
                   f"{fmt(t['lmax_med'] and t['lmax_med'] / 1000, 1)} / {fmt(t['lmax_max'] and t['lmax_max'] / 1000, 1, ' ms')} | "
                   f"{fmt(t['govlate'])} | {fmt(t['miss'])} | {fmt(t['unsup'], 2, ' s')} | {v} |")
    return "\n".join(out) + "\n"


def layers_section(rows):
    if not rows:
        return ""
    out = ["## E4 - Defence layers against escape attempts\n",
           "| Condition | n | Verdicts | Envelope held | What the controller reported |", "|---|---|---|---|---|"]
    for cid, rs in by_cond(rows).items():
        msgs = set()
        for r in rs:
            p = EXP / "layers" / "runs" / f"{cid}_r{r['rep']}" / "stdout.txt"
            if p.exists():
                for line in p.read_text(errors="replace").splitlines():
                    if line.startswith("[escape]") or line.startswith("[drive]"):
                        msgs.add(line.strip())
        v = ", ".join(f"{k} {n}" for k, n in collections.Counter(r.get("verdict") for r in rs).most_common())
        held = sum(1 for r in rs if truth(r, "envelope_held"))
        out.append(f"| {cid} | {len(rs)} | {v} | {held}/{len(rs)} | {'<br>'.join(sorted(msgs)) or '-'} |")
    return "\n".join(out) + "\n"


def figures():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    made = []
    pr = load("priority")
    if pr:
        g = by_cond(pr)
        loads = []
        for cid in g:
            l = cid.split("_", 1)[1]
            if l not in loads:
                loads.append(l)
        fig, ax = plt.subplots(figsize=(8, 3.6))
        w = 0.38
        for i, gp in enumerate(("gov70", "gov10")):
            vals = []
            for l in loads:
                rs = g.get(f"{gp}_{l}", [])
                vals.append(max([(r.get("timing") or {}).get("lateness_us_max") or 0 for r in rs] or [0]) / 1000)
            ax.bar([x + (i - 0.5) * w for x in range(len(loads))], vals, w,
                   label="governor at 70 (above any non-root app)" if gp == "gov70" else "governor at 10 (like a normal app)",
                   color="#2a6f97" if gp == "gov70" else "#c8553d")
        ax.set_xticks(range(len(loads)))
        ax.set_xticklabels(loads)
        ax.set_yscale("log")
        ax.set_ylabel("worst governor tick lateness (ms, log)")
        ax.set_title("Same VM, same controller: only the governor's priority changes")
        ax.legend(fontsize=8, frameon=False)
        fig.tight_layout()
        p = EXP / "priority.png"
        fig.savefig(p, dpi=150)
        plt.close(fig)
        made.append(p)
    return made


def main():
    parts = ["# Comparison experiments (QNX 8.0 VM, 500 Hz, 8 s sessions)\n",
             "Generated by `tools/exp_report.py` from `results/exp/*/runs.jsonl`. Each condition turns one mechanism "
             "off; everything else is identical. The joint process is the judge.\n"]
    parts.append(isolation_section(load("isolation")))
    parts.append(timing_table(load("priority"), "E3 - Priority bands: governor at 70 vs 10 under background load",
                              "Controller: good_rom. Loads: another process spinning at priority 30 or 63 (the highest a non-root "
                              "process can take), a 50 % duty-cycle load at 20, memory traffic, disk writes."))
    parts.append(timing_table(load("inheritance"), "E2 - Priority inheritance: fixed-priority channel vs plain channel",
                              "Controller: good_rom (priority 9), governor 70. A periodic load at priority 20 runs between them. "
                              "With a plain channel, receiving the controller's message hands the governor thread the controller's priority."))
    parts.append(layers_section(load("layers")))
    figs = figures()
    for f in figs:
        parts.append(f"![{f.stem}]({f.name})\n")
    (EXP / "REPORT.md").write_text("\n".join(p for p in parts if p), encoding="utf-8")
    print(f"wrote {EXP / 'REPORT.md'}" + (f" and {len(figs)} figure(s)" if figs else ""))


if __name__ == "__main__":
    main()
