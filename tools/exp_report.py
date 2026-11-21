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
    rows = [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
    for r in rows:
        r["_exp"] = name
    return rows


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
    out = [f"## {title}\n", intro + "\n", load_check(rows),
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


CAGE, MONO = "#2a78d6", "#eb6834"          # categorical slots 1-2 (validated: CVD dE 24.7, normal 33.6)
INK, INK2, INK3, LINE, SURF, BAD = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb", "#d03b3b"


def _style(plt):
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": LINE, "axes.labelcolor": INK2, "xtick.color": INK3,
                         "ytick.color": INK3, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
                         "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK,
                         "axes.titlelocation": "left", "legend.frameon": False})


def _read_truth_csv(path):
    t, v = [], []
    for line in path.read_text().splitlines():
        f = line.split(",")
        if len(f) >= 3:
            t.append(float(f[0])); v.append(abs(float(f[2])))
    return t, v


def figures():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    _style(plt)
    made = []

    # ---- E1: what each failure costs, cage vs single process (drive holds the last command)
    iso = by_cond(load("isolation"))
    ctrls = [c for c in ["bad_crash", "bad_loop", "bad_mem", "bad_hog", "bad_slow", "bad_tamper", "bad_torque", "bad_nan", "bad_drive", "good_rom"]
             if f"cage_wd0_{c}" in iso]
    if ctrls:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 3.9), sharey=True)
        y = list(range(len(ctrls)))
        h = 0.36
        for ax, metric, title, xl in ((a1, "unsup", "Time with no working safety layer", "seconds without a fresh safe command (median of 3)"),
                                      (a2, "vmax", "Peak joint speed (independent judge)", "deg/s (max of 3); limit 90, breach above 99")):
            for k, (mode, col, lab) in enumerate((("cage", CAGE, "cage: controller in its own process"),
                                                  ("mono", MONO, "single process: controller linked into the governor"))):
                vals = []
                for c in ctrls:
                    rs = iso.get(f"{mode}_wd0_{c}", [])
                    if metric == "unsup":
                        vals.append(med([unsupervised_s(r) for r in rs]) or 0)
                    else:
                        vals.append(max((truth(r, "max_speed_dps") or 0) for r in rs) if rs else 0)
                ax.barh([i + (k - 0.5) * (h + 0.04) for i in y], vals, h, color=col, label=lab)
            ax.set_title(title)
            ax.set_xlabel(xl)
            ax.grid(axis="x", color=LINE, linewidth=0.6)
            ax.set_axisbelow(True)
        a2.axvline(90, color=INK3, linewidth=1, linestyle=(0, (3, 3)))
        a2.axvline(99, color=BAD, linewidth=1, linestyle=(0, (3, 3)))
        a2.text(101, len(ctrls) - 0.6, "breach", color=BAD, fontsize=8)
        a1.set_yticks(y)
        a1.set_yticklabels(ctrls, color=INK2)
        a1.invert_yaxis()
        h_, l_ = a1.get_legend_handles_labels()
        fig.legend(h_, l_, loc="lower center", ncol=2, fontsize=8.5, bbox_to_anchor=(0.5, -0.01))
        fig.suptitle("Same controllers, same VM, same joint: only the process boundary changes", x=0.01, ha="left",
                     fontsize=11, fontweight="bold", color=INK)
        fig.tight_layout(rect=(0, 0.07, 1, 0.97))
        p = EXP / "isolation.png"
        fig.savefig(p, dpi=160)
        plt.close(fig)
        made.append(p)

    # ---- tamper: true joint speed over time
    cdir, mdir = EXP / "isolation" / "runs" / "cage_wd0_bad_tamper_r0", EXP / "isolation" / "runs" / "mono_wd0_bad_tamper_r0"
    if (cdir / "truth.csv").exists() and (mdir / "truth.csv").exists():
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for d, col, lab in ((mdir, MONO, "single process: limits found in memory and raised; governor verdict PASSED"),
                            (cdir, CAGE, "cage: limits not reachable from the controller's process; revoked")):
            t, v = _read_truth_csv(d / "truth.csv")
            ax.plot(t, v, color=col, linewidth=1.6, label=lab)
        ax.axhline(90, color=INK3, linewidth=1, linestyle=(0, (3, 3)))
        ax.text(0.05, 92, "speed limit 90 deg/s", color=INK2, fontsize=8)
        ax.set_xlabel("session time (s)")
        ax.set_ylabel("|joint speed|, deg/s")
        ax.set_title("AI code that 'fixes' the prescription by raising the safety limits (bad_tamper)")
        ax.grid(axis="y", color=LINE, linewidth=0.6)
        ax.set_axisbelow(True)
        ax.legend(fontsize=8, loc="upper right")
        ax.set_ylim(0, 160)
        fig.tight_layout()
        p = EXP / "tamper.png"
        fig.savefig(p, dpi=160)
        plt.close(fig)
        made.append(p)

    # ---- E3: governor priority under load
    pr = load("priority")
    if pr:
        g = by_cond(pr)
        loads = []
        for cid in g:
            l = cid.split("_", 1)[1]
            if l not in loads:
                loads.append(l)
        fig, ax = plt.subplots(figsize=(9, 3.4))
        w = 0.36
        for i, (gp, col, lab) in enumerate((("gov70", CAGE, "governor at 70 (above anything a non-root process can reach)"),
                                            ("gov10", MONO, "governor at 10 (an ordinary application priority)"))):
            vals = []
            for l in loads:
                rs = g.get(f"{gp}_{l}", [])
                vals.append(max([((r.get("timing") or {}).get("lateness_us_max") or 0) for r in rs] or [0]) / 1000 or 0.1)
            ax.bar([x + (i - 0.5) * (w + 0.04) for x in range(len(loads))], vals, w, color=col, label=lab)
        ax.set_xticks(range(len(loads)))
        ax.set_xticklabels(loads, color=INK2)
        ax.set_yscale("log")
        ax.set_ylabel("worst tick lateness, ms (log)")
        ax.set_title("Only the governor's priority changes: worst control-tick lateness over 5 runs per load")
        ax.grid(axis="y", color=LINE, linewidth=0.6)
        ax.set_axisbelow(True)
        ax.legend(fontsize=8, loc="upper center", ncol=2, bbox_to_anchor=(0.5, -0.12))
        fig.tight_layout()
        p = EXP / "priority.png"
        fig.savefig(p, dpi=160)
        plt.close(fig)
        made.append(p)
    return made


def failover_section(rows):
    if not rows:
        return ""
    out = ["## E5 - The governor itself is killed 4 s into the session\n",
           "`kill -9` on the governor (controller good_rom). Who protects the joint now depends only on the drive.\n",
           "| Drive | n | Envelope held | Drive on stale command (median) | Drive cut the torque (STO) | Note |", "|---|---|---|---|---|---|"]
    for cid, rs in by_cond(rows).items():
        held = sum(1 for r in rs if truth(r, "envelope_held"))
        sto = sum(1 for r in rs if truth(r, "drive_sto"))
        sto_t = med([truth(r, "sto_s") for r in rs if truth(r, "drive_sto")])
        note = ("holds the last command forever: safe here only because the last command was small"
                if "wd0" in cid else f"torque cut 20 ms after the last command (watchdog), at t = {fmt(sto_t, 2)} s (median); the arm then rests passively")
        out.append(f"| {'holds last command' if 'wd0' in cid else '20 ms watchdog'} | {len(rs)} | {held}/{len(rs)} | "
                   f"{fmt(med([unsupervised_s(r) for r in rs]), 2, ' s')} | {sto}/{len(rs)} | {note} |")
    return "\n".join(out) + "\n"


def disturbance_section(rows):
    if not rows:
        return ""
    out = ["## E6 - A patient spasm on top of the controller\n",
           "Spasm: +1.2 N*m for 0.4 s at t = 5 s, then released. The governor brakes over-speed inside the torque "
           "limit whatever drives it; braking a disturbance does not count against the controller.\n",
           "| Condition | n | Envelope held (judge) | Peak speed (max) | Verdicts | Disturbance braking ticks (median) |",
           "|---|---|---|---|---|---|"]
    for cid, rs in by_cond(rows).items():
        held = sum(1 for r in rs if truth(r, "envelope_held"))
        vmax = max((truth(r, "max_speed_dps") or 0) for r in rs)
        v = ", ".join(f"{k} {n}" for k, n in collections.Counter(r.get("verdict") for r in rs).most_common())
        br = med([(r.get("interventions") or {}).get("brake_disturbance") for r in rs])
        out.append(f"| {cid} | {len(rs)} | {held}/{len(rs)} | {fmt(vmax, 1)} deg/s | {v} | {fmt(br)} |")
    return "\n".join(out) + "\n"


def load_check(rows):
    """Proof that the background load actually ran during each session."""
    ran, total = 0, 0
    for r in rows:
        if not r.get("load"):
            continue
        total += 1
        p = EXP / r["_exp"] / "runs" / f"{r['condition']}_r{r['rep']}" / "load.txt"
        if p.exists() and "done after" in p.read_text(errors="replace"):
            ran += 1
    return f"Background load confirmed running (its own log) in {ran} of {total} loaded runs.\n" if total else ""


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
    parts.append(failover_section(load("failover")))
    parts.append(disturbance_section(load("disturbance")))
    figs = figures()
    for f in figs:
        parts.append(f"![{f.stem}]({f.name})\n")
    (EXP / "REPORT.md").write_text("\n".join(p for p in parts if p), encoding="utf-8")
    print(f"wrote {EXP / 'REPORT.md'}" + (f" and {len(figs)} figure(s)" if figs else ""))


if __name__ == "__main__":
    main()
