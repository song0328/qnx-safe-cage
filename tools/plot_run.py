"""Figures from governor output (run.csv + summary.json + events.jsonl).

  python tools/plot_run.py results/v0-suite/bad_torque            -> results/v0-suite/bad_torque/figure.png
  python tools/plot_run.py --grid results/v0-suite                -> results/v0-suite/grid.png
  python tools/plot_run.py --loop results/llm/<run>               -> results/llm/<run>/loop.png

CSV columns: t, angle_deg, velocity_dps, torque_requested, torque_applied, flags, lateness_us
"""
import argparse
import json
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
APPLIED, REQUESTED = "#2a78d6", "#eb6834"
GOOD, CRITICAL, LIMIT = "#0ca30c", "#d03b3b", "#8a8984"
F_FALLBACK = 64
TMAX, ROM = 4.0, (0.0, 90.0)

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
    "text.color": INK,
})


def load(d):
    d = pathlib.Path(d)
    rows = [list(map(float, l.split(","))) for l in (d / "run.csv").read_text().splitlines() if l.strip()]
    cols = list(zip(*rows))
    s = json.loads((d / "summary.json").read_text())
    ev = [json.loads(l) for l in (d / "events.jsonl").read_text().splitlines() if l.strip()] if (d / "events.jsonl").exists() else []
    return cols, s, ev


def fallback_spans(t, flags):
    spans, start = [], None
    for ti, f in zip(t, flags):
        on = int(f) & F_FALLBACK
        if on and start is None:
            start = ti
        if not on and start is not None:
            spans.append((start, ti)); start = None
    if start is not None:
        spans.append((start, t[-1]))
    return [sp for sp in spans if sp[1] - sp[0] > 0.02]


def end_event(ev):
    for e in ev:
        if e["kind"] in ("revoked", "controller_died", "controller_exited"):
            return e
    return None


def verdict_text(s):
    return ("✓ PASSED" if s["verdict"] == "PASSED" else "✕ REJECTED"), (GOOD if s["verdict"] == "PASSED" else CRITICAL)


def figure(d, title=None):
    (t, ang, vel, treq, tapp, flags, late), s, ev = load(d)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True, gridspec_kw={"height_ratios": [1.2, 1]})
    for lo, hi in fallback_spans(t, flags):
        for ax in (a1, a2):
            ax.axvspan(lo, hi, color="#efeeea", lw=0, zorder=0)
    a1.plot(t, ang, color=APPLIED, lw=2)
    for y in ROM:
        a1.axhline(y, color=LIMIT, lw=1, ls=(0, (4, 3)))
    a1.text(t[-1], ROM[1] + 2, "range-of-motion limit", ha="right", va="bottom", color=INK2, fontsize=8.5)
    a1.set_ylabel("elbow angle (deg)")
    a1.set_ylim(-8, 105)

    lim = 3 * TMAX
    req_clip = [max(-lim, min(lim, v)) if v == v else 0.0 for v in treq]
    a2.plot(t, req_clip, color=REQUESTED, lw=1.2, label="requested by the AI-written controller")
    a2.plot(t, tapp, color=APPLIED, lw=2, label="applied by the governor")
    for y in (-TMAX, TMAX):
        a2.axhline(y, color=LIMIT, lw=1, ls=(0, (4, 3)))
    a2.text(t[-1], TMAX + 0.3, "torque limit", ha="right", va="bottom", color=INK2, fontsize=8.5)
    big = max((abs(v) for v in treq if v == v), default=0)
    if big > lim:
        a2.text(t[0] + 0.05, lim - 0.6, f"requests up to {big:.0f} N·m (clipped)", color=INK2, fontsize=8.5, va="top")
    a2.set_ylim(-lim - 0.5, lim + 0.5)
    a2.set_ylabel("torque (N·m)")
    a2.set_xlabel("time (s)  ·  shaded = trusted fallback controller in charge")
    a2.legend(loc="lower right", frameon=False, fontsize=8.5)

    e = end_event(ev)
    if e:
        for ax in (a1, a2):
            ax.axvline(e["t"], color=CRITICAL, lw=1.5)
        a1.text(e["t"] + 0.06, 98, e["kind"].replace("_", " "), color=INK, fontsize=8.5, va="top")
    vt, vc = verdict_text(s)
    name = title or pathlib.Path(d).name
    fig.suptitle(f"{name}", x=0.06, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.text(0.06, 0.915, f"{vt}  {s['reason'] or 'accepted, no revocation'}", color=vc, fontsize=9.5, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    out = pathlib.Path(d) / "figure.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def grid(root, names=None):
    root = pathlib.Path(root)
    dirs = [root / n for n in names] if names else sorted(p for p in root.iterdir() if (p / "run.csv").exists())
    n = len(dirs)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(11, 2.6 * rows), sharex=True, sharey=True)
    for ax, d in zip(axes.flat, dirs):
        (t, ang, *_r), s, ev = load(d)
        flags = _r[3]
        for lo, hi in fallback_spans(t, flags):
            ax.axvspan(lo, hi, color="#efeeea", lw=0, zorder=0)
        ax.plot(t, ang, color=APPLIED, lw=1.6)
        for y in ROM:
            ax.axhline(y, color=LIMIT, lw=0.9, ls=(0, (4, 3)))
        e = end_event(ev)
        if e:
            ax.axvline(e["t"], color=CRITICAL, lw=1.2)
        vt, vc = verdict_text(s)
        ax.set_title(d.name, loc="left", fontsize=10)
        ax.text(0.99, 0.95, vt, transform=ax.transAxes, ha="right", va="top", color=vc, fontsize=9, fontweight="bold")
        ax.set_ylim(-8, 105)
    for ax in list(axes.flat)[n:]:
        ax.axis("off")
    fig.supylabel("elbow angle (deg)", color=INK2, fontsize=10)
    fig.supxlabel("time (s)  ·  dashed = range-of-motion limits  ·  shaded = fallback in charge  ·  red line = controller revoked or died",
                  color=INK2, fontsize=9)
    fig.tight_layout()
    out = root / "grid.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--title")
    a = ap.parse_args()
    order = ["good_rom", "bad_torque", "bad_nan", "bad_loop", "bad_slow", "bad_crash", "bad_mem", "bad_escape", "bad_hog"]
    if a.grid:
        root = pathlib.Path(a.path)
        names = [n for n in order if (root / n / "run.csv").exists()] or None
        print(grid(root, names))
    else:
        print(figure(a.path, a.title))


if __name__ == "__main__":
    main()
