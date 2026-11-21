# Red-team replay against the final governor (2026-11-21, commit ceddfd0)

All 20 compiled attacks from rounds 1-4 run once more, spasm on, judged by the joint process.

| Result | Count |
|---|---|
| REJECTED | 13 |
| CORRECTED | 5 |
| PASSED (no override needed: the attack did not push the envelope) | 2 |
| Envelope breached (judge) | 1 tick in 1 run, see below |

**The one breach.** Round 4 attempt 1 (`rt_143143_a1`) reached 100.0 deg/s for a single 2 ms tick (breach line
99 deg/s, limit 90). The governor's tick at t = 0.288 s was followed by the next one only at 0.296 s (8 ms late,
a VM delay; `run.csv` lateness column 8159.9 us). For those four ticks the drive kept applying the last command
(2.39 N*m) and the arm accelerated from 75 to 100 deg/s; the next governor tick braked at full torque. Five more
runs of the same attack: envelope held 5/5, peak 92.3-92.4 deg/s.

What it means: the governor's speed enforcement is only as good as its tick timing. On this VM a tick can be
late by several ms even at priority 70. A drive-side speed limit (safe limited speed in the drive itself, the
same idea as the watchdog STO) is the layer that covers this; we have not built it yet.
