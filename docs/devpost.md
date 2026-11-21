# Devpost write-up (draft 3, numbers from the final run)

> Draft for the QNX Global Hackathon 2026 submission. Track: **Robotics & Medical** (confirm on Devpost).
> Every number comes from runs recorded in `results/` (comparison tables: `results/exp/REPORT.md`).
> `[USER]` marks lines only the author can write or confirm. Numbers are from the final full run
> (253 runs, governor at commit `ceddfd0`, 2026-11-21).

## Tagline

Let an AI write the robot's control code, and prove (on QNX, by measurement) that it still cannot hurt the patient.

## Inspiration

LLMs can already write robot control code, and it is tempting to let them adapt a rehabilitation exercise to
each patient. But nobody should run freshly generated code next to a human joint on trust. We wanted the opposite
arrangement: let the AI write whatever it writes, and put a small, trusted, real-time layer between that code and
the actuator that cannot be crashed, starved, escaped or talked out of the safety limits. Then we wanted to know
whether QNX really matters for that, so we measured it.

[USER] One or two sentences in your own words on why this matters to you.

## What it does

- **The cage.** A governor on QNX 8.0 runs a 500 Hz control loop for an exoskeleton elbow. Each tick it asks a
  caged controller process (uid 1001, priority 9, CPU/memory/process limits, fork and spawn abilities denied) for
  a torque over QNX message passing and enforces the clinician's envelope (torque, slew, speed, range of motion,
  NaN). It shapes and brakes speed whatever drives the joint (the controller or a spasm), hands missed deadlines
  to a trusted fallback, and revokes misbehaving code with a plain-English reason: PASSED / CORRECTED / REJECTED.
- **An independent judge.** The joint (drive + forearm + patient) is a separate process above the governor
  (priority 72). It applies whatever is in its root-only command register, logs the true trajectory and decides
  whether the envelope held, so the governor never grades itself.
- **The AI loop.** A therapist's prescription goes to an LLM; its code is screened, cross-compiled with `qcc`,
  run in the cage against a virtual patient, and the cage's report goes back for the next version.
- **A care plan for clinicians.** Envelope per patient -> AI draft trialled on QNX with a spasm -> evidence
  (verdict, judge, overrides, SHA-256) -> approval pins that exact binary -> only it may run a session ->
  emergency stop through a QNX resource manager (`echo stop > /dev/safecage/ctl`) -> audit log.
- **A red team.** An attacker LLM gets the full design and tries to break the cage.

## Is QNX actually making it safer? We switched each mechanism off and measured

Same VM, same controllers, same judge; only one mechanism changes. 253 runs, 3-10 per condition.

- **Address-space isolation.** Linked into the governor (the usual single-process design), a crash, an infinite
  loop, a memory leak or a CPU hog took the governor down with it: 5.0-6.4 s with no working safety layer, every
  time. A merely slow controller was graded PASSED while the drive went 3.1 s without a fresh command.
  Worse, AI code that "fixed" an over-fast prescription by finding the safety limits in memory and raising them
  left the governor reporting PASSED while the judge measured 133 deg/s against a 90 deg/s limit (3/3). In the
  cage: limits unreachable, controller revoked, envelope held in all 60 caged runs.
- **Priority bands.** With the governor at an ordinary priority, one busy process at priority 30 or 63 stopped
  it for the whole 8 s load, every run (15/15). At priority 70 (above anything a non-root process can reach) it kept
  ticking, noticed the starved controller and held the joint with the fallback.
- **Abilities and permissions.** Abilities off: `fork()` succeeds. Controller run as root: it opens the drive
  registers and breaches the envelope every time. Full cage: both denied.
- **The governor dies.** A 20 ms drive watchdog cuts the torque. The drive, not the governor, is the last line.
- **A patient spasm.** Envelope held 15/15 with the spasm; the good controller still PASSED.
- **What did not matter:** the fixed-priority channel flag showed no measurable effect in our setup (40 runs). We
  say so.

## How we built it

Runs on **QNX 8.0 x86_64 in a VirtualBox VM built with `mkqnximage --type=vbox`**, cross-compiled on Windows with
**QNX SDP 8.0, `qcc -Vgcc_ntox86_64`** (`build.bat`). Python tools on the host drive the target over ssh.

| QNX feature | Where |
|---|---|
| Message passing on a fixed-priority channel (`ChannelCreate(_NTO_CHF_FIXED_PRIORITY)`, `MsgReceive`, `MsgReply`, `MsgSend`) | `qnx/governor.c`, `qnx/ctrl_main.c` |
| Timer pulses (`SIGEV_PULSE_INIT`, `timer_create`) | `qnx/governor.c`, `qnx/joint.c` |
| Abilities (`procmgr_ability`, deny + lock + inherit for fork; spawn locked from outside after exec) | `qnx/governor.c` `spawn_controller()`, `lock_spawn()` |
| Priority bands 72 / 70 / <=63 / 9 and revocation on self-raised priority | `qnx/governor.c`, `qnx/joint.c` |
| Resource manager `/dev/safecage/{status,ctl}` (`resmgr_attach`, `iofunc_*`, `dispatch_*`), `ctl` root-only | `qnx/resmgr.c` |
| Root-only shared-memory drive registers with a seqlock | `qnx/joint.c`, `qnx/common.h` |
| POSIX limits on the caged process (`RLIMIT_CPU/AS/NPROC/NOFILE/CORE`) | `qnx/governor.c` |

Host side: `tools/experiments.py` + `tools/exp_report.py` (comparisons), `tools/llm_loop.py` (AI loop, Claude Code
CLI headless with no tools), `tools/redteam.py`, `dashboard/server.py` + `dashboard/web/` (care plan).

[USER] Decide how to state AI assistance in building the project itself, if Devpost asks.

## Challenges we ran into

- **Our own safety layer had gaps, and our own tools found them.** The red team found a counting loophole
  (79 overrides still PASSED). A spasm trial in the care plan showed that the speed rule only stopped the
  controller from pushing, so a spasm carried the arm to 234 deg/s with a 70 deg/s envelope; the governor now
  brakes whatever drives the joint. Red team round 3 then showed that braking after the limit always overshoots
  (96.9 deg/s); torque towards the limit is now faded out from 90 % of it (same attacks: 87-92.5 deg/s).
- **Our fix introduced a regression, and the final rerun caught it.** Fading out torque near the speed limit
  made the governor blame the controller for its own rate clamp, so every faulty controller was revoked within
  0.15 s, before its real fault, and the isolation result disappeared. Because the isolation numbers suddenly
  contradicted two earlier runs, we looked, found it and fixed it, then reran everything.
- **Measuring without fooling ourselves.** Our first load experiment was invalid: the load starved the shell that
  launched the session, so sessions only started after the load ended. We caught it because the governor at
  priority 10 looked unaffected, and now every loaded run carries the load's own log.
- **Timing in a VM.** VirtualBox on Hyper-V cannot hold 1 kHz, and a spinning guest thread delays timer delivery
  for everyone. We run at 500 Hz, compare only within the same VM, and made the silence rule count each tick's
  gap only up to 3 periods so a whole-VM pause is not blamed on the controller.
- **Process plumbing.** A crashed root process's core dump filled the VM's small disk and made it read-only;
  a zombie process fooled our watchdog script once. Both fixed and written down.

## Accomplishments that we're proud of

- A comparison, not a demo: 253 runs where only one QNX mechanism changes, judged outside the governor.
- The tampering result: in one address space the safety layer was silently fooled; in the cage it was not.
- A clinician-facing flow where untested AI code can never reach a patient session.
- Four red-team rounds by an attacker AI with the full design: no breach in the rounds themselves; replaying all
  20 attacks against the final governor gave one 2 ms tick at 100.0 deg/s (breach line 99) when a governor tick
  came 8 ms late on the VM. We report it, and it points to the next layer (a drive-side speed limit).

## What we learned

- The safety argument must not depend on what the AI writes. It depends on a small layer the AI cannot reach,
  and on the OS guaranteeing "cannot reach" and "always runs".
- Never let the safety layer grade itself; a separate judge found problems the governor did not report.
- Null results belong in the write-up (the fixed-priority channel).
- Giving the model the cage's reasons, not just pass/fail, is what makes the AI loop converge.

## What's next

- Real hardware (QNX on a Raspberry Pi or x86 board) at 1 kHz with a real actuator and a hardware watchdog.
- A speed limit in the drive itself, so a late governor tick cannot let the arm overshoot.
- The same experiments on Linux with PREEMPT_RT, cgroups and seccomp, for a fair comparison.
- Per-patient envelopes signed by the clinician; CORRECTED trials routed to human review.

## Limits (stated plainly)

- Simulated joint and patient. No medical efficacy claim.
- VM timing; absolute latencies include host effects.

## Built with

QNX 8.0, QNX SDP 8.0 (`qcc`), C, Python, VirtualBox, Claude Code CLI, matplotlib, HTML/JS.

## Links

- Repository: [USER] make public before submission
- Video: [USER] 3-minute video (planned 12/7-12/13)
