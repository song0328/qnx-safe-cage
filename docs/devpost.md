# Devpost write-up (draft)

> Draft for the QNX Global Hackathon 2026 submission. Track: **Robotics & Medical** (confirm on Devpost).
> Every number below comes from runs recorded in `results/` and shown in the README.
> `[USER]` marks lines only the author can write or confirm.

## Tagline

An LLM writes the control code for a rehab-robot joint; a QNX governor cages it so the joint never
leaves the clinician's safety envelope, and tells the AI why its code was rejected.

## Inspiration

LLMs are getting good at writing robot control code, and it is tempting to let them adapt a
rehabilitation exercise to each patient. But nobody should run freshly generated code next to a
human joint on trust. We wanted the opposite arrangement: let the AI write whatever it writes, and
put a small, trusted, real-time layer between that code and the actuator that cannot be talked out
of the safety limits. QNX gives exactly the pieces such a layer needs: process isolation, fixed
priorities, synchronous message passing, and the ability to take privileges away from a process.

[USER] One or two sentences in your own words on why this matters to you.

## What it does

- A **governor** on QNX runs a 500 Hz control tick for a simulated elbow joint (inertia, damping,
  passive tissue stiffness, and a spasm disturbance at 5.0-5.4 s).
- Each tick it asks a **caged controller process** for a torque over QNX message passing. Before the
  torque reaches the joint, the governor enforces the clinician's envelope: torque limit 4 N·m, torque
  slew 60 N·m/s, speed 90 °/s, range of motion 0-90 ° with a 50 ms look-ahead.
- If the controller misbehaves (NaN, deadline misses, too many overrides, crashing, leaking memory,
  trying to escape), the governor **revokes** it and a trusted damping fallback takes over the joint.
- Every run gets a verdict: **PASSED** (≤ 5 overrides), **CORRECTED** (survived, but needed more
  corrections), **REJECTED** (revoked or died), with a plain-English reason.
- An **AI repair loop** sends a therapist's prescription to an LLM, which writes `ctrl_step()`. The
  code is screened, cross-compiled with `qcc`, run in the cage on QNX, and the cage's report goes back
  to the LLM for the next version.
- A **red-team mode** lets an attacker LLM, given the full design, try to break the cage.
- A **resource manager** exposes the cage at `/dev/safecage`: `cat /dev/safecage/status` shows live
  state as JSON from any shell; `echo stop > /dev/safecage/ctl` is a root-only emergency stop.
- A **dashboard** streams live telemetry from the QNX target, steps through the AI loop, and replays
  any recorded run.

## How we built it

Runs on **QNX 8.0 x86_64 in a VirtualBox VM built with `mkqnximage --type=vbox`**, cross-compiled on a
Windows host with **QNX SDP 8.0, `qcc -Vgcc_ntox86_64`** (`build.bat`). Python tools on the host drive
the target over ssh.

QNX features and where to find them:

| Feature | Where |
|---|---|
| Native message passing on a fixed-priority channel: `ChannelCreate(_NTO_CHF_FIXED_PRIORITY)`, `MsgReceive`, `MsgReply` | `qnx/governor.c` `main()` |
| Controller side: `ConnectAttach`, `MsgSend` | `qnx/ctrl_main.c` `main()` |
| Control tick from a timer pulse: `SIGEV_PULSE_INIT`, `timer_create` | `qnx/governor.c` `main()` |
| Dropping privileges: `procmgr_ability()` with `PROCMGR_AOP_DENY \| PROCMGR_AOP_LOCK \| PROCMGR_AOP_INHERIT_YES` for fork; spawn locked from outside after exec | `qnx/governor.c` `spawn_controller()`, `lock_spawn()` |
| Priority bands: governor FIFO 70, cage 9; a controller that raises itself (`sched_getparam` check) is revoked | `qnx/governor.c` `main()` |
| POSIX limits on the caged process: `RLIMIT_CPU`, `RLIMIT_AS`, `RLIMIT_NPROC`, `RLIMIT_NOFILE`, `RLIMIT_CORE` | `qnx/governor.c` `spawn_controller()` |
| Resource manager `/dev/safecage` (`resmgr_attach`, `iofunc_*`, `dispatch_*`); `ctl` is mode 0200 | `qnx/resmgr.c` `sc_resmgr_start()`, `io_read()`, `io_write()` |

Host side: `tools/run_case.py` (run controllers on the target), `tools/llm_loop.py` (AI repair loop
using the Claude Code CLI headless with no tools), `tools/redteam.py` (attacker LLM),
`tools/plot_run.py` (figures), `dashboard/server.py` + `dashboard/web/` (dashboard).

[USER] Decide how to state AI assistance in building the project itself, if Devpost asks.

## Results

- **9 hand-written controllers** (`results/v1-suite/`): the good range-of-motion controller PASSED with
  0 overrides; all 8 bad ones (huge gains, divide-by-zero, infinite loop, too slow, null pointer,
  memory leak, kill/fork/priority escape, priority-63 CPU hog) were REJECTED with a specific reason.
  The envelope held in all 9.
- **AI repair loop** with a prescription that deliberately conflicts with the envelope (20→85 ° every
  1.5 s needs ~136 °/s; the envelope allows 90 °/s):
  - Claude (default model): stayed under the speed limit on iteration 1, PASSED.
  - Claude Haiku: iteration 1 did not compile; iteration 2 was revoked after 25 overrides in 0.5 s;
    with that report, iteration 3 ran at 76 °/s over the full 20-85 °: PASSED.
  - Timing locked by the therapist: compile error, then revoked, then iteration 3 kept the rhythm and
    gave up amplitude (20-65 °, 71 °/s): PASSED.
- **Red team:** round 1 (6 attempts) found a real policy hole; after the fix, round 2 (6 attempts) had
  no attacker win. The envelope held in all 12.

## Challenges we ran into

- **Timing in a VM.** On a Windows host with Hyper-V active, VirtualBox runs through the slower NEM
  path and could not keep a 1 kHz timer; ticks were occasionally 30-90 ms late. We moved to 500 Hz
  and, when one red-team run showed a 170 ms tick, re-ran it three times next to the reference
  controller to show the same noise appeared without the attack.
- **Priorities are not a wall by themselves.** A normal QNX process can raise itself up to 63. We put
  the governor at 70 and revoke any controller that raises its own priority.
- **Spawn vs. abilities ordering.** `exec` itself needs the spawn ability, so the child can only deny
  and lock fork before exec; the governor denies and locks spawn from outside right after exec.
- **Our first override policy had a hole.** The red-team AI kept poking the envelope 79 times and was
  still labelled PASSED, because we only counted cumulative envelope ticks and ignored rate clamps. We
  now count every tick where the cage changes the controller's command, revoke at > 25 in 0.5 s or
  > 100 per session, and added the CORRECTED verdict.

## Accomplishments that we're proud of

- The envelope held for all 9 hand-written controllers and all 12 red-team attempts. In the AI loop,
  the iterations that pushed too fast peaked at about 91 °/s against the 90 °/s limit before the
  cage revoked them.
- The rejection reasons are good enough that a small model (Haiku) fixed its own code from them.
- An attacker AI found a real bug in our policy, and we could show the fix with a second round.
- The cage can be inspected and stopped with ordinary shell commands through a QNX resource manager.

## What we learned

- The safety argument should not depend on what the AI writes; it should depend on a small layer
  that the AI cannot change. Static screening helps, but isolation, priorities and revocation do the
  real work.
- Giving the model the cage's reasons, not just pass/fail, is what makes the loop converge.
- When the prescription conflicts with the envelope, the models chose different trade-offs (slow
  down vs. shrink the range). The cage enforces the clinician's limits; it does not decide whether the
  limits or the trade-off are clinically right. That stays with people.

## What's next

- Run on real hardware (e.g. Raspberry Pi with QNX) at 1 kHz with a real actuator in place of the
  simulated joint.
- Per-patient envelopes signed by the clinician, and an audit log of every override.
- Use the CORRECTED verdict as a signal for human review instead of automatic acceptance.

## Limits (stated plainly)

- The joint is simulated. No medical efficacy claim.
- Timing numbers come from a VM and include host effects.

## Built with

QNX 8.0, QNX SDP 8.0 (`qcc`), C, Python, VirtualBox, Claude Code CLI, matplotlib, HTML/JS.

## Links

- Repository: [USER] make public before submission
- Video: [USER] 3-minute video (planned 12/7-12/13)
