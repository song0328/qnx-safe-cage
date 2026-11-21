# SafeCage — QNX supervision for AI-written robot control

Category: Robotics & Medical

## Tagline

A QNX governor checks AI-written control commands for a simulated rehabilitation elbow.

## Inspiration

I’ve been exploring how robots turn a person’s instructions into motion. I chose a rehabilitation elbow because it makes the responsibility concrete: an AI can propose an exercise controller, but the person supervising the session should set the limits and decide what runs. I wanted to see what happens when the generated code is wrong, late, or simply too aggressive, and whether QNX can help catch those failures in practice.

## What it does

The operator sets limits for range of motion, speed, and torque. An LLM turns a plain-English exercise prescription into a C controller. SafeCage cross-compiles the controller and trials it on a simulated elbow, including a simulated disturbance. If the draft is rejected, the model receives the reason and can revise the code. In one recorded trial, the first draft failed to compile, the second was revoked after repeated interventions, and the third completed the motion with a peak speed of 76.1°/s.

On QNX, the controller runs in a separate restricted process. It proposes torque and a governor checks it at a nominal 500 Hz. The governor can use a fallback controller when commands are late or unsuitable. A separate joint process measures the resulting motion and independently checks it against the configured limits. The care-plan dashboard shows the trial evidence, allows an operator to approve a specific build by SHA-256, and records session and stop events.

## How it uses QNX 8.0

I ran the QNX target in an x86-64 VirtualBox VM created with the QNX SDP tool `mkqnximage`. The controller, governor, and simulated joint are distinct QNX processes. The governor uses native message passing, timer pulses, process abilities, and priority scheduling. It exposes status and operator stop through a QNX resource manager at `/dev/safecage`. The Windows dashboard connects over SSH. The [README](../README.md) links the APIs to the source and describes how to build and run the target.

## What we measured

- **253 comparison runs.** Each comparison changed one mechanism and kept the target, controller, and independent measurement consistent within that comparison. With a particular faulty controller sharing the governor’s process, the governor reported `PASSED` while the independent joint measured 133°/s. With the same controller isolated, the peak was 85°/s. The configured speed was 90°/s, with three runs per condition.
- **A negative result.** The fixed-priority message-channel setting produced no measurable safety difference under the load used in that experiment.
- **A separate stress replay.** One of 20 runs exceeded the independent judge’s 99°/s breach threshold for one 2 ms tick, peaking at 100°/s after a delayed VM tick.
- **A live operator-stop check.** The dashboard and QNX target both recorded an operator stop in one of four completed timed takes. Deadline-based automatic fallback occurred first in the other three. One additional attempt timed out. These are functional observations under VM scheduling, without an established stop-latency bound.

The comparison tables are in [the experiment report](../results/exp/REPORT.md). The VM retest used to make the video independently verified a hash match between the approved binary and the file read from the target.

## What we learned and what comes next

Keeping the measurement outside the governor exposed a failure the governor’s own verdict missed. The VM results show how process isolation changes failure behavior under the tested conditions. The elbow and patient are simulated. Next I would test on physical hardware, move speed limiting closer to the drive, and measure the operator stop separately from the controller’s automatic fallback.

## AI use

Claude generated and revised exercise controllers from prescriptions and trial feedback. AI coding agents also assisted extensively with implementation, experiments, analysis, drafting, and editing. The demo uses synthetic English narration. Its recorded trial footage and new QNX target capture are marked separately.

## Built with

QNX 8.0, QNX SDP 8.0, C, Python, VirtualBox, Claude Code, HTML/CSS/JavaScript.
## Links

- [Public source repository](https://github.com/song0328/qnx-safe-cage)
- [Public demonstration video](https://youtu.be/P3JAFVaphAA)
