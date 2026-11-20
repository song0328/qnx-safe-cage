# QNX Safe Cage (working title)

**Status: work in progress for the QNX Global Hackathon 2026.**

An LLM writes the control code for a rehabilitation robot joint. A trusted governor on QNX 8.0
runs that code in a cage and guarantees that, whatever the code does, the joint stays inside a
clinician-set safety envelope - and it tells the AI *why* the code was rejected so it can fix it.

## v0 result (QNX 8.0 VM, 500 Hz, 8 s sessions) - `results/v0-suite/`

| Controller | What it does | Verdict | Envelope held |
|---|---|---|---|
| good_rom | smooth 20-60 deg range-of-motion exercise | PASSED | yes |
| bad_torque | huge gains, aims past range of motion | REJECTED (pushed outside envelope) | yes |
| bad_nan | divides by zero after 2 s | REJECTED (non-finite torque) | yes |
| bad_loop | infinite loop after 3 s | REJECTED (missed control deadline) | yes |
| bad_crash | dereferences an unset pointer | REJECTED (memory protection fault) | yes |
| bad_mem | leaks 1 MB per step, no malloc check | REJECTED (hit 64 MB budget, then faulted) | yes |
| bad_escape | tries kill(governor), fork(), priority 60 | REJECTED (kill/fork denied, priority climb detected) | yes |
| bad_slow | 3 ms of work per 2 ms tick | REJECTED (CPU budget exhausted) | yes |
| bad_hog | raises itself to priority 63 and spins | REJECTED (priority climb detected; governor at 70 kept running) | yes |

## QNX features used (where to look)

- Native synchronous message passing on a fixed-priority channel: `ChannelCreate(_NTO_CHF_FIXED_PRIORITY)`, `MsgReceive`, `MsgReply` in `qnx/governor.c`; `ConnectAttach`, `MsgSend` in `qnx/ctrl_main.c`.
- Timer pulses for the control tick: `SIGEV_PULSE_INIT`, `timer_create` in `qnx/governor.c`.
- QNX abilities: `procmgr_ability()` with `PROCMGR_AOP_DENY | PROCMGR_AOP_LOCK | PROCMGR_AOP_INHERIT_YES` for fork, and spawn locked from outside after exec (`lock_spawn()`).
- Priority bands: governor at FIFO 70, above the 63 a non-root process can reach.
- POSIX resource limits on the caged process: `RLIMIT_CPU`, `RLIMIT_AS`, `RLIMIT_NPROC`, `RLIMIT_NOFILE`, `RLIMIT_CORE`.

## Build and run (Windows host, QNX SDP 8.0)

```
build.bat                                    # builds build/x86_64/* with qcc
scp build/x86_64/* <qnx-target>:/data/sc/
python tools/run_case.py good_rom bad_loop   # runs on the target over ssh (host alias "qnx")
```

## Known limits

- Runs in a VirtualBox VM on a Windows host where Hyper-V is active; the VM cannot keep a 1 kHz
  timer, so v0 uses 500 Hz and timing numbers include host effects.
- The joint is simulated. No medical claim.

License: Apache-2.0.
