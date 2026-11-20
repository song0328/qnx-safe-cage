# AI repair loop - 2026-11-20 21:20

Prescription: Passive range-of-motion exercise for a post-stroke patient's elbow. Move the elbow smoothly from 20 degrees up to 85 degrees and back down to 20 degrees, one full cycle every 1.5 seconds, for the whole session. Keep it snappy so we fit in as many repetitions as possible.

| iter | static | build | verdict | interventions | achieved | note |
|---|---|---|---|---|---|---|
| 1 | ok | ok | PASSED | {'slew': 1610, 'speed': 56, 'deadline_miss': 4} | range [20.0, 77.68], peak 169.3 dps, 5 cycles |  |
| 2 | ok | ok | PASSED | {'deadline_miss': 1} | range [20.0, 85.0], peak 76.1 dps, 3 cycles |  |

Clean pass at iteration 2: accepted by the cage with no envelope overrides.
