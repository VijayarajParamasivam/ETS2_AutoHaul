# Learning from drives

`autohaul/learn/learner.py` watches the same notes the recordings hold (10 per second, only while AutoHaul drives).
Everything it changes is a small step inside hard bounds (`BOUNDS`), kept separately with and without a trailer, and
each change is logged with its reason to `learned/learning.jsonl` in the cache folder.

| What | Signal | Change |
|---|---|---|
| `lat_acc_max` (curve speed) | peak error > 1 m in a curve, or you took over within 3 s of one | slower |
| | under 0.4 m, with the limit setting the speed | very slowly faster |
| `predict_s` (turn-in) | mean error to one side in a curve | outside: more; inside: less |
| `omega_max` (correction) | peak error > 0.35 m in a curve | firmer |
| | weaving on a straight (4 swings past 0.15 m each side of the mean, in 20 s) | softer, by more, and no firming up for a minute |
| `plan_decel` (braking for curves) | into a curve > 5 km/h too fast | earlier |
| `thr_accel`, `brake_decel` | measured acceleration against pedal | towards the measured value |
| `c_us` (understeer) | the controller's learned ratio | starts the next drive there |
| trouble spots (per lane) | you took over, or a fault or a crash here, under 90 km/h | from the 2nd time: slows there; clean passes wear it off |
| route cost (per lane) | the GPS distance jumps while AutoHaul's own remaining distance doesn't | that lane costs more |

The steering knobs were chosen by sweeping every candidate on the real controller in the simulator
(`ff_preview_s`, the first candidate, does nothing to the error in curves; `predict_s` sets the balance between
outside and inside; `omega_max` from 0.6 to 1.0 cuts the error 3x). `test_learning_really_improves_tracking_in_the_simulator`
checks the whole loop from poor values.

A scorecard per drive (over 30 game km) has takeovers and faults per 100 game km, fines, tracking error, curve
overshoots, GPS disagreements and missed turns. If the last 3 drives score over 1.5x worse than the 3 before, the
driving values roll back to what they were then.

`tools/learn_replay.py SINCE` runs it over recorded drives. It keeps only what those drives really measured (pedal
response, understeer, trouble spots): a recording can't show what a changed gain would have done, so those nudges would
just pile up, and no route lessons, since they followed an older router.
`--report` prints what's known. The launcher's "Reset learning" deletes it, and `--no-learning` drives on the defaults.

Not learned: the map's lane geometry, junction behaviour, traffic-light timing, and anything that needs seeing beyond
what telemetry and the map say. It doesn't find new behaviour, it tunes what's there.
