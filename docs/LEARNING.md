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

## From the driver

After any hand-over (a fault, a sharp turn, a toll gate, or the driver just taking control), the next 600 m or 2 minutes are a lesson. For each junction path and curved lane passed it keeps the driver's speed and line every 5 m (`bins`).

- Next time the truck follows that line (at most 1.5 m off the lane, eased in and out) and takes the place no faster than 5 % over that speed, braking in time. On those lanes the trailer swing makes way for the line, which already includes it.
- A sharp turn the driver has shown is no longer a reason to hand back.
- A line only counts on a junction's path driven steadily (within 1.5 m across the lane, and at most 1.5 m off it): a swing across is a lane change, and on roads only the speed is kept.
- A lesson with damage (a crash fine, or 0.5 % wear) is thrown away. Speeds within 40 m before or 30 m after a stop are ignored: that was a light or traffic. Each pass AutoHaul drives alone relaxes the speed by 3 % until the driver shows otherwise, so a slow lesson can't stay forever.
- At a toll booth where the driver paid: the other booths of the plaza cost 150 m more, the booth is no longer on the avoid list, and it stops where they stopped.
- While the driver drives, curve speed (`lat_acc_max`) and `accel_max` move 1 % of the way to theirs, per curve or second.

The lesson is what the map can express: speed and sideways line along lanes. It can't copy a move off the mapped lanes, like a three-point turn.

## Handing over at speed

An automatic hand-over above 7 km/h no longer lets go at once. AutoHaul holds the lane, brakes at 1.2 m/s^2 to a stop, puts the hazards on and sounds the alarm every 4 s, until a driving key or `/`. It lets go at once if it can't do that: it lost the lane, someone else is steering, the game doesn't take its inputs.

## Scorecards

A scorecard per drive (over 30 game km) has takeovers and faults per 100 game km, fines, tracking error, curve
overshoots, GPS disagreements and missed turns. If the last 3 drives score over 1.5x worse than the 3 before, the
driving values roll back to what they were then.

`tools/learn_replay.py SINCE` runs it over recorded drives. It keeps only what those drives really measured (pedal
response, understeer, trouble spots): a recording can't show what a changed gain would have done, so those nudges would
just pile up, and no route lessons, since they followed an older router.
`--report` prints what's known. The launcher's "Reset learning" deletes it, and `--no-learning` drives on the defaults.

Not learned: the map's lane geometry, junction rules, traffic-light timing, and anything that needs seeing beyond what
telemetry and the map say. It tunes what's there and repeats what the driver shows; it doesn't invent new behaviour.
