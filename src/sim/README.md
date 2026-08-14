# sim - steering-study simulations (historical)

These Python simulations are early design analysis, kept as part of the engineering record.
They informed the mechanical design before anything was built. They do NOT run on the car and
they do NOT describe the final steering - the build overruled their conclusion (see Decision #4
and README finding #1). Preserved unedited because the reasoning, and where it fell short, is
itself useful.

| File | What it modelled |
|---|---|
| steering_study.py | Single central pivot vs Ackermann vs parallel bell-crank on scrub, parking, slop. |
| steering_dyn.py | Dynamic-friction version at parking speed. |
| rear_axle_scrub.py | Solid axle vs open differential: scrub and odometry bias (still supports Decision #10). |
| optimize_layout.py | Early geometry/CG layout optimisation. |

## The lesson

`steering_study.py` scored candidates on scrub, parking and slop but never on the swept envelope
of the car body during a turn - the exact term that made the single central pivot wrong once
built. Simulate, but build the thing early.
