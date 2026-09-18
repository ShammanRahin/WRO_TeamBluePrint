# sim - steering-study simulations (historical)

These Python simulations are early design analysis, kept as part of the engineering record.
They informed the mechanical design before anything was built. They do NOT run on the car and
they do NOT describe the final steering - the build overruled their conclusion (see Decision #4
in [`DECISIONS.md`](../../DECISIONS.md) and finding #3 in
[`docs/engineering_findings.md`](../../docs/engineering_findings.md)). Preserved unedited
because the reasoning, and where it fell short, is itself useful.

Requirements: `pip install numpy matplotlib` (plus `pymunk` for `steering_dyn.py`).

| File | What it modelled |
|---|---|
| steering_study.py | Single central pivot vs Ackermann vs parallel bell-crank on scrub, parking, slop. |
| steering_dyn.py | Dynamic-friction version at parking speed. |
| rear_axle_scrub.py | Solid axle vs open differential: scrub and odometry bias (still supports Decision #10). |
| optimize_layout.py | Early geometry/CG layout optimisation. |
| geometry_sweep.py | Turn radius and two-arc park ratio R/L versus wheelbase and steering lock (Decisions #21, #28). |
| park_feasibility.py | Swept-polygon check of a symmetric two-arc parallel park (Decision #21). `--wheelbase` is required; the solver rejects wheelbases above 117 mm for a 165 mm body. |

## The lesson

`steering_study.py` scored candidates on scrub, parking and slop but never on the swept envelope
of the car body during a turn - the exact term that made the single central pivot wrong once
built. Simulate, but build the thing early.
