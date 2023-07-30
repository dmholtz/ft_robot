# Precision Report

## Absolute Position

Definitions:

- Expected: expected position in the world coordinate system
- Actual: actual position in the world coordinate system
- Compensated: compensated argument to reach the expected position
- Variance: estimated variance of the TCP when controlled with the compensated coordinate (without considering backlash)

| Expected             | Actual          | Compensated      | Variance     |
| -------------------- | --------------- | ---------------- | ------------ |
| (-82.5, -202.5, 60)  | (-88, -208, 55) |                  |              |
| (-82.5, -202.5, 10)  | (-82, -211, 2)  | (-86, -191, 23)  | (±1, ±1, ±1) |
| (-157.5, -202.5, 10) | ()              | (-158, -191, 25) | (±2, ±2, ±1) |

After Recalibration

| Expected            | Actual | Compensated     | Variance     |
| ------------------- | ------ | --------------- | ------------ |
| (-82.5, -202.5, 10) |        | (-90, -197, 23) | (±1, ±1, ±1) |

## Repetition Accuracy

Experiment

1. Robot repeats 10 pick and place tasks.
2. Robot heads to the compensated point (-158, -191, 25) from the first experiment.
3. Error at compensated point is measured (by hand).

Error at final, compensated point: (-1, +4, -4)

## Continuous Path Accuracy

Experiment

1. Robot follows a line (with varying interpolation step size)
2. Robot heads to the compensated point (-158, -191, 25) from the first experiment.
3. Error at compensated point is measured (by hand).

Error at final, compensated point (step-size=2mm): (-27, +3, -25)
Error at final, compensated point (step-size=2mm): (-12, +8, -20)
Error at final, compensated point (step-size=4mm): (-5, +2, -15)
Error at final, compensated point (step-size=8mm): (+5, +0, -5)

With reduced speed (300):
Error at final, compensated point (step-size=8mm): (-1, -5, -7)
