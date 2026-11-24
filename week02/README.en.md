# Week 2: Tilt to go where you want

[한국어](README.md)

A drone must tilt to move sideways. You use the week-1 PID on all three axes x, y, z and build the controller in the drone's
chip that turns a wanted acceleration into thrust and tilt. By the end of class your chip keeps the drone within 5 cm in the wind,
flies it through five waypoints, and follows a circle.

## Starting the class

```bash
dsp          # go to the repo and load the environment
git pull     # fetch today's session
code .
```

Open `week02/lesson_en.ipynb` in VS Code (the English notebook; `lesson.ipynb` is the same notebook in Korean). Check that the kernel in the top right is `ICARUS DSP (py3.12)`.
If VS Code asks for a kernel, pick `ICARUS DSP (py3.12)` from the `Jupyter Kernel…` list.
Run the first cell with `Shift+Enter`; when the "Simulator ready" line appears, you are set.

## How the class works

- Same as week 1. Type along in the `✍️` cells and check the plot right away with `Shift+Enter`. Blanks are `___`.
- Today's ①②③ run inside the drone's chip; ④ runs on the ground station (your laptop).
- Every stage prints a table that compares your result with the answer worked by hand. Guess the answer before you run it, and move on when it is green.
- Once the chip is complete (stage 5) you go straight to live flight and change the target, gains and wind of a flying drone yourself.

| Stage | What you type | What you see |
|---|---|---|
| 0 Setup | nothing | the week-1 PID on three axes at once, numpy warm-up |
| 1 Tilt | `g * tan(θ)` | flying while tilted, air drag |
| 2 ① Position controller | position and velocity errors, the output | compared with three inputs worked by hand |
| 3 ② Thrust and attitude | force, pitch, roll, thrust (the yaw rotation is given) | the force picture, four inputs compared, 64° at 20 m/s² |
| 4 ③ Tilt limit | the `if` condition and the ratio | 64° → 30° |
| 5 First flight | nothing (assemble the chip) | hover in the wind, PID vs PD |
| 6 Live flight | nothing | change target, gains, wind and the circle while it flies / PlotJuggler |
| 7 ④ Waypoints | the arrival check | ring-course path, tilt, video |
| 8 Circle and feed-forward | nothing (watch together) | feed-forward on / off compared |
| 9 Mission | nothing (try other gains) | four green cells / sliders |
| At home: Kalman filter | predict, gain, correct (optional) | thrust jitter: raw camera vs Kalman filter |

Mission criteria (default gains kp 6, kd 4, ki 2): hover in the wind with position error RMS < 5 cm, the ring course 5/5 within 40 s with max tilt ≤ 35°, circle trajectory RMS < 10 cm.

## Folder

```
week02/
├─ lesson_en.ipynb class notebook in English (with blanks). The only file you open
├─ lesson.ipynb    the same notebook in Korean
├─ background.py   background code: chip assembly, attitude controller, wind, courses, plots. No need to open it
└─ README.md       Korean version of this file (this one is README.en.md)
```
