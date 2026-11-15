# Week 1: Feedback that lifts one drone

[한국어](README.md)

One drone moves only up and down. You write the PID that watches the height and corrects the thrust.
By the end of class your PID climbs the drone from 0.5 m to 1.5 m and holds it there.

## Starting the class

```bash
dsp          # go to the repo and load the environment (alias created by the installer)
git pull     # fetch today's folder (week01/)
code .       # open the repo in VS Code
```

In VS Code open `week01/lesson.ipynb`. Check that the kernel in the top right is `ICARUS DSP (py3.12)`.
If VS Code asks for a kernel, pick `ICARUS DSP (py3.12)` from the `Jupyter Kernel…` list.
Run the first cell with `Shift+Enter`; when the "simulator ready" line (printed in Korean) appears, you are set.

## How the class works

- Follow the explanation and type along in cells marked `✍️`. Blanks are `___`.
- Run a cell with `Shift+Enter` and the plot appears right away.
- Change the numbers (`kp`, `kd`, `ki`, …), run again and watch how the plot changes.
- If you run a cell with a blank still empty, it tells you a blank (`___`) is not filled yet.
- Each stage's cell already holds the previous stage's code, so if you miss one you can rejoin at the next stage.

| Stage | What you type | What you see |
|---|---|---|
| 0 Setup | nothing | drone data |
| 1 Hover thrust | the thrust needed to hang still | exact mass / 10% heavier / lighter |
| 2 P control | the error, the P output | a step response that bounces and sags, `kp` 2·8·20 compared |
| 3 D control | the rate, the output | P vs PD, `kd` 2·4·8 compared |
| 4 I control | the integral, the output | the sag disappears, overshoot appears instead |
| 5 Integrate only when close | one `if` condition | the overshoot shrinks |
| 6 Mission | your own gain numbers | six green cells / sliders to try values instantly |
| 7 Live flight | nothing | change target and gains while it flies, view in PlotJuggler (optional) |

Mission criteria: overshoot < 10%, 2% settling < 2 s, error over the last second < 2 cm, for both the exact-mass drone and the 10% heavier one.

## Folder

```
week01/
├─ lesson.ipynb    class notebook (with blanks). The only file you open
├─ background.py   background code: physics engine, levelling, plots. No need to open it
└─ README.md       Korean version of this file (this one is README.en.md)
```
