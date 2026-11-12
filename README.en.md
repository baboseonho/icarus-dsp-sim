# ICARUS DSP simulation course

[한국어](README.md)

Simulation environment and lesson material for the ICARUS drone swarm course.
Members run the class on Ubuntu 24.04 in VS Code: during the lecture they fill the blanks (`___`) in a notebook
or change numbers, run the cell, and look at the plot right away. After five sessions, six drones change
formation and fly without hitting each other.

| Week | Topic | Status |
|---|---|---|
| 1 | Feedback for one drone (PID) | opens on the day |
| 2 | Cascaded control and waypoints | opens on the day |
| 3 | Formations and consensus (centralised) | opens on the day |
| 4 | Distributed consensus on ROS 2 | opens on the day |
| 5 | Collision avoidance and the final challenge | opens on the day |

The installer (`setup/`) installs everything the five weeks need in one go. Each week's folder (`week01/`, ...)
is pushed on the day of that session; install beforehand and `git pull` on the day.

## Install (once, on a fresh Ubuntu 24.04)

```bash
git clone https://github.com/baboseonho/icarus-dsp-sim.git ~/icarus-dsp-sim
cd ~/icarus-dsp-sim
bash setup/install.sh        # 20-40 min, asks for sudo; log in setup/install.log
```

The installer first checks the laptop (`setup/hw_profile.py`: CPU, memory, disk, GPU), then picks the ROS 2
variant (desktop or base) and the resource caps. Then it installs the base tools, ROS 2 Jazzy, a Python 3.12 venv
with the Crazyflow engine and Jupyter, builds `ros/dsp_ros`, installs VS Code with the Python and Jupyter extensions,
and runs `setup/check_env.py`.

With the Claude Code extension in VS Code, Claude can drive the install: open this folder and paste the prompt from
section 0 of the Korean README. Claude reads [`CLAUDE.md`](CLAUDE.md) for the steps and the rules (no passwords in chat,
hints rather than answers during class).

## Start a lesson

```bash
dsp          # alias added by the installer: cd here and source setup/env.sh
git pull     # today's week folder
code .
```

Open today's `weekNN/lesson.ipynb` and pick the `.venv (Python 3.12)` kernel.

## Resource caps (80 % of the laptop)

`import dsp` applies the caps ([`dsp/limits.py`](dsp/limits.py)):

- CPU: pinned to 80 % of the cores; child processes (simulator, ROS 2 nodes) inherit the same cores.
- Memory: the running cell is interrupted before the process tree reaches 80 % of RAM. On an Ubuntu desktop session a systemd slice (`icarus-dsp.slice`) adds a hard cap.
- GPU: not used for compute (CPU-only JAX); the 3D window is synced to the display refresh rate.

Lower it with `DSP_LIMIT_PCT=70 bash setup/install.sh --no-sudo`.

## Layout

- `setup/`: installer, hardware profile, environment, check
- `weekNN/`: `lesson.ipynb` (blanks) and `background.py` (provided code)
- `dsp/`: simulator (engine wrapper, onboard controllers, mocap, link, plotting helpers, resource caps); `dsp/ref/` gets each week's reference parts once that week is over
- `ros/dsp_ros/`: ROS 2 package (weeks 4-5)
- `docs/`: system design overview, engine comparison

New weeks arrive with `git pull` on the day.
