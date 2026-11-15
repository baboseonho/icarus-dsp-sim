# ICARUS DSP simulation course

[한국어](README.md)

Simulation environment and lesson material for the ICARUS drone swarm course.
Members run the class on Ubuntu 24.04 or 26.04 in VS Code: during the lecture they fill the blanks (`___`) in a notebook
or change numbers, run the cell, and look at the plot right away. After five sessions, six drones change
formation and fly without hitting each other.

| Week | Topic | Status |
|---|---|---|
| 1 | Feedback for one drone (PID) | open ([`week01/`](week01/README.en.md)) |
| 2 | Cascaded control and waypoints | opens on the day |
| 3 | Formations and consensus (centralised) | opens on the day |
| 4 | Distributed consensus on ROS 2 | opens on the day |
| 5 | Collision avoidance and the final challenge | opens on the day |

The installer (`setup/`) installs everything the five weeks need in one go. Each week's folder (`week01/`, ...)
is pushed on the day of that session; install beforehand and `git pull` on the day.

## Install (once, on a fresh Ubuntu 24.04 or 26.04)

| Ubuntu | ROS 2 | Python 3.12 (`.venv`) | Notes |
|---|---|---|---|
| 24.04 | Jazzy | system Python 3.12 | reference setup, all five weeks |
| 26.04 | Lyrical | downloaded by uv (system Python 3.14 is left alone) | weeks 1-3 work as is; weeks 4-5 (ROS 2) use `setup/docker/` |

The installer reads `/etc/os-release` and picks the row. On 26.04 a failed ROS 2 install only prints a warning;
the Python and week-1 setup still runs to the end.

```bash
git clone https://github.com/baboseonho/icarus-dsp-sim.git ~/icarus-dsp-sim
cd ~/icarus-dsp-sim
bash setup/install.sh        # 20-40 min, asks for sudo; log in setup/install.log
```

The installer first checks the laptop (`setup/hw_profile.py`: CPU, memory, disk, GPU), then picks the ROS 2
variant (desktop or base) and the resource caps. Then it installs the base tools, ROS 2 (Jazzy or Lyrical), a Python 3.12 venv
with the Crazyflow engine and Jupyter (kernel `ICARUS DSP (py3.12)`), builds `ros/dsp_ros`, installs VS Code with the Python and Jupyter extensions,
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

Open today's `weekNN/lesson.ipynb` (e.g. `week01/lesson.ipynb`) and pick the `ICARUS DSP (py3.12)` kernel.

On Ubuntu 26.04, ROS 2 Lyrical is built for Python 3.14, so the course kernel (Python 3.12) cannot import `rclpy`.
Weeks 1-3 and PlotJuggler's UDP plots work natively; for weeks 4-5 use the `setup/docker/` container (24.04 + Jazzy).

## Live view (from week 1)

Every lesson ends with a live-flight cell, `live(...)`. It flies the drone(s) in real time; move the sliders
(gains, target, wind, repulsion) while it flies and the notebook chart updates at once. ■ stops it, and
`window=True` also opens the MuJoCo 3D window.

The same values go to **PlotJuggler**: run `plotjuggler("week01")`, then click **Yes → OK** in its window.
In weeks 4–5 PlotJuggler plots the ROS 2 topics directly (Jazzy; the ROS plugin is not yet released for Lyrical), and from a terminal where you typed `dsp` you can use
`ros2 param set /agent_cf1 k 0.1`, `rqt_reconfigure`, `rqt_graph`, `rviz2 -d ros/dsp_ros/rviz/swarm.rviz`
(rqt and RViz come with the ROS 2 desktop variant only) and `ros2 bag record`.
These tools are separate programs and run outside the resource caps below.

## Resource caps (80 % of the laptop)

`import dsp` applies the caps ([`dsp/limits.py`](dsp/limits.py)):

- CPU: pinned to 80 % of the cores; child processes (simulator, ROS 2 nodes) inherit the same cores.
- Memory: the running cell is interrupted before the process tree reaches 80 % of RAM. On an Ubuntu desktop session a systemd slice (`icarus-dsp.slice`) adds a hard cap.
- GPU: not used for compute (CPU-only JAX); the 3D window is synced to the display refresh rate.

Lower it with `DSP_LIMIT_PCT=70 bash setup/install.sh --no-sudo`.

## Layout

- `setup/`: installer, hardware profile, environment, check
- `setup/docker/`: Ubuntu 24.04 + ROS 2 Jazzy container (other Ubuntu versions, or weeks 4-5 on 26.04)
- `weekNN/`: `lesson.ipynb` (blanks) and `background.py` (provided code)
- `dsp/`: simulator (engine wrapper, onboard controllers, mocap, link, plotting helpers, resource caps); `dsp/ref/` gets each week's reference parts once that week is over
- `ros/dsp_ros/`: ROS 2 package (weeks 4-5)
- `docs/`: system design overview, engine comparison

New weeks arrive with `git pull` on the day.
