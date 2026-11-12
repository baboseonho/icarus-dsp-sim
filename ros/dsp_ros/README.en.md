[한국어](README.md)

# dsp_ros

The ROS 2 package for weeks 4 and 5. The simulator, a virtual ceiling camera, the radio link, one agent per drone and the commander each run as their own node.

```
sim_bridge ──/sim/truth──▶ mocap ──/cfN/pose──┬──▶ agent_cfN ──/cfN/setpoint──▶ link ──UDP──▶ drone chips (inside sim_bridge)
                                              └──▶ commander ──/cfN/command───▶ link
                                                       └──/swarm/formation──▶ agent_cfN
```

## Nodes

| Node | What it does |
|---|---|
| `sim_bridge` | Runs the engine and the six drone chips (your week-2 `Onboard`) in real time. Commands arrive over real UDP; true positions go out on `/sim/truth`. Given `log_path`, it saves the trajectories to an npz file. |
| `mocap` | Adds noise, delay and dropped frames to `/sim/truth` and publishes `/cfN/pose`. Plays the ceiling camera. |
| `link` | Ground-side radio. Takes `/cfN/setpoint`, `/cfN/command` and `/swarm/emergency` and sends them to the drones as UDP packets. |
| `agent_cfN` | Distributed control for one drone. It is the `Agent` class you filled in `weekNN/exercise.py`, loaded as is (`DSP_SRC` switches to the solution or a submission). |
| `commander` | Decides take-off, formations, landing and emergency stop: `W4Machine` in week 4, your week-5 `ScenarioMachine` for the final run. When it exits, the whole launch shuts down. |

## Topics

| Topic | Type | Content |
|---|---|---|
| `/cfN/pose` | `PoseStamped` | Drone position, 100 Hz |
| `/cfN/setpoint` | `PoseStamped` | The agent's target point, 50 Hz |
| `/cfN/command` | `String` (JSON) | `{"type": "takeoff" \| "go_to" \| "land" \| "emergency_stop", ...}` |
| `/swarm/formation` | `String` (JSON) | `{"name": "hexagon", "center": [x, y, z] or null}` |
| `/swarm/emergency` | `Empty` | Emergency stop for every drone |
| `/swarm/status` | `String` (JSON) | Commander state (`state`, `formation`, `t`, `form_err`, `dmin`) |
| `/sim/truth` | `PoseArray` | Simulation only. True positions from the engine |
| `/mocap/disable` | `String` | Test hook. Send `"cf3"` to cut cf3's pose, `""` to bring it back |

`N` runs from 1 to 6. ROS 2 stays on the laptop; the drones are only ever spoken to over UDP.

## Running

```bash
ros2 launch dsp_ros swarm.launch.py                          # week 4: distributed hexagon
ros2 launch dsp_ros swarm.launch.py week:=5 scenario:=final  # final scenario
```

Usually `week04/run.py` and `week05/run.py --mode ros` start it for you with the right arguments.

| Argument | Default | Meaning |
|---|---|---|
| `week` | `4` | Which week's `exercise.py` the agents are loaded from |
| `graph` | `ring` | Communication graph: `ring`, `star`, `complete`, `split` |
| `scenario` | `w4` | `w4` or `final` |
| `formation` | `hexagon` | Formation for `w4`: `line`, `hexagon`, `triangle` |
| `hold` | `30.0` | How long `w4` holds the formation [s] |
| `center` | (none) | Formation centre for `w4`, as `"x,y,z"` |
| `seed` | `0` | Random seed shared by the sim, mocap and link |
| `init` | `grid` | Starting layout: `grid` (2×3) or `scatter` |
| `loss` | `0.02` | UDP packet loss rate |
| `noise` | `0.005` | Standard deviation of the mocap position noise [m] |
| `delay` | `0.020` | Mocap latency [s] |
| `drop` | `0.01` | Chance that mocap misses a frame |
| `link_delay` | `0.010` | Radio latency [s] |
| `downwash` | `true` | Downwash from a higher drone pushing down on the one below |
| `bag` | (none) | If set, `ros2 bag record` under that name |
| `log_path` | (none) | If set, sim_bridge saves the trajectories to this npz file |

## Building

Build from the repository root with the venv active.

```bash
source setup/env.sh
cd ros
python -m colcon build --symlink-install
```

Why run `colcon` with the venv's `python`: whichever Python runs the build ends up in the shebang of the node executables. Build with the system `python3` and the nodes start without crazyflow and the other venv packages, and die on import. Thanks to `--symlink-install`, editing a `.py` file needs no rebuild; you only rebuild when the list of executables in `setup.py` changes. After the very first build, run `source setup/env.sh` once more so that `ros2` can find the package.

## Moving to real hardware

| Simulation | Real hardware |
|---|---|
| `sim_bridge` | Gone. The real drones and the camera take its place. |
| `mocap` | An OpenCV ArUco node publishing the same `/cfN/pose` |
| `UdpLink` inside `link` | `EspUdpLink` |
| `agent_cfN`, `commander` | Unchanged |
