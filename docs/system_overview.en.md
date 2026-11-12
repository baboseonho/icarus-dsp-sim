# System overview

[한국어](시스템_구조.md)

The overall shape of the swarm system built in the course, and the contracts between its parts.

```
[ceiling camera mocap] --position--> [ground station] --target--> [radio link] --> [chip on each of 6 drones]
 sim:  virtual camera                 members' swarm code          sim:  localhost UDP     sim:  engine + members' cascaded PID
 real: ArUco ceiling camera           (formation, consensus,       real: ESP32 Wi-Fi UDP   real: firmware
                                       avoidance)
```

The ground-station code is the same in simulation and on the real drones. Moving to hardware only swaps `mocap` (for an ArUco node), `link` (for `EspUdpLink`), and drops the simulation-only `sim_bridge`.

### Interface contract

- Frame: world ENU, units m, s, rad. Drones are named `cf1` to `cf6`.
- State: `pose = {id, t, p:[x,y,z], q:[w,x,y,z]}`. The camera gives position only; the ground station gets velocity by differencing.
- Commands: `takeoff(h, dur)`, `go_to(p, yaw, dur)`, `cmd_position(p, yaw)` (50 Hz stream), `land(dur)`, `emergency_stop()`, the same names Crazyflie uses.
- ROS 2 topics: `/cfN/pose` (PoseStamped, 100 Hz), `/cfN/setpoint` (PoseStamped, 50 Hz), `/cfN/command` (String JSON), `/swarm/formation` (String JSON), `/swarm/emergency` (Empty), `/swarm/status` (String JSON).
- ROS 2 stays inside the laptop. The drones are reached over UDP only; DDS never goes over Wi-Fi.

The physics engine is Crazyflow 0.4.0 (JAX physics, MuJoCo rendering). The comparison with the alternative is in [`docs/engine_bakeoff.md`](engine_bakeoff.md) (Korean). The vehicle model is `cf2x_T350` (37.9 g).
