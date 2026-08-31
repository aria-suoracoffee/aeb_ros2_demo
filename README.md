# aeb_ros2_demo — Automatic Emergency Braking in ROS 2

A small, self-contained ROS 2 (Humble) workspace that implements an **Automatic
Emergency Braking (AEB)** function — the kind of longitudinal safety feature in
most modern ADAS stacks. It runs without Gazebo: a lightweight node simulates
the ego vehicle, a lead vehicle, and a forward-facing lidar, so the whole
perception → decision → control loop can run on a laptop.

```
                         /scan (LaserScan)
  ┌───────────┐  ───────────────────────────────►  ┌──────────────────┐
  │  sim_node │                                     │ perception_node  │
  │           │  ◄───────────────────────────────   │  scan → range,   │
  │  ego +    │        /cmd_vel (Twist)             │  closing speed,  │
  │  lead +   │                                     │  TTC             │
  │  fake     │                                     └────────┬─────────┘
  │  lidar    │                                       /aeb/hazard
  │           │        /driver/cmd_vel  ┌──────────┐          │
  │           │  ◄──────────────────────│ driver   │          ▼
  │           │                         │ _node    │   ┌──────────────┐
  │  /ego/odom├────────────────────────────────────►   │   aeb_node   │
  └───────────┘                             (ego v)    │ state machine│
                                                       │ CLEAR/WARN/  │
                                                       │ BRAKE  +     │
                                                       │ /cmd_vel out │
                                                       └──────────────┘
```

* **`sim_node`** — 1-D longitudinal model of ego + lead vehicle. Publishes
  `sensor_msgs/LaserScan` (`/scan`), `nav_msgs/Odometry` (`/ego/odom`),
  `/sim/collision`, RViz markers, and TF.
* **`perception_node`** — gates the scan to the vehicle's path, takes the
  nearest return, and estimates **closing speed** by differentiating successive
  scans (sensor-only — it does *not* use the ego odometry). Publishes
  `aeb_interfaces/Hazard` with range / closing speed / time-to-collision.
* **`aeb_node`** — the safety controller. Two independent brake gates
  (time-to-collision and required-deceleration), a 3-state machine
  (`CLEAR` / `WARN` / `BRAKE`), and a latch that holds the brake until the
  vehicle is stopped. Relays the driver command normally; overrides it with a
  full-brake command when a gate trips.
* **`driver_node`** — stand-in for a human driver / upstream planner; just
  commands a constant cruise speed so there is something for AEB to override.
* **`aeb_interfaces`** — the two custom messages (`Hazard`, `AEBStatus`).

## Layout

```
aeb_ros2_demo/
├── src/
│   ├── aeb_interfaces/        # custom msgs (ament_cmake)
│   │   └── msg/{Hazard,AEBStatus}.msg
│   └── aeb_demo/              # nodes (ament_python)
│       ├── aeb_demo/
│       │   ├── safety.py            # pure decision logic, unit-tested
│       │   ├── sim_node.py
│       │   ├── perception_node.py
│       │   ├── aeb_node.py
│       │   └── driver_node.py
│       ├── launch/aeb_sim.launch.py
│       ├── config/params.yaml
│       ├── rviz/aeb.rviz
│       └── test/test_safety.py
├── Dockerfile / .devcontainer/       # reproducible ROS 2 Humble environment
└── scripts/{install_ros2_humble.sh, run_demo.sh}
```

## Setup — Docker (recommended)

Works on any Docker host (Linux VPS, macOS, Windows) and in GitHub Codespaces.
Needs no ROS install on the host.

```bash
docker build -t aeb_demo .
docker run -it --rm --shm-size=1g -v "$PWD":/ws -w /ws aeb_demo bash
```

`--shm-size=1g` matters: the default 64 MB `/dev/shm` starves FastDDS shared
memory transport and the nodes silently fail to communicate.

Inside the container:

```bash
colcon build && source install/setup.bash
scripts/run_demo.sh stationary_lead
```

VS Code users: "Reopen in Container" uses `.devcontainer/` and builds on create.

## Setup — native (WSL2 + Ubuntu 22.04)

From Windows, install WSL2 with Ubuntu 22.04 if you don't have it:

```powershell
wsl --install -d Ubuntu-22.04
```

Then, **inside the Ubuntu shell**, install ROS 2 Humble and copy the workspace
out of `/mnt/c` into the Linux filesystem (colcon is much happier there):

```bash
cp -r /mnt/c/Users/ariars2/projects/aeb_ros2_demo ~/aeb_ros2_demo
cd ~/aeb_ros2_demo
bash scripts/install_ros2_humble.sh      # one-time
```

## Build

```bash
cd ~/aeb_ros2_demo
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run

`scripts/run_demo.sh <scenario> <enable_aeb>` builds if needed and launches.
Watch the `aeb_node` heartbeat (1 Hz) and state transitions.

```bash
scripts/run_demo.sh stationary_lead true    # stopped car 60 m ahead -> AEB stops the ego
scripts/run_demo.sh stationary_lead false   # AEB off -> ego drives into the lead (COLLISION)
scripts/run_demo.sh hard_brake              # lead matches speed then brakes hard at t=4 s
scripts/run_demo.sh slower_lead             # lead cruising 8 m/s slower than the ego
```

Or call the launch file directly (e.g. to add RViz):

```bash
ros2 launch aeb_demo aeb_sim.launch.py scenario:=hard_brake rviz:=true
```

Inspect topics from a second shell (`docker exec -it <container> bash`, or a new
sourced terminal):

```bash
ros2 topic echo /aeb/status
ros2 run rqt_graph rqt_graph
```

### What you should see (`stationary_lead`, AEB on)

```
[aeb_node] [CLEAR] valid=True range=  45.4 v_close=13.5 ttc=3.4 v_ego=14.0 cmd=14.0
[aeb_node] CLEAR -> WARN   ttc=2.36s  req_decel=3.2 m/s^2  v_ego=14.0 m/s
[aeb_node] WARN -> BRAKE   ttc=1.42s  req_decel=5.7 m/s^2  v_ego=14.0 m/s
[aeb_node] [BRAKE] valid=True range=  17.6 v_close=12.4 ttc=1.4 v_ego=12.1 cmd=0.0
[aeb_node] [BRAKE] valid=True range=   8.6 v_close=-0.0 ttc=999.9 v_ego=0.0 cmd=0.0   # stopped
[aeb_node] [BRAKE] valid=True range=   8.6 v_close= 0.2 ttc=  38.2 v_ego=0.0 cmd=0.0  # ...and held
```

The ego stops ~8–9 m short and holds the brake. With `enable_aeb:=false` the
same run ends in `COLLISION at t=... impact speed=14.0 m/s`.

## Tests

The decision logic in `safety.py` has no ROS dependency and is unit-tested:

```bash
colcon test --packages-select aeb_demo
colcon test-result --verbose
# or directly:
python3 -m pytest src/aeb_demo/test -q
```

## Design notes

* **Two brake gates (defense in depth).**
  * *TTC gate*: `range / closing_speed < ttc_brake`. Cheap, intuitive, but
    blind to how hard you'd actually have to brake.
  * *Required-deceleration gate*: `a_req = v_close² / (2·(range − standoff))`.
    Trips when `a_req` exceeds `brake_decel_frac · max_brake_decel`, i.e. when
    comfortable braking is no longer enough. Catches high-speed cases the TTC
    gate reacts to too late.
* **Hysteresis.** Leaving a more-severe state needs the signal to recover past
  a `hysteresis` factor (1.25×). Without it the noisy `required_decel` estimate
  chatters `WARN ↔ CLEAR` many times a second when it sits on the threshold.
* **Brake latch + stop-and-hold.** `BRAKE` holds until `v_ego < stop_speed`,
  then keeps holding (`cmd = 0`) while an obstacle stays within `hold_distance`.
  This is what production "AEB stop and hold" does; it also stops the ego from
  creeping forward again after the event (the stand-in driver keeps commanding
  cruise speed). Releases when the obstacle clears.
* **`standoff`** keeps the target stopping point ~2 m short of the obstacle.
* **Closing speed from the sensor, not the map.** `perception_node`
  differentiates the range signal and low-pass filters it — no dependence on
  knowing the lead vehicle's speed.
* **Separation of concerns.** All math is in `safety.py`; the nodes only do
  ROS I/O. That is what makes the logic testable without a running graph.

## Known simplifications (talking points, not hidden bugs)

* Purely longitudinal — no steering, no lateral path, single obstacle.
* The "lidar" is synthetic and noise is Gaussian; no false positives / clutter.
* First-order vehicle model; no actuator lag or brake-pressure ramp.
* Fixed thresholds; a production system would schedule them on speed, road
  friction estimate, and driver-attention state.
* Hysteresis damps *exiting* a state, not the first entry — you can still see a
  single brief `CLEAR→WARN→CLEAR` flap at the boundary. WARN is advisory only
  (no actuation), so this is cosmetic; the `BRAKE` decision does not flap.
* `closing_speed` from finite-differencing has a ~0.5 m/s noise floor at these
  settings; `min_closing_speed` gates it so a parked-car return doesn't
  produce a phantom TTC.

## Possible extensions

* Add a Forward Collision Warning HMI node subscribing to `/aeb/status`.
* Replace the synthetic scan with `ros2 bag` playback or the F1TENTH gym.
* Add an Adaptive Cruise Control mode (follow at a time-gap) that hands off to AEB.
* Record runs with `ros2 bag record -a` and add a regression test that replays
  a bag and asserts "no collision".
