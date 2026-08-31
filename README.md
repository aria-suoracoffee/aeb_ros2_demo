# aeb_ros2_demo — Automatic Emergency Braking in ROS 2

A small, self-contained ROS 2 (Humble) workspace that implements an **Automatic
Emergency Braking (AEB)** function — the kind of longitudinal safety feature in
most modern ADAS stacks. It runs without Gazebo: a lightweight node simulates
the ego vehicle, a lead vehicle, and two sensors (lidar + radar), so the whole
**sense → fuse → decide → act** loop can run on a laptop.

```
             /scan  ┌─────────────────┐ /aeb/meas/lidar ┐
  ┌────────┐ ──────►│ perception_node │ ───────────────► │
  │        │        └─────────────────┘                  │   ┌─────────────┐  /aeb/hazard
  │sim_node│                                             ├──►│ fusion_node │ ────────────┐
  │        │ /radar ┌─────────────────┐ /aeb/meas/radar  │   │  2-state KF │             │
  │ ego +  │ ──────►│   radar_node    │ ───────────────► ┘   └─────────────┘             │
  │ lead + │        └─────────────────┘                                                  ▼
  │ lidar +│                                          ┌──────────┐              ┌───────────────┐
  │ radar  │◄──── /cmd_vel ──────────────────────────│ aeb_node │◄──/driver────│  driver_node  │
  │        │──── /ego/odom ─────────────────────────►│  CLEAR / │   /cmd_vel    └───────────────┘
  └────────┘                                          │ WARN /   │
                                                      │ BRAKE    │
                                                      └──────────┘
```

* **`sim_node`** — 1-D longitudinal model of ego + lead vehicle. Publishes a
  synthetic `sensor_msgs/LaserScan` (`/scan`), a synthetic `RadarReturn`
  (`/radar`, with dropouts and false alarms), `/ego/odom`, `/sim/collision`,
  RViz markers, and TF.
* **`perception_node`** — gates the scan to the vehicle's path and reports an
  accurate in-path **range** (plus a loose differenced closing speed) as an
  `aeb_interfaces/Measurement` on `/aeb/meas/lidar`.
* **`radar_node`** — validity-gates raw radar returns and reports an accurate
  **closing speed** (Doppler) with a noisy range on `/aeb/meas/radar`.
* **`fusion_node`** — a constant-closing-speed **Kalman filter** over
  `[range, closing_speed]`. Weights each sensor by its reported variance,
  **Mahalanobis-gates** outliers (radar false alarms), and **coasts** on the
  model through dropouts. Publishes the fused `aeb_interfaces/Hazard`.
* **`aeb_node`** — the safety controller. Two independent brake gates
  (time-to-collision and required-deceleration), a 3-state machine
  (`CLEAR` / `WARN` / `BRAKE`), hysteresis, and a brake latch + stop-and-hold.
* **`driver_node`** — stand-in for a human driver; commands a constant cruise
  speed for AEB to supervise.
* **`aeb_interfaces`** — custom messages: `Hazard`, `AEBStatus`, `Measurement`,
  `RadarReturn`.

The two pure, ROS-free, unit-tested logic modules are the heart of it:
`safety.py` (the brake decision) and `fusion.py` (the Kalman filter).

## Layout

```
aeb_ros2_demo/
├── src/
│   ├── aeb_interfaces/        # custom msgs (ament_cmake)
│   │   └── msg/{Hazard,AEBStatus,Measurement,RadarReturn}.msg
│   └── aeb_demo/              # nodes (ament_python)
│       ├── aeb_demo/
│       │   ├── safety.py            # pure brake-decision logic, unit-tested
│       │   ├── fusion.py            # pure Kalman-filter math, unit-tested
│       │   ├── sim_node.py
│       │   ├── perception_node.py   # lidar  -> Measurement
│       │   ├── radar_node.py        # radar  -> Measurement
│       │   ├── fusion_node.py       # Measurements -> Hazard
│       │   ├── aeb_node.py
│       │   └── driver_node.py
│       ├── launch/aeb_sim.launch.py
│       ├── config/params.yaml
│       ├── rviz/aeb.rviz
│       └── test/{test_safety,test_fusion}.py
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

### Sensor fusion (`fusion.py` / `fusion_node`)

* **Complementary sensors.** Lidar range variance is ~0.02 m²; radar range
  variance is ~0.36 m². Radar rate variance (Doppler) is ~0.02 (m/s)²; the
  lidar's differenced rate is ~4 (m/s)². The Kalman gain therefore takes range
  mostly from lidar and closing speed mostly from radar — automatically, from
  the reported variances, with no hand-tuned switch.
* **Model.** State `[range, closing_speed]`, constant closing speed, unmodelled
  acceleration as process noise (`sigma_a`). `F = [[1, −dt], [0, 1]]`.
* **Outlier gating.** Every measurement is checked against the predicted state
  with a Mahalanobis distance; `d² > gate_chi2` (9.21, 2-dof 99%) is rejected.
  This is what drops the radar false alarms without a separate filter.
* **Graceful degradation.** No accepted measurement for `coast_timeout` → the
  filter coasts on the model with inflating covariance; past `drop_timeout` the
  track is invalidated and AEB loses the hazard (correct: a blind sensor stack
  should not brake on a guess).

### Braking (`safety.py` / `aeb_node`)

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
* **Separation of concerns.** The math is in `safety.py` and `fusion.py`; the
  nodes only do ROS I/O. That is what makes both testable without a running graph.

## Known simplifications (talking points, not hidden bugs)

* Purely longitudinal — no steering, no lateral path, single obstacle, so
  "fusion" is one track with fixed data association (no JPDA / MHT).
* Sensors are synthetic. Lidar noise is Gaussian with no clutter; radar has
  dropouts and false alarms but no multipath, no angle, no RCS model.
* No track confirmation logic — a radar false alarm as the *first* return can
  delay track acquisition by up to `drop_timeout` (~1.5 s).
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
