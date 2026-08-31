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
└── scripts/install_ros2_humble.sh
```

## Setup (WSL2 + Ubuntu 22.04)

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

Each scenario is one launch command. Watch the `aeb_node` log for the
state transitions and `sim_node` for the closing gap.

```bash
# 1. Stopped car 60 m ahead — AEB brings the ego to a stop before impact
ros2 launch aeb_demo aeb_sim.launch.py scenario:=stationary_lead

# 2. Same scenario, AEB disabled — the ego drives straight into the lead
ros2 launch aeb_demo aeb_sim.launch.py scenario:=stationary_lead enable_aeb:=false

# 3. Lead vehicle brakes hard from matched speed at t = 4 s
ros2 launch aeb_demo aeb_sim.launch.py scenario:=hard_brake

# 4. Slower-moving lead vehicle
ros2 launch aeb_demo aeb_sim.launch.py scenario:=slower_lead enable_aeb:=true

# Add RViz to any of the above
ros2 launch aeb_demo aeb_sim.launch.py scenario:=hard_brake rviz:=true
```

Inspect topics from a second sourced shell:

```bash
ros2 topic echo /aeb/status
ros2 topic echo /aeb/hazard
ros2 run rqt_graph rqt_graph
```

### What you should see (`stationary_lead`, AEB on)

```
[sim_node]  t=  2.0  v_ego= 14.0  v_lead=  0.0  gap=  32.1
[aeb_node]  CLEAR -> WARN   ttc=1.94s  req_decel=3.6 m/s^2  v_ego=14.0 m/s
[aeb_node]  WARN -> BRAKE   ttc=1.15s  req_decel=6.1 m/s^2  v_ego=14.0 m/s
[sim_node]  t=  4.5  v_ego=  3.2  v_lead=  0.0  gap=   4.0
[sim_node]  t=  5.1  v_ego=  0.0  v_lead=  0.0  gap=   2.4     # stopped, no collision
```

With `enable_aeb:=false` the same run ends in `COLLISION at t=... impact speed=14.0 m/s`.

## Tests

The decision logic in `safety.py` has no ROS dependency and is unit-tested:

```bash
colcon test --packages-select aeb_demo
colcon test-result --verbose
# or directly:
python3 -m pytest src/aeb_demo/test -q
```

## Design notes

* **Closing speed from the sensor, not the map.** `perception_node`
  differentiates the range signal and low-pass filters it. This mirrors how a
  radar/lidar AEB works — you don't assume you know the lead vehicle's speed.
* **Two brake gates (defense in depth).**
  * *TTC gate*: `range / closing_speed < ttc_brake`. Cheap, intuitive, but
    blind to how hard you'd actually have to brake.
  * *Required-deceleration gate*: `a_req = v_close² / (2·(range − standoff))`.
    Trips when `a_req` exceeds `brake_decel_frac · max_brake_decel`, i.e. when
    comfortable braking is no longer enough. Catches high-speed cases the TTC
    gate reacts to too late.
* **Latching.** Once `BRAKE` is entered it holds until `v_ego < stop_speed`.
  Real AEB is certified to not release mid-event; it also stops the state
  chattering around the threshold.
* **`standoff`** keeps the target stopping point ~2 m short of the obstacle.
* **Separation of concerns.** All math is in `safety.py`; the nodes only do
  ROS I/O. That is what makes the logic testable without a running graph.

## Known simplifications (talking points, not hidden bugs)

* Purely longitudinal — no steering, no lateral path, single obstacle.
* The "lidar" is synthetic and noise is Gaussian; no false positives / clutter.
* First-order vehicle model; no actuator lag or brake-pressure ramp.
* Fixed thresholds; a production system would schedule them on speed, road
  friction estimate, and driver-attention state.

## Possible extensions

* Add a Forward Collision Warning HMI node subscribing to `/aeb/status`.
* Replace the synthetic scan with `ros2 bag` playback or the F1TENTH gym.
* Add an Adaptive Cruise Control mode (follow at a time-gap) that hands off to AEB.
* Record runs with `ros2 bag record -a` and add a regression test that replays
  a bag and asserts "no collision".
