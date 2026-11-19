# rov_sediment

ROV **sediment-collection sequence** in ROS 2 C++ — a **lifecycle node** that
runs the seabed sampling sequence as a **ROS 2 action** with **explicit
fail-safes**.

Design rationale and the full-system context are in [ARCHITECTURE.md](ARCHITECTURE.md).

Scope: only the collection sequence is implemented. Navigation, localization,
station keeping and the physical vehicle are out of scope; the pump/sampler and
its sensors are simulated in-process so the sequence runs without hardware.

- Tested on: ROS 2 **Humble**, Ubuntu 22.04, C++17.

## Build

```bash
cd <this workspace>            # the folder containing src/
source /opt/ros/humble/setup.bash
colcon build --packages-select rov_sediment
source install/setup.bash
```

## Run

**Terminal 1 — bring the node up and auto-activate it:**

```bash
ros2 launch rov_sediment sediment.launch.py
```

**Terminal 2 — request a collection (target grams, timeout seconds):**

```bash
ros2 run rov_sediment collect_client 50 30
```

You will see phase-by-phase feedback and a `SUCCEEDED` result.

### Trigger a fail-safe

While a collection is running (use a big target so it lasts, e.g.
`collect_client 300 30`), in another terminal:

```bash
ros2 topic pub --once /rov/estop std_msgs/msg/Bool "{data: true}"   # emergency stop
ros2 topic pub --once /rov/leak  std_msgs/msg/Bool "{data: true}"   # water ingress
```

The goal aborts immediately with a safe-stow. Reset with `{data: false}`.
Pump-jam (overcurrent) fail-safe — inject a high current while PUMPING:

```bash
ros2 topic pub --once /rov/pump_current std_msgs/msg/Float64 "{data: 15.0}"
```

Reset after: `ros2 topic pub --once /rov/pump_current std_msgs/msg/Float64 "{data: 0.0}"`

### Manual lifecycle control (instead of the launch file)

```bash
ros2 run rov_sediment sediment_collector_node
ros2 lifecycle set /sediment_collector configure
ros2 lifecycle set /sediment_collector activate
# a goal sent before 'activate' is rejected (lifecycle fail-safe)
```

## Layout

```
src/rov_sediment/
├── action/CollectSediment.action      goal / result / feedback
├── src/sediment_collector_node.cpp    lifecycle node + action server + fail-safes
├── src/collect_client.cpp             C++ action client (demo / test driver)
└── launch/sediment.launch.py          bring-up + auto configure/activate
```
