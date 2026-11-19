# ROV Sediment-Collection — System Architecture

Design of a seabed sediment-collecting ROV, and the scope of what is
implemented in this submission.

## 1. Scope

The brief asks for the **complete system architecture** of the ROV, but only
the **sediment-collection sequence** is implemented in code. Navigation,
localization, station keeping and the physical vehicle are **out of scope** and
are shown here only as the surrounding context the collection sequence depends
on.

## 2. System overview

The ROV is tethered to a surface station. On-board compute runs ROS 2. The
surface sends high-level commands and receives telemetry/video over the tether.

```
  SURFACE                         TETHER            ROV (ROS 2)
  ┌──────────────┐                            ┌──────────────────────────────┐
  │ Operator GUI │   command / telemetry      │  ┌────────────┐              │
  │  + joystick  │◄───────────────────────────┤  │ navigation │ (out of     │
  └──────┬───────┘                            │  │ + station  │  scope)      │
         │ "collect sediment" (action goal)   │  │  keeping   │              │
         ▼                                     │  └────────────┘              │
  ┌──────────────┐                            │  ┌────────────────────────┐  │
  │ mission /    │───── action: CollectSediment──▶│ sediment_collector     │  │
  │ supervisor   │◄──── feedback / result ────│  │ (LIFECYCLE node +      │  │
  └──────────────┘                            │  │  ACTION server) ◄──IMPLEMENTED
                                              │  └───────┬───────▲────────┘  │
                              fail-safe inputs│          │ cmd   │ sensors   │
                               /rov/estop ───────────────┼───────┤           │
                               /rov/leak  ───────────────┘       │           │
                                              │  ┌───────▼───────┴────────┐  │
                                              │  │ actuators: pump,       │  │
                                              │  │ sampler arm, valves    │  │
                                              │  │ sensors: fill, current,│  │
                                              │  │ depth, leak            │  │
                                              │  └────────────────────────┘  │
                                              └──────────────────────────────┘
```

Only `sediment_collector` is built here. Its actuator/sensor layer is
**simulated in-process** so the sequence can be demonstrated without a vehicle;
the simulation sits exactly where real hardware drivers would plug in.

## 3. The sediment_collector node

A single ROS 2 **lifecycle node** that hosts one **action server**.

### Why a lifecycle node
Sediment collection moves hardware (a pump and a sampler arm). It must not run
until the vehicle is known-ready. The lifecycle gives explicit, inspectable
states:

| State       | Meaning                                         |
|-------------|-------------------------------------------------|
| Unconfigured| process up, nothing wired                       |
| Inactive    | action server exists but **rejects** goals      |
| Active      | ready — goals are accepted and executed         |
| Finalized   | shut down                                        |

Goals are only honoured in **Active**. `on_deactivate`/`on_cleanup` abort any
running goal and safe-stow — so an operator "deactivate" is itself a stop.

### Why an action
A collection takes seconds and must be observable and interruptible. An action
gives goal → **feedback** (phase, mass, elapsed) → **result**, plus **cancel**.
A service or topic would give neither progress nor a clean abort path.

### The sequence
`ARMING → LOWERING → PUMPING → SEALING → STOWING → DONE`

- **ARMING** – self-check / power the collection subsystem.
- **LOWERING** – deploy the sampler to the seabed.
- **PUMPING** – draw sediment until the requested mass is reached.
- **SEALING** – close the sample chamber so the sample is retained.
- **STOWING** – retract the sampler to a safe transit pose.

Feedback is published at 10 Hz throughout.

## 4. Fail-safes (explicit)

Guards are evaluated **every control cycle, in every phase**. Any trip runs
`safe_stow()` (pump OFF, sampler retracted) and then aborts the goal, so the
hardware always ends in a safe state.

| # | Fail-safe        | Trigger                                   | Outcome              |
|---|------------------|-------------------------------------------|----------------------|
| 1 | Lifecycle gating | goal arrives while not Active             | goal **rejected**    |
| 2 | Single-goal lock | goal arrives while one is running         | goal **rejected**    |
| 3 | E-stop           | `/rov/estop` = true                       | safe-stow + **abort**|
| 4 | Leak / ingress   | `/rov/leak` = true                        | safe-stow + **abort**|
| 5 | Operation timeout| elapsed > `goal.max_duration_s`           | safe-stow + **abort**|
| 6 | Pump overcurrent | pump current over limit (jam)             | safe-stow + **abort**|
| 7 | Operator cancel  | action cancel request                     | safe-stow + **cancel**|
| 8 | Leave Active     | `on_deactivate` / `on_cleanup`            | safe-stow + **abort**|

## 5. Interfaces

| Interface            | Type                          | Direction |
|----------------------|-------------------------------|-----------|
| `collect_sediment`   | action `CollectSediment`      | server    |
| `/rov/estop`         | `std_msgs/Bool`               | subscribe |
| `/rov/leak`          | `std_msgs/Bool`               | subscribe |
| `/rov/pump_current`  | `std_msgs/Float64`            | subscribe |

Parameters: `pump_rate_g_s` (25), `pump_current_limit` (10).

## 6. What a real deployment changes

Only the simulated hardware layer is replaced: `pump_current()` reads a real
driver, `safe_stow()` commands real actuators, and the fill/depth/leak values
come from real sensor topics. The lifecycle, action and fail-safe structure
stay exactly as they are.
