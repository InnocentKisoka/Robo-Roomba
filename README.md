# Robo Roomba

ROS 2 wall-following / room-clearing style controllers for **RoboMaster EP** in **CoppeliaSim** (USI robotics lab).

The robot uses ToF range sensors and odometry to approach walls, back off, and explore — Roomba-style behaviour in simulation.

## Features

- Open-loop motion controller
- Wall detection from ToF ranges
- Wall avoidance state machine
- Full Robo Roomba controller (and v2 when present)
- Launch files: `open_loop`, `standard`, `advanced`

## Package layout

```
robo_roomba/          Python nodes
launch/               ROS 2 launch files
demos/                Demo videos
package.xml / setup.py
test/
```

## Nodes

| Entry point | Role |
|---|---|
| `open_loop_controller` | Timed open-loop `cmd_vel` motion |
| `wall_detection` | ToF-based wall sensing |
| `wall_avoidance` | Approach / back-off avoidance |
| `robo_roomba_controller` | Combined Roomba-style controller |
| `robo_roomba_controller_v2` | Updated controller (if included) |

## Setup

Use the USI RoboMaster lab base project:

https://github.com/idsia-robotics/robotics-lab-usi-robomaster

Add this package under `src/`, then:

```bash
colcon build --symlink-install
source install/setup.zsh
```

## Usage

**Terminal 1 — CoppeliaSim**

```bash
cd robotics-lab-usi-robomaster
pixi run coppelia
```

Load `robomaster-random-wall-scene` or `robomaster-room-scene`, enable RT, start simulation.

**Terminal 2 — robot bringup**

```bash
pixi shell
source install/setup.zsh
ros2 launch robomaster_example ep_tof.launch.xml name:=/rm0
```

**Terminal 3 — Roomba controller**

```bash
pixi shell
source install/setup.zsh
ros2 launch robo_roomba standard_launch.py
# or
ros2 launch robo_roomba advanced_launch.py
# or
ros2 launch robo_roomba open_loop_launch.py
```

## Demos

See [`demos/`](demos/) for recorded simulation clips (Roomba run and wall avoidance).


