# Assembly Line OS

A visual prototyping platform for assembly line operations. Design, test, and run manufacturing workflows from a **drag-and-drop web UI** that talks to real hardware through **ROS 2** and an **Arduino** over USB serial.

Originally developed for a textiles startup; now open source for anyone building automated manufacturing systems.

---

## Table of contents

1. [What you are running](#what-you-are-running)
2. [Features](#features)
3. [ROS 2 in this project](#ros-2-in-this-project)
4. [Architecture](#architecture)
5. [What starts when you launch](#what-starts-when-you-launch)
6. [Repository layout](#repository-layout)
7. [Quick start (local)](#quick-start-local)
8. [Using the Control Center](#using-the-control-center)
9. [Workflows in the UI](#workflows-in-the-ui)
10. [ROS 2 topics](#ros-2-topics)
11. [Simulation vs hardware](#simulation-vs-hardware)
12. [Debugging](#debugging)
13. [Arduino firmware](#arduino-firmware)
14. [Production service (systemd)](#production-service-systemd)
15. [License](#license)

---

## What you are running

Assembly Line OS is a **web application** plus **ROS 2 middleware**. The browser hosts a visual block editor for motors, relays, and sensors; Python **nodes** publish and subscribe on **topics** so the UI stays in sync with the **Arduino** (single serial connection for motors and relays). You can also run with **simulation** enabled when no hardware is connected.

You do **not** need deep ROS experience to change the frontend (`static/`, `templates/`) or a single node (`assembly_line_control/*.py`).

---

## Features

- **Visual workflow builder** — Drag-and-drop blocks; connect them like Scratch; run parallel branches
- **Motor control** — Up to 2 stepper motors with step-based commands
- **Relay control** — Up to 4 relays (JSON commands on `/relay/command`)
- **Real-time monitoring** — Motor progress, relay states, potentiometer, sensors
- **Control Center** — SYSTEM tab: rosbridge, Arduino, telemetry, pre-flight checks, E-STOP, reconnect, incident log
- **ROS 2 + rosbridge** — Browser uses WebSocket to ROS; default stack targets **ROS 2 Humble**
- **Hardware-ready** — Arduino firmware under `arduino/assembly_line_control/`

---

## ROS 2 in this project

**ROS 2** runs several **nodes** (processes) and lets them exchange **messages** on named **topics**. Nodes do not call each other’s functions directly; they **publish** and **subscribe**.

| Term | Meaning here |
|------|----------------|
| **Node** | A process: e.g. `arduino_controller`, `web_interface`, `rosbridge_websocket` |
| **Topic** | A named channel; many nodes can publish or subscribe |
| **Message** | Typed data (or JSON in a string) on a topic |
| **Launch file** | Starts a defined set of nodes with ports and parameters |

The browser does not speak ROS natively. **rosbridge** exposes ROS over **WebSocket**; the JavaScript client in this repo connects to it (default port **9090**).

---

## Architecture

How a control action reaches hardware:

```text
Browser (HTML / JavaScript)
    │  WebSocket → rosbridge    HTTP → Flask (pages, REST)
    ▼
rosbridge_websocket  ←→  ROS graph on one machine
    │                        ├── arduino_controller   (USB serial ↔ Arduino)
    │                        ├── sensor_controller
    │                        └── potentiometer_speed_node → /motor_speed/setpoint
    ▼
web_interface (Flask) — serves UI and API used by the Control Center
```

1. User builds or runs a workflow in the browser.
2. JS publishes commands on motor/relay (and related) topics via rosbridge.
3. **arduino_controller** owns the serial port and talks to the Arduino firmware.
4. Status topics update the canvas and the **SYSTEM** (Control Center) tab.

---

## What starts when you launch

Entry point: **`ros2 launch assembly_line_control assembly_line_control.launch.py`**

| Component | Role |
|-----------|------|
| **rosbridge_websocket** | Browser ↔ ROS; default WebSocket port **9090** (launch argument `rosbridge_port`) |
| **web_interface** | Flask UI + APIs; default **1111** (`web_port`) |
| **arduino_controller** | Motors + relays over **one** serial link (avoids port contention) |
| **sensor_controller** | Sensors after registration; `/sensor/status` |
| **potentiometer_speed_node** | Pot / roll-OD mapping to `/motor_speed/setpoint` (launch parameters in `assembly_line_control.launch.py`) |

---

## Repository layout

| Path | Contents |
|------|----------|
| `src/assembly_line_control/` | ROS 2 package: CMake, `package.xml`, `launch/` |
| `src/assembly_line_control/assembly_line_control/` | Python nodes (`web_interface.py`, `arduino_controller.py`, …) |
| `src/assembly_line_control/static/` | Frontend: `app.js`, block system, `controlCenter.js`, CSS |
| `src/assembly_line_control/templates/` | `index.html`, `remote.html`, … |
| `arduino/assembly_line_control/` | Sketch + `README.md`, `SETUP.md` |
| `setup_service.py` | Optional: install **systemd** unit to run the launch file at boot |

---

## Quick start (local)

### Prerequisites

- **ROS 2 Humble** (or match your deployment distro): [Install ROS 2](https://docs.ros.org/en/humble/Installation.html)
- **colcon** and a C++/Python build chain for `ament_cmake` packages
- **Python 3** (3.8+)
- **Arduino** optional for real motion

### Dependencies

```bash
pip install flask flask-cors
sudo apt install ros-humble-rosbridge-suite
```

If you use Conda or another env for ROS tooling, activate it before sourcing ROS.

### Build and run

From the **workspace root** (this repository’s top level—the directory that contains `src/`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch assembly_line_control assembly_line_control.launch.py
```

Open **`http://localhost:1111`** (or `http://<host>:1111` on another machine on your LAN).

Every new terminal needs `source /opt/ros/humble/setup.bash` and `source install/setup.bash` before `ros2` commands, unless you add those to your shell profile.

---

## Using the Control Center

The desktop UI opens the **SYSTEM** sidebar tab by default.

- **Status** — Rosbridge, Arduino serial, motors, potentiometer, speed setpoint, registered sensors
- **Telemetry** — Motor progress, relays, pot strip + sparkline, sensor table, **Arduino serial** lines that are not high-rate `analog` / `motor_status` JSON (via `/arduino/serial_log`, rate-limited)
- **Pre-flight / guided checks** — Stored per browser session
- **Reconnect Arduino** — `POST /api/arduino/reconnect`; **E-STOP** next to other actions
- **Incident log** — Export UI/ROS messages; **`GET /api/version`** for build metadata

**Keyboard:** `Ctrl+`` (backtick) jumps to SYSTEM. Last sidebar tab is stored in `localStorage` as `assemblyLineSidebarTab`.

**Remote:** **`/remote`** — compact **ROS / ARD / RUN** line under the header on small screens.

---

## Workflows in the UI

1. **Create** — Drag motor and relay blocks from the palette onto the canvas  
2. **Connect** — Link blocks into sequences (and parallel stacks where supported)  
3. **Configure** — Steps, speeds, relay states per block  
4. **Execute** — Run against live topics (or simulation)

Under the hood, commands and status use the ROS topics listed below.

---

## ROS 2 topics

| Topic | Purpose |
|-------|---------|
| `/motor1/command`, `/motor2/command` | Motor step commands |
| `/relay/command` | Relay commands (JSON) |
| `/motor1/status`, `/motor2/status` | Motor status |
| `/relay1/status` … `/relay4/status` | Relay states |
| `/arduino/status` | Connection: `connected`, `port`, `baud` |
| `/potentiometer/raw` | Smoothed analog 0–1023 |
| `/motor_speed/setpoint` | Speed setpoint (e.g. from potentiometer node) |
| `/sensor/status` | Per-sensor JSON after `sensor/register` |
| `/assembly_line/execution_state` | Shared run/stop across browsers |
| `/arduino/serial_log` | Other Arduino / non-handled JSON lines for Control Center (rate-limited) |

Inspect live traffic:

```bash
ros2 topic list
ros2 topic echo /motor1/status
```

---

## Simulation vs hardware

With **simulation** ON, the header warns that there is **no physical motion**—useful for UI and logic tests without an Arduino.

---

## Debugging

| Symptom | Things to check |
|---------|------------------|
| Blank UI or no live data | rosbridge up? Browser console for WebSocket errors; port **9090** not blocked |
| Motors / relays dead | Arduino connected? `/arduino/status`; Control Center reconnect; only one process should own the serial device |
| Wrong distro / missing packages | `echo $ROS_DISTRO`; `ros2 pkg list \| grep rosbridge` |
| Stale code after edits | Rebuild: `colcon build --symlink-install` and re-`source install/setup.bash` |

Node logs appear in the terminal where you launched the launch file (or in `journalctl` if using systemd).

---

## Arduino firmware

See **`arduino/assembly_line_control/README.md`** and **`arduino/assembly_line_control/SETUP.md`** for flashing, wiring, and firmware behavior.

---

## Production service (systemd)

To run the same launch file at boot as a system service, use the repo root script (run on the deployment machine, from this workspace):

```bash
sudo python3 setup_service.py
```

The script prints **`systemctl`** commands to enable/start the unit it installs. If your environment uses a non-Humble distro, ensure `/opt/ros/<distro>/setup.bash` is what that machine sources before `colcon`/`ros2` (the script tries to detect `ROS_DISTRO`).

**Internal deployment steps** (SSH, pull, rebuild on a specific host) are **not** maintained in this repository as markdown—keep that runbook in your team’s private ops docs if needed.

---

## License

MIT License — open source and free to use.
