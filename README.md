# Assembly Line OS

A visual prototyping platform for assembly line operations. Build, test, and iterate on manufacturing workflows using a drag-and-drop interface that connects directly to your hardware via ROS 2.

## What is Assembly Line OS?

Assembly Line OS helps you prototype and control assembly line operations through an intuitive web-based interface. Whether you're testing motor sequences, coordinating relay switches, or building complex multi-step workflows, this tool makes it easy to visualize and execute your assembly line logic.

Originally developed for a textiles startup, this software is now open source and available for anyone building automated manufacturing systems.

## Features

- **Visual Workflow Builder**: Drag-and-drop blocks to create assembly line sequences
- **Motor Control**: Control up to 2 stepper motors with precise step-based movement
- **Relay Control**: Manage up to 4 relays for switching actuators and tools on/off
- **Parallel Workflows**: Run multiple sequences simultaneously, just like Scratch
- **Real-time Monitoring**: See motor status, steps remaining, and relay states in real-time
- **ROS 2 Integration**: Built on ROS 2 Humble with WebSocket communication
- **Hardware Ready**: Connects to Arduino-based hardware via USB serial

## Quick Start

### Prerequisites

- ROS 2 Humble installed
- Python 3.8+
- Arduino (optional, for hardware control)

### Installation

1. **Set up your environment:**
   ```bash
   conda activate ros_env
   source /opt/ros/humble/setup.bash
   ```

2. **Build the workspace:**
   ```bash
   cd /Users/danielhuinda/robotics/jmm
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Install dependencies:**
   ```bash
   pip install flask flask-cors
   sudo apt install ros-humble-rosbridge-suite
   ```

### Running the System

Launch everything with one command:

```bash
ros2 launch assembly_line_control assembly_line_control.launch.py
```

Then open your browser to `http://localhost:1111`

## How It Works

1. **Create Workflows**: Use the block palette to drag motor and relay control blocks onto the canvas
2. **Connect Blocks**: Link blocks together to create sequences that execute in order
3. **Configure Actions**: Set motor steps, speeds, and relay states for each block
4. **Execute**: Click the execute button to run your workflows in real-time

The system communicates with your hardware through ROS 2 topics, making it easy to integrate with existing robotics infrastructure.

## ROS 2 Topics

- `/motor1/command` - Motor 1 step commands
- `/motor2/command` - Motor 2 step commands  
- `/relay/command` - Relay commands (JSON format)
- `/motor1/status`, `/motor2/status` - Motor status updates
- `/relay1/status` through `/relay4/status` - Relay state updates

## License

MIT License - Open source and free to use
