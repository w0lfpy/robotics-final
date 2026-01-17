# 3CEB Robotics

[![Webots](https://img.shields.io/badge/Webots-R2025a-blue.svg)](https://cyberbotics.com/)
[![Python](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

> A sophisticated multi-agent robotics simulation implementing autonomous tag game behavior using state machine architecture, odometry-based navigation, and wireless inter-agent communication in the Webots environment.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Authors](#authors)
- [Game Mechanics](#game-mechanics)
- [System Architecture](#system-architecture)
- [Robot Specifications](#robot-specifications)
- [Technical Implementation](#technical-implementation)
- [Installation & Setup](#installation--setup)
- [Configuration](#configuration)
- [Known Limitations](#known-limitations)

## Overview

This project demonstrates advanced autonomous robotics concepts through a competitive tag game scenario. The simulation features one hunter robot pursuing three runner robots within a constrained arena environment. The implementation showcases real-time decision-making, sensor fusion, inter-agent communication, and reactive behaviors essential for multi-agent robotic systems.

## Features

- **State Machine Controllers**: Robust finite state machines for both hunter and runner agents
- **Odometry Integration**: Wheel encoder-based position tracking with GPS fallback
- **Wireless Communication**: Real-time position broadcasting between agents
- **Obstacle Avoidance**: IR sensor-based reactive collision avoidance
- **Recovery Mechanisms**: Automated stuck detection and escape behaviors
- **Dynamic Gameplay**: Adaptive time bonuses and real-time game state management

## Authors

| Name | Role |
|------|------|
| Adrián Pérez | Developer |
| Jose Suárez | Developer |
| Sergio Romano | Developer |
| Nicolás Pertierra | Developer |

## Game Mechanics

### Objective

The hunter robot (red) must capture all three runner robots (blue) before time expires. Runners must evade capture until the timer reaches zero.

### Rules

| Parameter | Value | Description |
|-----------|-------|-------------|
| Base Duration | 120 seconds | Initial game time |
| Time Bonus | +30 seconds | Added per successful capture |
| Capture Range | 0.12 meters | Distance threshold for capture |
| Arena Size | 1.6m × 1.6m | Rectangular playing field |
| Victory Condition (Hunter) | Capture all 3 runners | Before timeout |
| Victory Condition (Runners) | Survive | Until timer expires |

## System Architecture

### Directory Structure

```
robotics-final/
├── controllers/
│   ├── chaser_controller/
│   │   └── chaser_controller.py       # Hunter robot behavioral controller
│   ├── game_supervisor/
│   │   └── game_supervisor.py         # Simulation supervisor and referee
│   └── target_controller/
│       └── target_controller.py       # Runner robot behavioral controller
├── protos/
│   └── TagBot.proto                   # Custom robot PROTO definition
├── textures/
│   └── ...                            # Environment textures and assets
├── worlds/
│   ├── tarea_final.wbt                # Main simulation environment
│   └── .tarea_final.wbproj            # Webots project configuration
└── README.md
```

### Component Onfiguration

Each robot is equipped with the following components:

| Component | Type | Quantity | Specification |
|-----------|------|----------|---------------|
| Motors | Differential Drive | 2 | Independent left/right actuation |
| GPS | Global Positioning | 1 | Position estimation fallback |
| Compass | Orientation Sensor | 1 | Heading measurement |
| Infrared Sensors | Distance Measurement | 2 | Front-facing obstacle detection |
| Wheel Encoders | Position Sensors | 2 | Odometry calculation |
| Wireless Transceiver | Communication | 1 | Inter-agent messaging |

### Physical Parameters

```
Wheel Radius:     0.025 m
Wheelbase:        0.09 m
Body Radius:      0.035 m
Maximum Speed:    Variable by agent type
```

- **Differential Drive** : Two independent motors (motor_1, motor_2)
- **Sensors**:
  - GPS for global positioning
  - Compass for orientation
  - 2x Front infrared distance sensors
  - 2x Position sensors (encoders) for odometry
- **Communication**: [`chaser_controller.py`](controllers/chaser_controller/chaser_controller.py)

**Performance Profile**:

| Parameter | Value |
|-----------|-------|
| Maximum Speed | 9.0 rad/s |
| Base Speed | 7.0 rad/s |
| Proportional Gain | 3.5 |

**Behavioral States**:

- **SEARCH**: Executes random exploration patterns when no targets are detected
- **CHASE**: Pursues nearest runner using proportional heading control
- **AVOID**: Performs committed obstacle avoidance maneuvers
- **RECOVERY**: Activates stuck detection and escape sequences

**Key Capabilities**:
- Differential odometry with GPS fallback for position tracking
- Timeout-based [`target_controller.py`](controllers/target_controller/target_controller.py)

**Performance Profile**:

| Parameter | Value |
|-----------|-------|
| Maximum Speed | 6.0 rad/s |
| Base Speed | 5.0 rad/s |
| Speed Disadvantage | 33% slower than hunter |

**Behavioral States**:

- **WANDER**: Executes exploratory random movement patterns
- **FLEE**: Computes escape vectors opposite to hunter position
- **AVOID**: Performs reactive obstacle avoidance
- **RECOVERY**: Activates stuck escape sequences

**# Supervisor System

**Controller**: [`game_supervisor.py`](controllers/game_supervisor/game_supervisor.py)

The simulation supervisor manages all game logic and state:

**Responsibilities**:

| Function | Description |
|----------|-------------|
| Position Broadcasting | Real-time agent position transmission |
| Capture Detection | Distance-based collision detection |
| Time Management | Game timer and bonus time allocation |
| Victory Evaluation | Win/loss condition assessment |
| HUD Display | On-screen time remaining and capture count |
| Agent Lifecycle | Robot removal upon capture |

**Communication Architecture**:

| Channel | Direction | Payload |
|---------|-----------|---------|
| Channel 1 | Supervisor → Runners | Hunter position |
| Channel 2 | Supervisor → Hunter | Runner positions |

**Features**:
- Receives hunter position via wireless communication
- Stops movement when captured
- Lower speed than hunter (asymmetric gameplaydead reckoning with the following kinematic model:

```python
# Compute linear and angular displacement
d_center = (dist_left + dist_right) / 2
d_theta = (dist_right - dist_left) / wheelbase

# Update pose estimate
x += d_center * cos(heading)
y += d_center * sin(heading)
heading += d_theta
```

**Fallback Strategy**: GPS/compass measurements are used when wheel encoders are unavailable.

### State Machine Architecture

The behavioral controllers implement finite state machines with the following characteristics:

- **Timer-based Commitment**: State transitions include hysteresis to prevent oscillation
- **Priority Hierarchy**: Higher-priority states preempt lower-priority ones
- **Calibrated Thresholds**: Sensor thresholds are tuned for arena geometry

**State Transition Logic**:
```
RECOVERY (highest priority)
    ↓
AVOID
    ↓
CHASE/FLEE
    ↓
SEARCH/WANDER (lowest priority)
```Installation & Setup

### Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Webots | R2025a or later | [Download](https://cyberbotics.com/) |
| Python | 3.7+ | Included with Webots |
| Webots Python API | Bundled | Automatically available |

### Installation Steps

1. **Install Webots**:
   ```bash
   # Download from https://cyberbotics.com/
   # Follow platform-specific installation instructions
   ```

2. **Clone Repository**:
  # Arena Layout

The simulation environment features:

| Element | Specification | Quantity |
|---------|---------------|----------|
| Arena Dimensions | 1.6m × 1.6m | 1 |
| Perimeter Walls | Height: 0.15m | 4 |
| Cylindrical Pillars | Radius: 0.04m, Height: 0.15m | 4 |
| Internal Wall Segments | Various lengths | 5 |
| Box Obstacles | 0.12-0.15m dimensions | 2 |
| Floor Texture | Tiled grass pattern | 1 |

**Initial Positions**: Robot spawn points are strategically distributed to ensure balanced gameplay.

## Configuration

### Tunable Parameters

All key parameters can be adjusted in their respective controller files:

#### Game Supervisor ([`game_supervisor.py`](controllers/game_supervisor/game_supervisor.py))

```python
GAME_DURATION = 120        # Base game time (seconds)
TIME_BONUS = 30            # Bonus per capture (seconds)
CAPTURE_DISTANCE = 0.12    # Capture threshold (meters)
```

#### Hunter Controller ([`chaser_controller.py`](controllers/chaser_controller/chaser_controller.py))

```python
MAX_SPEED = 9.0           # Maximum angular velocity (rad/s)
BASE_SPEED = 7.0          # Cruising angular velocity (rad/s)
PROPORTIONAL_GAIN = 3.5   # Heading control gain
```

#### Runner Controller ([`target_controller.py`](controllers/target_controller/target_controller.py))

```python
MAX_SPEED = 6.0           # Maximum angular velocity (rad/s)
BASE_SPEED = 5.0          # Cruising angular velocity (rad/s)
```

### Performance Tuning

To modify gameplay balance:
- **Increase difficulty**: Reduce runner speed or increase hunter speed
- **Decrease difficulty**: Increase time bonus or base duration
- **Adjust responsiveness**: Modify proportional gains in controllers

## Known Limitations

The following behaviors are expected within the current implementation:

| Behavior | Cause | Mitigation |
|----------|-------|------------|
| Corner entrapment | Local minima in potential field | Recovery state with timeout |
| Oscillatory avoidance | Sensor noise near obstacles | State commitment timers |
| Suboptimal paths | Greedy target selection | Acceptable for reactive behavior |
| No path planning | Computational complexity | Direct vector navigation used |

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/improvement`)
5. Create a Pull Request

### Development Guidelines

- Follow PEP 8 style guidelines for Python code
- Add comments for complex logic
- Test changes in simulation before submitting
- Update README for significant feature additions

## Acknowledgments

- Webots development team for the simulation platform
- Course instructors and teaching assistants
- All team members for their contributions

---

### State Machine Architecture

Controllers use discrete state machines with:
- Timer-based state commitment to prevent oscillation
- Priority-based state transitions
- Obstacle detection thresholds calibrated for arena

### Control Law

Proportional control for heading:

```
error = target_angle - current_angle
turn_effort = gain * error
left_speed = base_speed - turn_effort
right_speed = base_speed + turn_effort
```

Speed normalization prevents exceeding motor limits.

## Requirements

- Webots R2025a or compatible version
- Python 3.x
- Webots Python API

## Running the Simulation

1. Open Webots
2. Load world file: `worlds/tarea_final.wbt`
3. Click play to start simulation
4. Monitor display for game status and remaining time

## Arena Layout

The simulation world includes:

- Rectangular arena with perimeter walls (height: 0.15m)
- 4 cylindrical pillars (radius: 0.04m, height: 0.15m)
- 5 internal wall segments for navigation complexity
- 2 box obstacles (0.12-0.15m dimensions)
- Textured grass floor with tile pattern

Initial positions are strategically distributed to ensure fair gameplay.

## Configuration Parameters

Key parameters can be adjusted in respective files:

**game_supervisor.py**:
- `GAME_DURATION`: Base game time (seconds)
- `TIME_BONUS`: Bonus per capture (seconds)
- `CAPTURE_DISTANCE`: Capture threshold (meters)

**chaser_controller.py**:
- `MAX_SPEED`: Hunter maximum velocity
- `BASE_SPEED`: Hunter cruising velocity

**target_controller.py**:
- `MAX_SPEED`: Runner maximum velocity
- `BASE_SPEED`: Runner cruising velocity