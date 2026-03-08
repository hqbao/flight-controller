# GPS-Denied Navigation Module

## Overview

Optical flow-based waypoint navigation for indoor/GPS-denied flight. Tracks waypoints defined in local coordinate frame (cm) using position estimates from the position estimation module.

## Data Flow

```
POSITION_STATE_UPDATE (current position)
    │
    ▼  SCHEDULER_10HZ: compute distance to next waypoint
    │
    └─► POSITION_TARGET_UPDATE (next waypoint in cm)
```

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `WAYPOINT_THRESHOLD` | 20.0 | Waypoint reached distance (cm) |
| `MAX_WAYPOINTS` | 10 | Maximum stored waypoints |

## Default Waypoints

1 m square pattern: `(0,0)` → `(100,0)` → `(100,100)` → `(0,100)` → `(0,0)`, then loops.

When no mission is active, holds current position.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `POSITION_STATE_UPDATE` | 500 Hz | Current position/velocity |
| `SCHEDULER_10HZ` | 10 Hz | Navigation update loop |

### Publications
| Topic | Data |
|-------|------|
| `POSITION_TARGET_UPDATE` | `vector3d_t` — target position (cm) |
