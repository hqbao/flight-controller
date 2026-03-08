# Fault Handler Module

## Overview

Safety module that responds to faults by immediately disarming all motors. Logs fault count to console and toggles LED.

## Behavior

On `FAULT_DETECTION` event:
1. Increment fault counter
2. Toggle LED
3. Force `STATE_DETECTION_UPDATE` → DISARMED (state = 0)
4. Print fault count to console

## PubSub Interface

### Subscriptions
| Topic | Purpose |
|-------|---------|
| `FAULT_DETECTION` | Fault event from any module |

### Publications
| Topic | Data |
|-------|------|
| `STATE_DETECTION_UPDATE` | `state_t` = DISARMED (0) |
