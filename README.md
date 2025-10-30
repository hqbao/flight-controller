# Flight Controller

This repository contains the embedded firmware for a custom **Flight Control System (FCS)**, developed primarily in **C**. The project is designed to manage and stabilize a multi-rotor aircraft by integrating sensor data, running a state estimation algorithm, and commanding the motor electronic speed controllers (ESCs).

## Overview

The Flight Controller (FC) acts as the **brain of the drone**, processing input from inertial sensors to determine the aircraft's orientation (attitude) and applying control algorithms (like **PID**) to maintain stable flight or execute pilot commands.

## Key Features (Inferred)

Based on the structure and language of the project, the flight controller is likely engineered to support the following core functionalities:

* **Attitude Stabilization:** Utilizes sensor data to actively stabilize the quadcopter's roll, pitch, and yaw.
* **PID Control:** Implementation of **Proportional-Integral-Derivative (PID) controllers** for precise and responsive motor control.
* **Sensor Fusion:** Algorithms (e.g., complementary filter or Extended Kalman Filter) to merge data from multiple sensors for an accurate state estimate.
* **Modular Architecture:** Organized code structure (`Modules` and `Libs`) to allow for easy integration of new hardware components and flight modes.
* **Telemetry & Logging:** Support for debugging and post-flight analysis through a logging system (often called Blackbox).



## Technology Stack

| Category | Detail |
| :--- | :--- |
| **Language** | **C** (99.4%) |
| **Target Platform** | Embedded Microcontroller (Commonly **STM32** series) |
| **Sensors** | **IMU** (Gyroscope & Accelerometer) is essential. Magnetometer (Compass) and Barometer (Altimeter) are highly likely. |
| **Control** | Custom/Standardized ESC Protocols (e.g., PWM, DShot) |

## Project Structure

The repository is organized into distinct directories for a clear separation of concerns:

| Directory | Purpose |
| :--- | :--- |
| `base` | Contains core functionalities like memory management, hardware abstraction layers (HAL), and general utilities. |
| `Libs` | External libraries or custom low-level drivers for communicating with sensors (IMU, GPS) and peripherals. |
| `Modules` | Higher-level flight control modules, including flight modes, sensor fusion algorithms, and the PID controller logic. |
| `pytest` | Test scripts and configuration files for unit and integration testing of the firmware components. |

## Getting Started

### Prerequisites

* C/C++ toolchain for the target microcontroller (e.g., GCC for ARM Embedded).
* Microcontroller hardware with necessary programming and debugging tools (e.g., J-Link, ST-Link).
* A ground control station software for configuration and tuning (if implemented).

### Compilation

```bash
# Example command for cross-compilation (may vary based on build system)
# Replace <TargetMCU> with the actual chip, e.g., 'stm32f405'
make TARGET=<TargetMCU>
