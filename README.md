# UWB-IPS — Ultra-Wideband Indoor Positioning System

> Embedded firmware for multi-tag indoor positioning research based on Ultra-Wideband (UWB) technology, using a **RING** (round-robin) channel access protocol.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Protocol](#protocol)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation & Build](#installation--build)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

**UWB-IPS** is an embedded firmware project for researching **Ultra-Wideband (UWB)** based Indoor Positioning Systems (IPS). The system implements **Two-Way Ranging (TWR)** to measure distances between a mobile tag and a set of fixed anchors, then applies **multilateration** to compute 2D/3D coordinates.

The project implements the **RING** MAC (Medium Access Control) protocol for the tag, providing a simple and collision-free round-robin channel access scheme.

---

## Features

- Centimeter-level distance measurement using UWB Two-Way Ranging (TWR / DS-TWR)
- Multi-tag simultaneous operation without signal collisions
- RING MAC protocol for collision-free multi-tag channel access
- Position estimation via multilateration / trilateration
- Lightweight pure-C firmware, easy to port to different embedded platforms
- Suitable for both academic research and real-world prototyping

---

## System Architecture

```
┌─────────────────────────────────────────────┐
│               Indoor Environment             │
│                                             │
│   [Anchor 0]       [Anchor 1]               │
│       │   \         /   │                   │
│       │    \       /    │   ← UWB ranging   │
│       │     \     /     │                   │
│   [Anchor 3]   [Tag]   [Anchor 2]           │
│                  │                          │
│             (mobile device)                 │
└─────────────────────────────────────────────┘
              ↓ Range measurements
     ┌────────────────────────┐
     │  Positioning Algorithm │  ← Multilateration
     │   (x, y) / (x, y, z)  │
     └────────────────────────┘
```

**Components:**
- **Anchor**: Fixed UWB device at a known position. Responds to ranging requests from tags.
- **Tag**: Mobile UWB device. Initiates ranging to anchors and computes its own position.
- **MAC Protocol**: Coordinates channel access order to prevent collisions (RING).

---

## Repository Structure

```
UWB-IPS/
├── RING_Tag/          # Tag firmware using RING (round-robin) protocol
│   ├── Core/          # Main C source code
│   ├── Drivers/       # UWB and peripheral drivers
│   └── ...
├── LICENSE
└── README.md
```

---

## Protocol

### RING Tag

The **RING** protocol grants each tag exclusive channel access in a sequential, round-robin order:

```
Superframe:
┌──────────┬──────────┬──────────┬──────────┐
│  Tag 0   │  Tag 1   │  Tag 2   │  Tag N   │  → repeats
│ (ranging)│ (ranging)│ (ranging)│ (ranging)│
└──────────┴──────────┴──────────┴──────────┘
```

- A tag receives the token → performs ranging with all anchors → passes the token to the next tag.
- Simple to implement; best suited for small tag counts.

---

## Hardware Requirements

| Component | Recommendation |
|-----------|---------------|
| UWB Module | Qorvo/Decawave DWM1001, DWM3001C, or equivalent |
| UWB Chip | DW1000 / DW3000 (IEEE 802.15.4 UWB) |
| Microcontroller | ARM Cortex-M (on-module or external) |
| Anchors | Minimum 3 for 2D positioning, 4+ for 3D |
| Debug Probe | JTAG/SWD (J-Link, ST-Link, or compatible) |

---

## Software Requirements

- **Toolchain**: `arm-none-eabi-gcc` (GCC for ARM Cortex-M)
- **Build System**: GNU Make or CMake (or an IDE: STM32CubeIDE / SEGGER Embedded Studio / VS Code + Cortex-Debug)
- **UWB SDK**: Qorvo/Decawave UWB Stack (DW1000 API / QORVO UWB SDK)
- **RTOS** *(optional)*: FreeRTOS or Mynewt

---

## Installation & Build

```bash
# 1. Clone the repository
git clone https://github.com/leonas-kratos/UWB-IPS.git
cd UWB-IPS

# 2. Enter the firmware directory
cd RING_Tag

# 3. Build
make all

# 4. Flash to device
make flash     # or use your IDE / OpenOCD
```

> **Note**: System parameters (anchor coordinates, UWB radio settings, tag order) can be configured in the header files under `Core/Inc/`.

---

## Usage

1. **Deploy anchors** — Place anchors at fixed, known positions and flash the anchor firmware.

2. **Configure the system** — Set UWB addresses (PAN ID, node ID) for each device and define the tag order in the RING sequence.

3. **Flash the tag firmware** — Flash the firmware from `RING_Tag/` onto the mobile tag device.

4. **Run the system** — Power on all devices. The tag automatically starts ranging with the anchors and outputs distance or position data via UART or another interface.

5. **Read results** — Connect a host PC via UART (default baud rate is typically `115200`) to receive distance measurements and estimated coordinates.

---

## How It Works

The system uses **SS-TWR / DS-TWR (Single/Double-Sided Two-Way Ranging)** to measure distances:

```
Tag                              Anchor
 │──── Poll          (T1) ──────►│
 │                           (T2)│
 │◄─── Response      (T3) ───────│
 │(T4)                           │

 ToF      = [(T4 − T1) − (T3 − T2)] / 2
 Distance = ToF × c     (c ≈ 3×10⁸ m/s)
```

Once distances to ≥ 3 anchors are known, a **Least Squares / Gauss-Newton** solver computes the tag's (x, y) coordinates from the multilateration equations:

```
(x − xᵢ)² + (y − yᵢ)² = dᵢ²    for each anchor i
```

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork this repository.
2. Create a new branch: `git checkout -b feature/your-feature-name`
3. Commit your changes: `git commit -m "feat: short description"`
4. Push and open a Pull Request.

For bug reports or feature requests, please open an [Issue](https://github.com/leonas-kratos/UWB-IPS/issues).

---

## License

This project is released under the **MIT License**. See [LICENSE](./LICENSE) for details.

---

<p align="center">
  Made with ❤️ for UWB indoor positioning research
</p>
