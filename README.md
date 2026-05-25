# AMPS-EIS: Adaptive Multi-Peripheral Orchestration with Safety Guarantees for Embedded Instrumentation Systems

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-STM32F407VG-orange.svg)]()
[![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B%20%7C%20Rust-lightgrey.svg)]()
[![Simulator](https://img.shields.io/badge/Simulator-Proteus%208.17-green.svg)]()

> Mid-Term Exam (ETS) — Controller Programming Course  
> Instrumentation Engineering Technology, Semester Genap 2025/2026

---

## Background

This project proposes **AMPS-EIS**, a novel firmware orchestration method for sensor-actuator embedded systems on the **STM32F407VGT6** microcontroller. The method integrates 15 *future work* directions identified from recent (2021–2026) Scopus/WoS journal articles across six topic areas:

- Microcontroller technology & architecture
- Peripheral subsystems (ADC, timer, serial communication)
- Embedded programming in C/C++ and Rust
- Safety-critical embedded systems

No prior work has integrated all these components into a single, unified firmware stack on this platform.

---

## Novel Method: AMPS-EIS

**Core idea:** A real-time orchestration layer that simultaneously manages:

| Component | Source Future Work | Key Innovation |
|---|---|---|
| 16× ADC oversampling via DMA burst | FW4 (Jurnal 4) | Zero-CPU 16-bit effective resolution |
| Multi-bus zero-copy driver (SPI/I2C/UART) | FW14 (Jurnal 14) | Hot-swap sensor support |
| Adaptive task scheduler | FW3 (Jurnal 3) | Runtime load-based priority tuning |
| Adaptive EKF sensor fusion | FW13 (Jurnal 13) | Auto-tuning noise covariance via ADC feedback |
| Dual-layer watchdog + anomaly detection | FW11 (Jurnal 11) | <2 ms fault detection latency |
| PID auto-tune controller | FW15 (Jurnal 15) | Relay-feedback without external tuning tool |
| Adaptive PWM dead-time compensation | FW6 (Jurnal 6) | No external temperature sensor needed |
| CAN FD with dynamic priority (TSN) | FW5 (Jurnal 5) | <2 µs jitter at 2 Mbps |
| Rust safe DMA abstraction HAL | FW7, FW9 (Jurnal 7, 9) | Zero unsafe blocks for DMA transfers |
| Runtime SIL-2 check (IEC 61508) | FW10 (Jurnal 10) | Compile-time safety validation via const fn |

---

## Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        STM32F407VGT6                            │
│                                                                 │
│  [IMU SPI] ──┐                                                  │
│  [Temp I2C]──┤─→ Multi-Bus Driver (FW14) ──→ Adaptive          │
│  [ADC x4] ───┴─→ Oversampling DMA (FW4)  ──→ Scheduler (FW3)  │
│                                                  │              │
│                                          ┌───────┴───────┐      │
│                                          ↓               ↓      │
│                                    EKF Fusion      Watchdog     │
│                                      (FW13)        (FW11)       │
│                                          │               │      │
│                                          └───────┬───────┘      │
│                                                  ↓              │
│                                          PID Auto-Tune (FW15)   │
│                                                  │              │
│                                    ┌─────────────┴──────────┐   │
│                                    ↓                        ↓   │
│                             PWM+DeadTime (FW6)      CAN FD (FW5) │
│                                    │                        │   │
│                             [ACTUATOR]                [MASTER/HMI]│
│                                                                 │
│  └─────────────── Rust HAL Safety Layer (FW7,9,10) ───────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
AMPS-EIS-STM32/
├── README.md
├── docs/
│   ├── block_diagram.png        # System block diagram
│   ├── flowchart.png            # Algorithm flowchart
│   └── simulation_results.pdf  # GNUPlot output
├── firmware/
│   ├── keil_4B/                 # Class 4B: Embedded C project
│   │   ├── Core/Src/main.c
│   │   ├── Core/Src/adc_dma.c
│   │   ├── Core/Src/ekf.c
│   │   ├── Core/Src/pid.c
│   │   ├── Core/Src/watchdog.c
│   │   ├── Core/Src/canfd.c
│   │   └── AMPS_EIS.uvprojx
│   └── rust_4C/                 # Class 4C: Rust project
│       ├── src/
│       │   ├── main.rs
│       │   ├── adc_dma.rs
│       │   ├── ekf.rs
│       │   ├── pid.rs
│       │   ├── watchdog.rs
│       │   └── canfd.rs
│       ├── Cargo.toml
│       └── memory.x
├── proteus/
│   └── AMPS_EIS.pdsprj          # Proteus simulation project
├── cubemx/
│   └── AMPS_EIS.ioc             # STM32CubeMX config file
├── gnuplot/
│   ├── plot_results.gp          # GNUPlot script
│   ├── adc_raw.csv
│   ├── adc_os.csv
│   ├── ekf_raw.csv
│   ├── ekf_fused.csv
│   ├── pid_resp.csv
│   └── canfd.csv
└── LICENSE
```

---

## Running the Simulation

### Prerequisites

**Class 4B (Embedded C + Keil):**
- Proteus Professional 8.17+
- STM32CubeMX 6.11+
- Keil MDK-ARM µVision 5.38+
- ARM Compiler 6 (AC6)

**Class 4C (Rust):**
- Proteus Professional 8.17+
- Rust toolchain: `rustup target add thumbv7em-none-eabihf`
- probe-rs: `cargo install probe-rs-tools`
- Required crates (see `Cargo.toml`): `stm32f4xx-hal`, `cortex-m-rt`, `embedded-hal`

### Step 1 — Open Proteus Simulation

1. Open `proteus/AMPS_EIS.pdsprj` in Proteus 8.17
2. Verify component list: STM32F407, MPU-6050, LM35, MCP2551, L298N motor driver

### Step 2 — Build Firmware (Class 4B: Keil)

```bash
# Open project in Keil µVision 5
# File → Open → firmware/keil_4B/AMPS_EIS.uvprojx
# Project → Build Target (F7)
# Output: firmware/keil_4B/Objects/AMPS_EIS.hex
```

Load the `.hex` file into the STM32F407 component in Proteus:
- Right-click STM32F407 → Edit Properties → Program File → select `.hex`

### Step 3 — Build Firmware (Class 4C: Rust)

```bash
cd firmware/rust_4C
cargo build --release --target thumbv7em-none-eabihf

# Convert to hex for Proteus
arm-none-eabi-objcopy -O ihex \
  target/thumbv7em-none-eabihf/release/amps_eis \
  AMPS_EIS.hex
```

Load the `.hex` into Proteus as described above.

### Step 4 — Run Simulation & Export Data

1. Click **Play** (▶) in Proteus
2. Open the Virtual Serial Monitor to see telemetry output
3. Copy the CSV output to `gnuplot/` directory:
   - `adc_raw.csv`, `adc_os.csv`, `ekf_fused.csv`, `pid_resp.csv`, `canfd.csv`

### Step 5 — Plot Results with GNUPlot

```bash
cd gnuplot
gnuplot plot_results.gp
# Output: hasil_simulasi.pdf (4-panel plot)
```

---

## Simulation Results

| Parameter | Baseline | AMPS-EIS | Improvement |
|---|---|---|---|
| ADC effective resolution | 12-bit | 16-bit | +4 bits |
| Angle RMSE (EKF) | 2.3° | 0.42° | −81.7% |
| PID overshoot | 18% | 4.7% | −73.9% |
| PID settling time | 320 ms | 195 ms | −39.1% |
| CAN FD throughput | 1 Mbps | 2 Mbps | +100% |
| Communication jitter | 12 µs | 1.8 µs | −85% |
| CPU load (168 MHz) | 78% | 61% | −17 pp |
| Fault detection latency | N/A | <2 ms | New feature |

---

## Key Advantages

1. **First integrated implementation** of all 15 future-work components on STM32F407 in a single firmware stack
2. **Memory safety** via Rust ownership model — eliminates buffer overflow and use-after-free bugs by construction
3. **Partial IEC 61508 SIL-2 compliance** via dual-layer watchdog + runtime safety checks
4. **Zero-CPU DMA pipeline** frees processor for EKF and PID computation
5. **Portability** — Rust HAL abstraction allows porting to other Cortex-M targets with minimal changes

---

## Authors

- Wisnu Dwipayana — NRP [2042241002] — Kelas 4B
- Rafif Agung Makarim Putra  — NRP [2042241066] — Kelas 4B

**Supervisor:** Ahmad Radhy S.si M.si  
Program Studi Rekayasa Teknologi Instrumentasi  
Semester Genap 2025/2026

---

## License

MIT License — see [LICENSE](LICENSE) for details.
