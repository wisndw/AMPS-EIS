# Simulation — AMPS-EIS Proteus & GNUPlot

Folder ini berisi file simulasi rangkaian Proteus dan skrip plotting
GNUPlot untuk metode **AMPS-EIS** pada STM32F407VGT6.

## Struktur

```
Simulation/
├── AMPS_EIS.pdsprj          -- proyek Proteus (buat manual di Proteus 8.17)
├── schematic_proteus.png    -- skematik rangkaian (referensi visual)
├── plot_results.gp          -- skrip GNUPlot 4-panel
└── README.md                -- file ini
```

## Komponen Rangkaian Proteus

| Komponen | Library Proteus | Fungsi |
|---|---|---|
| STM32F407VG | STM32F4xx | Mikrokontroler utama |
| MPU-6050 | Custom/GPIO | Sensor IMU (SPI1) |
| LM35 | Analogue | Sensor suhu (ADC1) |
| Potensiometer | POT-HG | Setpoint RPM (ADC1) |
| L298N | Motor Drive | H-bridge motor driver |
| Motor DC | Electromech | Aktuator (12V, 3000 RPM) |
| MCP2551 | CAN Bus | CAN transceiver |
| Oscilloscope | Virtual Instruments | Capture sinyal ADC/PWM |

## Net List Singkat

| Pin STM32F407 | Terhubung ke |
|---|---|
| PA0–PA3 | ADC1: encoder, LM35, potensiometer, Vref |
| PA4–PA7 | SPI1: MPU-6050 (NSS/SCK/MISO/MOSI) |
| PE9/PE8 | TIM1 PWM → L298N ENA/ENB |
| PD0/PD1 | CAN1 → MCP2551 RXD/TXD |

Lihat detail lengkap pin mapping pada `Report/main.tex` Tabel "Net List".

## Menjalankan Simulasi

1. Buka `AMPS_EIS.pdsprj` di Proteus Professional 8.17
2. Load file `.hex` hasil compile dari `../Firmware/`
3. Klik Play (▶)
4. Buka Virtual Oscilloscope untuk mengamati sinyal ADC/PWM/CAN
5. Export data ke CSV via klik kanan → Export Data

## Plotting dengan GNUPlot

```bash
gnuplot plot_results.gp
```

Skrip ini menghasilkan `hasil_simulasi.pdf` 4-panel:
1. ADC oversampling (raw 12-bit vs 16-bit)
2. EKF sensor fusion (estimasi sudut)
3. PID step response (auto-tune vs fixed)
4. CAN FD throughput comparison

Pastikan file CSV (`adc_raw.csv`, `ekf_fused.csv`, dll.) berada di folder
`../Data_Analysis/` atau disalin ke folder ini sebelum menjalankan skrip.
