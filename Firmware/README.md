# Firmware — AMPS-EIS (Kelas 4B, Embedded C)

Folder ini berisi firmware Embedded C untuk mikrokontroler
**STM32F407VGT6**, dikompilasi menggunakan **Keil µVision 5.38**
dengan **ARM Compiler 6 (AC6)**.

## Struktur

```
Firmware/
├── Core/Src/
│   └── main.c          -- firmware utama AMPS-EIS
├── AMPS_EIS.uvprojx     -- file proyek Keil uVision (buat manual di Keil)
└── README.md            -- file ini
```

## Komponen yang Diimplementasikan

| Fungsi | Future Work | Keterangan |
|---|---|---|
| `ADC_DMA_Init()` / `ADC_Decimate()` | FW4 | ADC 16× oversampling via DMA2, zero-CPU |
| `PWM_Init()` / `PWM_UpdateDeadTime()` | FW6 | PWM TIM1 center-aligned 20 kHz, dead-time adaptif |
| `EKF_Init()` / `EKF_Update()` | FW13 | Extended Kalman Filter 2-state untuk estimasi sudut IMU |
| `PID_Init()` / `PID_Compute()` | FW15 | PID relay auto-tune (Ziegler-Nichols) |
| Watchdog (IWDG + WWDG) | FW11 | Dual-layer watchdog + anomaly detection |
| CAN1 driver | FW5 | CAN FD dengan prioritas pesan dinamis |
| Multi-bus driver (SPI1) | FW14 | Driver IMU zero-copy via DMA |
| Adaptive Scheduler | FW3 | Penjadwalan task berbasis beban sensor run-time |

## Cara Compile

1. Buka Keil µVision 5
2. File → New → Project, pilih target **STM32F407VG**
3. Tambahkan `Core/Src/main.c` ke proyek
4. Tambahkan library STM32F4xx HAL Driver (via Pack Installer)
5. Project → Build Target (F7)
6. Output `.hex` ada di `Objects/AMPS_EIS.hex`

## Konfigurasi Clock

- HSE 8 MHz → PLL → SYSCLK 168 MHz
- APB1 = 42 MHz, APB2 = 84 MHz

## Load ke Proteus

1. Buka proyek Proteus di folder `../Simulation/`
2. Klik kanan komponen STM32F407 → Edit Properties
3. Program File → pilih `.hex` hasil compile
4. Klik Play (▶) untuk menjalankan simulasi
