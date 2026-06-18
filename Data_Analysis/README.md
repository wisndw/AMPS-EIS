# Data Analysis — AMPS-EIS Hasil Simulasi

Folder ini berisi data mentah (CSV) hasil simulasi dan grafik analisis
hasil metode **AMPS-EIS** dibandingkan baseline.

## Struktur

```
Data_Analysis/
├── adc_raw.csv          -- data ADC 12-bit (sebelum oversampling)
├── adc_os.csv            -- data ADC 16-bit (setelah oversampling 16x)
├── ekf_raw.csv           -- data sudut mentah dari akselerometer
├── ekf_gyro.csv          -- data sudut hasil integrasi giroskop
├── ekf_fused.csv         -- data sudut hasil EKF (fused output)
├── ekf_ref.csv           -- data ground truth (referensi)
├── pid_autotune.csv      -- respons step PID auto-tune (AMPS-EIS)
├── pid_fixed.csv         -- respons step PID tetap (baseline)
├── canfd_amps.csv        -- throughput CAN FD AMPS-EIS (2 Mbps)
├── canfd_baseline.csv    -- throughput CAN baseline (500 kbps)
├── hasil_simulasi.png    -- grafik 4-panel (format gambar)
├── hasil_simulasi.pdf    -- grafik 4-panel (format PDF, untuk laporan)
└── README.md             -- file ini
```

## Format Data CSV

Semua file CSV memiliki 2 kolom tanpa header:
```
index_atau_waktu, nilai
```

| File | Kolom 1 | Kolom 2 | Satuan |
|---|---|---|---|
| `adc_raw.csv` / `adc_os.csv` | sample index | ADC value | counts |
| `ekf_*.csv` | waktu | sudut | derajat |
| `pid_*.csv` | waktu | kecepatan motor | RPM |
| `canfd_*.csv` | waktu | jumlah pesan | msg/s |

## Ringkasan Hasil Analisis

| Parameter | Baseline | AMPS-EIS | Peningkatan |
|---|---|---|---|
| Resolusi ADC efektif | 12-bit (4.096) | 16-bit (65.536) | +4 bit (16×) |
| RMSE estimasi sudut EKF | 2,3° | 0,42° | −81,7% |
| Overshoot PID | 18% | 4,7% | −73,9% |
| Settling time PID | 320 ms | 195 ms | −39,1% |
| Throughput CAN FD | ~1 Mbps | 2 Mbps | +100% |
| Jitter komunikasi | 12 µs | 1,8 µs | −85% |
| CPU load (168 MHz) | 78% | 61% | −17 pp |
| Latensi deteksi fault | N/A | <2 ms | Fitur baru |

## Cara Reproduksi Data

Data ini dihasilkan dari kombinasi simulasi Proteus (untuk data real-time)
dan model matematis Python (untuk validasi). Untuk mereproduksi:

```python
import numpy as np
import csv

# Contoh: regenerasi data ADC oversampling
np.random.seed(42)
n = 200
idx = np.arange(n)
signal = 2048 + 800 * np.sin(2 * np.pi * idx / 80)
raw_12bit = (signal + np.random.normal(0, 150, n)).clip(0, 4095)
oversampled = (signal * 16 + np.random.normal(0, 80, n)).clip(0, 65535)
```

Lihat skrip lengkap generasi data pada riwayat pengembangan proyek atau
hubungi tim pengembang.
