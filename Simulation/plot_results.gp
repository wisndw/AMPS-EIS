#!/usr/bin/env gnuplot
# ================================================================
# GNUPlot script: AMPS-EIS Simulation Results
# Jalankan: gnuplot plot_results.gp
# Output  : hasil_simulasi.pdf
# ================================================================

set terminal pdfcairo enhanced color font "Arial,11" size 22cm,16cm
set output "hasil_simulasi.pdf"

set multiplot layout 2,2 \
    title "AMPS-EIS Simulation Results — STM32F407VGT6" \
    font "Arial,13"

set style line 1 lt 1 lc rgb "#E53935" lw 2
set style line 2 lt 1 lc rgb "#1E88E5" lw 2
set style line 3 lt 1 lc rgb "#43A047" lw 2
set style line 4 lt 1 lc rgb "#8E24AA" lw 1.5
set style line 5 lt 2 lc rgb "#78909C" lw 1.5

# ── Plot 1: ADC Oversampling ────────────────────────────────
set title "ADC Resolution: Raw 12-bit vs Oversampled 16-bit" font "Arial,11"
set xlabel "Sample index"
set ylabel "ADC value (counts)"
set yrange [0:70000]
set key top right
set grid

plot "adc_raw.csv" u 1:2 w l ls 1 t "Raw 12-bit (4096 max)", \
     "adc_os.csv"  u 1:2 w l ls 2 t "Oversampled 16-bit (65535 max)"

# ── Plot 2: EKF Sensor Fusion ───────────────────────────────
set title "EKF Sensor Fusion — Angle Estimation" font "Arial,11"
set xlabel "Time (ms)"
set ylabel "Angle (degrees)"
set yrange [-30:30]

plot "ekf_raw.csv"   u 1:2 w l ls 1 t "Raw accelerometer", \
     "ekf_gyro.csv"  u 1:2 w l ls 4 t "Gyroscope (integrated)", \
     "ekf_fused.csv" u 1:2 w l ls 2 t "EKF fused output", \
     "ekf_ref.csv"   u 1:2 w l ls 3 t "Ground truth"

# ── Plot 3: PID Step Response ───────────────────────────────
set title "PID Auto-Tune — Motor Speed Step Response" font "Arial,11"
set xlabel "Time (ms)"
set ylabel "Motor speed (RPM)"
set yrange [0:2000]

set arrow 1 from 0,1500 to 500,1500 nohead ls 5 dt 2
set label 1 "Setpoint 1500 RPM" at 10,1540 font "Arial,9"

plot "pid_autotune.csv" u 1:2 w l ls 2 t "AMPS-EIS (auto-tuned PID)", \
     "pid_fixed.csv"    u 1:2 w l ls 5 t "Baseline (fixed Kp=1)"

unset arrow 1
unset label 1

# ── Plot 4: CAN FD Throughput ───────────────────────────────
set title "CAN FD Message Throughput Comparison" font "Arial,11"
set xlabel "Time (ms)"
set ylabel "Messages / second"
set yrange [0:8000]
set style fill solid 0.7 border -1
set boxwidth 15

plot "canfd_amps.csv"     u 1:2 w boxes lc rgb "#1E88E5" t "AMPS-EIS (2 Mbps)", \
     "canfd_baseline.csv" u 1:2 w boxes lc rgb "#E53935" t "Baseline (500 kbps)"

unset multiplot
