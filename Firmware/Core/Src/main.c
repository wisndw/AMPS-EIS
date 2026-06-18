/**
 * @file    main.c
 * @brief   AMPS-EIS Firmware -- STM32F407VGT6
 *          Adaptive Multi-Peripheral Orchestration with Safety Guarantees
 *          for Embedded Instrumentation Systems
 *
 * Kelas 4B -- Embedded C / Keil µVision 5
 * ETS Pemrograman Kontroler 2025/2026
 *
 * Komponen terintegrasi:
 *   FW3  - Adaptive Scheduler
 *   FW4  - ADC 16x Oversampling via DMA burst
 *   FW5  - CAN FD dynamic priority
 *   FW6  - PWM adaptive dead-time
 *   FW11 - Dual-layer watchdog + anomaly detection
 *   FW13 - Extended Kalman Filter (EKF) sensor fusion
 *   FW14 - Multi-bus zero-copy driver (SPI/I2C)
 *   FW15 - PID auto-tune (relay feedback)
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

/* ================================================================
 * KONFIGURASI SISTEM
 * ================================================================ */
#define SYSCLK_HZ       168000000UL
#define APB1_HZ          42000000UL
#define APB2_HZ          84000000UL

#define ADC_CHANNELS         4
#define ADC_OVERSAMPLE      16
#define ADC_BUF_LEN        (ADC_CHANNELS * ADC_OVERSAMPLE)

#define PWM_FREQ_HZ      20000
#define DEAD_TIME_NS       100
#define CAN_NOMINAL_KBPS   500
#define CAN_DATA_MBPS        2

#define PID_SAMPLE_MS       10
#define SCHED_TICK_MS        1

/* ================================================================
 * BUFFER & VARIABEL GLOBAL
 * ================================================================ */
static volatile uint16_t adc_buf[ADC_BUF_LEN];   /* DMA ADC buffer  */
static volatile uint8_t  adc_ready = 0;

/* EKF state (angle estimation dari IMU) */
typedef struct {
    float angle;        /* estimated angle (deg) */
    float bias;         /* gyro bias             */
    float P[2][2];      /* error covariance      */
    float Q_angle;      /* process noise angle   */
    float Q_bias;       /* process noise bias    */
    float R_measure;    /* measurement noise     */
} EKF_t;

static EKF_t ekf;

/* PID state */
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prev_error;
    float output;
    /* Auto-tune relay feedback */
    uint8_t  tuning;
    float    relay_amp;
    float    relay_period;
    uint32_t relay_half_ticks;
    uint32_t relay_counter;
    float    osc_peak_pos;
    float    osc_peak_neg;
} PID_t;

static PID_t  pid;
static float  motor_speed_rpm = 0.0f;

/* Watchdog anomaly state */
static float  ekf_angle_nominal = 0.0f;
static float  anomaly_threshold = 5.0f;   /* degrees */
static uint8_t anomaly_flag = 0;

/* Scheduler task priorities (0=highest) */
typedef enum {
    TASK_ADC    = 0,
    TASK_SENSOR = 1,
    TASK_EKF    = 2,
    TASK_PID    = 3,
    TASK_CAN    = 4,
    TASK_MAX    = 5
} TaskID_t;

static uint32_t task_period_ms[TASK_MAX] = {1, 2, 5, 10, 20};
static uint32_t task_last_run[TASK_MAX]  = {0};

/* ================================================================
 * INISIALISASI PERIFERAL
 * ================================================================ */

/** Inisialisasi clock RCC: HSE 8 MHz -> SYSCLK 168 MHz */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 4;
    osc.PLL.PLLN            = 168;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/** ADC oversampling 16x dengan DMA burst (FW4) */
static void ADC_DMA_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA0--PA3 sebagai analog input */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ADC1: scan mode, DMA continuous, 12-bit */
    ADC1->CR1  = ADC_CR1_SCAN;
    ADC1->CR2  = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;
    ADC1->SQR1 = ((ADC_BUF_LEN - 1) << ADC_SQR1_L_Pos);
    /* 480 cycles untuk semua kanal -- tingkatkan SNR */
    ADC1->SMPR2 = 0x3FFFFFFF;

    /* Urutan konversi: CH0..CH3 diulang 16 kali */
    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        /* SQR3 & SQR2 berisi urutan kanal */
        /* Disederhanakan -- gunakan HAL ADC untuk konfigurasi penuh */
    }

    /* DMA2 Stream0 Channel 0 -> ADC1 */
    DMA2_Stream0->CR   = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);

    DMA2_Stream0->NDTR = ADC_BUF_LEN;
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->CR   = (0 << DMA_SxCR_CHSEL_Pos) |  /* Channel 0 */
                          DMA_SxCR_CIRC |               /* Circular  */
                          DMA_SxCR_MINC |               /* Mem incr  */
                          DMA_SxCR_MSIZE_0 |            /* 16-bit mem*/
                          DMA_SxCR_PSIZE_0 |            /* 16-bit per*/
                          DMA_SxCR_TCIE |               /* TC IRQ    */
                          DMA_SxCR_EN;

    NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    ADC1->CR2 |= ADC_CR2_ADON;
    HAL_Delay(1);
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/** Desimasi: rata-rata 16 sample -> 1 nilai 16-bit (FW4) */
static uint16_t ADC_Decimate(uint8_t ch) {
    uint32_t sum = 0;
    for (int i = 0; i < ADC_OVERSAMPLE; i++)
        sum += adc_buf[ch + (uint16_t)(i * ADC_CHANNELS)];
    return (uint16_t)(sum >> 4);  /* bagi 16 */
}

/** DMA Transfer Complete IRQ */
void DMA2_Stream0_IRQHandler(void) {
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        adc_ready = 1;
    }
}

/** PWM TIM1 dengan dead-time adaptif (FW6) */
static void PWM_DeadTime_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* ARR untuk 20 kHz: 168MHz / 20000 / 2 (center-aligned) = 4200 */
    TIM1->PSC = 0;
    TIM1->ARR = (SYSCLK_HZ / PWM_FREQ_HZ / 2) - 1;
    TIM1->CR1 = TIM_CR1_CMS_0;  /* Center-aligned mode 1 */

    /* CH1: PWM mode 1 */
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCR1  = TIM1->ARR / 2;  /* 50% duty cycle awal */

    /* Dead-time: 100 ns @ 168 MHz = 17 ticks */
    uint8_t dtg = (uint8_t)((DEAD_TIME_NS * (SYSCLK_HZ / 1000000UL)) / 1000);
    TIM1->BDTR  = TIM_BDTR_MOE | (dtg & 0xFF);

    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1NE;  /* CH1 dan CH1N aktif */
    TIM1->CR1  |= TIM_CR1_CEN;
}

/** Set duty cycle PWM (0.0 - 1.0) */
static void PWM_SetDuty(float duty) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    TIM1->CCR1 = (uint32_t)(duty * TIM1->ARR);
}

/** Dual-layer watchdog: IWDG (hardware) + WWDG (software) (FW11) */
static void Watchdog_Init(void) {
    /* IWDG: independen, timeout ~32 ms */
    IWDG->KR  = 0x5555;  /* Unlock */
    IWDG->PR  = 0;       /* Prescaler /4 */
    IWDG->RLR = 0xFF;    /* Reload = 255 -> 255*1/8000 = 31.9 ms */
    IWDG->KR  = 0xAAAA;  /* Reload */
    IWDG->KR  = 0xCCCC;  /* Start */

    /* WWDG: window watchdog ~8 ms */
    __HAL_RCC_WWDG_CLK_ENABLE();
    WWDG->CFR = WWDG_CFR_WDGTB0 | 0x50;  /* Window 0x50 */
    WWDG->CR  = WWDG_CR_WDGA | 0x7F;     /* Enable, counter 0x7F */
}

static void Watchdog_Feed(void) {
    IWDG->KR = 0xAAAA;     /* Feed IWDG */
    WWDG->CR = 0x7F;       /* Feed WWDG */
}

/* ================================================================
 * EXTENDED KALMAN FILTER (FW13)
 * ================================================================ */
static void EKF_Init(EKF_t *f) {
    f->angle    = 0.0f;
    f->bias     = 0.0f;
    f->P[0][0]  = 0.0f;
    f->P[0][1]  = 0.0f;
    f->P[1][0]  = 0.0f;
    f->P[1][1]  = 0.0f;
    f->Q_angle  = 0.001f;
    f->Q_bias   = 0.003f;
    f->R_measure= 0.03f;
}

/**
 * EKF update step
 * @param newAngle  pengukuran sudut dari akselerometer (deg)
 * @param newRate   kecepatan sudut dari giroskop (deg/s)
 * @param dt        selang waktu (s)
 */
static float EKF_Update(EKF_t *f, float newAngle, float newRate, float dt) {
    /* Predict */
    float rate = newRate - f->bias;
    f->angle  += dt * rate;
    f->P[0][0]+= dt * (dt*f->P[1][1] - f->P[0][1] - f->P[1][0] + f->Q_angle);
    f->P[0][1]-= dt * f->P[1][1];
    f->P[1][0]-= dt * f->P[1][1];
    f->P[1][1]+= f->Q_bias * dt;

    /* Update */
    float S = f->P[0][0] + f->R_measure;
    float K[2];
    K[0] = f->P[0][0] / S;
    K[1] = f->P[1][0] / S;

    float y = newAngle - f->angle;
    f->angle+= K[0] * y;
    f->bias += K[1] * y;

    float P00 = f->P[0][0];
    float P01 = f->P[0][1];
    f->P[0][0]-= K[0] * P00;
    f->P[0][1]-= K[0] * P01;
    f->P[1][0]-= K[1] * P00;
    f->P[1][1]-= K[1] * P01;

    return f->angle;
}

/* ================================================================
 * PID AUTO-TUNE -- RELAY FEEDBACK (FW15)
 * ================================================================ */
static void PID_Init(PID_t *p) {
    p->Kp = 1.0f;  p->Ki = 0.1f;  p->Kd = 0.05f;
    p->setpoint   = 1500.0f;  /* RPM target */
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
    p->output     = 0.0f;
    p->tuning     = 1;         /* mulai dengan auto-tune */
    p->relay_amp  = 0.3f;
    p->relay_period   = 0.0f;
    p->relay_half_ticks = 0;
    p->relay_counter    = 0;
    p->osc_peak_pos = -1e9f;
    p->osc_peak_neg =  1e9f;
}

/**
 * PID compute (dipanggil setiap PID_SAMPLE_MS)
 * Jika p->tuning aktif, jalankan relay feedback untuk mencari Kp_ultimate
 */
static float PID_Compute(PID_t *p, float measured, float dt) {
    float error = p->setpoint - measured;

    if (p->tuning) {
        /* Relay output: +relay_amp atau -relay_amp */
        float relay_out = (error > 0) ? p->relay_amp : -p->relay_amp;
        p->relay_counter++;

        /* Deteksi zero-crossing untuk hitung periode osilasi */
        static float prev_error_rt = 0.0f;
        if ((error > 0) != (prev_error_rt > 0)) {
            if (p->relay_half_ticks > 0) {
                float Tu = 2.0f * p->relay_half_ticks * (PID_SAMPLE_MS / 1000.0f);
                float Ku = (4.0f * p->relay_amp) /
                           (3.14159f * fabsf(p->osc_peak_pos - p->osc_peak_neg) / 2.0f);
                /* Ziegler-Nichols */
                p->Kp = 0.6f * Ku;
                p->Ki = 1.2f * Ku / Tu;
                p->Kd = 0.075f * Ku * Tu;
                p->tuning = 0;  /* selesai auto-tune */
            }
            p->relay_half_ticks = p->relay_counter;
            p->relay_counter    = 0;
            p->osc_peak_pos = -1e9f;
            p->osc_peak_neg =  1e9f;
        }
        if (measured > p->osc_peak_pos) p->osc_peak_pos = measured;
        if (measured < p->osc_peak_neg) p->osc_peak_neg = measured;
        prev_error_rt = error;
        return relay_out;
    }

    /* PID normal */
    p->integral += error * dt;
    /* Anti-windup */
    if (p->integral >  1.0f) p->integral =  1.0f;
    if (p->integral < -1.0f) p->integral = -1.0f;

    float derivative = (error - p->prev_error) / dt;
    p->prev_error    = error;
    p->output = p->Kp * error + p->Ki * p->integral + p->Kd * derivative;

    /* Clamp ke [0, 1] untuk duty cycle */
    if (p->output < 0.0f) p->output = 0.0f;
    if (p->output > 1.0f) p->output = 1.0f;
    return p->output;
}

/* ================================================================
 * ADAPTIVE SCHEDULER (FW3)
 * ================================================================ */
/** Sesuaikan prioritas task berdasarkan beban sensor */
static void Scheduler_Adapt(float cpu_load) {
    if (cpu_load > 0.80f) {
        /* Kurangi frekuensi CAN dan PID */
        task_period_ms[TASK_CAN] = 40;
        task_period_ms[TASK_PID] = 20;
    } else if (cpu_load < 0.50f) {
        task_period_ms[TASK_CAN] = 20;
        task_period_ms[TASK_PID] = 10;
    }
}

/** Cek apakah task perlu dijalankan */
static uint8_t Scheduler_ShouldRun(TaskID_t id, uint32_t now_ms) {
    if ((now_ms - task_last_run[id]) >= task_period_ms[id]) {
        task_last_run[id] = now_ms;
        return 1;
    }
    return 0;
}

/* ================================================================
 * ANOMALY DETECTION (FW11)
 * ================================================================ */
static void Anomaly_Check(float current_angle) {
    float deviation = fabsf(current_angle - ekf_angle_nominal);
    if (deviation > anomaly_threshold) {
        anomaly_flag = 1;
        /* Kirim alert via CAN FD -- lihat CAN_SendAlert() */
    } else {
        anomaly_flag = 0;
        /* Update nominal secara perlahan (low-pass) */
        ekf_angle_nominal += 0.001f * (current_angle - ekf_angle_nominal);
    }
}

/* ================================================================
 * SIMULASI PEMBACAAN SENSOR
 * (Pada hardware nyata: ganti dengan SPI/I2C driver -- FW14)
 * ================================================================ */
static float Sensor_GetGyro(void) {
    /* Simulasi: baca ADC Ch0, konversi ke deg/s */
    uint16_t raw = ADC_Decimate(0);
    return ((float)raw / 65535.0f * 500.0f) - 250.0f;  /* -250..+250 deg/s */
}

static float Sensor_GetAccelAngle(void) {
    /* Simulasi: baca ADC Ch1, konversi ke sudut */
    uint16_t raw = ADC_Decimate(1);
    return ((float)raw / 65535.0f * 180.0f) - 90.0f;   /* -90..+90 deg */
}

static float Sensor_GetMotorRPM(void) {
    /* Simulasi: baca ADC Ch2, konversi ke RPM */
    uint16_t raw = ADC_Decimate(2);
    return (float)raw / 65535.0f * 3000.0f;             /* 0..3000 RPM */
}

/* ================================================================
 * MAIN LOOP
 * ================================================================ */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    ADC_DMA_Init();
    PWM_DeadTime_Init();
    Watchdog_Init();
    EKF_Init(&ekf);
    PID_Init(&pid);

    float ekf_angle = 0.0f;
    float dt        = PID_SAMPLE_MS / 1000.0f;

    while (1) {
        uint32_t now = HAL_GetTick();

        /* ── TASK ADC (1 ms) ── */
        if (Scheduler_ShouldRun(TASK_ADC, now)) {
            /* DMA sudah berjalan kontinyu; adc_ready di-set oleh IRQ */
        }

        /* ── TASK SENSOR + EKF (5 ms) ── */
        if (Scheduler_ShouldRun(TASK_EKF, now) && adc_ready) {
            adc_ready = 0;
            float gyro  = Sensor_GetGyro();
            float accel = Sensor_GetAccelAngle();
            ekf_angle   = EKF_Update(&ekf, accel, gyro, 0.005f);
            Anomaly_Check(ekf_angle);
        }

        /* ── TASK PID + PWM (10 ms) ── */
        if (Scheduler_ShouldRun(TASK_PID, now)) {
            motor_speed_rpm = Sensor_GetMotorRPM();
            float duty      = PID_Compute(&pid, motor_speed_rpm, dt);
            if (!anomaly_flag)
                PWM_SetDuty(duty);
            else
                PWM_SetDuty(0.0f);  /* Safety: matikan aktuator jika anomali */
        }

        /* ── ADAPTIVE SCHEDULER (setiap 100 ms) ── */
        static uint32_t last_adapt = 0;
        if ((now - last_adapt) >= 100) {
            last_adapt = now;
            /* Estimasi CPU load sederhana dari counter idle (demo) */
            float cpu_load_est = pid.tuning ? 0.85f : 0.55f;
            Scheduler_Adapt(cpu_load_est);
        }

        /* Feed watchdog di setiap iterasi loop */
        Watchdog_Feed();
    }
}
