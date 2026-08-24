/*
 * wave_generator_internal.h
 *
 * Внутрішній інтерфейс модуля генератора: типи, спільний стан і дрібні
 * хелпери, потрібні кільком файлам модуля. Прикладний код цей файл не
 * вмикає - для нього є wave_generator_task.h.
 *
 * Модуль розкладено так:
 *
 *   wave_generator_task.c      гарячий шлях: синтез, регулятор, ISR
 *   wave_generator_params.c    перерахунок похідних величин
 *   wave_generator_schedule.c  черга запланованих змін і профілі
 *   wave_generator_cal.c       автостарт і самокалібрування
 *   wave_generator_hw.c        налаштування периферії, увесь HAL
 *   wave_generator_api.c       гетери й сетери для рівня зв'язку
 */

#ifndef WAVE_GENERATOR_INTERNAL_H_
#define WAVE_GENERATOR_INTERNAL_H_

#include "wave_generator_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define WG_APERIODIC_Q_SHIFT      20
#define WG_DECAY_Q_SHIFT          30
#define WG_DECAY_Q_ONE            (1L << WG_DECAY_Q_SHIFT)
#define WG_DEGREES_PER_TURN_MILLI 360000L

/* Межі, за якими відлік ЗЗ вважається насиченим (обрив/перевантаження). */
#define WG_FB_SAT_LOW             8
#define WG_FB_SAT_HIGH            (WG_ADC_FULL_SCALE - 8)

/* Гарячі дані контуру живуть у CCM RAM: окрема шина, нуль wait states.
 * Секція .ccm_bss оголошена NOLOAD - startup її не чистить і не копіює,
 * тому все, що сюди кладеться, заповнюється явно у BindHardware / Init.
 * g_adc_dma_buffer сюди НЕ переноситься: DMA на STM32F3 до CCM доступу не має. */
#define WG_CCM __attribute__((section(".ccm_bss")))

/* ------------------------------------------------------------------------- */
/*                            Внутрішні структури                            */
/* ------------------------------------------------------------------------- */

/* Компактний опис активної (ненульової) гармоніки - те, по чому бігає
 * гарячий цикл. Перебудовується лише при зміні конфігурації. */
typedef struct
{
    uint32_t bias;    /* фазовий зсув гармоніки, Q32 */
    int16_t  amp;     /* амплітуда у відліках ADC    */
    uint8_t  order;   /* 1..WG_HARMONIC_MAX_ORDER    */
} WgActiveHarmonic;

/* Поля, які підтримують лінійне наростання. */
typedef struct
{
    int32_t  amplitude_adc;
    uint32_t freq_millihz;
    int32_t  phase_deg_milli;
} WgRampPoint;

typedef struct
{
    /* --- залізо --- */
    WaveGeneratorChannelHardware hw;
    volatile uint32_t *cmp_reg;          /* прямий вказівник на CMPxxR */

    /* --- конфігурація --- */
    WaveGeneratorChannelConfig cfg;
    WaveGeneratorLoopGains     gains;
    WaveGeneratorFeedbackCal   cal;

    /* --- похідні величини (перераховуються поза гарячим циклом) --- */
    uint32_t phase_step;
    uint32_t phase_bias;
    WgActiveHarmonic active[WG_HARMONIC_COUNT];
    uint8_t  active_count;
    int32_t  aperiodic_decay_q;
    float    res_b;                      /* Ts * w0            */
    float    res_c;                      /* Ts * wc            */
    float    res_out_gain;               /* 2 * wc * kr        */
    float    ki_ts;                      /* ki * Ts            */

    /* --- стан --- */
    uint32_t phase_acc;
    int32_t  aperiodic_current_q;
    float    integrator;
    float    res_xa;
    float    res_xb;
    uint16_t feedback_adc;
    uint16_t reference_adc;
    uint16_t compare_ticks;
    uint16_t fb_peak_track;
    uint16_t fb_peak_hold;
    uint16_t fault_counter;
    int16_t  error_adc;
    bool     feedback_fault;
    bool     loop_running;

    /* --- самокалібрування об'єкта --- */
    int32_t  cal_target_amp;      /* робоча амплітуда, відновлюється після проби */
    float    cal_sum_rf;          /* Sum(ref_dev * fb_dev) */
    float    cal_sum_rr;          /* Sum(ref_dev^2)        */
    float    cal_fb_acc;          /* накопичення ЗЗ для offset */
    uint32_t cal_count;
    uint8_t  cal_attempt;
    bool     cal_ok;
    bool     cal_failed;
    int32_t  last_ref_dev;
    float    last_fb_dev;
    int32_t  cal_amp_max;         /* досяжна амплітуда при знайденому ff */
    bool     cal_amp_limited;

    /* --- активний профіль --- */
    bool        profile_active;
    uint64_t    profile_start_sample;
    uint32_t    profile_duration;
    uint32_t    profile_mask;
    WgRampPoint ramp_start;
    WgRampPoint ramp_target;
} WgChannel;

/* ------------------------------------------------------------------------- */
/*                              Глобальний стан                              */
/* ------------------------------------------------------------------------- */

extern WG_CCM WaveGeneratorHardwareConfig g_hw;
extern WG_CCM WgChannel g_ch[WG_CHANNEL_COUNT];

typedef struct
{
    bool     initialized;
    bool     outputs_running;
    bool     signal_active;
    bool     start_armed;
    bool     overrun;
    uint64_t sample_counter;
    uint64_t signal_start_sample;
    uint64_t signal_elapsed_samples;
    uint32_t isr_cycles_max;             /* найдовший такт, такти ядра */
    uint16_t adc_sample[WAVE_GENERATOR_FEEDBACK_SOURCE_COUNT];
} WgRuntime;

extern WG_CCM WgRuntime g_rt;

extern uint32_t g_sample_rate_hz;
extern float    g_sample_period_s;
extern uint32_t g_master_period;
extern int32_t  g_duty_center;
extern int32_t  g_cmp_min;
extern int32_t  g_cmp_max;

extern WG_CCM int16_t g_sine_lut[WG_LUT_SIZE];
/* DMA збирає WG_ADC_GROUPS_PER_TICK груп «rank1 + rank2» за такт контуру.
 * У звичайній SRAM: DMA на STM32F3 до CCM доступу не має. */
extern uint32_t g_adc_dma_buffer[2U * WG_ADC_GROUPS_PER_TICK];

extern WG_CCM WaveGeneratorScheduledUpdate g_queue[WG_UPDATE_QUEUE_CAPACITY];
extern uint8_t g_queue_count;

/* --- автостарт і самокалібрування --------------------------------------- */
typedef enum
{
    WGB_SETTLE = 0,   /* амплітуда 0, чекаємо LC, міряємо offset ЗЗ  */
    WGB_PROBE_SKIP,   /* проба почалась, пропускаємо перехідний      */
    WGB_PROBE_MEAS,   /* накопичуємо кореляційні суми                */
    WGB_APPLY,        /* застосовуємо результат                      */
    WGB_RUN           /* штатна робота                               */
} WgBootState;

extern WgBootState g_boot_state;
extern uint32_t g_boot_ticks;
extern uint32_t g_boot_limit;
extern uint32_t g_samples_per_period;
extern uint8_t  g_boot_attempt;

/* ------------------------------------------------------------------------- */
/*                                 Утиліти                                   */
/* ------------------------------------------------------------------------- */

static inline uint32_t WgLock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void WgUnlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float clamp_f(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline int32_t lerp_i32(int32_t a, int32_t b, uint32_t t, uint32_t span)
{
    if (span == 0U)
    {
        return b;
    }
    return a + (int32_t)(((int64_t)(b - a) * (int64_t)t) / (int64_t)span);
}

static inline uint32_t lerp_u32(uint32_t a, uint32_t b, uint32_t t, uint32_t span)
{
    if (span == 0U)
    {
        return b;
    }
    return (uint32_t)((int64_t)a + (((int64_t)b - (int64_t)a) * (int64_t)t) / (int64_t)span);
}

/* ---- Функції, спільні між файлами модуля --------------------------------- */

void WgApplyDueUpdates(void);
void WgApplyMaskedPatch(WaveGeneratorChannelConfig *base, uint32_t mask, const WaveGeneratorChannelConfig *patch);
void WgBootTick(void);
void WgNormalizeChannelConfig(WaveGeneratorChannelConfig *c);
void WgNormalizeGains(WaveGeneratorLoopGains *g);
bool WgQueueInsert(const WaveGeneratorScheduledUpdate *update);
void WgRecomputeDerived(WgChannel *c, uint32_t mask);
void WgRecomputeGains(WgChannel *c);
uint32_t WgSamplesPerPeriod(void);
void WgSetChannelIdle(WgChannel *c);

#endif /* WAVE_GENERATOR_INTERNAL_H_ */
