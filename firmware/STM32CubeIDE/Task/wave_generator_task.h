/*
 * wave_generator_task.h
 *
 *  Created on: May 11, 2025
 *      Author: Oleksandr
 *
 * Багатоканальний генератор сигналів із замкненим контуром керування.
 *
 * Швидкий контур повністю живе в ISR завершення передачі ADC->DMA
 * (HAL_ADC_ConvCpltCallback). Задач FreeRTOS для генерації немає й не
 * потрібно: усе, що робить прикладний рівень, - це змінює конфігурацію
 * через API нижче, а ISR її підхоплює.
 */

#ifndef WAVE_GENERATOR_TASK_H_
#define WAVE_GENERATOR_TASK_H_

#include "main.h"
#include "wave_generator_config.h"

#include <stdbool.h>
#include <stdint.h>

/* ---- Маска полів для часткового оновлення конфігурації ------------------ */
#define WAVE_GENERATOR_UPDATE_AMPLITUDE            (1UL << 0)  /* амплітуда основної */
#define WAVE_GENERATOR_UPDATE_FREQUENCY            (1UL << 1)
#define WAVE_GENERATOR_UPDATE_PHASE                (1UL << 2)
#define WAVE_GENERATOR_UPDATE_APERIODIC_AMPLITUDE  (1UL << 3)
#define WAVE_GENERATOR_UPDATE_APERIODIC_TAU        (1UL << 4)
#define WAVE_GENERATOR_UPDATE_HARMONICS            (1UL << 5)  /* увесь масив гармонік */
#define WAVE_GENERATOR_UPDATE_LOOP_MODE            (1UL << 6)
#define WAVE_GENERATOR_UPDATE_GAINS                (1UL << 7)
#define WAVE_GENERATOR_UPDATE_ALL                  ((1UL << 8) - 1UL)

/* Поля, які підтримують лінійне наростання (ramp) у запланованих змінах.
 * Решта застосовується стрибком навіть у режимі LINEAR. */
#define WAVE_GENERATOR_RAMPABLE   (WAVE_GENERATOR_UPDATE_AMPLITUDE | \
                                   WAVE_GENERATOR_UPDATE_FREQUENCY | \
                                   WAVE_GENERATOR_UPDATE_PHASE)

/* ---- Джерело зворотного зв'язку ----------------------------------------- *
 * Dual regular simultaneous + DMAACCESSMODE_12_10_BITS: кожне 32-бітне слово
 * DMA містить master (ADC1) у молодших 16 бітах і slave (ADC2) у старших.
 * Слово 0 = rank 1, слово 1 = rank 2.                                       */
typedef enum
{
    WAVE_GENERATOR_FEEDBACK_MASTER_RANK1 = 0,  /* ADC1 CH3  diff PA2/PA3 */
    WAVE_GENERATOR_FEEDBACK_SLAVE_RANK1,       /* ADC2 CH1  diff PA4     */
    WAVE_GENERATOR_FEEDBACK_MASTER_RANK2,      /* ADC1 CH11      PB0     */
    WAVE_GENERATOR_FEEDBACK_SLAVE_RANK2,       /* ADC2 CH3  diff PA6     */
    WAVE_GENERATOR_FEEDBACK_SOURCE_COUNT
} WaveGeneratorFeedbackSource;

typedef enum
{
    WAVE_GENERATOR_PROFILE_STEP = 0,
    WAVE_GENERATOR_PROFILE_LINEAR
} WaveGeneratorProfileMode;

/* ---- Опис одного каналу на рівні заліза --------------------------------- */
typedef struct
{
    uint32_t hrtim_timer_index;   /* HRTIM_TIMERINDEX_TIMER_A / _B / ...      */
    uint32_t hrtim_compare_unit;  /* HRTIM_COMPAREUNIT_1 / _2                 */
    uint32_t hrtim_output;        /* HRTIM_OUTPUT_TA1 / TA2 / TB1 / TB2       */
    WaveGeneratorFeedbackSource feedback_source;
    bool     feedback_enabled;    /* false -> канал завжди в розімкненому контурі */
} WaveGeneratorChannelHardware;

typedef struct
{
    ADC_HandleTypeDef   *adc_master;   /* ADC1 */
    ADC_HandleTypeDef   *adc_slave;    /* ADC2 */
    HRTIM_HandleTypeDef *hrtim;
    uint32_t pwm_output_mask;          /* сума HRTIM_OUTPUT_xx усіх каналів   */
    uint32_t counter_start_mask;       /* HRTIM_TIMERID_MASTER | ...          */
    uint32_t sampling_trigger_index;   /* HRTIM_ADCTRIGGER_1                  */
    WaveGeneratorChannelHardware channel[WG_CHANNEL_COUNT];
} WaveGeneratorHardwareConfig;

/* ---- Одна гармоніка ------------------------------------------------------ */
typedef struct
{
    int32_t amplitude_adc;    /* пікова амплітуда у відліках ADC, 0..2047     */
    int32_t phase_deg_milli;  /* фаза відносно основної, тисячні градуса      */
} WaveGeneratorHarmonic;

/* ---- Завдання одного каналу ---------------------------------------------- */
typedef struct
{
    uint32_t fundamental_freq_millihz;   /* тисячні герца                     */
    int32_t  phase_deg_milli;            /* фаза каналу, тисячні градуса      */
    /* harmonic[0] - основна (1-а), harmonic[k] - гармоніка порядку k+1       */
    WaveGeneratorHarmonic harmonic[WG_HARMONIC_COUNT];
    int32_t  aperiodic_amplitude_adc;    /* початкове значення аперіодичної   */
    uint32_t aperiodic_tau_ms;           /* 0 -> постійне зміщення без згасання */
    bool     closed_loop;                /* false -> тільки feedforward       */
} WaveGeneratorChannelConfig;

/* ---- Коефіцієнти регулятора каналу --------------------------------------- */
typedef struct
{
    float feedforward_gain;   /* тіки duty на відлік ADC                      */
    float kp;                 /* тіки duty на відлік ADC                      */
    float ki;                 /* тіки duty на (відлік ADC * с)                */
    float kr;                 /* підсилення резонансного члена, безрозмірне   */
    float wc;                 /* смуга резонатора, рад/с                      */
    float output_limit;       /* межа |PI + резонатор|, тіки duty             */
} WaveGeneratorLoopGains;

/* ---- Калібрування каналу вимірювання ------------------------------------- */
typedef struct
{
    int32_t offset_adc;   /* код ADC при нульовому сигналі                    */
    float   scale;        /* відліки ЗЗ -> шкала завдання; від'ємний = інверсія */
} WaveGeneratorFeedbackCal;

/* ---- Запланована зміна ---------------------------------------------------- */
typedef struct
{
    uint64_t apply_at_sample;      /* такт контуру, коли застосувати          */
    uint32_t duration_samples;     /* 0 або MODE=STEP -> стрибком             */
    uint32_t update_mask;
    uint8_t  channel;
    WaveGeneratorProfileMode mode;
    WaveGeneratorChannelConfig target_config;
} WaveGeneratorScheduledUpdate;

/* ---- Стан каналу для телеметрії ------------------------------------------- */
typedef struct
{
    bool     closed_loop;        /* реально працює по ЗЗ просто зараз        */
    bool     feedback_fault;     /* канал ЗЗ у насиченні / обрив             */
    bool     calibrated;         /* самокалібрування об'єкта вдалося         */
    bool     calibration_failed; /* спроби вичерпані, лишились розімкнені    */
    bool     amplitude_limited;  /* задану амплітуду обрізано до досяжної    */
    int32_t  amplitude_max_adc;  /* максимум, який тягне силовий каскад      */
    bool     profile_active;
    uint16_t reference_adc;
    uint16_t feedback_adc;
    uint16_t feedback_peak_adc;  /* пік ЗЗ за останній період основної       */
    uint16_t compare_ticks;
    int16_t  error_adc;
} WaveGeneratorChannelState;

typedef struct
{
    bool     initialized;
    bool     outputs_running;
    bool     signal_active;
    bool     start_armed;
    bool     overrun;            /* такт контуру не встиг завершитись        */
    uint64_t sample_counter;
    uint64_t signal_start_sample;
    uint64_t signal_elapsed_samples;
    uint32_t sample_rate_hz;
    uint8_t  pending_updates;
    WaveGeneratorChannelState channel[WG_CHANNEL_COUNT];
} WaveGeneratorRuntimeState;

/* ---- API ------------------------------------------------------------------ */
void WaveGenerator_BindHardware(const WaveGeneratorHardwareConfig *config);
void WaveGenerator_Init(void);

/* Фактична частота контуру керування (100 kHz / WG_CONTROL_DECIMATION). */
uint32_t WaveGenerator_GetSampleRateHz(void);

bool WaveGenerator_GetChannelConfig(uint8_t channel, WaveGeneratorChannelConfig *config);
bool WaveGenerator_UpdateChannelConfig(uint8_t channel,
                                       uint32_t update_mask,
                                       const WaveGeneratorChannelConfig *patch);

bool WaveGenerator_GetLoopGains(uint8_t channel, WaveGeneratorLoopGains *gains);
bool WaveGenerator_SetLoopGains(uint8_t channel, const WaveGeneratorLoopGains *gains);

bool WaveGenerator_GetFeedbackCal(uint8_t channel, WaveGeneratorFeedbackCal *cal);
bool WaveGenerator_SetFeedbackCal(uint8_t channel, const WaveGeneratorFeedbackCal *cal);

bool WaveGenerator_QueueScheduledUpdate(const WaveGeneratorScheduledUpdate *update);
void WaveGenerator_ClearScheduledUpdates(void);

void WaveGenerator_StartNow(void);
void WaveGenerator_ArmStartAtSample(uint64_t start_sample);
void WaveGenerator_Stop(void);

void WaveGenerator_GetRuntimeState(WaveGeneratorRuntimeState *state);
bool WaveGenerator_GetSampleCounter(uint64_t *sample_counter);

/* Діагностика навантаження: найдовший такт контуру у проміле від періоду.
 * Значення > ~700 означає, що пора збільшувати WG_CONTROL_DECIMATION. */
uint32_t WaveGenerator_GetLoadPermille(void);
void     WaveGenerator_ResetLoadMeasurement(void);

#endif /* WAVE_GENERATOR_TASK_H_ */
