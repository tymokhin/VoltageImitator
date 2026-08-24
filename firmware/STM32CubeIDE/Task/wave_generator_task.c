/*
 * wave_generator_task.c
 *
 * ЯДРО МОДУЛЯ: гарячий шлях контуру керування.
 *
 * ТОЧКА ВХОДУ - HAL_ADC_ConvCpltCallback() наприкінці файлу. Задачі FreeRTOS
 * для генератора немає і не повинно бути: такт триває 40 мкс, а пробудження
 * задачі семафором коштує 200-400 тактів на перемикання контексту, тобто
 * 10-15 % бюджету, плюс джитер, який у силовому каскаді стає спотворенням
 * форми. Швидкий контур живе в перериванні - стандартна схема для приводів
 * і перетворювачів. Задачі лишаються для повільного: зараз це лише uart_task.
 *
 * Потік даних одного такту:
 *
 *   HRTIM master CMP1..CMP4 -> ADC trigger -> ADC1/ADC2 dual simultaneous
 *     -> DMA1_Ch1 (16 груп по 2 слова) -> HAL_ADC_ConvCpltCallback
 *       -> усереднення вибірок (гасить пульсацію комутації)
 *       -> застосування запланованих змін     [wave_generator_schedule.c]
 *       -> синтез завдання -> регулятор -> CMPx
 *       -> крок автостарту/калібрування       [wave_generator_cal.c]
 */


/* Такт контуру виконується 25 000 разів на секунду, тому файл оптимізується
 * завжди - навіть у Debug, який STM32CubeIDE збирає з -O0. Без цього такт
 * розтягується приблизно втричі і контур перестає встигати. */
#if defined(__GNUC__) && !defined(__OPTIMIZE__)
#pragma GCC optimize ("O2", "inline-functions")
#endif


#include "wave_generator_internal.h"


/* ---- Спільний стан модуля -------------------------------------------------
 * Оголошення - у wave_generator_internal.h; означення зібрані тут, бо цей
 * файл є ядром модуля. */
WG_CCM WaveGeneratorHardwareConfig g_hw;
WG_CCM WgChannel g_ch[WG_CHANNEL_COUNT];
WG_CCM WgRuntime g_rt;

uint32_t g_sample_rate_hz  = WG_PWM_FREQ_HZ / WG_CONTROL_DECIMATION;
float    g_sample_period_s = 1.0f / (float)(WG_PWM_FREQ_HZ / WG_CONTROL_DECIMATION);
uint32_t g_master_period   = WG_CONTROL_DECIMATION * WG_MASTER_TICKS_PER_PWM;
int32_t  g_duty_center     = (int32_t)(WG_PWM_PERIOD_TICKS / 2U);
int32_t  g_cmp_min         = (int32_t)WG_DUTY_MARGIN_TICKS;
int32_t  g_cmp_max         = (int32_t)(WG_PWM_PERIOD_TICKS - WG_DUTY_MARGIN_TICKS);

WG_CCM int16_t g_sine_lut[WG_LUT_SIZE];

/* Буфер DMA - у звичайній SRAM: DMA на STM32F3 до CCM доступу не має. */
uint32_t g_adc_dma_buffer[2U * WG_ADC_GROUPS_PER_TICK];

WG_CCM WaveGeneratorScheduledUpdate g_queue[WG_UPDATE_QUEUE_CAPACITY];
uint8_t g_queue_count;

WgBootState g_boot_state;
uint32_t g_boot_ticks;
uint32_t g_boot_limit;
uint32_t g_samples_per_period;
uint8_t  g_boot_attempt;




#if (WG_DUTY_QUANT_TICKS > 1U)

/* Стан генератора дизеринга. У звичайній SRAM з ініціалізатором, а не в
 * .ccm_bss: там NOLOAD, і xorshift із нульового стану залипає назавжди. */
static uint32_t g_dither_state = 0x2545F491U;

static inline uint32_t WgPrng(void)
{
    uint32_t x = g_dither_state;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_dither_state = x;
    return x;
}

/* Емуляція грубішої сітки compare - див. розділ 10 у wave_generator_config.h.
 * Дизеринг додає рівномірний шум в один крок квантування ПЕРЕД усіченням:
 * це декорелює похибку від сигналу, перетворюючи гармоніки квантування на
 * рівномірний шумовий фон. */
static inline int32_t WgQuantizeCompare(int32_t cmp)
{
    const int32_t q = (int32_t)WG_DUTY_QUANT_TICKS;

#if WG_DUTY_DITHER
    cmp += (int32_t)(WgPrng() % (uint32_t)q);
#endif

    return (cmp / q) * q;
}

#endif /* WG_DUTY_QUANT_TICKS > 1 */

/* Вихід сконфігуровано як SET = CMPx, RESET = PERIOD, тобто імпульс триває
 * від compare до кінця періоду: t_on = period - compare. */
static inline void WgWriteDuty(WgChannel *c, int32_t duty_ticks)
{
    int32_t cmp = (int32_t)WG_PWM_PERIOD_TICKS - duty_ticks;

    cmp = clamp_i32(cmp, g_cmp_min, g_cmp_max);

#if (WG_DUTY_QUANT_TICKS > 1U)
    /* Квантування після першого обмеження (щоб ділити завідомо додатне) і
     * повторне обмеження після нього: усічення може зсунути значення нижче
     * апаратного мінімуму HRTIM. */
    cmp = clamp_i32(WgQuantizeCompare(cmp), g_cmp_min, g_cmp_max);
#endif

    c->compare_ticks = (uint16_t)cmp;
    *c->cmp_reg = (uint32_t)cmp;
}


void WgSetChannelIdle(WgChannel *c)
{
    c->reference_adc       = (uint16_t)WG_ADC_MIDPOINT;
    c->error_adc           = 0;
    c->integrator          = 0.0f;
    c->res_xa              = 0.0f;
    c->res_xb              = 0.0f;
    c->aperiodic_current_q = 0;
    c->phase_acc           = 0U;
    c->fb_peak_track       = 0U;
    c->loop_running        = false;
    WgWriteDuty(c, g_duty_center);
}


/* ------------------------------------------------------------------------- */
/*                              Гарячий цикл                                 */
/* ------------------------------------------------------------------------- */

/* Синтез миттєвого значення завдання: сума активних гармонік із лінійною
 * інтерполяцією по LUT + аперіодична складова. Повертає ВІДХИЛЕННЯ від
 * середини шкали, у відліках ADC. */
static inline int32_t WgSynthesizeReference(WgChannel *c)
{
    const WgActiveHarmonic *h = c->active;
    int32_t acc = 0;
    uint32_t prev_phase = c->phase_acc;

    for (uint8_t i = 0U; i < c->active_count; ++i, ++h)
    {
        uint32_t ph   = (c->phase_acc * (uint32_t)h->order) + h->bias;
        uint32_t idx  = ph >> WG_LUT_INDEX_SHIFT;
        int32_t  frac = (int32_t)((ph >> WG_LUT_FRAC_SHIFT) & 0xFFU);
        int32_t  s0   = g_sine_lut[idx];
        int32_t  s1   = g_sine_lut[(idx + 1U) & WG_LUT_MASK];

        acc += (s0 + (((s1 - s0) * frac) >> 8)) * (int32_t)h->amp;
    }

    c->phase_acc = prev_phase + c->phase_step;

    /* Перехід через нуль фази - момент оновлення виміряного піка. */
    if (c->phase_acc < prev_phase)
    {
        c->fb_peak_hold  = c->fb_peak_track;
        c->fb_peak_track = 0U;
    }

    return (acc >> 15) + (c->aperiodic_current_q >> WG_APERIODIC_Q_SHIFT);
}


static inline void WgDecayAperiodic(WgChannel *c)
{
    if (c->aperiodic_decay_q == (int32_t)WG_DECAY_Q_ONE)
    {
        return;                                   /* постійне зміщення */
    }

    if (c->aperiodic_decay_q == 0)
    {
        c->aperiodic_current_q = 0;
        return;
    }

    c->aperiodic_current_q =
        (int32_t)(((int64_t)c->aperiodic_current_q * (int64_t)c->aperiodic_decay_q) >>
                  WG_DECAY_Q_SHIFT);
}


static void WgProcessChannel(WgChannel *c)
{
    int32_t ref_offset;
    float   duty;

    ref_offset = WgSynthesizeReference(c);
    c->reference_adc = (uint16_t)clamp_i32(WG_ADC_MIDPOINT + ref_offset, 0, WG_ADC_FULL_SCALE);
    c->last_ref_dev  = ref_offset;
    c->last_fb_dev   = 0.0f;

    /* --- feedforward: пряме відображення завдання в шпаруватість --- */
    duty = (float)g_duty_center + c->gains.feedforward_gain * (float)ref_offset;

    if (c->hw.feedback_enabled)
    {
        int32_t raw = (int32_t)g_rt.adc_sample[c->hw.feedback_source];
        int32_t dev = raw - c->cal.offset_adc;
        uint16_t mag = (uint16_t)((dev < 0) ? -dev : dev);

        c->feedback_adc = (uint16_t)raw;
        /* Відхилення ЗЗ рахуємо завжди, а не лише в замкненому контурі:
         * воно потрібне самокалібруванню, яке працює саме в розімкненому. */
        c->last_fb_dev = (float)dev * c->cal.scale;

        if (mag > c->fb_peak_track)
        {
            c->fb_peak_track = mag;
        }

        /* Насичення/обрив каналу вимірювання - переходимо в розімкнений
         * контур, щоб регулятор не заганяв вихід у полицю. */
        if ((raw <= WG_FB_SAT_LOW) || (raw >= WG_FB_SAT_HIGH))
        {
            if (c->fault_counter < WG_FEEDBACK_FAULT_TICKS)
            {
                c->fault_counter++;
            }
            else
            {
                c->feedback_fault = true;
            }
        }
        else
        {
            c->fault_counter  = 0U;
            c->feedback_fault = false;
        }
    }

    if (c->hw.feedback_enabled && c->cfg.closed_loop && !c->feedback_fault)
    {
        float err = (float)ref_offset - c->last_fb_dev;
        float u;
        float lim = c->gains.output_limit;

        c->error_adc = (int16_t)clamp_i32((int32_t)err, -32768, 32767);

        /* Заморожування станів при завеликій похибці: одночасно anti-windup
         * і захист від "розгону" при пошкодженні кола ЗЗ. */
        if ((err < (float)WG_ERROR_FREEZE_ADC) && (err > -(float)WG_ERROR_FREEZE_ADC))
        {
            /* Демпфований резонатор на основній частоті (напівнеявний Ейлер):
             *   xa' = e - wc*xa - w0*xb
             *   xb' =     w0*xa - wc*xb
             * Передатна 2*wc*s/(s^2 + 2*wc*s + w0^2): одиничне підсилення
             * рівно на w0, нульове на постійному струмі. */
            c->res_xa += (g_sample_period_s * err) - (c->res_c * c->res_xa) -
                         (c->res_b * c->res_xb);
            c->res_xb += (c->res_b * c->res_xa) - (c->res_c * c->res_xb);
            c->integrator += c->ki_ts * err;
            c->integrator = clamp_f(c->integrator, -lim, lim);
        }

        u = (c->gains.kp * err) + c->integrator + (c->res_out_gain * c->res_xa);
        u = clamp_f(u, -lim, lim);
        duty += u;
        c->loop_running = true;
    }
    else
    {
        c->integrator   = 0.0f;
        c->res_xa       = 0.0f;
        c->res_xb       = 0.0f;
        c->error_adc    = 0;
        c->loop_running = false;
    }

    WgWriteDuty(c, (int32_t)duty);
    WgDecayAperiodic(c);
}


static void WgActivateSignal(void)
{
    g_rt.signal_active          = true;
    g_rt.start_armed            = false;
    g_rt.signal_elapsed_samples = 0ULL;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        g_ch[i].phase_acc           = 0U;
        g_ch[i].integrator          = 0.0f;
        g_ch[i].res_xa              = 0.0f;
        g_ch[i].res_xb              = 0.0f;
        g_ch[i].aperiodic_current_q = g_ch[i].cfg.aperiodic_amplitude_adc << WG_APERIODIC_Q_SHIFT;
    }
}


/* Усереднення WG_ADC_GROUPS_PER_TICK вибірок, рівномірно рознесених
 * усередині періоду PWM. Саме рознесення ВСЕРЕДИНІ періоду (а не через
 * період) робить це ковзне середнє фільтром із нулем АЧХ рівно на частоті
 * комутації - пульсація гаситься за побудовою.
 *
 * Кожна група - два слова: слово 0 = rank 1, слово 1 = rank 2; у кожному
 * слові master (ADC1) у молодших 16 бітах, slave (ADC2) у старших. */
static void WgUnpackAdc(void)
{
    const uint32_t *p = g_adc_dma_buffer;
    uint32_t m1 = 0U;
    uint32_t s1 = 0U;
    uint32_t m2 = 0U;
    uint32_t s2 = 0U;

    for (uint32_t i = 0U; i < WG_ADC_GROUPS_PER_TICK; ++i)
    {
        uint32_t w0 = *p++;
        uint32_t w1 = *p++;

        m1 += (w0 & 0xFFFFU);
        s1 += (w0 >> 16);
        m2 += (w1 & 0xFFFFU);
        s2 += (w1 >> 16);
    }

    g_rt.adc_sample[WAVE_GENERATOR_FEEDBACK_MASTER_RANK1] = (uint16_t)(m1 / WG_ADC_GROUPS_PER_TICK);
    g_rt.adc_sample[WAVE_GENERATOR_FEEDBACK_SLAVE_RANK1]  = (uint16_t)(s1 / WG_ADC_GROUPS_PER_TICK);
    g_rt.adc_sample[WAVE_GENERATOR_FEEDBACK_MASTER_RANK2] = (uint16_t)(m2 / WG_ADC_GROUPS_PER_TICK);
    g_rt.adc_sample[WAVE_GENERATOR_FEEDBACK_SLAVE_RANK2]  = (uint16_t)(s2 / WG_ADC_GROUPS_PER_TICK);
}


static void WgControlTick(void)
{
    g_rt.sample_counter++;
    WgUnpackAdc();

    if (g_rt.start_armed && (g_rt.sample_counter >= g_rt.signal_start_sample))
    {
        WgActivateSignal();
    }

    WgApplyDueUpdates();

    if (!g_rt.signal_active)
    {
        for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
        {
            WgSetChannelIdle(&g_ch[i]);
        }
        return;
    }

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        WgProcessChannel(&g_ch[i]);
    }

#if WG_AUTOCAL
    WgBootTick();
#endif

    g_rt.signal_elapsed_samples++;
}


/* ------------------------------------------------------------------------- */
/*                                  ISR                                      */
/* ------------------------------------------------------------------------- */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t t0;
    uint32_t dt;

    if ((hadc != g_hw.adc_master) || (!g_rt.initialized))
    {
        return;
    }

    t0 = DWT->CYCCNT;

    WgControlTick();

    dt = DWT->CYCCNT - t0;

    if (dt > g_rt.isr_cycles_max)
    {
        g_rt.isr_cycles_max = dt;
    }

    /* Такт не вклався в бюджет - контур почав відставати. */
    if (dt > (SystemCoreClock / g_sample_rate_hz))
    {
        g_rt.overrun = true;
    }
}


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == g_hw.adc_master)
    {
        g_rt.overrun = true;
    }
}
