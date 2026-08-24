/*
 * wave_generator_hw.c
 *
 * Налаштування й запуск периферії. ТУТ ЗОСЕРЕДЖЕНО ВЕСЬ HAL модуля:
 * HAL_HRTIM_TimeBaseConfig, HAL_HRTIM_ADCTriggerConfig,
 * HAL_HRTIM_WaveformCompareConfig, HAL_HRTIM_WaveformOutputConfig,
 * HAL_ADCEx_Calibration_Start, HAL_ADC_Start, HAL_ADCEx_MultiModeStart_DMA,
 * HAL_HRTIM_WaveformOutputStart, HAL_HRTIM_WaveformCounterStart.
 *
 * Повз HAL тут зроблено дві речі: кешування вказівника на регістр compare
 * (щоб у гарячому шляху був один store замість дерева порівнянь усередині
 * макроса __HAL_HRTIM_SETCOMPARE) і ввімкнення лічильника тактів DWT, для
 * якого в HAL немає API взагалі.
 */


#include "wave_generator_internal.h"


/* ------------------------------------------------------------------------- */
/*                        Ініціалізація периферії                            */
/* ------------------------------------------------------------------------- */

static void WgInitSineLut(void)
{
    for (uint32_t i = 0U; i < WG_LUT_SIZE; ++i)
    {
        float angle = (2.0f * (float)M_PI * (float)i) / (float)WG_LUT_SIZE;
        g_sine_lut[i] = (int16_t)lrintf(sinf(angle) * 32767.0f);
    }
}


static volatile uint32_t *WgCompareRegister(uint32_t timer_index, uint32_t compare_unit)
{
    HRTIM_TypeDef *inst = g_hw.hrtim->Instance;

    if (compare_unit == HRTIM_COMPAREUNIT_2)
    {
        return &inst->sTimerxRegs[timer_index].CMP2xR;
    }

    return &inst->sTimerxRegs[timer_index].CMP1xR;
}


/* Тактування master і схема запуску АЦП.
 *
 * У згенерованому CubeMX коді master мав PRESCALERRATIO_MUL4, а Timer A/B -
 * MUL32, при однаковому Period = 46080. Через це master (а з ним і ADC-тригер)
 * ішов у 8 разів повільніше, ніж PWM, тоді як увесь розрахунок частоти й фази
 * припускав 100 кГц.
 *
 * Тепер період master дорівнює РІВНО одному періоду PWM, а частоту такту
 * контуру задає довжина буфера DMA: переривання приходить, коли зібрано
 * WG_ADC_GROUPS_PER_TICK груп, тобто раз на WG_CONTROL_DECIMATION періодів.
 *
 * Усередині періоду ставимо WG_ADC_TRIGGERS_PER_PWM рівномірно рознесених
 * точок вибірки через master CMP1..CMP4. Регістр ADC1R приймає МАСКУ подій,
 * тому кілька compare складаються в один ADC-тригер. Це і є придушення
 * пульсації: ковзне середнє по вибірках, що рівномірно вкривають період
 * комутації, має нуль АЧХ рівно на частоті комутації. */
static void WgConfigureTiming(void)
{
    static const uint32_t cmp_unit[4] = {
        HRTIM_COMPAREUNIT_1, HRTIM_COMPAREUNIT_2,
        HRTIM_COMPAREUNIT_3, HRTIM_COMPAREUNIT_4
    };
    static const uint32_t cmp_event[4] = {
        HRTIM_ADCTRIGGEREVENT13_MASTER_CMP1, HRTIM_ADCTRIGGEREVENT13_MASTER_CMP2,
        HRTIM_ADCTRIGGEREVENT13_MASTER_CMP3, HRTIM_ADCTRIGGEREVENT13_MASTER_CMP4
    };

    HRTIM_TimeBaseCfgTypeDef tb = {0};
    HRTIM_ADCTriggerCfgTypeDef trig = {0};
    uint32_t trigger_mask = 0U;

    g_master_period   = WG_MASTER_TICKS_PER_PWM;
    g_sample_rate_hz  = WG_PWM_FREQ_HZ / WG_CONTROL_DECIMATION;
    g_sample_period_s = 1.0f / (float)g_sample_rate_hz;

    tb.Period            = g_master_period;
    tb.RepetitionCounter = 0x00U;
    tb.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL4;
    tb.Mode              = HRTIM_MODE_CONTINUOUS;

    if (HAL_HRTIM_TimeBaseConfig(g_hw.hrtim, HRTIM_TIMERINDEX_MASTER, &tb) != HAL_OK)
    {
        Error_Handler();
    }

    for (uint32_t i = 0U; i < WG_ADC_TRIGGERS_PER_PWM; ++i)
    {
        /* Точки вибірки в (2i+1)/(2N) періоду: рівномірно, без збігу з краями,
         * де комутаційні викиди найбільші. */
        uint32_t pos = (g_master_period * ((2U * i) + 1U)) / (2U * WG_ADC_TRIGGERS_PER_PWM);

        __HAL_HRTIM_SETCOMPARE(g_hw.hrtim, HRTIM_TIMERINDEX_MASTER, cmp_unit[i], pos);
        trigger_mask |= cmp_event[i];
    }

    trig.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
    trig.Trigger      = trigger_mask;

    if (HAL_HRTIM_ADCTriggerConfig(g_hw.hrtim, g_hw.sampling_trigger_index, &trig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* Усі чотири виходи приводяться до однакової форми SET = CMPx, RESET = PERIOD,
 * щоб арифметика duty була спільною. У згенерованому коді TB1 був інвертований,
 * а TB2 взагалі не мав джерел. */
static void WgConfigureOutputs(void)
{
    HRTIM_OutputCfgTypeDef out = {0};
    HRTIM_CompareCfgTypeDef cmp = {0};

    out.Polarity             = HRTIM_OUTPUTPOLARITY_HIGH;
    out.ResetSource          = HRTIM_OUTPUTRESET_TIMPER;
    out.IdleMode             = HRTIM_OUTPUTIDLEMODE_NONE;
    out.IdleLevel            = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    out.FaultLevel           = HRTIM_OUTPUTFAULTLEVEL_NONE;
    out.ChopperModeEnable    = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    out.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

    cmp.AutoDelayedMode    = HRTIM_AUTODELAYEDMODE_REGULAR;
    cmp.AutoDelayedTimeout = 0x0000U;
    cmp.CompareValue       = (uint32_t)(WG_PWM_PERIOD_TICKS - (uint32_t)g_duty_center);

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        const WaveGeneratorChannelHardware *hw = &g_ch[i].hw;

        out.SetSource = (hw->hrtim_compare_unit == HRTIM_COMPAREUNIT_2)
                            ? HRTIM_OUTPUTSET_TIMCMP2
                            : HRTIM_OUTPUTSET_TIMCMP1;

        if (HAL_HRTIM_WaveformCompareConfig(g_hw.hrtim, hw->hrtim_timer_index,
                                            hw->hrtim_compare_unit, &cmp) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_HRTIM_WaveformOutputConfig(g_hw.hrtim, hw->hrtim_timer_index,
                                           hw->hrtim_output, &out) != HAL_OK)
        {
            Error_Handler();
        }
    }
}


/* Лічильник тактів ядра для вимірювання завантаження контуру. Працює і без
 * підключеного відлагоджувача, достатньо ввімкнути TRCENA. */
static void WgEnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}


static void WgStartPeripherals(void)
{
    /* Калібрування ADC свідомо винесене з-під __disable_irq: воно чекає
     * апаратний прапорець із таймаутом по HAL_GetTick, а з вимкненими
     * перериваннями системний тік не рухається. */
    if (HAL_ADCEx_Calibration_Start(g_hw.adc_slave, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(g_hw.adc_master, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start(g_hw.adc_slave) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_MultiModeStart_DMA(g_hw.adc_master, g_adc_dma_buffer,
                                     (uint32_t)(2U * WG_ADC_GROUPS_PER_TICK)) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_HRTIM_WaveformOutputStart(g_hw.hrtim, g_hw.pwm_output_mask) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_HRTIM_WaveformCounterStart(g_hw.hrtim, g_hw.counter_start_mask) != HAL_OK)
    {
        Error_Handler();
    }

    g_rt.outputs_running = true;
}


/* ------------------------------------------------------------------------- */
/*                                   API                                     */
/* ------------------------------------------------------------------------- */

void WaveGenerator_BindHardware(const WaveGeneratorHardwareConfig *config)
{
    uint32_t key;

    if (config == NULL)
    {
        return;
    }

    key = WgLock();
    /* g_hw і g_ch лежать у NOLOAD-секції CCM, тому їх треба обнулити явно:
     * startup цього не робить. BindHardware завжди викликається першим. */
    memset(&g_rt, 0, sizeof(g_rt));
    memset(g_ch, 0, sizeof(g_ch));
    g_hw = *config;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        g_ch[i].hw = config->channel[i];
    }

    WgUnlock(key);
}


void WaveGenerator_Init(void)
{
    static const int32_t default_phase[WG_CHANNEL_COUNT] = {
        WG_DEFAULT_PHASE_CH0_DEG_MILLI,
        WG_DEFAULT_PHASE_CH1_DEG_MILLI,
        WG_DEFAULT_PHASE_CH2_DEG_MILLI,
        WG_DEFAULT_PHASE_CH3_DEG_MILLI,
    };

    if (g_rt.initialized)
    {
        return;
    }

    if ((g_hw.adc_master == NULL) || (g_hw.adc_slave == NULL) || (g_hw.hrtim == NULL))
    {
        Error_Handler();
        return;
    }

    memset(&g_rt, 0, sizeof(g_rt));
    memset(g_queue, 0, sizeof(g_queue));
    g_queue_count = 0U;

    g_duty_center = (int32_t)(WG_PWM_PERIOD_TICKS / 2U);
    g_cmp_min     = (int32_t)WG_DUTY_MARGIN_TICKS;
    g_cmp_max     = (int32_t)(WG_PWM_PERIOD_TICKS - WG_DUTY_MARGIN_TICKS);

    WgInitSineLut();
    WgConfigureTiming();

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        WgChannel *c = &g_ch[i];
        WaveGeneratorChannelHardware hw = c->hw;

        memset(c, 0, sizeof(*c));
        c->hw      = hw;
        c->cmp_reg = WgCompareRegister(hw.hrtim_timer_index, hw.hrtim_compare_unit);

        c->cfg.fundamental_freq_millihz  = WG_DEFAULT_FREQ_MILLIHZ;
        c->cfg.phase_deg_milli           = default_phase[i];
        c->cfg.aperiodic_amplitude_adc   = WG_DEFAULT_APERIODIC_ADC;
        c->cfg.aperiodic_tau_ms          = WG_DEFAULT_TAU_MS;
        c->cal_target_amp                = WG_DEFAULT_AMPLITUDE_ADC;

#if WG_AUTOCAL
        /* Стартуємо з нульовою амплітудою й розімкненим контуром: спершу
         * треба виміряти об'єкт, і лише потім замикати. Замикати наосліп
         * не можна - невгаданий ЗНАК тракту дав би додатний зворотний
         * зв'язок і миттєвий вихід у полицю. */
        c->cfg.harmonic[0].amplitude_adc = 0;
        c->cfg.closed_loop               = false;

        c->gains.feedforward_gain = WG_CAL_FF_PROBE;
        c->gains.kp               = 0.0f;
        c->gains.ki               = 0.0f;
        c->gains.kr               = 0.0f;
#else
        /* Без самокалібрування нормовані коефіцієнти беруться як є, тобто
         * неявно припускається підсилення об'єкта P = 1. Майже напевно це
         * не так - коефіцієнти доведеться підбирати вручну по UART. */
        c->cfg.harmonic[0].amplitude_adc = WG_DEFAULT_AMPLITUDE_ADC;
        c->cfg.closed_loop               = hw.feedback_enabled;

        c->gains.feedforward_gain = WG_FF_NORM;
        c->gains.kp               = WG_PI_KP_NORM;
        c->gains.ki               = WG_PI_KI_NORM;
        c->gains.kr               = WG_PR_KR_NORM;
#endif
        c->gains.wc               = WG_PR_WC_DEFAULT;
        c->gains.output_limit     = WG_LOOP_OUT_LIMIT_TICKS;

        c->cal.offset_adc = WG_FB_OFFSET_DEFAULT;
        c->cal.scale      = WG_FB_SCALE_DEFAULT;

        WgNormalizeChannelConfig(&c->cfg);
        WgNormalizeGains(&c->gains);
        WgRecomputeDerived(c, WAVE_GENERATOR_UPDATE_ALL);
        WgSetChannelIdle(c);
    }

    WgEnableCycleCounter();
    WgConfigureOutputs();

#if WG_AUTOCAL
    g_boot_state         = WGB_SETTLE;
    g_boot_attempt       = 0U;
    g_boot_ticks         = 0U;
    g_samples_per_period = WgSamplesPerPeriod();
    g_boot_limit         = (uint32_t)(((uint64_t)WG_CAL_SETTLE_MS * g_sample_rate_hz) / 1000ULL);
#else
    g_boot_state = WGB_RUN;
#endif

    WgStartPeripherals();

    g_rt.initialized = true;

#if WG_AUTOSTART
    /* Без UART прошивка має почати сама. Генерація стартує одразу, але з
     * нульовою амплітудою: спершу відпрацює калібрування, і лише потім
     * амплітуда плавно підніметься до робочої. */
    g_rt.signal_active          = true;
    g_rt.start_armed            = false;
    g_rt.signal_elapsed_samples = 0ULL;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        g_ch[i].aperiodic_current_q =
            g_ch[i].cfg.aperiodic_amplitude_adc << WG_APERIODIC_Q_SHIFT;
    }
#endif
}
