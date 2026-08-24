/*
 * wave_generator_cal.c
 *
 * Автостарт і самокалібрування об'єкта.
 *
 * Замикати контур наосліп не можна: невгаданий ЗНАК тракту дає додатний
 * зворотний зв'язок і миттєвий вихід у полицю. Тому на старті прошивка сама
 * вимірює передавальну характеристику «тіки duty -> відліки ADC» і лише
 * після цього замикає контур. Опис фаз - розділ 11 у wave_generator_config.h.
 */


#include "wave_generator_internal.h"




/* Симетричний запас шпаруватості навколо центру, у тіках. */
static int32_t WgDutySpanTicks(void)
{
    int32_t below = g_duty_center - ((int32_t)WG_PWM_PERIOD_TICKS - g_cmp_max);
    int32_t above = ((int32_t)WG_PWM_PERIOD_TICKS - g_cmp_min) - g_duty_center;

    return (below < above) ? below : above;
}


/* Найбільша амплітуда основної, яку силовий каскад відтворить без обрізання
 * при вже відкаліброваному feedforward. */
static int32_t WgMaxAmplitudeAdc(const WgChannel *c)
{
    float ff = c->gains.feedforward_gain;
    float mag = fabsf(ff);

    if (mag < 1e-6f)
    {
        return WG_ADC_HALF_RANGE;
    }

    return clamp_i32((int32_t)((WG_AMP_HEADROOM * (float)WgDutySpanTicks()) / mag),
                     0, WG_ADC_HALF_RANGE);
}


uint32_t WgSamplesPerPeriod(void)
{
    uint64_t n = ((uint64_t)g_sample_rate_hz * 1000ULL) /
                 (uint64_t)g_ch[0].cfg.fundamental_freq_millihz;

    return (uint32_t)clamp_u32((uint32_t)n, 16U, 100000U);
}


/* Запуск чергової спроби проби. Кожна наступна вчетверо «гучніша», але
 * модуляція duty жорстко обмежена WG_CAL_MAX_MODULATION_TICKS. */
static void WgBootBeginProbe(uint8_t attempt)
{
    float ff = WG_CAL_FF_PROBE;
    float ff_cap = (float)WG_CAL_MAX_MODULATION_TICKS / (float)WG_CAL_PROBE_AMPLITUDE_ADC;

    for (uint8_t i = 0U; i < attempt; ++i)
    {
        ff *= 4.0f;
    }

    if (ff > ff_cap)
    {
        ff = ff_cap;
    }

    g_boot_attempt = attempt;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        WgChannel *c = &g_ch[i];

        c->cal_sum_rf = 0.0f;
        c->cal_sum_rr = 0.0f;
        c->cal_count  = 0U;

        if (!c->hw.feedback_enabled || c->cal_ok)
        {
            continue;
        }

        c->gains.feedforward_gain = ff;
        c->gains.kp = 0.0f;
        c->gains.ki = 0.0f;
        c->gains.kr = 0.0f;
        WgRecomputeGains(c);

        c->cfg.closed_loop = false;
        c->cfg.harmonic[0].amplitude_adc = WG_CAL_PROBE_AMPLITUDE_ADC;
        WgRecomputeDerived(c, WAVE_GENERATOR_UPDATE_AMPLITUDE);
    }

    g_samples_per_period = WgSamplesPerPeriod();
    g_boot_ticks = 0U;
    g_boot_limit = WG_CAL_SKIP_PERIODS * g_samples_per_period;
    g_boot_state = WGB_PROBE_SKIP;
}


/* Завершення старту: застосувати знайдені коефіцієнти, замкнути контур і
 * плавно підняти амплітуду до робочої. */
static void WgBootApply(void)
{
    uint32_t ramp = (uint32_t)(((uint64_t)WG_AUTOSTART_RAMP_MS * g_sample_rate_hz) / 1000ULL);

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        WgChannel *c = &g_ch[i];
        WaveGeneratorScheduledUpdate up;

        if (!c->cal_ok)
        {
            /* Немає ЗЗ або калібрування не вдалося - працюємо розімкнено
             * з безпечним feedforward. Сигнал усе одно буде. */
            c->gains.feedforward_gain = WG_CAL_FF_PROBE;
            c->gains.kp = 0.0f;
            c->gains.ki = 0.0f;
            c->gains.kr = 0.0f;
            WgRecomputeGains(c);
        }

        c->cfg.closed_loop = c->cal_ok;
        c->integrator = 0.0f;
        c->res_xa     = 0.0f;
        c->res_xb     = 0.0f;

        /* Обмежуємо цільову амплітуду тим, що каскад реально тягне: краще
         * менша, але чиста синусоїда, ніж задана, але обрізана. */
        c->cal_amp_max     = WgMaxAmplitudeAdc(c);
        c->cal_amp_limited = (c->cal_target_amp > c->cal_amp_max);

        if (c->cal_amp_limited)
        {
            c->cal_target_amp = c->cal_amp_max;
        }

        c->cfg.harmonic[0].amplitude_adc = 0;
        WgRecomputeDerived(c, WAVE_GENERATOR_UPDATE_AMPLITUDE);

        memset(&up, 0, sizeof(up));
        up.channel          = (uint8_t)i;
        up.apply_at_sample  = g_rt.sample_counter + 1ULL;
        up.duration_samples = ramp;
        up.mode             = WAVE_GENERATOR_PROFILE_LINEAR;
        up.update_mask      = WAVE_GENERATOR_UPDATE_AMPLITUDE;
        up.target_config    = c->cfg;
        up.target_config.harmonic[0].amplitude_adc = c->cal_target_amp;
        (void)WgQueueInsert(&up);
    }

    g_boot_state = WGB_RUN;
}


/* Оцінка результату проби. g = Sum(ref*fb)/Sum(ref^2) - оцінка наскрізного
 * підсилення методом найменших квадратів, разом зі знаком. Усе, що не
 * корельоване із завданням (пульсація, шум), у цій сумі прямує до нуля. */
static void WgBootEvaluate(void)
{
    bool retry = false;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        WgChannel *c = &g_ch[i];
        float g;
        float mag;
        float k;
        float ff;

        if (!c->hw.feedback_enabled || c->cal_ok)
        {
            continue;
        }

        if (c->cal_sum_rr <= 0.0f)
        {
            retry = true;
            continue;
        }

        g   = c->cal_sum_rf / c->cal_sum_rr;      /* = P * ff_probe */
        mag = fabsf(g);

        if ((mag < WG_CAL_G_MIN) || (mag > WG_CAL_G_MAX))
        {
            retry = true;
            continue;
        }

        k  = c->gains.feedforward_gain / g;       /* = 1 / P */
        ff = WG_FF_NORM * k;
        mag = fabsf(ff);

        if ((mag < WG_FF_MIN) || (mag > WG_FF_MAX))
        {
            retry = true;
            continue;
        }

        /* Усі коефіцієнти множаться на 1/P: так нормовані значення з
         * конфігурації стають реальними тіками duty на відлік ADC, а знак
         * інверсного тракту врахований автоматично. */
        c->gains.feedforward_gain = ff;
        c->gains.kp           = WG_PI_KP_NORM * k;
        c->gains.ki           = WG_PI_KI_NORM * k;
        c->gains.kr           = WG_PR_KR_NORM * k;
        c->gains.wc           = WG_PR_WC_DEFAULT;
        c->gains.output_limit = WG_LOOP_OUT_LIMIT_TICKS;
        WgNormalizeGains(&c->gains);
        WgRecomputeGains(c);
        c->cal_ok = true;
    }

    if (retry && ((uint32_t)(g_boot_attempt + 1U) < WG_CAL_ATTEMPTS))
    {
        WgBootBeginProbe((uint8_t)(g_boot_attempt + 1U));
        return;
    }

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        if (g_ch[i].hw.feedback_enabled && !g_ch[i].cal_ok)
        {
            g_ch[i].cal_failed = true;
        }
    }

    WgBootApply();
}


void WgBootTick(void)
{
    switch (g_boot_state)
    {
        case WGB_SETTLE:
            for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
            {
                if (g_ch[i].hw.feedback_enabled)
                {
                    g_ch[i].cal_fb_acc += (float)g_ch[i].feedback_adc;
                    g_ch[i].cal_count++;
                }
            }

            if (++g_boot_ticks >= g_boot_limit)
            {
                for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
                {
                    WgChannel *c = &g_ch[i];

                    if (c->hw.feedback_enabled && (c->cal_count > 0U))
                    {
                        /* Код АЦП при нульовому сигналі - це і є справжній
                         * нуль каналу вимірювання, а не теоретичні 2048. */
                        c->cal.offset_adc =
                            clamp_i32((int32_t)(c->cal_fb_acc / (float)c->cal_count),
                                      0, WG_ADC_FULL_SCALE);
                    }
                }

                WgBootBeginProbe(0U);
            }
            break;

        case WGB_PROBE_SKIP:
            if (++g_boot_ticks >= g_boot_limit)
            {
                for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
                {
                    g_ch[i].cal_sum_rf = 0.0f;
                    g_ch[i].cal_sum_rr = 0.0f;
                }

                g_boot_ticks = 0U;
                g_boot_limit = WG_CAL_MEASURE_PERIODS * g_samples_per_period;
                g_boot_state = WGB_PROBE_MEAS;
            }
            break;

        case WGB_PROBE_MEAS:
            for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
            {
                WgChannel *c = &g_ch[i];
                float r;

                if (!c->hw.feedback_enabled || c->cal_ok)
                {
                    continue;
                }

                r = (float)c->last_ref_dev;
                c->cal_sum_rf += r * c->last_fb_dev;
                c->cal_sum_rr += r * r;
            }

            if (++g_boot_ticks >= g_boot_limit)
            {
                WgBootEvaluate();
            }
            break;

        case WGB_APPLY:
        case WGB_RUN:
        default:
            break;
    }
}
