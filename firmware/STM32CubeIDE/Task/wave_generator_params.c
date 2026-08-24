/*
 * wave_generator_params.c
 *
 * Перерахунок похідних величин конфігурації: крок фази, фазові зсуви,
 * коефіцієнт згасання аперіодичної, коефіцієнти регулятора, перебудова
 * списку активних гармонік.
 *
 * Усе це виконується ПОЗА гарячим шляхом - лише коли змінилась конфігурація.
 * Гарячий цикл користується вже готовими числами.
 */


#include "wave_generator_internal.h"


static uint32_t WgPhaseStep(uint32_t freq_millihz)
{
    return (uint32_t)(((uint64_t)freq_millihz << 32) / ((uint64_t)g_sample_rate_hz * 1000ULL));
}


static uint32_t WgPhaseBias(int32_t deg_milli)
{
    int64_t n = (int64_t)deg_milli % WG_DEGREES_PER_TURN_MILLI;

    if (n < 0)
    {
        n += WG_DEGREES_PER_TURN_MILLI;
    }

    return (uint32_t)(((uint64_t)n << 32) / (uint64_t)WG_DEGREES_PER_TURN_MILLI);
}


/* Коефіцієнт згасання аперіодичної складової за один такт контуру.
 * tau_ms == 0 трактується як "постійне зміщення без згасання" - так
 * зручніше для перевірок із DC-складовою. */
static int32_t WgDecayQ(uint32_t tau_ms)
{
    uint64_t tau_samples;
    uint64_t attenuation;

    if (tau_ms == 0U)
    {
        return (int32_t)WG_DECAY_Q_ONE;
    }

    tau_samples = ((uint64_t)tau_ms * (uint64_t)g_sample_rate_hz) / 1000ULL;
    if (tau_samples == 0ULL)
    {
        return 0;
    }

    attenuation = (uint64_t)WG_DECAY_Q_ONE / tau_samples;
    if (attenuation >= (uint64_t)WG_DECAY_Q_ONE)
    {
        return 0;
    }

    return (int32_t)((uint64_t)WG_DECAY_Q_ONE - attenuation);
}


void WgNormalizeChannelConfig(WaveGeneratorChannelConfig *c)
{
    c->fundamental_freq_millihz = clamp_u32(c->fundamental_freq_millihz, 1U, 1000000UL);
    c->aperiodic_amplitude_adc  = clamp_i32(c->aperiodic_amplitude_adc,
                                            -WG_ADC_HALF_RANGE, WG_ADC_HALF_RANGE);
    c->aperiodic_tau_ms         = clamp_u32(c->aperiodic_tau_ms, 0U, 600000U);

    for (uint32_t i = 0U; i < WG_HARMONIC_COUNT; ++i)
    {
        c->harmonic[i].amplitude_adc = clamp_i32(c->harmonic[i].amplitude_adc,
                                                 -WG_ADC_HALF_RANGE, WG_ADC_HALF_RANGE);
    }
}


void WgNormalizeGains(WaveGeneratorLoopGains *g)
{
    g->feedforward_gain = clamp_f(g->feedforward_gain, -1000.0f, 1000.0f);
    g->kp               = clamp_f(g->kp, -1000.0f, 1000.0f);
    g->ki               = clamp_f(g->ki, -1.0e6f, 1.0e6f);
    g->kr               = clamp_f(g->kr, -1000.0f, 1000.0f);
    /* wc обмежене зверху умовою стійкості явної схеми: wc*Ts << 1 */
    g->wc               = clamp_f(g->wc, 0.0f, 0.05f / g_sample_period_s);
    g->output_limit     = clamp_f(g->output_limit, 0.0f, (float)WG_PWM_PERIOD_TICKS);
}


/* ------------------------------------------------------------------------- */
/*                          Перерахунок похідних величин                     */
/* ------------------------------------------------------------------------- */

static void WgRebuildHarmonics(WgChannel *c)
{
    uint8_t n = 0U;

    for (uint32_t i = 0U; i < WG_HARMONIC_COUNT; ++i)
    {
        int32_t amp = c->cfg.harmonic[i].amplitude_adc;

        if (amp == 0)
        {
            continue;
        }

        c->active[n].order = (uint8_t)(i + 1U);
        c->active[n].amp   = (int16_t)amp;
        /* Фаза гармоніки складається з фази каналу (помноженої на порядок,
         * бо зсув осі часу зсуває k-ту гармоніку на k*phi) і власної фази
         * гармоніки відносно основної. */
        c->active[n].bias  = (c->phase_bias * (uint32_t)(i + 1U)) +
                             WgPhaseBias(c->cfg.harmonic[i].phase_deg_milli);
        n++;
    }

    c->active_count = n;
}


static void WgRecomputeFrequency(WgChannel *c)
{
    float w0;

    c->phase_step = WgPhaseStep(c->cfg.fundamental_freq_millihz);

    w0        = 2.0f * (float)M_PI * ((float)c->cfg.fundamental_freq_millihz / 1000.0f);
    c->res_b  = g_sample_period_s * w0;
}


void WgRecomputeGains(WgChannel *c)
{
    c->ki_ts        = c->gains.ki * g_sample_period_s;
    c->res_c        = g_sample_period_s * c->gains.wc;
    c->res_out_gain = 2.0f * c->gains.wc * c->gains.kr;
}


void WgRecomputeDerived(WgChannel *c, uint32_t mask)
{
    if ((mask & WAVE_GENERATOR_UPDATE_FREQUENCY) != 0U)
    {
        WgRecomputeFrequency(c);
    }

    if ((mask & WAVE_GENERATOR_UPDATE_PHASE) != 0U)
    {
        c->phase_bias = WgPhaseBias(c->cfg.phase_deg_milli);
    }

    if ((mask & (WAVE_GENERATOR_UPDATE_AMPLITUDE |
                 WAVE_GENERATOR_UPDATE_HARMONICS |
                 WAVE_GENERATOR_UPDATE_PHASE)) != 0U)
    {
        WgRebuildHarmonics(c);
    }

    if ((mask & WAVE_GENERATOR_UPDATE_APERIODIC_TAU) != 0U)
    {
        c->aperiodic_decay_q = WgDecayQ(c->cfg.aperiodic_tau_ms);
    }

    if ((mask & WAVE_GENERATOR_UPDATE_GAINS) != 0U)
    {
        WgRecomputeGains(c);
    }
}


void WgApplyMaskedPatch(WaveGeneratorChannelConfig *base,
                               uint32_t mask,
                               const WaveGeneratorChannelConfig *patch)
{
    if ((mask & WAVE_GENERATOR_UPDATE_AMPLITUDE) != 0U)
    {
        base->harmonic[0].amplitude_adc = patch->harmonic[0].amplitude_adc;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_FREQUENCY) != 0U)
    {
        base->fundamental_freq_millihz = patch->fundamental_freq_millihz;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_PHASE) != 0U)
    {
        base->phase_deg_milli = patch->phase_deg_milli;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_APERIODIC_AMPLITUDE) != 0U)
    {
        base->aperiodic_amplitude_adc = patch->aperiodic_amplitude_adc;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_APERIODIC_TAU) != 0U)
    {
        base->aperiodic_tau_ms = patch->aperiodic_tau_ms;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_HARMONICS) != 0U)
    {
        memcpy(base->harmonic, patch->harmonic, sizeof(base->harmonic));
    }

    if ((mask & WAVE_GENERATOR_UPDATE_LOOP_MODE) != 0U)
    {
        base->closed_loop = patch->closed_loop;
    }

    WgNormalizeChannelConfig(base);
}
