/*
 * wave_generator_api.c
 *
 * Публічне API модуля: гетери й сетери, якими користується рівень зв'язку
 * (зараз uart_task, у майбутньому - канал передачі даних).
 *
 * Усі функції захоплюють короткий критичний інтервал, бо змінюють стан, який
 * читає переривання контуру.
 */


#include "wave_generator_internal.h"


uint32_t WaveGenerator_GetSampleRateHz(void)
{
    return g_sample_rate_hz;
}


bool WaveGenerator_GetChannelConfig(uint8_t channel, WaveGeneratorChannelConfig *config)
{
    uint32_t key;

    if ((channel >= WG_CHANNEL_COUNT) || (config == NULL))
    {
        return false;
    }

    key = WgLock();
    *config = g_ch[channel].cfg;
    WgUnlock(key);
    return true;
}


bool WaveGenerator_UpdateChannelConfig(uint8_t channel,
                                       uint32_t update_mask,
                                       const WaveGeneratorChannelConfig *patch)
{
    uint32_t key;
    WgChannel *c;

    if ((channel >= WG_CHANNEL_COUNT) || (patch == NULL) || (update_mask == 0U))
    {
        return false;
    }

    c = &g_ch[channel];

    key = WgLock();
    c->profile_active = false;
    WgApplyMaskedPatch(&c->cfg, update_mask, patch);
    WgRecomputeDerived(c, update_mask);

    /* Зміна аперіодичної складової перезапускає її з нового значення -
     * інакше вона б продовжила згасати зі старого. */
    if (((update_mask & (WAVE_GENERATOR_UPDATE_APERIODIC_AMPLITUDE |
                         WAVE_GENERATOR_UPDATE_APERIODIC_TAU)) != 0U) &&
        g_rt.signal_active)
    {
        c->aperiodic_current_q = c->cfg.aperiodic_amplitude_adc << WG_APERIODIC_Q_SHIFT;
    }

    /* Повернення в замкнений контур скидає накопичений стан регулятора. */
    if ((update_mask & WAVE_GENERATOR_UPDATE_LOOP_MODE) != 0U)
    {
        c->integrator     = 0.0f;
        c->res_xa         = 0.0f;
        c->res_xb         = 0.0f;
        c->feedback_fault = false;
        c->fault_counter  = 0U;
    }

    WgUnlock(key);
    return true;
}


bool WaveGenerator_GetLoopGains(uint8_t channel, WaveGeneratorLoopGains *gains)
{
    uint32_t key;

    if ((channel >= WG_CHANNEL_COUNT) || (gains == NULL))
    {
        return false;
    }

    key = WgLock();
    *gains = g_ch[channel].gains;
    WgUnlock(key);
    return true;
}


bool WaveGenerator_SetLoopGains(uint8_t channel, const WaveGeneratorLoopGains *gains)
{
    uint32_t key;

    if ((channel >= WG_CHANNEL_COUNT) || (gains == NULL))
    {
        return false;
    }

    key = WgLock();
    g_ch[channel].gains = *gains;
    WgNormalizeGains(&g_ch[channel].gains);
    WgRecomputeGains(&g_ch[channel]);
    WgUnlock(key);
    return true;
}


bool WaveGenerator_GetFeedbackCal(uint8_t channel, WaveGeneratorFeedbackCal *cal)
{
    uint32_t key;

    if ((channel >= WG_CHANNEL_COUNT) || (cal == NULL))
    {
        return false;
    }

    key = WgLock();
    *cal = g_ch[channel].cal;
    WgUnlock(key);
    return true;
}


bool WaveGenerator_SetFeedbackCal(uint8_t channel, const WaveGeneratorFeedbackCal *cal)
{
    uint32_t key;

    if ((channel >= WG_CHANNEL_COUNT) || (cal == NULL))
    {
        return false;
    }

    key = WgLock();
    g_ch[channel].cal.offset_adc = clamp_i32(cal->offset_adc, 0, WG_ADC_FULL_SCALE);
    g_ch[channel].cal.scale      = clamp_f(cal->scale, -100.0f, 100.0f);
    WgUnlock(key);
    return true;
}


void WaveGenerator_StartNow(void)
{
    uint32_t key = WgLock();

    g_rt.signal_active       = false;
    g_rt.start_armed         = true;
    g_rt.signal_start_sample = g_rt.sample_counter + 1ULL;

    WgUnlock(key);
}


void WaveGenerator_ArmStartAtSample(uint64_t start_sample)
{
    uint32_t key = WgLock();

    g_rt.signal_active       = false;
    g_rt.start_armed         = true;
    g_rt.signal_start_sample = start_sample;

    WgUnlock(key);
}


void WaveGenerator_Stop(void)
{
    uint32_t key = WgLock();

    g_rt.signal_active          = false;
    g_rt.start_armed            = false;
    g_rt.signal_elapsed_samples = 0ULL;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        g_ch[i].profile_active = false;
        WgSetChannelIdle(&g_ch[i]);
    }

    WgUnlock(key);
}


void WaveGenerator_GetRuntimeState(WaveGeneratorRuntimeState *state)
{
    uint32_t key;

    if (state == NULL)
    {
        return;
    }

    key = WgLock();

    state->initialized            = g_rt.initialized;
    state->outputs_running        = g_rt.outputs_running;
    state->signal_active          = g_rt.signal_active;
    state->start_armed            = g_rt.start_armed;
    state->overrun                = g_rt.overrun;
    state->sample_counter         = g_rt.sample_counter;
    state->signal_start_sample    = g_rt.signal_start_sample;
    state->signal_elapsed_samples = g_rt.signal_elapsed_samples;
    state->sample_rate_hz         = g_sample_rate_hz;
    state->pending_updates        = g_queue_count;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        state->channel[i].closed_loop        = g_ch[i].loop_running;
        state->channel[i].feedback_fault     = g_ch[i].feedback_fault;
        state->channel[i].calibrated         = g_ch[i].cal_ok;
        state->channel[i].calibration_failed = g_ch[i].cal_failed;
        state->channel[i].amplitude_limited  = g_ch[i].cal_amp_limited;
        state->channel[i].amplitude_max_adc  = g_ch[i].cal_amp_max;
        state->channel[i].profile_active    = g_ch[i].profile_active;
        state->channel[i].reference_adc     = g_ch[i].reference_adc;
        state->channel[i].feedback_adc      = g_ch[i].feedback_adc;
        state->channel[i].feedback_peak_adc = g_ch[i].fb_peak_hold;
        state->channel[i].compare_ticks     = g_ch[i].compare_ticks;
        state->channel[i].error_adc         = g_ch[i].error_adc;
    }

    WgUnlock(key);
}


bool WaveGenerator_GetSampleCounter(uint64_t *sample_counter)
{
    uint32_t key;

    if (sample_counter == NULL)
    {
        return false;
    }

    key = WgLock();
    *sample_counter = g_rt.sample_counter;
    WgUnlock(key);
    return true;
}


/* Завантаження контуру у проміле від періоду - головна діагностика для
 * відповіді на питання "чи встигає ядро". Вимірюється лічильником master. */
uint32_t WaveGenerator_GetLoadPermille(void)
{
    uint32_t budget = SystemCoreClock / g_sample_rate_hz;

    if (budget == 0U)
    {
        return 0U;
    }

    return (g_rt.isr_cycles_max * 1000U) / budget;
}


void WaveGenerator_ResetLoadMeasurement(void)
{
    g_rt.isr_cycles_max = 0U;
    g_rt.overrun = false;
}
