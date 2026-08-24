/*
 * wave_generator_schedule.c
 *
 * Черга запланованих змін і лінійні профілі - основа для відтворення
 * перехідних процесів.
 *
 * Перехідний процес описується як послідовність записів «канал, момент,
 * тривалість, режим, набір параметрів». Записи зберігаються відсортованими
 * за моментом застосування, а WgApplyDueUpdates() вичерпує ВСІ, чий час
 * настав, ще до обробки каналів - завдяки цьому кілька записів з однаковим
 * моментом застосовуються в одному такті, тобто трифазне КЗ виникає
 * синхронно на всіх каналах.
 *
 * Наростання підтримують амплітуда, частота й фаза; гармоніки та аперіодична
 * складова застосовуються стрибком - саме це й потрібно для моменту
 * виникнення пошкодження.
 */


#include "wave_generator_internal.h"


/* ------------------------------------------------------------------------- */
/*                     Черга запланованих змін і профілі                     */
/* ------------------------------------------------------------------------- */

static void WgCaptureRampPoint(const WgChannel *c, WgRampPoint *p)
{
    p->amplitude_adc   = c->cfg.harmonic[0].amplitude_adc;
    p->freq_millihz    = c->cfg.fundamental_freq_millihz;
    p->phase_deg_milli = c->cfg.phase_deg_milli;
}


static void WgApplyRampPoint(WgChannel *c, const WgRampPoint *p, uint32_t mask)
{
    if ((mask & WAVE_GENERATOR_UPDATE_AMPLITUDE) != 0U)
    {
        c->cfg.harmonic[0].amplitude_adc = p->amplitude_adc;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_FREQUENCY) != 0U)
    {
        c->cfg.fundamental_freq_millihz = p->freq_millihz;
    }

    if ((mask & WAVE_GENERATOR_UPDATE_PHASE) != 0U)
    {
        c->cfg.phase_deg_milli = p->phase_deg_milli;
    }

    WgRecomputeDerived(c, mask);
}


static void WgStartProfile(WgChannel *c, const WaveGeneratorScheduledUpdate *u)
{
    uint32_t immediate = u->update_mask & ~WAVE_GENERATOR_RAMPABLE;
    uint32_t rampable  = u->update_mask & WAVE_GENERATOR_RAMPABLE;

    /* Усе, що не вміє наростати (гармоніки, аперіодична, режим контуру),
     * застосовується стрибком у момент apply_at_sample. */
    if (immediate != 0U)
    {
        WgApplyMaskedPatch(&c->cfg, immediate, &u->target_config);
        WgRecomputeDerived(c, immediate);

        if ((immediate & (WAVE_GENERATOR_UPDATE_APERIODIC_AMPLITUDE |
                          WAVE_GENERATOR_UPDATE_APERIODIC_TAU)) != 0U)
        {
            c->aperiodic_current_q = c->cfg.aperiodic_amplitude_adc << WG_APERIODIC_Q_SHIFT;
        }
    }

    if (rampable == 0U)
    {
        c->profile_active = false;
        return;
    }

    if ((u->mode == WAVE_GENERATOR_PROFILE_STEP) || (u->duration_samples == 0U))
    {
        WgApplyMaskedPatch(&c->cfg, rampable, &u->target_config);
        WgRecomputeDerived(c, rampable);
        c->profile_active = false;
        return;
    }

    WgCaptureRampPoint(c, &c->ramp_start);
    c->ramp_target = c->ramp_start;

    if ((rampable & WAVE_GENERATOR_UPDATE_AMPLITUDE) != 0U)
    {
        c->ramp_target.amplitude_adc = clamp_i32(u->target_config.harmonic[0].amplitude_adc,
                                                 -WG_ADC_HALF_RANGE, WG_ADC_HALF_RANGE);
    }

    if ((rampable & WAVE_GENERATOR_UPDATE_FREQUENCY) != 0U)
    {
        c->ramp_target.freq_millihz = clamp_u32(u->target_config.fundamental_freq_millihz,
                                                1U, 1000000UL);
    }

    if ((rampable & WAVE_GENERATOR_UPDATE_PHASE) != 0U)
    {
        c->ramp_target.phase_deg_milli = u->target_config.phase_deg_milli;
    }

    c->profile_active       = true;
    c->profile_mask         = rampable;
    c->profile_start_sample = u->apply_at_sample;
    c->profile_duration     = u->duration_samples;
}


static void WgInterpolateProfile(WgChannel *c)
{
    uint64_t elapsed64;
    uint32_t elapsed;
    WgRampPoint p;

    if (!c->profile_active)
    {
        return;
    }

    elapsed64 = g_rt.sample_counter - c->profile_start_sample;

    if (elapsed64 >= (uint64_t)c->profile_duration)
    {
        WgApplyRampPoint(c, &c->ramp_target, c->profile_mask);
        c->profile_active = false;
        return;
    }

    elapsed = (uint32_t)elapsed64;
    p.amplitude_adc   = lerp_i32(c->ramp_start.amplitude_adc,
                                 c->ramp_target.amplitude_adc,
                                 elapsed, c->profile_duration);
    p.freq_millihz    = lerp_u32(c->ramp_start.freq_millihz,
                                 c->ramp_target.freq_millihz,
                                 elapsed, c->profile_duration);
    p.phase_deg_milli = lerp_i32(c->ramp_start.phase_deg_milli,
                                 c->ramp_target.phase_deg_milli,
                                 elapsed, c->profile_duration);
    WgApplyRampPoint(c, &p, c->profile_mask);
}


static void WgDequeueFirst(void)
{
    if (g_queue_count == 0U)
    {
        return;
    }

    for (uint8_t i = 1U; i < g_queue_count; ++i)
    {
        g_queue[i - 1U] = g_queue[i];
    }

    g_queue_count--;
}


/* Внутрішня вставка без захоплення - викликається і з ISR (автостарт),
 * де переривання й так заборонені. */
bool WgQueueInsert(const WaveGeneratorScheduledUpdate *update)
{
    uint8_t insert;

    if (g_queue_count >= WG_UPDATE_QUEUE_CAPACITY)
    {
        return false;
    }

    insert = g_queue_count;
    while ((insert > 0U) && (g_queue[insert - 1U].apply_at_sample > update->apply_at_sample))
    {
        g_queue[insert] = g_queue[insert - 1U];
        insert--;
    }

    g_queue[insert] = *update;
    WgNormalizeChannelConfig(&g_queue[insert].target_config);
    g_queue_count++;
    return true;
}


void WgApplyDueUpdates(void)
{
    while ((g_queue_count > 0U) && (g_rt.sample_counter >= g_queue[0].apply_at_sample))
    {
        uint8_t ch = g_queue[0].channel;

        if (ch < WG_CHANNEL_COUNT)
        {
            WgStartProfile(&g_ch[ch], &g_queue[0]);
        }

        WgDequeueFirst();
    }

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        if (g_ch[i].profile_active)
        {
            WgInterpolateProfile(&g_ch[i]);
        }
    }
}


bool WaveGenerator_QueueScheduledUpdate(const WaveGeneratorScheduledUpdate *update)
{
    uint32_t key;
    bool ok;

    if ((update == NULL) || (update->update_mask == 0U) ||
        (update->channel >= WG_CHANNEL_COUNT))
    {
        return false;
    }

    key = WgLock();
    ok = WgQueueInsert(update);
    WgUnlock(key);
    return ok;
}


void WaveGenerator_ClearScheduledUpdates(void)
{
    uint32_t key = WgLock();

    g_queue_count = 0U;

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        g_ch[i].profile_active = false;
    }

    WgUnlock(key);
}
