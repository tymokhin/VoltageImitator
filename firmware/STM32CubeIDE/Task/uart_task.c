/*
 * uart_task.c
 *
 * Текстовий протокол керування генератором через USART3, 115200 8N1.
 * Рядки закінчуються \n (\r ігнорується).
 *
 * Команди:
 *
 *   HELP
 *   GET STATE [CH=n]        стан контуру (усі канали або один)
 *   GET CONFIG CH=n         поточне завдання каналу
 *   GET GAINS  CH=n         коефіцієнти регулятора
 *   GET CAL    CH=n         калібрування каналу вимірювання
 *   GET TIME                поточний такт контуру
 *
 *   SET CH=n [AMP=..] [FREQ=..] [PHASE=..] [APER=..] [TAU=..] [LOOP=0|1]
 *            [H2=..] [H2PH=..] ... [H10=..] [H10PH=..]
 *       AMP, H<k>  - амплітуда у відліках ADC (0..2047); H1 = AMP
 *       H<k>PH     - фаза k-ї гармоніки відносно основної, тисячні градуса
 *       FREQ       - тисячні герца (50000 = 50.000 Hz)
 *       PHASE      - фаза каналу, тисячні градуса
 *       APER       - початкова аперіодична складова, відліки ADC
 *       TAU        - постійна часу аперіодичної, мс (0 = постійне зміщення)
 *       LOOP       - 1 замкнений контур, 0 тільки feedforward
 *
 *   GAINS CH=n [FF=..] [KP=..] [KI=..] [KR=..] [WC=..] [LIM=..]
 *       FF/KP/KI/KR/WC у тисячних (KP=500 -> 0.5), LIM у тіках duty
 *
 *   CAL CH=n [OFFS=..] [SCALE=..]
 *       OFFS у відліках ADC, SCALE у тисячних (1000 -> 1.0, від'ємне = інверсія)
 *
 *   SCHEDULE CH=n [AT=|ATMS=] [DUR=|DURMS=] [MODE=STEP|LINEAR] <поля як у SET>
 *       наростання підтримують лише AMP, FREQ, PHASE; решта - стрибком
 *
 *   START [NOW | AT=<такт> | ATMS=<мс>]
 *   STOP
 *   CLEAR SCHEDULE
 *   RESET LOAD              скинути вимірювання завантаження контуру
 */

#include "uart_task.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include "wave_generator_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_RX_FRAME_SIZE      96U
#define UART_STREAM_SIZE        256U
#define UART_LINE_SIZE          192U
#define UART_TASK_STACK_WORDS   384U
#define UART_TASK_PRIORITY      2U

static UartControlHardwareConfig g_uart_hw = {0};
static bool g_uart_initialized = false;
static uint8_t g_rx_frame[UART_RX_FRAME_SIZE] = {0};
static uint8_t g_stream_storage[UART_STREAM_SIZE] = {0};
static StaticStreamBuffer_t g_stream_struct;
static StreamBufferHandle_t g_stream = NULL;
static StaticTask_t g_task_tcb;
static StackType_t g_task_stack[UART_TASK_STACK_WORDS] = {0};

/* Робочі буфери задачі - статичні, щоб не роздувати її стек. */
static char g_line[UART_LINE_SIZE];
static char g_response[192];

/* ------------------------------------------------------------------------- */
/*                              Дрібні хелпери                               */
/* ------------------------------------------------------------------------- */

static bool UartControl_StartReception(void)
{
    return (g_uart_hw.uart != NULL) &&
           (HAL_UARTEx_ReceiveToIdle_IT(g_uart_hw.uart, g_rx_frame, sizeof(g_rx_frame)) == HAL_OK);
}

static char UartControl_Upper(char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char)(c - ('a' - 'A')) : c;
}

static bool UartControl_Equals(const char *left, const char *right)
{
    while ((*left != '\0') && (*right != '\0'))
    {
        if (UartControl_Upper(*left) != UartControl_Upper(*right))
        {
            return false;
        }
        left++;
        right++;
    }

    return (*left == '\0') && (*right == '\0');
}

static void UartControl_Send(const char *text)
{
    if ((g_uart_hw.uart != NULL) && (text != NULL))
    {
        (void)HAL_UART_Transmit(g_uart_hw.uart, (uint8_t *)text, (uint16_t)strlen(text), 100U);
    }
}

static uint64_t UartControl_MsToSamples(uint32_t ms)
{
    return ((uint64_t)ms * (uint64_t)WaveGenerator_GetSampleRateHz()) / 1000ULL;
}

static bool UartControl_ParseI32(const char *text, int32_t *value)
{
    char *end;
    long parsed;

    if ((text == NULL) || (*text == '\0'))
    {
        return false;
    }

    parsed = strtol(text, &end, 10);
    if (*end != '\0')
    {
        return false;
    }

    *value = (int32_t)parsed;
    return true;
}

static bool UartControl_ParseU32(const char *text, uint32_t *value)
{
    int32_t signed_value;

    if (!UartControl_ParseI32(text, &signed_value) || (signed_value < 0))
    {
        return false;
    }

    *value = (uint32_t)signed_value;
    return true;
}

static bool UartControl_ParseU64(const char *text, uint64_t *value)
{
    char *end;
    unsigned long long parsed;

    if ((text == NULL) || (*text == '\0'))
    {
        return false;
    }

    parsed = strtoull(text, &end, 10);
    if (*end != '\0')
    {
        return false;
    }

    *value = (uint64_t)parsed;
    return true;
}

static bool UartControl_SplitKeyValue(char *token, char **key, char **value)
{
    char *separator = (token != NULL) ? strchr(token, '=') : NULL;

    if (separator == NULL)
    {
        return false;
    }

    *separator = '\0';
    *key = token;
    *value = separator + 1;
    return true;
}

#define UART_MAX_TOKENS 28U

static char *g_keys[UART_MAX_TOKENS];
static char *g_vals[UART_MAX_TOKENS];

/* Розбирає решту рядка на пари key=value ОДНИМ проходом.
 * Один прохід принциповий: strtok_r і SplitKeyValue руйнують рядок,
 * замінюючи роздільники та '=' на '\0', тому пройтись по ньому вдруге
 * вже неможливо. Повертає кількість пар або -1 при помилці. */
static int UartControl_Tokenize(char *context)
{
    char *saved = context;
    char *token;
    int count = 0;

    while ((token = strtok_r(NULL, " \t", &saved)) != NULL)
    {
        if ((uint32_t)count >= UART_MAX_TOKENS)
        {
            return -1;
        }

        if (!UartControl_SplitKeyValue(token, &g_keys[count], &g_vals[count]))
        {
            return -1;
        }

        count++;
    }

    return count;
}

/* Знаходить значення CH= серед розібраних токенів. */
static bool UartControl_FindChannel(int count, uint8_t *channel)
{
    for (int i = 0; i < count; ++i)
    {
        if (UartControl_Equals(g_keys[i], "CH"))
        {
            int32_t value;

            if (!UartControl_ParseI32(g_vals[i], &value) ||
                (value < 0) || (value >= (int32_t)WG_CHANNEL_COUNT))
            {
                return false;
            }

            *channel = (uint8_t)value;
            return true;
        }
    }

    return false;
}

/* Розбір ключа виду H<k> або H<k>PH. Повертає true, якщо ключ гармонічний. */
static bool UartControl_ParseHarmonicKey(const char *key, uint32_t *order, bool *is_phase)
{
    uint32_t value = 0U;
    const char *p = key;

    if (UartControl_Upper(*p) != 'H')
    {
        return false;
    }

    p++;
    if ((*p < '0') || (*p > '9'))
    {
        return false;
    }

    while ((*p >= '0') && (*p <= '9'))
    {
        value = (value * 10U) + (uint32_t)(*p - '0');
        p++;
    }

    if (*p == '\0')
    {
        *is_phase = false;
    }
    else if (UartControl_Equals(p, "PH"))
    {
        *is_phase = true;
    }
    else
    {
        return false;
    }

    if ((value < 1U) || (value > WG_HARMONIC_MAX_ORDER))
    {
        return false;
    }

    *order = value;
    return true;
}

/* ------------------------------------------------------------------------- */
/*                     Розбір полів конфігурації каналу                      */
/* ------------------------------------------------------------------------- */

/* Повертає false, якщо ключ не належить до конфігурації каналу (щоб
 * викликач міг спробувати розібрати його як свій - AT, DUR, MODE...). */
static bool UartControl_ApplyConfigField(const char *key,
                                         const char *value,
                                         WaveGeneratorChannelConfig *patch,
                                         uint32_t *mask,
                                         bool *parse_error)
{
    int32_t signed_value;
    uint32_t unsigned_value;
    uint32_t order;
    bool is_phase;

    if (UartControl_ParseHarmonicKey(key, &order, &is_phase))
    {
        if (!UartControl_ParseI32(value, &signed_value))
        {
            *parse_error = true;
            return true;
        }

        if (is_phase)
        {
            patch->harmonic[order - 1U].phase_deg_milli = signed_value;
        }
        else
        {
            patch->harmonic[order - 1U].amplitude_adc = signed_value;
        }

        /* Змінюючи будь-яку гармоніку, передаємо увесь масив: він у патчі
         * попередньо заповнений поточним станом каналу. */
        *mask |= WAVE_GENERATOR_UPDATE_HARMONICS;
        if (order == 1U)
        {
            *mask |= WAVE_GENERATOR_UPDATE_AMPLITUDE;
        }
        return true;
    }

    if (UartControl_Equals(key, "AMP"))
    {
        if (!UartControl_ParseI32(value, &signed_value)) { *parse_error = true; return true; }
        patch->harmonic[0].amplitude_adc = signed_value;
        *mask |= WAVE_GENERATOR_UPDATE_AMPLITUDE | WAVE_GENERATOR_UPDATE_HARMONICS;
        return true;
    }

    if (UartControl_Equals(key, "FREQ"))
    {
        if (!UartControl_ParseU32(value, &unsigned_value)) { *parse_error = true; return true; }
        patch->fundamental_freq_millihz = unsigned_value;
        *mask |= WAVE_GENERATOR_UPDATE_FREQUENCY;
        return true;
    }

    if (UartControl_Equals(key, "PHASE"))
    {
        if (!UartControl_ParseI32(value, &signed_value)) { *parse_error = true; return true; }
        patch->phase_deg_milli = signed_value;
        *mask |= WAVE_GENERATOR_UPDATE_PHASE;
        return true;
    }

    if (UartControl_Equals(key, "APER"))
    {
        if (!UartControl_ParseI32(value, &signed_value)) { *parse_error = true; return true; }
        patch->aperiodic_amplitude_adc = signed_value;
        *mask |= WAVE_GENERATOR_UPDATE_APERIODIC_AMPLITUDE;
        return true;
    }

    if (UartControl_Equals(key, "TAU"))
    {
        if (!UartControl_ParseU32(value, &unsigned_value)) { *parse_error = true; return true; }
        patch->aperiodic_tau_ms = unsigned_value;
        *mask |= WAVE_GENERATOR_UPDATE_APERIODIC_TAU;
        return true;
    }

    if (UartControl_Equals(key, "LOOP"))
    {
        if (!UartControl_ParseI32(value, &signed_value)) { *parse_error = true; return true; }
        patch->closed_loop = (signed_value != 0);
        *mask |= WAVE_GENERATOR_UPDATE_LOOP_MODE;
        return true;
    }

    return false;
}

/* ------------------------------------------------------------------------- */
/*                                 Звіти                                     */
/* ------------------------------------------------------------------------- */

static void UartControl_ReportConfig(uint8_t channel)
{
    WaveGeneratorChannelConfig cfg;
    size_t used;

    if (!WaveGenerator_GetChannelConfig(channel, &cfg))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    (void)snprintf(g_response, sizeof(g_response),
                   "CFG CH=%u FREQ=%lu PHASE=%ld APER=%ld TAU=%lu LOOP=%u\r\n",
                   (unsigned)channel,
                   (unsigned long)cfg.fundamental_freq_millihz,
                   (long)cfg.phase_deg_milli,
                   (long)cfg.aperiodic_amplitude_adc,
                   (unsigned long)cfg.aperiodic_tau_ms,
                   cfg.closed_loop ? 1U : 0U);
    UartControl_Send(g_response);

    /* Перелік ненульових гармонік у форматі H<порядок>=<амплітуда>/<фаза>. */
    used = (size_t)snprintf(g_response, sizeof(g_response), "HRM CH=%u", (unsigned)channel);

    for (uint32_t i = 0U; i < WG_HARMONIC_COUNT; ++i)
    {
        int written;

        if (cfg.harmonic[i].amplitude_adc == 0)
        {
            continue;
        }

        /* Лишаємо запас на "\r\n\0"; при переповненні рядок обривається. */
        if (used + 32U >= sizeof(g_response))
        {
            break;
        }

        written = snprintf(&g_response[used], sizeof(g_response) - used,
                           " H%lu=%ld/%ld",
                           (unsigned long)(i + 1U),
                           (long)cfg.harmonic[i].amplitude_adc,
                           (long)cfg.harmonic[i].phase_deg_milli);

        if (written <= 0)
        {
            break;
        }

        used += (size_t)written;
    }

    if (used > (sizeof(g_response) - 3U))
    {
        used = sizeof(g_response) - 3U;
    }

    g_response[used]      = '\r';
    g_response[used + 1U] = '\n';
    g_response[used + 2U] = '\0';
    UartControl_Send(g_response);
}

static void UartControl_ReportGains(uint8_t channel)
{
    WaveGeneratorLoopGains g;

    if (!WaveGenerator_GetLoopGains(channel, &g))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    (void)snprintf(g_response, sizeof(g_response),
                   "GAINS CH=%u FF=%ld KP=%ld KI=%ld KR=%ld WC=%ld LIM=%ld\r\n",
                   (unsigned)channel,
                   (long)(g.feedforward_gain * 1000.0f),
                   (long)(g.kp * 1000.0f),
                   (long)(g.ki * 1000.0f),
                   (long)(g.kr * 1000.0f),
                   (long)(g.wc * 1000.0f),
                   (long)g.output_limit);
    UartControl_Send(g_response);
}

static void UartControl_ReportCal(uint8_t channel)
{
    WaveGeneratorFeedbackCal cal;

    if (!WaveGenerator_GetFeedbackCal(channel, &cal))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    (void)snprintf(g_response, sizeof(g_response),
                   "CAL CH=%u OFFS=%ld SCALE=%ld\r\n",
                   (unsigned)channel,
                   (long)cal.offset_adc,
                   (long)(cal.scale * 1000.0f));
    UartControl_Send(g_response);
}

static void UartControl_ReportState(int32_t only_channel)
{
    WaveGeneratorRuntimeState st;

    WaveGenerator_GetRuntimeState(&st);

    (void)snprintf(g_response, sizeof(g_response),
                   "STATE INIT=%u RUN=%u ACT=%u ARMED=%u OVR=%u FS=%lu SMP=%llu Q=%u LOAD=%lu\r\n",
                   st.initialized ? 1U : 0U,
                   st.outputs_running ? 1U : 0U,
                   st.signal_active ? 1U : 0U,
                   st.start_armed ? 1U : 0U,
                   st.overrun ? 1U : 0U,
                   (unsigned long)st.sample_rate_hz,
                   (unsigned long long)st.sample_counter,
                   st.pending_updates,
                   (unsigned long)WaveGenerator_GetLoadPermille());
    UartControl_Send(g_response);

    for (uint32_t i = 0U; i < WG_CHANNEL_COUNT; ++i)
    {
        if ((only_channel >= 0) && ((uint32_t)only_channel != i))
        {
            continue;
        }

        (void)snprintf(g_response, sizeof(g_response),
                       "CH%lu LOOP=%u CAL=%u FLT=%u PROF=%u REF=%u FB=%u FBPK=%u CMP=%u ERR=%d AMAX=%ld%s\r\n",
                       (unsigned long)i,
                       st.channel[i].closed_loop ? 1U : 0U,
                       st.channel[i].calibrated ? 1U : (st.channel[i].calibration_failed ? 2U : 0U),
                       st.channel[i].feedback_fault ? 1U : 0U,
                       st.channel[i].profile_active ? 1U : 0U,
                       st.channel[i].reference_adc,
                       st.channel[i].feedback_adc,
                       st.channel[i].feedback_peak_adc,
                       st.channel[i].compare_ticks,
                       st.channel[i].error_adc,
                       (long)st.channel[i].amplitude_max_adc,
                       st.channel[i].amplitude_limited ? "!" : "");
        UartControl_Send(g_response);
    }
}

/* ------------------------------------------------------------------------- */
/*                              Обробники команд                             */
/* ------------------------------------------------------------------------- */

static void UartControl_HandleSet(char *context)
{
    WaveGeneratorChannelConfig patch;
    uint32_t mask = 0U;
    uint8_t channel;
    int count = UartControl_Tokenize(context);
    bool parse_error = false;

    if (count < 0)
    {
        UartControl_Send("ERR ARG\r\n");
        return;
    }

    /* Патч заповнюється поточним станом каналу: масив гармонік передається
     * цілком, тому він має бути актуальним, а не порожнім. */
    if (!UartControl_FindChannel(count, &channel) ||
        !WaveGenerator_GetChannelConfig(channel, &patch))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    for (int i = 0; i < count; ++i)
    {
        if (UartControl_Equals(g_keys[i], "CH"))
        {
            continue;
        }

        if (!UartControl_ApplyConfigField(g_keys[i], g_vals[i], &patch, &mask, &parse_error) ||
            parse_error)
        {
            UartControl_Send("ERR ARG\r\n");
            return;
        }
    }

    if ((mask == 0U) || !WaveGenerator_UpdateChannelConfig(channel, mask, &patch))
    {
        UartControl_Send("ERR SET\r\n");
        return;
    }

    UartControl_Send("OK SET\r\n");
}

static void UartControl_HandleGains(char *context)
{
    WaveGeneratorLoopGains gains;
    uint8_t channel;
    int count = UartControl_Tokenize(context);

    if (count < 0)
    {
        UartControl_Send("ERR ARG\r\n");
        return;
    }

    if (!UartControl_FindChannel(count, &channel) ||
        !WaveGenerator_GetLoopGains(channel, &gains))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    for (int i = 0; i < count; ++i)
    {
        int32_t milli;

        if (UartControl_Equals(g_keys[i], "CH"))
        {
            continue;
        }

        if (!UartControl_ParseI32(g_vals[i], &milli))
        {
            UartControl_Send("ERR ARG\r\n");
            return;
        }

        if      (UartControl_Equals(g_keys[i], "FF"))  { gains.feedforward_gain = (float)milli / 1000.0f; }
        else if (UartControl_Equals(g_keys[i], "KP"))  { gains.kp               = (float)milli / 1000.0f; }
        else if (UartControl_Equals(g_keys[i], "KI"))  { gains.ki               = (float)milli / 1000.0f; }
        else if (UartControl_Equals(g_keys[i], "KR"))  { gains.kr               = (float)milli / 1000.0f; }
        else if (UartControl_Equals(g_keys[i], "WC"))  { gains.wc               = (float)milli / 1000.0f; }
        else if (UartControl_Equals(g_keys[i], "LIM")) { gains.output_limit     = (float)milli; }
        else
        {
            UartControl_Send("ERR ARG\r\n");
            return;
        }
    }

    if (!WaveGenerator_SetLoopGains(channel, &gains))
    {
        UartControl_Send("ERR GAINS\r\n");
        return;
    }

    UartControl_Send("OK GAINS\r\n");
}

static void UartControl_HandleCal(char *context)
{
    WaveGeneratorFeedbackCal cal;
    uint8_t channel;
    int count = UartControl_Tokenize(context);

    if (count < 0)
    {
        UartControl_Send("ERR ARG\r\n");
        return;
    }

    if (!UartControl_FindChannel(count, &channel) ||
        !WaveGenerator_GetFeedbackCal(channel, &cal))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    for (int i = 0; i < count; ++i)
    {
        int32_t number;

        if (UartControl_Equals(g_keys[i], "CH"))
        {
            continue;
        }

        if (!UartControl_ParseI32(g_vals[i], &number))
        {
            UartControl_Send("ERR ARG\r\n");
            return;
        }

        if      (UartControl_Equals(g_keys[i], "OFFS"))  { cal.offset_adc = number; }
        else if (UartControl_Equals(g_keys[i], "SCALE")) { cal.scale = (float)number / 1000.0f; }
        else
        {
            UartControl_Send("ERR ARG\r\n");
            return;
        }
    }

    if (!WaveGenerator_SetFeedbackCal(channel, &cal))
    {
        UartControl_Send("ERR CAL\r\n");
        return;
    }

    UartControl_Send("OK CAL\r\n");
}

static void UartControl_HandleSchedule(char *context)
{
    WaveGeneratorScheduledUpdate update;
    uint8_t channel;
    uint32_t milliseconds;
    int count = UartControl_Tokenize(context);
    bool parse_error = false;

    if (count < 0)
    {
        UartControl_Send("ERR ARG\r\n");
        return;
    }

    memset(&update, 0, sizeof(update));
    update.mode = WAVE_GENERATOR_PROFILE_STEP;

    if (!UartControl_FindChannel(count, &channel) ||
        !WaveGenerator_GetChannelConfig(channel, &update.target_config))
    {
        UartControl_Send("ERR CH\r\n");
        return;
    }

    update.channel = channel;

    for (int i = 0; i < count; ++i)
    {
        const char *key = g_keys[i];
        const char *value = g_vals[i];

        if (UartControl_Equals(key, "CH"))
        {
            continue;
        }

        if (UartControl_Equals(key, "AT"))
        {
            if (!UartControl_ParseU64(value, &update.apply_at_sample)) { parse_error = true; }
            continue;
        }

        if (UartControl_Equals(key, "ATMS"))
        {
            uint64_t now = 0ULL;

            if (!UartControl_ParseU32(value, &milliseconds)) { parse_error = true; continue; }
            (void)WaveGenerator_GetSampleCounter(&now);
            update.apply_at_sample = now + UartControl_MsToSamples(milliseconds);
            continue;
        }

        if (UartControl_Equals(key, "DUR"))
        {
            if (!UartControl_ParseU32(value, &update.duration_samples)) { parse_error = true; }
            continue;
        }

        if (UartControl_Equals(key, "DURMS"))
        {
            if (!UartControl_ParseU32(value, &milliseconds)) { parse_error = true; continue; }
            update.duration_samples = (uint32_t)UartControl_MsToSamples(milliseconds);
            continue;
        }

        if (UartControl_Equals(key, "MODE"))
        {
            if (UartControl_Equals(value, "STEP"))        { update.mode = WAVE_GENERATOR_PROFILE_STEP; }
            else if (UartControl_Equals(value, "LINEAR")) { update.mode = WAVE_GENERATOR_PROFILE_LINEAR; }
            else                                          { parse_error = true; }
            continue;
        }

        if (!UartControl_ApplyConfigField(key, value, &update.target_config,
                                          &update.update_mask, &parse_error))
        {
            parse_error = true;
        }

        if (parse_error)
        {
            break;
        }
    }

    if (parse_error || (update.update_mask == 0U) ||
        !WaveGenerator_QueueScheduledUpdate(&update))
    {
        UartControl_Send("ERR SCHEDULE\r\n");
        return;
    }

    UartControl_Send("OK SCHEDULE\r\n");
}

static void UartControl_HandleStart(char *context)
{
    char *saved = context;
    char *token = strtok_r(NULL, " \t", &saved);
    char *key;
    char *value;
    uint64_t sample;
    uint32_t milliseconds;

    if ((token == NULL) || UartControl_Equals(token, "NOW"))
    {
        WaveGenerator_StartNow();
        UartControl_Send("OK START\r\n");
        return;
    }

    if (UartControl_SplitKeyValue(token, &key, &value))
    {
        if (UartControl_Equals(key, "AT") && UartControl_ParseU64(value, &sample))
        {
            WaveGenerator_ArmStartAtSample(sample);
            UartControl_Send("OK START\r\n");
            return;
        }

        if (UartControl_Equals(key, "ATMS") && UartControl_ParseU32(value, &milliseconds))
        {
            uint64_t now = 0ULL;
            (void)WaveGenerator_GetSampleCounter(&now);
            WaveGenerator_ArmStartAtSample(now + UartControl_MsToSamples(milliseconds));
            UartControl_Send("OK START\r\n");
            return;
        }
    }

    UartControl_Send("ERR START\r\n");
}

static void UartControl_HandleGet(char *context)
{
    char *saved = context;
    char *what = strtok_r(NULL, " \t", &saved);
    char *token;
    char *key;
    char *value;
    int32_t channel = -1;

    if (what == NULL)
    {
        UartControl_Send("ERR GET\r\n");
        return;
    }

    while ((token = strtok_r(NULL, " \t", &saved)) != NULL)
    {
        if (UartControl_SplitKeyValue(token, &key, &value) && UartControl_Equals(key, "CH"))
        {
            (void)UartControl_ParseI32(value, &channel);
        }
    }

    if (UartControl_Equals(what, "STATE"))
    {
        UartControl_ReportState(channel);
        return;
    }

    if (channel < 0)
    {
        if (UartControl_Equals(what, "TIME"))
        {
            uint64_t sample = 0ULL;
            (void)WaveGenerator_GetSampleCounter(&sample);
            (void)snprintf(g_response, sizeof(g_response), "TIME SAMPLE=%llu FS=%lu\r\n",
                           (unsigned long long)sample,
                           (unsigned long)WaveGenerator_GetSampleRateHz());
            UartControl_Send(g_response);
            return;
        }

        UartControl_Send("ERR CH\r\n");
        return;
    }

    if (UartControl_Equals(what, "CONFIG")) { UartControl_ReportConfig((uint8_t)channel); return; }
    if (UartControl_Equals(what, "GAINS"))  { UartControl_ReportGains((uint8_t)channel);  return; }
    if (UartControl_Equals(what, "CAL"))    { UartControl_ReportCal((uint8_t)channel);    return; }

    UartControl_Send("ERR GET\r\n");
}

static void UartControl_ProcessLine(char *line)
{
    char *context = line;
    char *command = strtok_r(line, " \t", &context);
    char *subcommand;

    if (command == NULL)
    {
        return;
    }

    if (UartControl_Equals(command, "SET"))      { UartControl_HandleSet(context);      return; }
    if (UartControl_Equals(command, "GAINS"))    { UartControl_HandleGains(context);    return; }
    if (UartControl_Equals(command, "CAL"))      { UartControl_HandleCal(context);      return; }
    if (UartControl_Equals(command, "SCHEDULE")) { UartControl_HandleSchedule(context); return; }
    if (UartControl_Equals(command, "START"))    { UartControl_HandleStart(context);    return; }
    if (UartControl_Equals(command, "GET"))      { UartControl_HandleGet(context);      return; }

    if (UartControl_Equals(command, "STOP"))
    {
        WaveGenerator_Stop();
        UartControl_Send("OK STOP\r\n");
        return;
    }

    if (UartControl_Equals(command, "CLEAR"))
    {
        subcommand = strtok_r(NULL, " \t", &context);
        if ((subcommand != NULL) && UartControl_Equals(subcommand, "SCHEDULE"))
        {
            WaveGenerator_ClearScheduledUpdates();
            UartControl_Send("OK CLEAR\r\n");
            return;
        }

        UartControl_Send("ERR CLEAR\r\n");
        return;
    }

    if (UartControl_Equals(command, "RESET"))
    {
        subcommand = strtok_r(NULL, " \t", &context);
        if ((subcommand != NULL) && UartControl_Equals(subcommand, "LOAD"))
        {
            WaveGenerator_ResetLoadMeasurement();
            UartControl_Send("OK RESET\r\n");
            return;
        }

        UartControl_Send("ERR RESET\r\n");
        return;
    }

    if (UartControl_Equals(command, "HELP"))
    {
        UartControl_Send("CMD SET|GAINS|CAL|SCHEDULE|START|STOP|CLEAR SCHEDULE|RESET LOAD\r\n");
        UartControl_Send("CMD GET STATE|CONFIG|GAINS|CAL|TIME  (CH=0..3)\r\n");
        UartControl_Send("SET CH=n AMP= FREQ= PHASE= APER= TAU= LOOP= H2..H10= H2PH..H10PH=\r\n");
        UartControl_Send("GAINS CH=n FF= KP= KI= KR= WC= LIM=   (тисячні, LIM у тіках)\r\n");
        return;
    }

    UartControl_Send("ERR CMD\r\n");
}

/* ------------------------------------------------------------------------- */
/*                                  Задача                                   */
/* ------------------------------------------------------------------------- */

static void UartControl_Task(void *argument)
{
    uint8_t chunk[64];
    size_t line_length = 0U;
    size_t received;

    (void)argument;

    UartControl_Send("UART READY\r\n");

    for (;;)
    {
        received = xStreamBufferReceive(g_stream, chunk, sizeof(chunk), portMAX_DELAY);

        for (size_t i = 0U; i < received; ++i)
        {
            char current = (char)chunk[i];

            if (current == '\r')
            {
                continue;
            }

            if (current == '\n')
            {
                g_line[line_length] = '\0';
                UartControl_ProcessLine(g_line);
                line_length = 0U;
                continue;
            }

            if (line_length < (sizeof(g_line) - 1U))
            {
                g_line[line_length++] = current;
            }
            else
            {
                line_length = 0U;
                UartControl_Send("ERR LINE\r\n");
            }
        }
    }
}

void UartControl_BindHardware(const UartControlHardwareConfig *config)
{
    if (config != NULL)
    {
        g_uart_hw = *config;
    }
}

void UartControl_Init(void)
{
    if (g_uart_initialized)
    {
        return;
    }

    if (g_uart_hw.uart == NULL)
    {
        Error_Handler();
        return;
    }

    g_stream = xStreamBufferCreateStatic(sizeof(g_stream_storage), 1U,
                                         g_stream_storage, &g_stream_struct);
    if (g_stream == NULL)
    {
        Error_Handler();
        return;
    }

    HAL_NVIC_SetPriority(g_uart_hw.irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(g_uart_hw.irqn);

    if (!UartControl_StartReception())
    {
        Error_Handler();
        return;
    }

    (void)xTaskCreateStatic(UartControl_Task, "uart_ctl", UART_TASK_STACK_WORDS, NULL,
                            UART_TASK_PRIORITY, g_task_stack, &g_task_tcb);

    g_uart_initialized = true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((huart != g_uart_hw.uart) || (g_stream == NULL) || (size == 0U))
    {
        (void)UartControl_StartReception();
        return;
    }

    (void)xStreamBufferSendFromISR(g_stream, g_rx_frame, size, &higher_priority_task_woken);
    (void)UartControl_StartReception();
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_uart_hw.uart)
    {
        (void)UartControl_StartReception();
    }
}
