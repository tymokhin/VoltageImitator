# VoltageImitator Firmware

Прошивка для `STM32F334C8T6`, створена в `STM32CubeIDE` для генерації керованих PWM/SPWM-сигналів, синхронного вимірювання через ADC і подальшої побудови замкненого контуру керування для імітації напруги або струму в експериментах з релейним захистом.

Проєкт використовує `STM32 HAL`, `CMSIS` і `FreeRTOS`. У коді вже є базова апаратна конфігурація та каркас прикладної логіки, але повний робочий контур `HRTIM -> ADC -> DMA -> обробка -> оновлення PWM` ще не завершений.

## Що це за проєкт

За задумом і за наявним кодом цей firmware має:

- генерувати високочастотний PWM через `HRTIM1`;
- формувати вихідну хвилю через LUT і оновлення `CMPx`;
- синхронно з PWM запускати вибірку `ADC1/ADC2`;
- використовувати `DMA` для перенесення вибірок і, імовірно, для burst-оновлення `HRTIM`;
- у перспективі виконувати PI/PID-корекцію на основі виміряного сигналу;
- приймати параметри сигналу через `USART3`;
- взаємодіяти з енкодером і LCD.

Із контекстного опису проєкту також випливає цільова архітектура:

- частота PWM порядку `100 kHz`;
- цільова вихідна хвиля порядку `50 Hz`;
- LUT-підхід замість важких обчислень у швидкому контурі;
- підтримка кількох каналів або фаз;
- використання `Master Timer` HRTIM для синхронізації та ADC-trigger.

Ці пункти добре узгоджуються з кодом і `.ioc`, але не все з цього вже реалізовано в runnable-логіці.

## Цільова платформа

- MCU: `STM32F334C8T6`
- Сімейство: `STM32F3`
- IDE: `STM32CubeIDE`
- RTOS: `FreeRTOS`
- Toolchain у `Debug`: `GNU Tools for STM32 (13.3.rel1)`

У [Core/Src/main.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/main.c) система налаштована на роботу від `HSE` з PLL. Частота ядра в [conf/FreeRTOSConfig.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/conf/FreeRTOSConfig.h) очікується як `SystemCoreClock`, а тік `FreeRTOS` задано `1000 Hz`.

## Апаратні блоки, які реально налаштовані

За [VoltageImitator.ioc](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/VoltageImitator.ioc), [Core/Src/main.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/main.c) і [Core/Src/stm32f3xx_hal_msp.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/stm32f3xx_hal_msp.c) в проєкті використані:

- `HRTIM1`
- `ADC1`
- `ADC2`
- `DMA1_Channel1` для `ADC1`
- `USART3`
- `SPI1`
- `TIM2` у режимі encoder
- GPIO для LCD, енкодера, службових виходів і SWD

Ключові виводи з [Core/Inc/main.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Inc/main.h):

- `PA8` -> `HRTIM1_CHA1`
- `PA9` -> `HRTIM1_CHA2`
- `PA10` -> `HRTIM1_CHB1`
- `PA11` -> `HRTIM1_CHB2`
- `PB10` / `PB11` -> `USART3_TX/RX`
- `PB3` / `PB5` -> `SPI1_SCK/MOSI`
- `PA0` / `PA1` -> енкодер `TIM2`
- `PA15`, `PB4`, `PB6` -> сигнали LCD

## Поточна структура проєкту

- [Core](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core) - точка входу, startup, IRQ, HAL MSP, системний код.
- [Task](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task) - прикладні задачі генератора та UART.
- [Task/HAL](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL) - окремі заготовки для роботи з ADC/HRTIM.
- [conf](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/conf) - `FreeRTOSConfig`.
- [Drivers](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Drivers) - HAL/CMSIS від ST.
- [FreeRTOS](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/FreeRTOS) - ядро FreeRTOS.
- [Debug](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Debug) - згенеровані артефакти збірки.
- [VoltageImitator.ioc](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/VoltageImitator.ioc) - апаратна конфігурація CubeMX.

## Фактична послідовність запуску

У [Core/Src/main.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/main.c) виконується:

1. `HAL_Init()`
2. `SystemClock_Config()`
3. `MX_GPIO_Init()`
4. `MX_DMA_Init()`
5. `MX_HRTIM1_Init()`
6. `MX_ADC1_Init()`
7. `MX_ADC2_Init()`
8. `MX_USART3_UART_Init()`
9. `MX_TIM2_Init()`
10. `MX_SPI1_Init()`
11. `CreatePWMTask()`
12. `vTaskStartScheduler()`

Що важливо:

- калібрування `ADC1/ADC2` зараз закоментоване;
- старт виходів HRTIM і старт лічильників HRTIM зараз закоментовані;
- явного старту `ADC + DMA` в `main()` теж немає.

Тобто ініціалізація периферії відбувається, але власне генерація та вимірювання в поточній ревізії не запускаються.

## Реальна конфігурація HRTIM

У [Core/Src/main.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/main.c) для `HRTIM1` налаштовано:

- `ADC Trigger 1`: `HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD`
- `ADC trigger update source`: `HRTIM_ADCTRIGGERUPDATE_MASTER`
- burst mode: `HRTIM_BURSTMODE_CONTINOUS`
- burst trigger: `HRTIM_BURSTMODETRIGGER_TIMERA_RESET`
- `PWM_PERIOD = 46080`
- Master timer prescaler: `HRTIM_PRESCALERRATIO_MUL4`
- Timer A prescaler: `HRTIM_PRESCALERRATIO_MUL32`
- Timer B prescaler: `HRTIM_PRESCALERRATIO_MUL32`

Виходи налаштовані так:

- `TA1`: `SET = TIMCMP1`, `RESET = TIMPER`
- `TA2`: `SET = TIMCMP2`, `RESET = TIMPER`
- `TB1`: `SET = TIMPER`, `RESET = TIMCMP1`
- `TB2`: `SET = NONE`, `RESET = NONE`

Початкові compare-значення:

- `Timer A / CMP1 = 96`
- `Timer A / CMP2 = 96`
- `Timer B / CMP1 = PWM_PERIOD / 5`

Для Timer A увімкнено:

- `PreloadEnable = HRTIM_PRELOAD_ENABLED`
- `UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT`
- `UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_A`
- `ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED`

Це добре узгоджується з вашим контекстом про чутливість HRTIM до preload/update-конфігурації при переході від одного каналу до кількох.

## Реальна конфігурація ADC

### ADC1

У [Core/Src/main.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/main.c) для `ADC1` налаштовано:

- режим `ADC_DUALMODE_REGSIMULT`
- зовнішній тригер `ADC_EXTERNALTRIGCONVHRTIM_TRG1`
- фронт тригера `RISING`
- `NbrOfConversion = 2`
- `DMAContinuousRequests = ENABLE`
- `DMA circular` через `DMA1_Channel1`

Порядок regular ranks:

- rank 1 -> `ADC_CHANNEL_3`
- rank 2 -> `ADC_CHANNEL_11`

За [Core/Src/stm32f3xx_hal_msp.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/stm32f3xx_hal_msp.c) це відповідає:

- `PA2` / `ADC1_IN3`
- `PB0` / `ADC1_IN11`

### ADC2

Для `ADC2` налаштовано:

- `NbrOfConversion = 2`
- `DMAContinuousRequests = DISABLE`
- same dual regular simultaneous scheme разом із `ADC1`

Порядок regular ranks:

- rank 1 -> `ADC_CHANNEL_1`
- rank 2 -> `ADC_CHANNEL_3`

За MSP-конфігурацією це відповідає:

- `PA4` / `ADC2_IN1`
- `PA6` / `ADC2_IN3`

### Практичний зміст

Контекстний файл правильно підкреслює, що для такого проєкту треба явно фіксувати порядок rank-ів і layout DMA-буфера. У поточному коді сама ідея синхронного вимірювання є, але структура даних для чіткої прив’язки `rank -> signal` ще не доведена до цілісного рішення в продакшн-коді.

## Прикладна логіка та стан реалізації

### `wave_generator_task.c`

У [Task/wave_generator_task.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/wave_generator_task.c):

- створюється задача `pwm_task`;
- задано `SAMPLE_RATE = 100000UL`, що збігається з очікуваним PWM-порядком із контексту;
- описано LUT-буфер `cmp_lut`;
- є заготовка побудови синусоподібного профілю через `sinf()`;
- є заготовки для `HAL_DMA_Start(... -> BDMADR)` та запуску `TA1/TA2`.

Водночас у поточному стані:

- головна робоча логіка майже повністю закоментована;
- `pwm_task()` фактично містить порожній `for (;;)` без обчислень і без старту периферії;
- використовується лише `CreatePWMTask()`, але не реальний fast control loop.

### `uart_task.c`

У [Task/uart_task.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/uart_task.c) є тільки заглушка `handle_uart_command(...)`. Ідея з контекстного файлу про передачу параметрів типу `A`, `alpha`, `phi`, `freq` поки в коді не реалізована.

### `Task/HAL`

У [Task/HAL/hrtim.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/hrtim.h) і [Task/HAL/adc.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/adc.h) є окремі визначення:

- `PWM_BUFFER_LEN = 128`
- `PWM_CHANNELS = 4`
- `SignalParams { A, alpha, phi, freq, dt }`
- `adc_dma_buffer[2][ADC_BUFFER_LENGTH]`

Це добре стикується з вашим загальним задумом:

- кілька каналів;
- LUT-параметризація;
- буферизація виміряних даних;
- підготовка до регулятора.

## Виявлені неузгодженості в коді

Нижче перелік речей, які варто знати перед продовженням роботи. Це не припущення, а те, що видно з поточної кодової бази.

### 1. Генерація і вимірювання не стартують у `main`

У `main()` закоментовані:

- `HAL_HRTIM_WaveformOutputStart(...)`
- `HAL_HRTIM_WaveformCounterStart(...)`
- калібрування ADC

Також немає виклику на кшталт `HAL_ADCEx_MultiModeStart_DMA(...)` або аналогічного старту перетворень.

### 2. Callback-и ADC реалізовані двічі

Одна реалізація є в [Task/wave_generator_task.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/wave_generator_task.c), друга - в [Task/HAL/adc.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/adc.c).

Для weak callback-механізму HAL це означає, що треба залишити лише одну узгоджену реалізацію, інакше збірка або поведінка будуть проблемними.

### 3. `ProcessAdcData(...)` лише оголошена

У [Task/HAL/adc.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/adc.c) є:

- `extern void ProcessAdcData(uint16_t* adc_buffer, uint16_t buf_length);`

Але реалізації цієї функції в проєкті не знайдено.

### 4. `hdma_hrtim1_a` використовується як `extern`

У [Task/wave_generator_task.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/wave_generator_task.c) та [Task/HAL/hrtim.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/hrtim.c) присутній:

- `extern DMA_HandleTypeDef hdma_hrtim1_a;`

Але визначення цього DMA handle в поточних файлах проєкту не видно. Якщо HRTIM DMA планується використовувати далі, цю частину треба або доробити, або прибрати застарілі посилання.

### 5. `Task/HAL` треба перевірити на реальну участь у збірці

У [Debug/Task/HAL/subdir.mk](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Debug/Task/HAL/subdir.mk) є правила для `adc.c` і `hrtim.c`, але у [Debug/sources.mk](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Debug/sources.mk) каталог `Task/HAL` відсутній у `SUBDIRS`.

Тобто код `Task/HAL` явно існує як частина задуму, але актуальний стан його включення в згенеровану збірку треба перевірити окремо в IDE.

## FreeRTOS

У [conf/FreeRTOSConfig.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/conf/FreeRTOSConfig.h):

- `configMAX_PRIORITIES = 5`
- `configMINIMAL_STACK_SIZE = 130`
- `configTOTAL_HEAP_SIZE = 4096`
- `configUSE_MUTEXES = 1`
- `configSUPPORT_STATIC_ALLOCATION = 1`
- `configSUPPORT_DYNAMIC_ALLOCATION = 1`
- software timers вимкнені: `configUSE_TIMERS = 0`

У [Task/wave_generator_task.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/wave_generator_task.c) задача `PWM` створюється з пріоритетом `configMAX_PRIORITIES - 1`, тобто майже максимальним.

Це відповідає задуму, що важка периферійна логіка має бути ближче до high-priority execution, хоча з вашого контексту видно, що критичний fast loop все одно краще тримати на рівні timer/DMA/ISR, а не у blocking task.

## Збірка

### Через STM32CubeIDE

1. Відкрити каталог проєкту в `STM32CubeIDE`.
2. Імпортувати існуючий проєкт Eclipse/STM32CubeIDE.
3. За потреби відкрити [VoltageImitator.ioc](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/VoltageImitator.ioc) і перегенерувати код.
4. Зібрати конфігурацію `Debug`.

У каталозі [Debug](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Debug) очікуються:

- `VoltageImitator.elf`
- `VoltageImitator.hex`
- `VoltageImitator.map`

### Через makefile

STM32CubeIDE генерує make-based збірку в каталозі `Debug`, тому проєкт зазвичай можна збирати і поза IDE, якщо доступний toolchain від STM32CubeIDE.

## Прошивка та налагодження

У проєкті є конфігурація запуску [VoltageImitator Debug.launch](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/VoltageImitator%20Debug.launch), яку можна використати для запуску або відлагодження через ST-LINK.

Типовий сценарій:

- збірка в `Debug`;
- прошивка `VoltageImitator.elf` або `VoltageImitator.hex`;
- контроль сигналів `PA8/PA9/PA10/PA11`;
- окрема перевірка фактичного старту HRTIM counters, ADC trigger та DMA callback.

## Рекомендовані наступні кроки

1. Узгодити єдину точку входу для `HAL_ADC_ConvHalfCpltCallback` і `HAL_ADC_ConvCpltCallback`.
2. Додати реальний старт послідовності `ADC calibration -> HRTIM start -> ADC/DMA start`.
3. Явно описати формат DMA-буфера для пари `ADC1/ADC2` і прив’язку `rank -> signal`.
4. Доробити або видалити незавершений шлях із `hdma_hrtim1_a`.
5. Реалізувати `ProcessAdcData(...)` і визначити, де саме виконується fast control.
6. Завершити UART-протокол зміни параметрів сигналу.
7. Після стабілізації коду оновити README ще раз уже під фактичний runtime-потік, а не під поточний каркас.

## Файли, з яких варто починати

- [Core/Src/main.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/main.c) - ініціалізація та порядок запуску.
- [Core/Src/stm32f3xx_hal_msp.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Core/Src/stm32f3xx_hal_msp.c) - відповідність периферії та GPIO.
- [Task/wave_generator_task.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/wave_generator_task.c) - основна ідея генератора та LUT.
- [Task/HAL/adc.c](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/adc.c) - заготовка ADC DMA обробки.
- [Task/HAL/hrtim.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/Task/HAL/hrtim.h) - параметри каналів і структура сигналу.
- [conf/FreeRTOSConfig.h](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/conf/FreeRTOSConfig.h) - параметри RTOS.
- [VoltageImitator.ioc](D:/Projects/PowerNetworkImitator/VoltageImitator/firmware/STM32CubeIDE/VoltageImitator.ioc) - істина щодо периферії CubeMX.
