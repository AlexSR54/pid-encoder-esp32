#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"

static const char *TAG = "LAB2_PID";

/* ===================== CONFIGURACION ===================== */
// Motor (L298)
#define MOTOR_PWM_GPIO      GPIO_NUM_16   // ENA
#define MOTOR_IN1_GPIO      GPIO_NUM_17
#define MOTOR_IN2_GPIO      GPIO_NUM_23
#define MOTOR_PWM_FREQ_HZ   5000

// Si el eje gira al reves de lo que dice la pantalla, cambiar a 0
#define MOTOR_CW_IN1_LEVEL  1

// Encoder
#define ENCODER_PIN_A       GPIO_NUM_18
#define ENCODER_PIN_B       GPIO_NUM_19
// Si al girar a la derecha salen RPM negativas, poner -1 (o cruzar los cables A y B)
#define ENCODER_SIGN        1

// Botones
#define BTN_PWM             GPIO_NUM_32
#define BTN_MODE            GPIO_NUM_33
#define BTN_DEBOUNCE_US     50000ULL
#define LONG_PRESS_MS       1000      // mantener el boton PWM este tiempo alterna PWM/PID

// OLED
#define OLED_SDA_GPIO       GPIO_NUM_21
#define OLED_SCL_GPIO       GPIO_NUM_22
#define OLED_I2C_PORT       I2C_NUM_0

// 11 pulsos por vuelta, reductora 34:1, cuadratura x4
#define ENCODER_PPR          (11.0f * 34.0f)
#define TOTAL_COUNTS_PER_REV (ENCODER_PPR * 4.0f)

#define TWO_PI              6.2831853f
#define RPM_PER_RAD_S       (60.0f / TWO_PI)
#define SAMPLE_PERIOD_US    100000
#define SAMPLE_PERIOD_SEC   (SAMPLE_PERIOD_US / 1000000.0f)

#define PCNT_HIGH_LIMIT      30000
#define PCNT_LOW_LIMIT      -30000

#define SPEED_QUEUE_LEN     10
#define STALE_MS            700       // sin muestras por mas de esto = sin datos

// Lazo abierto
#define PWM_STEP            10
#define RAMP_STEP           5.0f      // % de duty cada 100 ms
#define RAMP_PERIOD_US      100000LL

// Cambio de giro: espera a que el motor este quieto
#define STOP_RPM            3.0f
#define DIR_QUIET_MS        300
#define DIR_TIMEOUT_MS      2000

// PID (lazo cerrado)
#define MAX_RPM             200
#define RPM_STEP            20
#define MIN_DUTY            25.0f

#define KP                  (0.5f * 100.0f / MAX_RPM)
#define KI                  (2.0f * 100.0f / MAX_RPM)
#define KD                  (0.05f * 100.0f / MAX_RPM)
#define RPM_FILTER_ALPHA    0.6f

#define TXT_DER             "DERECHA"
#define TXT_IZQ             "IZQUIERDA"
#define TXT_DIR(d)          ((d) ? TXT_DER : TXT_IZQ)
/* ========================================================= */

typedef enum
{
    UI_MONITOR = 0,
    UI_CFG_PWM,       // PWM en lazo abierto o PID, segun pid_activo
    UI_CFG_DIR
} ui_mode_t;

typedef enum
{
    DIR_NINGUNO = 0,
    DIR_ESPERANDO,
    DIR_CAMBIADO,
    DIR_FALLO
} dir_fase_t;

typedef struct
{
    int pwm_ref;
    int rpm_ref;
    int motor_dir;    // 1 = derecha, 0 = izquierda
    bool pid_activo;  // true = PID dentro del menu PWM
    ui_mode_t mode;
} ui_state_t;

typedef struct
{
    float speed_rad_s;
    int32_t pulses;
    uint64_t timestamp_us;
} speed_msg_t;

typedef struct
{
    int gpio;
    uint64_t t_us;
} button_event_t;

typedef struct
{
    int gpio;
    uint64_t last_us;
} debounce_state_t;

static debounce_state_t g_debounce[] = {{BTN_PWM, 0}, {BTN_MODE, 0}};
#define NUM_BTNS (sizeof(g_debounce) / sizeof(g_debounce[0]))

static ui_state_t g_ui = {.pwm_ref = 0, .rpm_ref = 0, .motor_dir = 1, .pid_activo = false, .mode = UI_MONITOR};

static pcnt_unit_handle_t g_pcnt_unit = NULL;
static esp_timer_handle_t g_speed_timer_handle = NULL;
static SemaphoreHandle_t g_state_mutex = NULL;
static QueueHandle_t g_speed_queue = NULL;
static QueueHandle_t g_ctrl_queue = NULL;
static QueueHandle_t g_button_queue = NULL;
static ssd1306_handle_t g_oled = NULL;

static int32_t g_last_count = 0;
static int64_t g_last_time_us = 0;

static volatile float g_duty_applied = 0;
static volatile float g_duty_target = 0;
static volatile float g_rpm_meas = 0;
static volatile dir_fase_t g_dir_fase = DIR_NINGUNO;

// Terminos del PID, para mostrarlos en pantalla y en el log
static volatile float g_err = 0;
static volatile float g_p = 0;
static volatile float g_i = 0;
static volatile float g_d = 0;

/* ============================ PROTOTIPOS ============================ */
void motor_init(void);
void motor_set_duty(float percent);
static void motor_apply_dir(int d);
static void motor_toggle_direction(void);
static void ramp_step(void);

static void init_encoder_pcnt(void);
static void init_speed_timer(void);
static void timer_callback(void *arg);

static void init_uibtn_gpios(void);
static void button_isr(void *arg);

static void ui_state_get(ui_state_t *out);
static ui_mode_t ui_mode_get(void);
static void ui_mode_set(ui_mode_t m);
static int pwm_ref_get(void);
static void pwm_ref_set(int v);
static int pwm_ref_step(void);
static void rpm_ref_set(int v);
static int rpm_ref_step(void);
static void motor_dir_set(int d);
static void pid_set(bool activo);

static float con_signo(float v, int dir);
static void entrar_a_pid(void);
static void salir_de_pid(void);

static void oled_init(void);
static void valor_grande(int y, float v, int dec, const char *unidad);
static void barra(int y, int valor, int maximo);
static const char *estado_motor(float rpm);
static void vista_monitor(float rad_s, float rpm, const ui_state_t *s, bool sin_datos);
static void vista_pwm(float rpm, const ui_state_t *s, bool sin_datos);
static void vista_pid(float rpm, const ui_state_t *s, bool sin_datos);
static void vista_giro(float rpm, const ui_state_t *s, bool sin_datos);

static void boton_pwm_config(void);
static void manejar_boton(const button_event_t *ev);
static void display_task(void *arg);
static void ui_task(void *arg);
static void control_task(void *arg);

/* ============================== APP MAIN ============================== */
void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando sistema");

    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL)
    {
        ESP_LOGE(TAG, "no se pudo crear el mutex");
        return;
    }

    g_speed_queue = xQueueCreate(SPEED_QUEUE_LEN, sizeof(speed_msg_t));
    g_ctrl_queue = xQueueCreate(SPEED_QUEUE_LEN, sizeof(speed_msg_t));
    g_button_queue = xQueueCreate(10, sizeof(button_event_t));
    if (g_speed_queue == NULL || g_ctrl_queue == NULL || g_button_queue == NULL)
    {
        ESP_LOGE(TAG, "no se pudieron crear las colas");
        return;
    }

    init_encoder_pcnt();
    oled_init();
    motor_init();
    motor_set_duty(0);
    init_uibtn_gpios();

    xTaskCreate(display_task, "display", 4096, NULL, 2, NULL);
    xTaskCreate(ui_task, "ui", 4096, NULL, 3, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 4, NULL);

    init_speed_timer();

    ESP_LOGI(TAG, "Sistema activo");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ============================== MOTOR ============================== */
void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    motor_apply_dir(1);
}

void motor_set_duty(float percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    uint32_t pwm = (uint32_t)(percent * 1023.0f / 100.0f + 0.5f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    g_duty_applied = percent;
}

static void motor_apply_dir(int d)
{
    int in1 = d ? MOTOR_CW_IN1_LEVEL : !MOTOR_CW_IN1_LEVEL;
    gpio_set_level(MOTOR_IN1_GPIO, in1);
    gpio_set_level(MOTOR_IN2_GPIO, !in1);
}

// Sube de a poco hacia el duty pedido y baja de una vez
static void ramp_step(void)
{
    float obj = g_duty_target;
    float act = g_duty_applied;
    if (act == obj)
        return;
    if (act < obj)
    {
        act += RAMP_STEP;
        if (act > obj)
            act = obj;
    }
    else
    {
        act = obj;
    }
    motor_set_duty(act);
}

// Corta el PWM y espera a que el motor quede quieto. Si no para, no invierte y deja el motor apagado
static void motor_toggle_direction(void)
{
    ui_state_t s;
    ui_state_get(&s);
    int nuevo = !s.motor_dir;

    g_duty_target = 0;
    motor_set_duty(0);
    g_dir_fase = DIR_ESPERANDO;

    int quieto_ms = 0;
    int t = 0;
    while (t < DIR_TIMEOUT_MS && quieto_ms < DIR_QUIET_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        t += 50;
        if (fabsf(g_rpm_meas) < STOP_RPM)
            quieto_ms += 50;
        else
            quieto_ms = 0;
    }

    if (quieto_ms < DIR_QUIET_MS)
    {
        pwm_ref_set(0);
        g_dir_fase = DIR_FALLO;
        ESP_LOGW(TAG, "el motor no paro, giro cancelado");
        return;
    }

    motor_dir_set(nuevo);
    motor_apply_dir(nuevo);
    g_dir_fase = DIR_CAMBIADO;
    g_duty_target = (float)s.pwm_ref;
    ESP_LOGI(TAG, "giro -> %s", TXT_DIR(nuevo));
}

/* =========================== ENCODER PCNT =========================== */
static void init_encoder_pcnt(void)
{
    gpio_config_t pines = {
        .pin_bit_mask = (1ULL << ENCODER_PIN_A) | (1ULL << ENCODER_PIN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pines));

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &g_pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {.edge_gpio_num = ENCODER_PIN_A, .level_gpio_num = ENCODER_PIN_B};
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {.edge_gpio_num = ENCODER_PIN_B, .level_gpio_num = ENCODER_PIN_A};
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // necesarios para que el contador acumule al pasar los limites
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcnt_unit, PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit));

    g_last_count = 0;
    g_last_time_us = esp_timer_get_time();
}

// El contador nunca se borra: se resta la cuenta anterior y se usa el tiempo real entre muestras
static void timer_callback(void *arg)
{
    int count = 0;
    if (pcnt_unit_get_count(g_pcnt_unit, &count) != ESP_OK)
        return;

    int64_t ahora = esp_timer_get_time();
    int64_t delta = (int64_t)count - (int64_t)g_last_count;
    int64_t dt_us = ahora - g_last_time_us;
    g_last_count = count;
    g_last_time_us = ahora;
    if (dt_us <= 0)
        return;

    int32_t pulses = (int32_t)(delta * ENCODER_SIGN);
    speed_msg_t msg = {
        .pulses = pulses,
        .speed_rad_s = ((float)pulses / TOTAL_COUNTS_PER_REV) * TWO_PI * (1000000.0f / (float)dt_us),
        .timestamp_us = (uint64_t)ahora,
    };
    xQueueSend(g_speed_queue, &msg, 0);
    xQueueSend(g_ctrl_queue, &msg, 0);
}

static void init_speed_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "encoder_speed_timer",
        .dispatch_method = ESP_TIMER_TASK,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &g_speed_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_speed_timer_handle, SAMPLE_PERIOD_US));
}

/* ============================== BOTONES ============================== */
static void IRAM_ATTR button_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    button_event_t ev = {.gpio = (int)(intptr_t)arg, .t_us = esp_timer_get_time()};
    xQueueSendFromISR(g_button_queue, &ev, &hp);
    if (hp)
        portYIELD_FROM_ISR();
}

static void init_uibtn_gpios(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BTN_PWM) | (1ULL << BTN_MODE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_PWM, button_isr, (void *)(intptr_t)BTN_PWM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_MODE, button_isr, (void *)(intptr_t)BTN_MODE));
}

/* ================= ESTADO COMPARTIDO (MUTEX) ================= */
static void ui_state_get(ui_state_t *out)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    *out = g_ui;
    xSemaphoreGive(g_state_mutex);
}

static ui_mode_t ui_mode_get(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    ui_mode_t m = g_ui.mode;
    xSemaphoreGive(g_state_mutex);
    return m;
}

static void ui_mode_set(ui_mode_t m)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.mode = m;
    xSemaphoreGive(g_state_mutex);
}

static int pwm_ref_get(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    int v = g_ui.pwm_ref;
    xSemaphoreGive(g_state_mutex);
    return v;
}

static void pwm_ref_set(int v)
{
    if (v < 0)   v = 0;
    if (v > 100) v = 100;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.pwm_ref = v;
    xSemaphoreGive(g_state_mutex);
}

// Sube PWM_STEP y vuelve a 0 despues de 100
static int pwm_ref_step(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.pwm_ref += PWM_STEP;
    if (g_ui.pwm_ref > 100)
        g_ui.pwm_ref = 0;
    int v = g_ui.pwm_ref;
    xSemaphoreGive(g_state_mutex);
    return v;
}

static void rpm_ref_set(int v)
{
    if (v < 0)       v = 0;
    if (v > MAX_RPM) v = MAX_RPM;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.rpm_ref = v;
    xSemaphoreGive(g_state_mutex);
}

// Al pasar MAX_RPM vuelve a RPM_STEP, no a 0, para no parar el motor sin querer
static int rpm_ref_step(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.rpm_ref += RPM_STEP;
    if (g_ui.rpm_ref > MAX_RPM)
        g_ui.rpm_ref = RPM_STEP;
    int v = g_ui.rpm_ref;
    xSemaphoreGive(g_state_mutex);
    return v;
}

static void motor_dir_set(int d)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.motor_dir = d ? 1 : 0;
    xSemaphoreGive(g_state_mutex);
}

static void pid_set(bool activo)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.pid_activo = activo;
    xSemaphoreGive(g_state_mutex);
}

// A la izquierda el encoder cuenta al reves
static float con_signo(float v, int dir)
{
    return dir ? v : -v;
}

// Entra al PID con la velocidad actual como meta, asi el motor no se frena
static void entrar_a_pid(void)
{
    rpm_ref_set((int)(g_rpm_meas + 0.5f));
    pid_set(true);
    ESP_LOGI(TAG, "modo PID, meta inicial = %d RPM", (int)(g_rpm_meas + 0.5f));
}

// Sale del PID dejando el PWM en el duty que tenia el controlador
static void salir_de_pid(void)
{
    pwm_ref_set((int)(g_duty_applied + 0.5f));
    g_duty_target = (float)pwm_ref_get();
    pid_set(false);
    ESP_LOGI(TAG, "modo PWM, duty = %d%%", pwm_ref_get());
}

/* ============================== TAREAS ============================== */
// Toque corto: sube el valor. Mantener LONG_PRESS_MS: alterna PWM / PID
static void boton_pwm_config(void)
{
    int ms = 0;
    while (gpio_get_level(BTN_PWM) == 0 && ms < LONG_PRESS_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        ms += 20;
    }

    if (gpio_get_level(BTN_PWM) == 0)
    {
        ui_state_t s;
        ui_state_get(&s);
        if (s.pid_activo)
            salir_de_pid();
        else
            entrar_a_pid();

        while (gpio_get_level(BTN_PWM) == 0)
            vTaskDelay(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(50));
        xQueueReset(g_button_queue);
        return;
    }

    ui_state_t s;
    ui_state_get(&s);
    if (s.pid_activo)
    {
        int ref = rpm_ref_step();
        ESP_LOGI(TAG, "meta = %d RPM", ref);
    }
    else
    {
        int ref = pwm_ref_step();
        g_duty_target = (float)ref;
        ESP_LOGI(TAG, "PWM = %d%%", ref);
    }
}

static void manejar_boton(const button_event_t *ev)
{
    vTaskDelay(pdMS_TO_TICKS(20));
    if (gpio_get_level(ev->gpio) != 0)
        return;

    debounce_state_t *st = NULL;
    for (size_t i = 0; i < NUM_BTNS; i++)
        if (g_debounce[i].gpio == ev->gpio)
            st = &g_debounce[i];
    if (st == NULL)
        return;
    if (ev->t_us - st->last_us < BTN_DEBOUNCE_US)
        return;
    st->last_us = ev->t_us;

    if (ev->gpio == BTN_MODE)
    {
        ui_state_t s;
        ui_state_get(&s);
        if (s.mode == UI_CFG_PWM && s.pid_activo)
            salir_de_pid();

        ui_mode_t nuevo = (ui_mode_t)((s.mode + 1) % 3);
        g_dir_fase = DIR_NINGUNO;
        ui_mode_set(nuevo);
        g_duty_target = (float)pwm_ref_get();
        ESP_LOGI(TAG, "vista -> %d", nuevo);
    }
    else if (ev->gpio == BTN_PWM)
    {
        switch (ui_mode_get())
        {
        case UI_CFG_PWM:
            boton_pwm_config();
            break;
        case UI_CFG_DIR:
            motor_toggle_direction();
            break;
        case UI_MONITOR:
        default:
            break;
        }
    }
}

// Atiende botones y aplica la rampa de PWM (menos cuando manda el PID)
static void ui_task(void *arg)
{
    button_event_t ev;
    int64_t ultima_rampa = 0;

    while (1)
    {
        if (xQueueReceive(g_button_queue, &ev, pdMS_TO_TICKS(20)) == pdTRUE)
            manejar_boton(&ev);

        int64_t ahora = esp_timer_get_time();
        if (ahora - ultima_rampa >= RAMP_PERIOD_US)
        {
            ultima_rampa = ahora;
            ui_state_t s;
            ui_state_get(&s);
            if (!(s.mode == UI_CFG_PWM && s.pid_activo))
                ramp_step();
        }
    }
}

// Mide siempre las RPM; solo con el PID activo calcula y aplica el control
static void control_task(void *arg)
{
    speed_msg_t m;
    float i_term = 0, rpm_f = 0, rpm_prev = 0;
    bool active = false;
    bool was_zero = true;

    while (1)
    {
        if (xQueueReceive(g_ctrl_queue, &m, portMAX_DELAY) != pdTRUE)
            continue;

        ui_state_t s;
        ui_state_get(&s);

        float rpm = con_signo(m.speed_rad_s * RPM_PER_RAD_S, s.motor_dir);
        rpm_f = RPM_FILTER_ALPHA * rpm + (1.0f - RPM_FILTER_ALPHA) * rpm_f;
        g_rpm_meas = rpm_f;

        if (!(s.mode == UI_CFG_PWM && s.pid_activo))
        {
            active = false;
            continue;
        }

        if (!active)
        {
            active = true;
            i_term = g_duty_applied;
            rpm_prev = rpm_f;
            was_zero = (g_duty_applied < 1.0f);
        }

        if (s.rpm_ref == 0)
        {
            i_term = 0;
            was_zero = true;
            motor_set_duty(0);
            g_err = g_p = g_i = g_d = 0;
        }
        else
        {
            if (was_zero)
            {
                i_term = MIN_DUTY;
                rpm_prev = rpm_f;
                was_zero = false;
            }

            float err = (float)s.rpm_ref - rpm_f;                 // error = meta - medida
            float p = KP * err;                                    // proporcional
            float d = -KD * (rpm_f - rpm_prev) / SAMPLE_PERIOD_SEC; // derivativo sobre la medicion
            float i = i_term;                                      // integral acumulada
            float u = p + i + d;

            // anti-windup: no integra si ya esta saturado y el error empuja mas
            bool sat_alta = (u > 100.0f && err > 0);
            bool sat_baja = (u < 0.0f && err < 0);
            if (!sat_alta && !sat_baja)
            {
                i_term += KI * err * SAMPLE_PERIOD_SEC;
                if (i_term < 0)   i_term = 0;
                if (i_term > 100) i_term = 100;
            }

            motor_set_duty(u);
            g_err = err;
            g_p = p;
            g_i = i;
            g_d = d;
        }
        rpm_prev = rpm_f;

        ESP_LOGI(TAG, "PID meta=%d real=%.1f err=%.1f | P=%.1f I=%.1f D=%.1f | duty=%.0f%%",
                 s.rpm_ref, rpm_f, g_err, g_p, g_i, g_d, g_duty_applied);
    }
}

/* ============================== OLED ============================== */
// Busca la pantalla en 0x3C y 0x3D. Si no aparece, el programa sigue sin ella
static void oled_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = OLED_I2C_PORT,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    uint8_t addr = 0;
    if (i2c_master_probe(bus, 0x3C, 100) == ESP_OK)
        addr = 0x3C;
    else if (i2c_master_probe(bus, 0x3D, 100) == ESP_OK)
        addr = 0x3D;

    if (addr == 0)
    {
        ESP_LOGE(TAG, "no hay pantalla en el i2c, revisar cables");
        return;
    }
    ESP_LOGI(TAG, "pantalla encontrada en 0x%02X", addr);

    ssd1306_config_t cfg = {
        .bus = SSD1306_I2C,
        .width = 128,
        .height = 64,
        .iface.i2c = {.port = OLED_I2C_PORT, .addr = addr, .rst_gpio = GPIO_NUM_NC},
    };
    if (ssd1306_new_i2c(&cfg, &g_oled) != ESP_OK)
    {
        ESP_LOGE(TAG, "la pantalla no respondio al iniciar");
        g_oled = NULL;
        return;
    }
    ssd1306_clear(g_oled);
    ssd1306_display(g_oled);
}

// Numero grande con su unidad en letra chica al lado
static void valor_grande(int y, float v, int dec, const char *unidad)
{
    char buf[16];
    snprintf(buf, sizeof buf, "%.*f", dec, v);
    ssd1306_draw_text_scaled(g_oled, 0, y, buf, true, 2);
    ssd1306_draw_text(g_oled, (int)strlen(buf) * 12 + 4, y + 7, unidad, true);
}

static void barra(int y, int valor, int maximo)
{
    if (valor < 0)      valor = 0;
    if (valor > maximo) valor = maximo;
    ssd1306_draw_rect(g_oled, 0, y, 128, 8, false);
    int w = (126 * valor) / maximo;
    if (w > 0)
        ssd1306_draw_rect(g_oled, 1, y + 1, w, 6, true);
}

static const char *estado_motor(float rpm)
{
    if (g_duty_applied < 1.0f)
        return (fabsf(rpm) < STOP_RPM) ? "DETENIDO" : "FRENANDO";
    if (g_duty_applied < g_duty_target - 0.5f)
        return "ACELERANDO";
    return "EN MARCHA";
}

static void vista_monitor(float rad_s, float rpm, const ui_state_t *s, bool sin_datos)
{
    char line[32];
    ssd1306_draw_text(g_oled, 0, 0, "MONITOR", true);
    valor_grande(9, rad_s, 2, "rad/s");
    valor_grande(27, rpm, 1, "RPM");
    snprintf(line, sizeof line, "PWM:%.0f%%  %s", g_duty_applied, TXT_DIR(s->motor_dir));
    ssd1306_draw_text(g_oled, 0, 45, line, true);
    ssd1306_draw_text(g_oled, 0, 55, sin_datos ? "ENCODER SIN DATOS" : estado_motor(rpm), true);
}

// Lazo abierto: el usuario fija el PWM
static void vista_pwm(float rpm, const ui_state_t *s, bool sin_datos)
{
    char line[32];
    ssd1306_draw_text(g_oled, 0, 0, "CONFIG PWM", true);
    snprintf(line, sizeof line, "%d%%", s->pwm_ref);
    ssd1306_draw_text_scaled(g_oled, 0, 11, line, true, 2);
    snprintf(line, sizeof line, "REAL:%.0f%%", g_duty_applied);
    ssd1306_draw_text(g_oled, 66, 18, line, true);
    barra(30, (int)(g_duty_applied + 0.5f), 100);
    if (sin_datos)
        snprintf(line, sizeof line, "ENCODER SIN DATOS");
    else
        snprintf(line, sizeof line, "%.1f RPM", rpm);
    ssd1306_draw_text(g_oled, 0, 43, line, true);
    ssd1306_draw_text(g_oled, 0, 55, "PWM:+10 largo:PID", true);
}

// Lazo cerrado: el usuario fija la meta de RPM y se ven los tres terminos
static void vista_pid(float rpm, const ui_state_t *s, bool sin_datos)
{
    char line[32];
    ssd1306_draw_text(g_oled, 0, 0, "PID", true);
    snprintf(line, sizeof line, "META:%d", s->rpm_ref);
    ssd1306_draw_text(g_oled, 128 - (int)strlen(line) * 6, 0, line, true);

    valor_grande(9, rpm, 1, "RPM");

    // la barra muestra la RPM real y la marca la meta
    barra(26, (int)(rpm + 0.5f), MAX_RPM);
    int mx = (126 * s->rpm_ref) / MAX_RPM;
    ssd1306_draw_rect(g_oled, mx, 24, 2, 12, true);

    snprintf(line, sizeof line, "P%.1f I%.1f D%.1f", g_p, g_i, g_d);
    ssd1306_draw_text(g_oled, 0, 37, line, true);

    if (sin_datos)
        snprintf(line, sizeof line, "ENCODER SIN DATOS");
    else
        snprintf(line, sizeof line, "PWM:%.0f%%  %s", g_duty_applied, TXT_DIR(s->motor_dir));
    ssd1306_draw_text(g_oled, 0, 46, line, true);
    ssd1306_draw_text(g_oled, 0, 55, "PWM:+20 largo:PWM", true);
}

static void vista_giro(float rpm, const ui_state_t *s, bool sin_datos)
{
    char line[32];
    const char *msg;
    ssd1306_draw_text(g_oled, 0, 0, "CONFIG GIRO", true);
    ssd1306_draw_text_scaled(g_oled, 0, 11, TXT_DIR(s->motor_dir), true, 2);

    switch (g_dir_fase)
    {
    case DIR_ESPERANDO: msg = "ESPERANDO PARO...";   break;
    case DIR_CAMBIADO:  msg = "GIRO CAMBIADO";       break;
    case DIR_FALLO:     msg = "SIN PARO: MOTOR OFF"; break;
    default:            msg = "PULSA PARA INVERTIR"; break;
    }
    ssd1306_draw_text(g_oled, 0, 31, msg, true);

    if (sin_datos)
        snprintf(line, sizeof line, "ENCODER SIN DATOS");
    else
        snprintf(line, sizeof line, "%.1f RPM  PWM:%.0f%%", rpm, g_duty_applied);
    ssd1306_draw_text(g_oled, 0, 43, line, true);
    ssd1306_draw_text(g_oled, 0, 55, "PWM:girar  MODO:SIG", true);
}

// Dibuja la vista activa; si no llegan muestras en STALE_MS muestra SIN DATOS
static void display_task(void *arg)
{
    speed_msg_t msg = {0};
    int cuenta = 0;

    while (1)
    {
        bool sin_datos = false;
        if (xQueueReceive(g_speed_queue, &msg, pdMS_TO_TICKS(STALE_MS)) != pdTRUE)
        {
            sin_datos = true;
            msg.speed_rad_s = 0;
            msg.pulses = 0;
        }

        // si la pantalla no se encontro al arrancar, no dibuja nada
        if (g_oled == NULL)
            continue;

        ui_state_t s;
        ui_state_get(&s);

        float rad_s = con_signo(msg.speed_rad_s, s.motor_dir);
        float rpm = rad_s * RPM_PER_RAD_S;
        if (fabsf(rad_s) < 0.005f) rad_s = 0;
        if (fabsf(rpm) < 0.05f)    rpm = 0;

        ssd1306_clear(g_oled);
        switch (s.mode)
        {
        case UI_MONITOR:
            vista_monitor(rad_s, rpm, &s, sin_datos);
            if (++cuenta >= 5)
            {
                cuenta = 0;
                ESP_LOGI(TAG, "pulsos=%ld | %.2f rad/s | %.1f RPM", (long)msg.pulses, rad_s, rpm);
            }
            break;
        case UI_CFG_PWM:
            if (s.pid_activo)
                vista_pid(rpm, &s, sin_datos);
            else
                vista_pwm(rpm, &s, sin_datos);
            break;
        case UI_CFG_DIR:
            vista_giro(rpm, &s, sin_datos);
            break;
        }
        ssd1306_display(g_oled);
    }
}