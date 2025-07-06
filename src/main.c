#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"

// ==== Definições de hardware ====
#define LED_RED         13
#define LED_GREEN       11
#define LED_BLUE        12
#define BUZZER_GPIO     21

#define BTN_A_GPIO       5
#define BTN_B_GPIO       6
#define JOY_SW_GPIO     22

#define ADC_JOY_Y        0   // GPIO 26
#define ADC_JOY_X        1   // GPIO 27
#define ADC_MICROPHONE   2   // GPIO 28

// ==== Buzzer PWM 2kHz ====
static uint buzzer_slice;

static void buzzer_pwm_init(void)
{
    gpio_set_function(BUZZER_GPIO, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_GPIO);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);   // 125 MHz / 125 = 1 MHz
    pwm_config_set_wrap(&cfg, 499);        // 1 MHz / 500 = 2 kHz
    pwm_init(buzzer_slice, &cfg, false);
    pwm_set_gpio_level(BUZZER_GPIO, 0);    // duty = 0%
}

static void buzzer_on(void)
{
    pwm_set_gpio_level(BUZZER_GPIO, 250);  // 50% duty
    pwm_set_enabled(buzzer_slice, true);
}

static void buzzer_off(void)
{
    pwm_set_enabled(buzzer_slice, false);
    pwm_set_gpio_level(BUZZER_GPIO, 0);
}

// ==== Funções auxiliares ADC ====
static uint16_t adc_raw(uint input)
{
    adc_select_input(input);
    return adc_read();
}

static float adc_volt(uint input)
{
    return (adc_raw(input) * 3.3f) / 4095.0f;
}

// ==== TAREFA 1: Self-Test ====
static void vSelfTestTask(void *pvParameters)
{
    (void)pvParameters;
    printf("\n===== [ TAREFA 1 – SELF-TEST INICIADO ] =====\n");

    // LEDs RGB
    const uint rgb[3] = { LED_RED, LED_GREEN, LED_BLUE };
    const char *nome_cores[] = { "VERMELHO", "VERDE", "AZUL" };
    printf("Etapa 1: Testando LEDs RGB…\n");
    for (int i = 0; i < 3; i++) {
        printf("  • %s ON\n", nome_cores[i]);
        gpio_put(rgb[i], true);
        vTaskDelay(pdMS_TO_TICKS(800));
        gpio_put(rgb[i], false);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    // Beep após LEDs
    printf("Beep de confirmação…\n");
    buzzer_on(); vTaskDelay(pdMS_TO_TICKS(700)); buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(300));

    // Botões
    const uint btns[3] = { BTN_A_GPIO, BTN_B_GPIO, JOY_SW_GPIO };
    const char *nbtn[3] = { "BOTÃO A", "BOTÃO B", "JOYSTICK SW" };
    printf("Etapa 2: Teste de Botões – Pressione na ordem A → B → SW\n");
    for (int i = 0; i < 3; i++) {
        printf("  • Aguarde %s...\n", nbtn[i]);
        while (gpio_get(btns[i])) vTaskDelay(pdMS_TO_TICKS(20));
        printf("    ✅ %s pressionado!\n", nbtn[i]);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    // Joystick
    printf("\nEtapa 3: Leitura do Joystick (ADC0 = Y, ADC1 = X)\n");
    for (int i = 1; i <= 3; i++) {
        uint16_t rawX = adc_raw(ADC_JOY_X);
        uint16_t rawY = adc_raw(ADC_JOY_Y);
        float vx = (rawX * 3.3f) / 4095.0f;
        float vy = (rawY * 3.3f) / 4095.0f;
        printf("  • #%d → X = %4d (%.2f V), Y = %4d (%.2f V)\n",
               i, rawX, vx, rawY, vy);
        vTaskDelay(pdMS_TO_TICKS(700));
    }

    // Microfone
    printf("\nEtapa 4: Leitura do Microfone (ADC2)\n");
    for (int i = 1; i <= 3; i++) {
        uint16_t rawM = adc_raw(ADC_MICROPHONE);
        float vm = (rawM * 3.3f) / 4095.0f;
        printf("  • #%d → Mic = %4d (%.2f V)\n", i, rawM, vm);
        vTaskDelay(pdMS_TO_TICKS(700));
    }

    // Beep final
    printf("Self-Test concluído – beep final!\n");
    buzzer_on(); vTaskDelay(pdMS_TO_TICKS(700)); buzzer_off();

    printf("===== [ TAREFA 1 – FINALIZADA ] =====\n\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    vTaskDelete(NULL);
}

// ==== TAREFA 2: LED Alive ====
static void vAliveTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        gpio_put(LED_RED, true);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(LED_RED, false);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ==== TAREFA 3: Monitor de Joystick e Alarme ====
static void vMonitorTask(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t delay = pdMS_TO_TICKS(50);

    for (;;) {
        float vx = adc_volt(ADC_JOY_X);
        float vy = adc_volt(ADC_JOY_Y);
        printf("[MONITOR] Joy X: %.2f V | Joy Y: %.2f V\n", vx, vy);

        if (vx > 3.0f || vy > 3.0f)
            buzzer_on();
        else
            buzzer_off();

        vTaskDelay(delay);
    }
}

// ==== MAIN ====
int main(void)
{
    stdio_init_all();

    // Aguarda terminal por até 4s
    const absolute_time_t timeout = make_timeout_time_ms(4000);
    while (!stdio_usb_connected() &&
           absolute_time_diff_us(get_absolute_time(), timeout) > 0) {}

    printf("\n\n===== Sistema BitDogLab com FreeRTOS =====\n");

    // Inicialização GPIOs
    const uint outs[] = { LED_RED, LED_GREEN, LED_BLUE };
    for (int i = 0; i < 3; i++) {
        gpio_init(outs[i]);
        gpio_set_dir(outs[i], GPIO_OUT);
        gpio_put(outs[i], false);
    }

    const uint ins[] = { BTN_A_GPIO, BTN_B_GPIO, JOY_SW_GPIO };
    for (int i = 0; i < 3; i++) {
        gpio_init(ins[i]);
        gpio_set_dir(ins[i], GPIO_IN);
        gpio_pull_up(ins[i]);
    }

    // Buzzer e ADC
    buzzer_pwm_init();
    adc_init();
    adc_gpio_init(26); // ADC0
    adc_gpio_init(27); // ADC1
    adc_gpio_init(28); // ADC2

    // Tarefas
    xTaskCreate(vSelfTestTask, "SelfTest", 1024, NULL, 3, NULL);
    xTaskCreate(vAliveTask,    "AliveLED",  256, NULL, 1, NULL);
    xTaskCreate(vMonitorTask,  "Joystick",  512, NULL, 1, NULL);

    // Inicia escalonador
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (true) { tight_loop_contents(); }
}