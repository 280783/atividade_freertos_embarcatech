#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ==== Definições de hardware ====
#define ADC_JOY_Y      0    // GPIO26
#define ADC_JOY_X      1    // GPIO27
#define JOY_BTN_GPIO   22   // botão
#define BUZZER_GPIO    21

typedef struct {
    uint16_t vrx;
    uint16_t vry;
    bool     button_pressed;
} JoystickEvent;

static QueueHandle_t queueJoystick;
static SemaphoreHandle_t mutexUart;
static SemaphoreHandle_t semBuzzer;

static uint buzzer_slice;

// ==== Buzzer PWM 2kHz ====
static void buzzer_pwm_init(void) {
    gpio_set_function(BUZZER_GPIO, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_GPIO);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 499); // 1MHz/500 = 2kHz
    pwm_init(buzzer_slice, &cfg, false);
    pwm_set_gpio_level(BUZZER_GPIO, 0);
}

static void buzzer_on(void) {
    pwm_set_gpio_level(BUZZER_GPIO, 250); // 50% duty
    pwm_set_enabled(buzzer_slice, true);
}

static void buzzer_off(void) {
    pwm_set_enabled(buzzer_slice, false);
    pwm_set_gpio_level(BUZZER_GPIO, 0); // Garante 0V
}

// ==== Tarefa 1: Leitura dos eixos ====
static void vTaskReadAxes(void *pvParameters) {
    JoystickEvent ev;
    (void)pvParameters;
    while (1) {
        adc_select_input(ADC_JOY_X);
        ev.vrx = adc_read();
        adc_select_input(ADC_JOY_Y);
        ev.vry = adc_read();
        ev.button_pressed = false;
        xQueueSend(queueJoystick, &ev, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==== Tarefa 2: Leitura do botão ====
static void vTaskReadButton(void *pvParameters) {
    JoystickEvent ev;
    (void)pvParameters;
    bool last = gpio_get(JOY_BTN_GPIO);
    while (1) {
        bool curr = gpio_get(JOY_BTN_GPIO);
        if (!curr && last) {
            ev.vrx = ev.vry = 0;
            ev.button_pressed = true;
            xQueueSend(queueJoystick, &ev, portMAX_DELAY);
        }
        last = curr;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==== Tarefa 3/4: Processamento + Controle do Buzzer ====
static void vTaskProcess(void *pvParameters) {
    JoystickEvent ev;
    const uint16_t DEADZONE = 200; // maior zona morta evita ruído
    (void)pvParameters;
    while (1) {
        if (xQueueReceive(queueJoystick, &ev, portMAX_DELAY)) {
            xSemaphoreTake(mutexUart, portMAX_DELAY);
            printf("RX=%u RY=%u BTN=%d\n", ev.vrx, ev.vry, ev.button_pressed);
            xSemaphoreGive(mutexUart);

            bool moved = (abs((int)ev.vrx - 2048) > DEADZONE) ||
                         (abs((int)ev.vry - 2048) > DEADZONE);

            if (ev.button_pressed || moved) {
                if (xSemaphoreTake(semBuzzer, pdMS_TO_TICKS(10)) == pdPASS) {
                    buzzer_on();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    buzzer_off();
                    xSemaphoreGive(semBuzzer);
                }
            }
        }
    }
}

// ==== MAIN ====
int main(void) {
    stdio_init_all();
    adc_init();
    adc_gpio_init(26); // ADC0 - Y
    adc_gpio_init(27); // ADC1 - X

    gpio_init(JOY_BTN_GPIO);
    gpio_set_dir(JOY_BTN_GPIO, GPIO_IN);
    gpio_pull_up(JOY_BTN_GPIO);

    buzzer_pwm_init();

    queueJoystick = xQueueCreate(10, sizeof(JoystickEvent));
    mutexUart = xSemaphoreCreateMutex();
    semBuzzer = xSemaphoreCreateCounting(2, 2);

    xTaskCreate(vTaskReadAxes,   "AZ", 256, NULL, 3, NULL);
    xTaskCreate(vTaskReadButton, "BTN",256, NULL, 3, NULL);
    xTaskCreate(vTaskProcess,    "PRC",512, NULL, 2, NULL);

    vTaskStartScheduler();
    while (1) tight_loop_contents();
}
