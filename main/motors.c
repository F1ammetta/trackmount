#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <stdio.h>

#define STEP_PIN1 21
#define DIR_PIN1 22

// #define STEP_PIN2 33
// #define DIR_PIN2 32

#define STEPS_PER_REV 200

#define MOTOR1 1
#define MOTOR2 2
#define LED1 18
#define LED2 19

#define TAG "MOTORS"

void motor_init(int motor) {
  if (motor == MOTOR1) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << STEP_PIN1) | (1ULL << DIR_PIN1) |
                           (1ULL << LED1) | (1ULL << LED2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
  } else if (motor == MOTOR2) {
  }
}

void motor_move(int motor, int steps) {
  if (motor == MOTOR1) {
    for (int i = 0; i < steps; i++) {
      gpio_set_level(STEP_PIN1, 1);
      vTaskDelay(13 / portTICK_PERIOD_MS);
      gpio_set_level(STEP_PIN1, 0);
      vTaskDelay(13 / portTICK_PERIOD_MS);
    }
  } else if (motor == MOTOR2) {
  }
}

typedef struct {
  int steps;
  int dir;
  int led1;
  int led2;
} motor_params_t;

void motor_main(void *pvParameters) {
  motor_params_t *motor_params = (motor_params_t *)pvParameters;
  gpio_set_level(LED1, motor_params->led1);
  gpio_set_level(LED2, motor_params->led2);

  motor_init(MOTOR1);
  gpio_set_level(DIR_PIN1, motor_params->dir);
  gpio_set_level(STEP_PIN1, 1);
  vTaskDelay(13 / portTICK_PERIOD_MS);
  gpio_set_level(STEP_PIN1, 0);

  motor_move(MOTOR1, motor_params->steps);
}
