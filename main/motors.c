#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <stdio.h>

#define STEP_PIN1 26
#define DIR_PIN1 25

// #define STEP_PIN2 33
// #define DIR_PIN2 32

#define STEPS_PER_REV 200

#define MOTOR1 1
#define MOTOR2 2

#define TAG "MOTORS"

void motor_init(int motor) {
  if (motor == MOTOR1) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << STEP_PIN1) | (1ULL << DIR_PIN1);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
  } else if (motor == MOTOR2) {
    // gpio_config_t io_conf;
    // io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pin_bit_mask = (1ULL << STEP_PIN2) | (1ULL << DIR_PIN2);
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 0;
    // gpio_config(&io_conf);
  }
}

void motor_move(int motor, int steps) {
  if (motor == MOTOR1) {
    steps = abs(steps);
    for (int i = 0; i < steps; i++) {
      gpio_set_level(STEP_PIN1, 1);
      ets_delay_us(5);
      gpio_set_level(STEP_PIN1, 0);
      ets_delay_us(5);
    }
  } else if (motor == MOTOR2) {
  }
}

void motor_main(void *pvParameters) {
  motor_init(MOTOR1);
  gpio_set_level(DIR_PIN1, 1);

  while (1) {
    motor_move(MOTOR1, 200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
