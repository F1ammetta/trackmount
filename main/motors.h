
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <stdio.h>

// void motor_init();
typedef struct {
  int steps;
  int dir;
  int led1;
  int led2;
} motor_params_t;

void motor_main();
void motor_init(int motor);
