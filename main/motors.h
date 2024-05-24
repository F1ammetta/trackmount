
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <stdio.h>

enum motor { MOTOR1, MOTOR2 };

// void motor_init();
typedef struct {
  enum motor *motor;
  int steps;
  int dir;
} motor_params_t;

void motor_set_speed(enum motor motor, uint32_t speed);
void motor_main(void *pvParameters);
void motor_init(enum motor motor);
