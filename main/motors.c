#include "motors.h"
#include "driver/mcpwm_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/mcpwm_types.h"
#include "soc/clk_tree_defs.h"
#include <driver/gpio.h>
#include <driver/mcpwm_gen.h>
#include <driver/mcpwm_oper.h>
#include <driver/mcpwm_timer.h>
#include <rom/ets_sys.h>
#include <stdio.h>

#define STEP_PIN1 21
#define DIR_PIN1 22

#define STEP_PIN2 16
#define DIR_PIN2 17

#define STEPS_PER_REV 200

#define TAG "MOTORS"

#define MHZ 1000000

// speed in steps per second
int g_speed = 360;

mcpwm_timer_handle_t timer1_handle;
mcpwm_oper_handle_t oper1_handle;
mcpwm_gen_handle_t gen1_handle;

mcpwm_timer_handle_t timer2_handle;
mcpwm_oper_handle_t oper2_handle;
mcpwm_gen_handle_t gen2_handle;

void motor_init(enum motor motor) {
  if (motor == MOTOR1) {
    mcpwm_timer_config_t timer_config;

    timer_config.group_id = 0;
    timer_config.intr_priority = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = MHZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = 1000;

    mcpwm_operator_config_t oper_config;

    oper_config.group_id = 0;
    oper_config.intr_priority = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper1_handle));

    mcpwm_generator_config_t gen_config;

    gen_config.gen_gpio_num = STEP_PIN1;

    ESP_ERROR_CHECK(
        mcpwm_new_generator(oper1_handle, &gen_config, &gen1_handle));

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer1_handle));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1_handle));
  } else if (motor == MOTOR2) {
    // mcpwm_timer_config_t timer_config;
    //
    // timer_config.group_id = 1;
    // timer_config.intr_priority = 0;
    // timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    // timer_config.resolution_hz = MHZ;
    // timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    // timer_config.period_ticks = 500;
    //
    // mcpwm_operator_config_t oper_config;
    //
    // oper_config.group_id = 1;
    // oper_config.intr_priority = 0;
    // ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper2_handle));
    //
    // mcpwm_generator_config_t gen_config;
    //
    // gen_config.gen_gpio_num = STEP_PIN1;
    //
    // ESP_ERROR_CHECK(
    //     mcpwm_new_generator(oper2_handle, &gen_config, &gen2_handle));
    //
    // ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer2_handle));
  }
}

void motor_set_speed(enum motor motor, uint32_t speed) {
  // speed in steps per second
  if (motor == MOTOR1) {
    mcpwm_timer_set_period(timer1_handle, (MHZ / speed));
    g_speed = speed;
  } else if (motor == MOTOR2) {
    mcpwm_timer_set_period(timer2_handle, (MHZ / speed) * 2);
  }
}

void motor_move(enum motor motor, int steps) {
  if (motor == MOTOR1) {
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1_handle, timer1_handle));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen1_handle, (mcpwm_gen_timer_event_action_t){
                         .direction = MCPWM_TIMER_DIRECTION_UP,
                         .event = MCPWM_TIMER_EVENT_FULL,
                         .action = MCPWM_GEN_ACTION_TOGGLE,
                     }));

    ESP_LOGI(TAG, "Moving motor 1 %d steps", steps);
    ESP_ERROR_CHECK(
        mcpwm_timer_start_stop(timer1_handle, MCPWM_TIMER_START_NO_STOP));

    uint32_t dead_time = steps * 1000 / g_speed;

    vTaskDelay(dead_time / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(
        mcpwm_timer_start_stop(timer1_handle, MCPWM_TIMER_STOP_FULL));

  } else if (motor == MOTOR2) {
  }
}

void motor_main(void *pvParameters) {
  motor_params_t *motor_params = (motor_params_t *)pvParameters;

  // enum motor motor = *motor_params->motor;
  enum motor motor = MOTOR1;
  gpio_set_level(DIR_PIN1, motor_params->dir);

  motor_move(motor, motor_params->steps);
}
