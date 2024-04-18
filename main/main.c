#include "bt.h"
#include "mpu6050.h"
#include <stdio.h>

#define TIME_ZONE (-5)   // Bogota, Lima, Quito time zone
#define YEAR_BASE (2000) // date in GPS starts from 2000

#define I2C_MASTER_NUM 0

char *bt_data_str;

void read_imu_task(void *pvParameters) {
  mpu6050_handle_t imu_handle = (mpu6050_handle_t)pvParameters;
  mpu6050_acce_value_t acce;
  mpu6050_gyro_value_t gyro;

  while (1) {
    ESP_ERROR_CHECK(mpu6050_get_acce(imu_handle, &acce));
    ESP_ERROR_CHECK(mpu6050_get_gyro(imu_handle, &gyro));

    sprintf(bt_data_str, "%.4f %.4f %.4f %.4f %.4f %.4f\n", acce.acce_x,
            acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(bt_init());
  bt_data_str = get_data_str();

  mpu6050_handle_t imu_handle =
      mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);

  ESP_ERROR_CHECK(mpu6050_wake_up(imu_handle));
  ESP_ERROR_CHECK(mpu6050_config(imu_handle, ACCE_FS_2G, GYRO_FS_250DPS));

  xTaskCreate(read_imu_task, "read_imu_task", 2048, (void *const)imu_handle, 10,
              NULL);
}
