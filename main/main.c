#include "bmx280.h"
#include "bt.h"
#include "mpu6050.h"
#include <stdio.h>

#define TIME_ZONE (-5)   // Bogota, Lima, Quito time zone
#define YEAR_BASE (2000) // date in GPS starts from 2000

#define I2C_MASTER_NUM 0

char *bt_data_str;

typedef struct {
  mpu6050_handle_t imu_handle;
  bmx280_t *bmx280;
} sensor_data_t;

void read_sensors_task(void *pvParameters) {
  sensor_data_t *sensor_data = (sensor_data_t *)pvParameters;
  mpu6050_handle_t imu_handle = sensor_data->imu_handle;
  mpu6050_acce_value_t acce;
  mpu6050_gyro_value_t gyro;
  float temp = 0, pres = 0;

  bmx280_t *bmx280 = sensor_data->bmx280;

  while (1) {
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
    do {
      vTaskDelay(pdMS_TO_TICKS(1));
    } while (bmx280_isSampling(bmx280));

    ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, NULL));
    ESP_ERROR_CHECK(mpu6050_get_acce(imu_handle, &acce));
    ESP_ERROR_CHECK(mpu6050_get_gyro(imu_handle, &gyro));

    sprintf(bt_data_str, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
            acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y,
            gyro.gyro_z, temp, pres);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(bt_init());
  bt_data_str = get_data_str();

  mpu6050_handle_t imu_handle =
      mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);

  ESP_ERROR_CHECK(mpu6050_wake_up(imu_handle));
  ESP_ERROR_CHECK(mpu6050_config(imu_handle, ACCE_FS_4G, GYRO_FS_250DPS));

  bmx280_t *bmx280 = bmx280_create(I2C_MASTER_NUM);

  if (!bmx280) {
    ESP_LOGE("BMX280", "BMX280 initialization failed");
    return;
  }

  ESP_ERROR_CHECK(bmx280_init(bmx280));

  bmx280_config_t config = ((bmx280_config_t){
      BMX280_TEMPERATURE_OVERSAMPLING_X16, BMX280_PRESSURE_OVERSAMPLING_X16,
      BME280_STANDBY_20M, BMX280_IIR_X16, BMX280_HUMIDITY_OVERSAMPLING_X16});

  ESP_ERROR_CHECK(bmx280_configure(bmx280, &config));

  sensor_data_t sensor_data = {.imu_handle = imu_handle, .bmx280 = bmx280};

  xTaskCreate(read_sensors_task, "read_imu_task", 2048, (void *)&sensor_data,
              10, NULL);
}
