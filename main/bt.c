#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "nmea_parser.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sys/time.h"
#include "time.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPP_TAG "TrackMount"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "Sidus Mount"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA /*Choose show mode: show data or speed*/

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t *bda, char *str, size_t size) {
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }

  uint8_t *p = bda;
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4],
          p[5]);
  return str;
}

char *data;
bool connected = false;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  char bda_str[18] = {0};

  switch (event) {
  case ESP_SPP_INIT_EVT:
    if (param->init.status == ESP_SPP_SUCCESS) {
      ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
      esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
    } else {
      ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
    }
    break;
  case ESP_SPP_DISCOVERY_COMP_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
    break;
  case ESP_SPP_OPEN_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");

    break;
  case ESP_SPP_CLOSE_EVT:
    ESP_LOGI(SPP_TAG,
             "ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32
             " close_by_remote:%d",
             param->close.status, param->close.handle, param->close.async);
    break;
  case ESP_SPP_START_EVT:
    if (param->start.status == ESP_SPP_SUCCESS) {
      ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d",
               param->start.handle, param->start.sec_id, param->start.scn);
      esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    } else {
      ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
    }
    break;
  case ESP_SPP_CL_INIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
    break;
  case ESP_SPP_DATA_IND_EVT:
    /*
     * We only show the data in which the data length is less than 128 here.
     * If you want to print the data and the data rate is high, it is
     * strongly recommended to process them in other lower priority
     * application task rather than in this callback directly. Since the
     * printing takes too much time, it may stuck the Bluetooth stack and
     * also have a effect on the throughput!
     */
    ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%" PRIu32,
             param->data_ind.len, param->data_ind.handle);
    if (param->data_ind.len < 128) {
      esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
      // parse the data into a string
      char *data = (char *)malloc(param->data_ind.len + 1);
      memcpy(data, param->data_ind.data, param->data_ind.len);
      data[param->data_ind.len] = '\0';
      ESP_LOGI(SPP_TAG, "Received data: %s", data);
      // parse the data
      motor_params_t *motor_params =
          (motor_params_t *)malloc(sizeof(motor_params_t));
      char *token = strtok(data, ",");
      motor_params->steps = atoi(token);
      token = strtok(NULL, ",");
      motor_params->dir = atoi(token);
      ESP_LOGI(SPP_TAG, "Motor params: steps: %d, dir: %d", motor_params->steps,
               motor_params->dir);
      // move the motor
      motor_main(motor_params);
    }
    break;
  case ESP_SPP_CONG_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
    // check if the client is ready to receive data
    // if not, wait for 100ms and try again
    if (!param->cong.cong && connected) {
      // vTaskDelay(100 / portTICK_PERIOD_MS);
      // esp_spp_write(param->cong.handle, strlen(data), (uint8_t *)data);
    }
    break;
  case ESP_SPP_WRITE_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
    if (!param->write.cong && connected) {
      // vTaskDelay(100 / portTICK_PERIOD_MS);
      // esp_spp_write(param->cong.handle, strlen(data), (uint8_t *)data);
    }
    break;
  case ESP_SPP_SRV_OPEN_EVT:
    ESP_LOGI(SPP_TAG,
             "ESP_SPP_SRV_OPEN_EVT status:%d handle:%" PRIu32 ", rem_bda:[%s]",
             param->srv_open.status, param->srv_open.handle,
             bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
    // send data to the client send greeting message
    esp_spp_write(param->srv_open.handle, strlen("Starting"),
                  (uint8_t *)"Starting");

    connected = true;
    break;
  case ESP_SPP_SRV_STOP_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
    connected = false;
    break;
  case ESP_SPP_UNINIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
    break;
  default:
    break;
  }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  char bda_str[18] = {0};

  switch (event) {
  case ESP_BT_GAP_AUTH_CMPL_EVT: {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]",
               param->auth_cmpl.device_name,
               bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
    } else {
      ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
               param->auth_cmpl.stat);
    }
    break;
  }
  case ESP_BT_GAP_PIN_REQ_EVT: {
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d",
             param->pin_req.min_16_digit);
    if (param->pin_req.min_16_digit) {
      ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
      esp_bt_pin_code_t pin_code = {0};
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
    } else {
      ESP_LOGI(SPP_TAG, "Input pin code: 1234");
      esp_bt_pin_code_t pin_code;
      pin_code[0] = '1';
      pin_code[1] = '2';
      pin_code[2] = '3';
      pin_code[3] = '4';
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
    break;
  }

  case ESP_BT_GAP_CFM_REQ_EVT:
    ESP_LOGI(
        SPP_TAG,
        "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %" PRIu32,
        param->cfm_req.num_val);
    esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
    break;
  case ESP_BT_GAP_KEY_NOTIF_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%" PRIu32,
             param->key_notif.passkey);
    break;
  case ESP_BT_GAP_KEY_REQ_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
    break;
  case ESP_BT_GAP_MODE_CHG_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]",
             param->mode_chg.mode,
             bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
    break;

  default: {
    ESP_LOGI(SPP_TAG, "event: %d", event);
    break;
  }
  }
  return;
}

char *get_data_str() { return data; }

esp_err_t bt_init() {
  data = (char *)malloc(100);

  char bda_str[18] = {0};
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__,
             esp_err_to_name(ret));
    return ret;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__,
             esp_err_to_name(ret));
    return ret;
  }

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__,
             esp_err_to_name(ret));
    return ret;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__,
             esp_err_to_name(ret));
    return ret;
  }

  if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__,
             esp_err_to_name(ret));
    return ret;
  }

  if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__,
             esp_err_to_name(ret));
    return ret;
  }

  esp_spp_cfg_t bt_spp_cfg = {
      .mode = esp_spp_mode,
      .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
      .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
  };
  if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
    return ret;
  }

  /* Set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  /*
   * Set default parameters for Legacy Pairing
   * Use variable pin, input pin code when pairing
   */
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
  esp_bt_pin_code_t pin_code;
  esp_bt_gap_set_pin(pin_type, 0, pin_code);

  ESP_LOGI(
      SPP_TAG, "Own address:[%s]",
      bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
  return ESP_OK;
}
