/*
 * Copyright 2018, Brian Lovin, All rights reserved.
 */

#include "bme280_task.h"
#include "message_buffer.h"
#include <string.h>

#define ACK_VAL 0x0  /*!< I2C ack value */
#define NACK_VAL 0x1 /*!< I2C nack value */
static char *LOG_BME = "BME280";

extern MessageBufferHandle_t xMessageBuffer;

/* Doesn't work if BME280_FLOAT_ENABLE is set */
double centigrade_to_fahrenheit(uint32_t centigrade)
{
  return ((double)centigrade) / 100 * 9 / 5 + 32.0;
}

/*********************************************************************************************************/
/* Temperature is in DegC, resolution is 0.01 DegC.                                                      */
/* Value of “5123” equals 51.23 DegC.                                                                    */
/* Pressure is in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits). */
/* Value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa                                */
/* Humidity is in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).   */
/* Value of “47445” represents 47445/1024 = 46.333 %RH                                                   */
/*********************************************************************************************************/
/* If BME280_FLOAT_ENABLE is set, output temp in degrees C. */
void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
  ESP_LOGI(LOG_BME, "temp %.2fC, p %f, hum %f\r\n",
           comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
  ESP_LOGI(LOG_BME, "temp %.2fF, p %zu, hum %.2f\r\n",
           centigrade_to_fahrenheit(comp_data->temperature),
           comp_data->pressure,
           (float)comp_data->humidity / 1024);
#endif
}

/* Only writes 1 byte to i2c for now, but the driver only wants to write 1 byte, so it works. */
static uint8_t write_register(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  ESP_LOGD(LOG_BME, "----write_register()----\n");
  ESP_LOGD(LOG_BME, "\treg_addr: %#x\n", reg_addr);
  ESP_LOGD(LOG_BME, "\tlen: %d\n", len);
  ESP_LOGD(LOG_BME, "\tdata: %#x\n", *data);

  if (len > 1)
  {
    ESP_LOGE(LOG_BME, "Cannot write more than 1 byte!\n");
    return ESP_FAIL;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t err = BME280_OK;
  i2c_master_start(cmd);
  /* id equals BME280_I2C_ADDR_SEC (0x77) */
  i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
  i2c_master_write_byte(cmd, reg_addr, 1);
  i2c_master_write(cmd, data, len, 1);
  i2c_master_stop(cmd);

  err = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (err == ESP_OK)
  {
    ESP_LOGD(LOG_BME, "Write OK\n");
  }
  else if (err == ESP_ERR_TIMEOUT)
  {
    ESP_LOGE(LOG_BME, "Bus is busy\n");
  }
  else
  {
    ESP_LOGE(LOG_BME, "Write Failed\n");
  }
  return err;
}

static uint8_t read_register(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  ESP_LOGD(LOG_BME, "----read_register()----\n");
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t err = BME280_OK;
  i2c_master_start(cmd);
  //id equals BME280_I2C_ADDR_SEC (0x77)
  i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
  i2c_master_write_byte(cmd, reg_addr, 1);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_READ, 1 /* expect ack */);
  if (len > 1)
    err = i2c_master_read(cmd, data, len - 1, ACK_VAL);
  if (err != BME280_OK)
    return err;

  err = i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  ESP_LOGD(LOG_BME, "\tdata: %#x\n", *data);
  ESP_LOGD(LOG_BME, "\terr: %#x\n", err);
  ESP_LOGD(LOG_BME, "----end read_register()----\n");
  return err;
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  ESP_LOGD(LOG_BME, "----user_i2c_read()----\n");
  ESP_LOGD(LOG_BME, "\treg_addr: %#x\n", reg_addr);
  ESP_LOGD(LOG_BME, "\tlen: %d\n", len);
  if (read_register(id, reg_addr, data, len) != BME280_OK)
    return 1;
  ESP_LOGD(LOG_BME, "\tdata: %#x\n", *data);
  ESP_LOGD(LOG_BME, "----end user_i2c_read()----\n");
  return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  return write_register(id, reg_addr, data, len);
  ;
}

void user_delay_ms(uint32_t period)
{
  vTaskDelay(pdMS_TO_TICKS(period));
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;
  struct bme280_data comp_data;

  /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings(settings_sel, dev);
  if (rslt != BME280_OK)
  {
    ESP_LOGE(LOG_BME, "Error setting BME280 settings: %d\n", rslt);
    abort();
  }

  while (1)
  {
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
    /* Wait for the measurement to complete and print data @25Hz */
    dev->delay_ms(1000);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    if (rslt != BME280_OK)
    {
      ESP_LOGE(LOG_BME, "Error getting BME280 data: %d\n", rslt);
      abort();
    }
    print_sensor_data(&comp_data);
  }
  return rslt;
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;
  struct bme280_data comp_data;

  /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;
  dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  settings_sel = BME280_OSR_PRESS_SEL;
  settings_sel |= BME280_OSR_TEMP_SEL;
  settings_sel |= BME280_OSR_HUM_SEL;
  settings_sel |= BME280_STANDBY_SEL;
  settings_sel |= BME280_FILTER_SEL;
  rslt = bme280_set_sensor_settings(settings_sel, dev);
  if (rslt != BME280_OK)
  {
    ESP_LOGE(LOG_BME, "Error setting BME280 settings: %d\n", rslt);
    abort();
  }
  rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
  if (rslt != BME280_OK)
  {
    ESP_LOGE(LOG_BME, "Error setting BME280 mode: %d\n", rslt);
    abort();
  }

  /* Continuously stream sensor data */
  const TickType_t x100ms = pdMS_TO_TICKS(100);
  char pcStringToSend[100];
  size_t xBytesSent;

  while (1)
  {
    ESP_LOGD(LOG_BME, "HighWaterMark: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

    /* Delay while the sensor completes a measurement */
    dev->delay_ms(70);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    if (rslt != BME280_OK)
    {
      ESP_LOGE(LOG_BME, "Error getting BME280 data: %d\n", rslt);
      abort();
    }
    print_sensor_data(&comp_data);

    configASSERT(xMessageBuffer != NULL);
    snprintf(pcStringToSend, 100, "{\"temp\": %.2f, \"p\": %zu, \"hum\": %.2f}\r\n",
             centigrade_to_fahrenheit(comp_data.temperature),
             comp_data.pressure,
             (float)comp_data.humidity / 1024);

    xBytesSent = xMessageBufferSend(xMessageBuffer,
                                    (void *)pcStringToSend,
                                    strlen(pcStringToSend),
                                    0);
    if (xBytesSent == strlen(pcStringToSend))
    {
      ESP_LOGI(LOG_BME, "message sent.");
    }
    else
    {
      ESP_LOGI(LOG_BME, "no enough space. %s, %d", pcStringToSend, xBytesSent);
    }
  }

  return rslt;
}

void task_bme280(void *ignore)
{
  struct bme280_dev dev;
  esp_err_t e;

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 32;
  conf.scl_io_num = 33;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  ESP_LOGD(LOG_BME, "Setting up I2C");
  e = i2c_param_config(I2C_NUM_1, &conf);
  if (e == ESP_OK)
  {
    e = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
    if (e == ESP_OK)
    {
    }
    else
    {
      ESP_LOGE(LOG_BME, "Error during I2C driver installation: %s", esp_err_to_name(e));
    }
  }
  else
  {
    ESP_LOGE(LOG_BME, "Error setting up I2C: %s", esp_err_to_name(e));
  }

  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  int8_t rslt = bme280_init(&dev);
  if (rslt != BME280_OK)
  {
    ESP_LOGE(LOG_BME, "Error initializing BME280: %d\n", rslt);
    abort();
  }

  //Loops forever, never returns.
  //stream_sensor_data_forced_mode(&dev);
  stream_sensor_data_normal_mode(&dev);

  vTaskDelete(NULL);
}
