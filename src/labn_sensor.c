/**
 * @file labn_sensor.c
 * @brief LabN: Receive BME280 sensor data and send MQTT message.
 *
 * (C) 2020 - Tatsuhiro Iida 
 * This code is licensed under the MIT License.
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Set up logging for this demo. */
#include "iot_demo_logging.h"

/* Platform layer includes. */
#include "platform/iot_clock.h"

/* MQTT include. */
#include "iot_mqtt.h"

#include "aws_demo.h"
#include "types/iot_network_types.h"
#include "esp_log.h"

#include "device.h"
#include "lab_config.h"
#include "lab_connection.h"
#include "labn_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "message_buffer.h"

#include "esp_system.h"
#include "esp_spi_flash.h"

#include "bme280.h"
#include "bme280_defs.h"

static const char *TAG = "labn_sensor";

#ifndef IOT_DEMO_MQTT_TOPIC_PREFIX
#define IOT_DEMO_MQTT_TOPIC_PREFIX "data/area-1"
#endif
/** @endcond */

/**
 * @brief The timeout for MQTT operations in this demo.
 */
#define MQTT_TIMEOUT_MS (5000)

/**
 * @brief The first characters in the client identifier. A timestamp is appended
 * to this prefix to create a unique client identifer.
 *
 * This prefix is also used to generate topic names and topic filters used in this
 * demo.
 */
#define CLIENT_IDENTIFIER_PREFIX "mydevice"

/**
 * @brief The longest client identifier that an MQTT server must accept (as defined
 * by the MQTT 3.1.1 spec) is 23 characters. Add 1 to include the length of the NULL
 * terminator.
 */
#define CLIENT_IDENTIFIER_MAX_LENGTH (24)

/**
 * @brief The keep-alive interval used for this demo.
 *
 * An MQTT ping request will be sent periodically at this interval.
 */
#define KEEP_ALIVE_SECONDS (60)

/**
 * @brief The Last Will and Testament topic name in this demo.
 *
 * The MQTT server will publish a message to this topic name if this client is
 * unexpectedly disconnected.
 */
#define WILL_TOPIC_NAME IOT_DEMO_MQTT_TOPIC_PREFIX "/will"

/**
 * @brief The length of #WILL_TOPIC_NAME.
 */
#define WILL_TOPIC_NAME_LENGTH ((uint16_t)(sizeof(WILL_TOPIC_NAME) - 1))

/**
 * @brief The message to publish to #WILL_TOPIC_NAME.
 */
#define WILL_MESSAGE "MQTT demo unexpectedly disconnected."

/**
 * @brief The length of #WILL_MESSAGE.
 */
#define WILL_MESSAGE_LENGTH ((size_t)(sizeof(WILL_MESSAGE) - 1))

/**
 * @brief The length of topic filter.
 *
 * For convenience, all topic filters are the same length.
 */
#define TOPIC_FORMAT IOT_DEMO_MQTT_TOPIC_PREFIX "/%s"
#define TOPIC_BUFFER_LENGTH ((uint16_t)(sizeof(TOPIC_FORMAT) + 12))

/**
 * @brief Format string of the PUBLISH messages in this demo.
 */
#define PUBLISH_PAYLOAD_FORMAT_SINGLE "{\"serialNumber\": \"%s\",\"clickType\": \"SINGLE\"}"
#define PUBLISH_PAYLOAD_FORMAT_HOLD "{\"serialNumber\": \"%s\",\"clickType\": \"HOLD\"}"

/**
 * @brief Size of the buffer that holds the PUBLISH messages in this demo.
 */
#define PUBLISH_PAYLOAD_BUFFER_LENGTH (sizeof(PUBLISH_PAYLOAD_FORMAT_SINGLE) + 12)

/**
 * @brief The maximum number of times each PUBLISH in this demo will be retried.
 */
#define PUBLISH_RETRY_LIMIT (10)

/**
 * @brief A PUBLISH message is retried if no response is received within this
 * time.
 */
#define PUBLISH_RETRY_MS (1000)

/**
 * @brief The topic name on which acknowledgement messages for incoming publishes
 * should be published.
 */
#define ACKNOWLEDGEMENT_TOPIC_NAME IOT_DEMO_MQTT_TOPIC_PREFIX "/acknowledgements"

/**
 * @brief The length of #ACKNOWLEDGEMENT_TOPIC_NAME.
 */
#define ACKNOWLEDGEMENT_TOPIC_NAME_LENGTH ((uint16_t)(sizeof(ACKNOWLEDGEMENT_TOPIC_NAME) - 1))

/**
 * @brief Format string of PUBLISH acknowledgement messages in this demo.
 */
#define ACKNOWLEDGEMENT_MESSAGE_FORMAT "Client has received PUBLISH %.*s from server."

/**
 * @brief Size of the buffers that hold acknowledgement messages in this demo.
 */
#define ACKNOWLEDGEMENT_MESSAGE_BUFFER_LENGTH (sizeof(ACKNOWLEDGEMENT_MESSAGE_FORMAT) + 2)

void task_bme280(void *ignore);

/*-----------------------------------------------------------*/

/**
 * @brief Called by the MQTT library when an operation completes.
 *
 * The demo uses this callback to determine the result of PUBLISH operations.
 * @param[in] param1 The number of the PUBLISH that completed, passed as an intptr_t.
 * @param[in] pOperation Information about the completed operation passed by the
 * MQTT library.
 */
static void _operationCompleteCallback(void *param1,
                                       IotMqttCallbackParam_t *const pOperation)
{
    intptr_t publishCount = (intptr_t)param1;

    /* Silence warnings about unused variables. publishCount will not be used if
     * logging is disabled. */
    (void)publishCount;

    /* Print the status of the completed operation. A PUBLISH operation is
     * successful when transmitted over the network. */
    if (pOperation->u.operation.result == IOT_MQTT_SUCCESS)
    {
        IotLogInfo("MQTT %s %d successfully sent.",
                   IotMqtt_OperationType(pOperation->u.operation.type),
                   (int)publishCount);
    }
    else
    {
        IotLogError("MQTT %s %d could not be sent. Error %s.",
                    IotMqtt_OperationType(pOperation->u.operation.type),
                    (int)publishCount,
                    IotMqtt_strerror(pOperation->u.operation.result));
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief Transmit message.
 *
 * @param[in] pTopicName The topic name for publishing.
 * @param[in] pPayload The payload for publishing.
 *
 * @return `EXIT_SUCCESS` if all messages are published; `EXIT_FAILURE` otherwise.
 */
static int _publishMessage(const char *pTopicName,
                           const char *pPayload)
{
    int status = EXIT_SUCCESS;

    IotMqttPublishInfo_t publishInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    IotMqttCallbackInfo_t publishComplete = IOT_MQTT_CALLBACK_INFO_INITIALIZER;

    /* The MQTT library should invoke this callback when a PUBLISH message
     * is successfully transmitted. */
    publishComplete.function = _operationCompleteCallback;

    /* Set the common members of the publish info. */
    publishInfo.qos = IOT_MQTT_QOS_1;
    publishInfo.topicNameLength = strlen(pTopicName);
    publishInfo.pPayload = pPayload;
    publishInfo.payloadLength = strlen(pPayload);
    publishInfo.pTopicName = pTopicName;
    publishInfo.retryMs = PUBLISH_RETRY_MS;
    publishInfo.retryLimit = PUBLISH_RETRY_LIMIT;

    status = eLabConnectionPublish(&publishInfo, &publishComplete);

    return status;
}

MessageBufferHandle_t xMessageBuffer = NULL;

void task_publish_sensor_data(void *ignore)
{
    uint8_t ucRxData[100];
    size_t xReceivedBytes;
    const TickType_t xBlockTime = pdMS_TO_TICKS(11000);

    configASSERT(xMessageBuffer != NULL);

    for (;;)
    {
        xReceivedBytes = xMessageBufferReceive(xMessageBuffer,
                                               (void *)ucRxData,
                                               sizeof(ucRxData),
                                               xBlockTime);

        if (xReceivedBytes > 0)
        {
            char pTopic[TOPIC_BUFFER_LENGTH] = {0};

            esp_err_t res = ESP_FAIL;
            snprintf(pTopic, TOPIC_BUFFER_LENGTH, TOPIC_FORMAT,"M5StickC");
            
            res = _publishMessage(pTopic, (const char *)ucRxData);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to publish");
            }
        }
        else
        {
            ESP_LOGE(TAG, "cannot receive. xReceivedBytes: %d", xReceivedBytes);
        }
    }
}

/*-----------------------------------------------------------*/

esp_err_t eLabNInit(const char *const strID)
{
    ESP_LOGI(TAG, "eLabNInit: Init");

    static iot_connection_params_t connectionParams;

    connectionParams.strID = (char *)strID;
    connectionParams.useShadow = false;
    connectionParams.networkConnectedCallback = NULL;
    connectionParams.networkDisconnectedCallback = NULL;

    const size_t xMessageBufferSizeBytes = 200;

    /* Create a stream buffer that can hold 100 bytes.  The memory used to hold
    both the stream buffer structure and the data in the stream buffer is
    allocated dynamically. */
    xMessageBuffer = xMessageBufferCreate(xMessageBufferSizeBytes);

    xTaskCreate(&task_bme280, "task_bme280", 4096, NULL, 5, NULL);

    xTaskCreate(&task_publish_sensor_data, "task_publish_sensor_data", 4096, NULL, 5, NULL);

    return eLabConnectionInit(&connectionParams);
}

/*-----------------------------------------------------------*/

esp_err_t eLabNAction(const char *strID, int32_t buttonID)
{
    esp_err_t res = ESP_FAIL;

    if (bIsLabConnectionMqttConnected())
    {
        ESP_LOGI(TAG, "labn_action: %d", buttonID);

        /* Topic and Payload buffers */
        char pTopic[TOPIC_BUFFER_LENGTH] = {0};
        char pPublishPayload[PUBLISH_PAYLOAD_BUFFER_LENGTH] = {0};

        /* Generate the payload for the PUBLISH. */
        int status = -1;

        if (buttonID == BUTTON_CLICK)
        {
            status = snprintf(pPublishPayload, PUBLISH_PAYLOAD_BUFFER_LENGTH, PUBLISH_PAYLOAD_FORMAT_SINGLE, strID);
        }
        if (buttonID == BUTTON_HOLD)
        {
            status = snprintf(pPublishPayload, PUBLISH_PAYLOAD_BUFFER_LENGTH, PUBLISH_PAYLOAD_FORMAT_HOLD, strID);
        }

        /* Check for errors from snprintf. */
        if (status < 0)
        {
            IotLogError("Failed to generate MQTT PUBLISH payload: %d.", (int)status);
            return ESP_FAIL;
        }
        else
        {
            /* Generate the topic. */
            status = snprintf(pTopic, TOPIC_BUFFER_LENGTH, TOPIC_FORMAT, strID);
        }

        if (status < 0)
        {
            IotLogError("Failed to generate MQTT payload topic: %d.", (int)status);
            return ESP_FAIL;
        }

        res = _publishMessage(pTopic, pPublishPayload);
    }
    else
    {
        ESP_LOGI(TAG, "labn_action: %d. Skipped due to lack of available MQTT connection", buttonID);
    }

    return res;
}

/*-----------------------------------------------------------*/

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_READ, 1);

    esp_err_t err = ESP_OK;
    if (len > 1)
        err = i2c_master_read(cmd, data, len - 1, 0x00);
    if (err != BME280_OK)
        return err;

    err = i2c_master_read_byte(cmd, data + len - 1, 0x01);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
        return 1;
    return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (len > 1) {
        ESP_LOGE(TAG, "Cannot write more than 1 byte!\n");
        return ESP_FAIL;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_write(cmd, data, len, 1);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write Failed %d\n", err);
    }
    return err;
}

void user_delay_ms(uint32_t period) {
    vTaskDelay(pdMS_TO_TICKS(period));
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev) {
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

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
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Error setting BME280 settings: %d\n", rslt);
        abort();
    }
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Error setting BME280 mode: %d\n", rslt);
        abort();
    }

    char pcStringToSend[100];
    char pcStringToDisplay[20];
    size_t xBytesSent;

    while (1) {
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK) {
            ESP_LOGE(TAG, "Error getting BME280 data: %d\n", rslt);
            abort();
        }

        snprintf(pcStringToDisplay, 100, "h:%5.1f, t:%5.1f",
                 (float) comp_data.humidity,
                 (float) comp_data.temperature);
        TFT_print(pcStringToDisplay, CENTER, 44/*SCREEN_LINE_4*/);


        configASSERT(xMessageBuffer != NULL);
        snprintf(pcStringToSend, 100, "{\"humidity\":%5.2f, \"pressure\":%7.2f, \"temperature\":%5.2f}\r\n",
                 (float) comp_data.humidity,
                 (float) comp_data.pressure / 100,
                 (float) comp_data.temperature);

        xBytesSent = xMessageBufferSend(xMessageBuffer,
                                        (void *) pcStringToSend,
                                        strlen(pcStringToSend),
                                        0);

        if (xBytesSent == strlen(pcStringToSend)) {
            ESP_LOGI(TAG, "message sent.");
        } else {
            ESP_LOGI(TAG, "no enough space. %s, %d", pcStringToSend, xBytesSent);
        }
        user_delay_ms(10000);
    }

    return rslt;
}

void task_bme280(void *ignore) {

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 32;
    conf.scl_io_num = 33;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    esp_err_t err = i2c_param_config(I2C_NUM_1, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting up I2C: %s", esp_err_to_name(err));
    }

    err = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
    }

    struct bme280_dev dev;
    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    int8_t rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Error initializing BME280: %d\n", rslt);
        abort();
    }

    stream_sensor_data_normal_mode(&dev);
    vTaskDelete(NULL);
}
