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

#include "bme280_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "message_buffer.h"

#include "esp_system.h"
#include "esp_spi_flash.h"

static const char *TAG = "labn_sensor";

#ifndef IOT_DEMO_MQTT_TOPIC_PREFIX
#define IOT_DEMO_MQTT_TOPIC_PREFIX "data/mydevice"
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
    const TickType_t xBlockTime = pdMS_TO_TICKS(1000);

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
