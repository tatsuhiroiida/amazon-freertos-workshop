/**
 * @file labn_sensor.h
 *
 * (C) 2020 - Tatsuhiro Iida
 * This code is licensed under the MIT License.
 */

#ifndef _LABN_SENSOR_H_
#define _LABN_SENSOR_H_

#include "esp_err.h"

#if defined(LAB_INIT)
    #undef LAB_INIT
    #define LAB_INIT(x) eLabNInit(x)
#endif

esp_err_t eLabNInit(const char *strID);
esp_err_t eLabNAction( const char * strID, int32_t buttonID );

#endif /* ifndef _LABN_SENSOR_H_ */
