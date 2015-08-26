/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "TMP_nrf51.h"
#include "nrf_soc.h" // for internal thermometer sensor

/**
 * @brief Get the temperature value.
 *
 * @return Die temperature in °C
 */
TMP_nrf51::TempSensorValue_t TMP_nrf51::get(void)
{
    int32_t p_temp;
    sd_temp_get(&p_temp);

    return ((TempSensorValue_t)p_temp * 0.25); /* 0.25 is temperature sensor resolution */
}
