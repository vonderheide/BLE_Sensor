
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

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "TMP_nrf51/TMP_nrf51.h"

#define COMP_ID_TEST 0xFEFE

#pragma pack(1)
typedef struct manufacturerData {
    uint16_t companyId;
    /* User defined manufacture data */
    TMP_nrf51::tmpSensorValue_t tmpSensorValue;
} manufacturerData_t;
#pragma pack()

BLE ble;
TMP_nrf51 tempSensor;
DigitalOut alivenessLED(LED1, 1);
static bool triggerTempValueRead = true;

void periodicCallback(void)
{
    /* Do blinky on LED1 while we're waiting for BLE events */
    alivenessLED = !alivenessLED;
    triggerTempValueRead = true;
}

void temperatureValueAdvertising(void)
{
    manufacturerData_t manuData;
    manuData.companyId = COMP_ID_TEST;
    
    /* Read a new temperature value */
    manuData.tmpSensorValue = tempSensor.get();
    printf("Temp is %f\r\n", (float)manuData.tmpSensorValue);
    
    /* Stop advertising and clear the payload if in advertising state */
    if((ble.gap().getState()).advertising == 1) {
        ble.gap().stopAdvertising();
        ble.gap().clearAdvertisingPayload();
    }
    
    /* Setup advertising payload */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE); /* Set flag */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_THERMOMETER); /* Set appearance */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)&manuData, sizeof(manufacturerData_t)); /* Set data */
    /* Setup advertising parameters */
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(500);
    /* Start advertising */
    ble.gap().startAdvertising();
}

int main(void)
{
    Ticker ticker;
    /* Refresh temperature value every 2 seconds */
    ticker.attach(periodicCallback, 2);

    ble.init();

    while (true) {
        if (triggerTempValueRead) {
            temperatureValueAdvertising();
            triggerTempValueRead = false;
        }
        ble.waitForEvent();
    }
}
