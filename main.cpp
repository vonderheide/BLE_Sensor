
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
#include "TMP_nrf51/TMP_nrf51.h"

#define APP_SPECIFIC_ID_TEST 0xFEFE

#pragma pack(1)
struct ApplicationData_t {
    uint16_t applicationSpecificId;             /* An ID used to identify temperature value
                                                   in the manufacture specific AD data field */
    TMP_nrf51::tmpSensorValue_t tmpSensorValue; /* User defined application data */
};
#pragma pack()

BLE ble;
TMP_nrf51 tempSensor;
DigitalOut alivenessLED(LED1, 1);
static bool triggerTempValueUpdate = false;

void periodicCallback(void)
{
    /* Do blinky on LED1 while we're waiting for BLE events */
    alivenessLED = !alivenessLED;
    triggerTempValueUpdate = true;
}

void accumulateApplicationData(ApplicationData_t &appData)
{
    appData.applicationSpecificId = APP_SPECIFIC_ID_TEST;
    /* Read a new temperature value */
    appData.tmpSensorValue = tempSensor.get();
}

void temperatureValueAdvertising(void)
{
    ApplicationData_t appData;
    
    accumulateApplicationData(appData);
    //printf("Temp is %f\r\n", (float)appData.tmpSensorValue);
    
    /* Setup advertising payload */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE); /* Set flag */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_THERMOMETER); /* Set appearance */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)&appData, sizeof(ApplicationData_t)); /* Set data */
    /* Setup advertising parameters */
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(500);
    /* Start advertising */
    ble.gap().startAdvertising();
}

void updateSensorValueInAdvPayload(void)
{
    ApplicationData_t appData;
    
    accumulateApplicationData(appData);
    
    /* Stop advertising first */
    ble.gap().stopAdvertising();
    /* Only update temperature value field */
    ble.gap().updateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)&appData, sizeof(ApplicationData_t));
    /* Start advertising again */
    ble.gap().startAdvertising();
}

int main(void)
{
    Ticker ticker;
    /* Enable trigger every 2 seconds */
    ticker.attach(periodicCallback, 2);

    ble.init();
    /* Start temperature advertising */
    temperatureValueAdvertising();
    
    while (true) {
        if (triggerTempValueUpdate) {
            /* Update temperature value */
            updateSensorValueInAdvPayload();
            triggerTempValueUpdate = false;
        }
        ble.waitForEvent();
    }
}
