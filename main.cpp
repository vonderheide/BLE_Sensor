
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



BLE ble;

DigitalOut alivenessLED(LED1, 1);
TMP_nrf51 tempSensor;

bool triggerLedCharacteristic = false;

uint8_t ADV_INFO[6] = {0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00}; /* Special character || temperature value */
uint8_t fNewTempValue;

void periodicCallback(void) {
    alivenessLED = !alivenessLED; /* Do blinky on LED1 while we're waiting for BLE events */
    fNewTempValue = 1;
}



int main(void) {
    Ticker ticker;
    /* Refresh temperature value every 2 seconds */
    ticker.attach(periodicCallback, 2);

    ble.init();

    while (true) {
        if (fNewTempValue) {
            float tempVal;
            tempVal = tempSensor.get();
            memcpy(&ADV_INFO[2], &tempVal, 4); /* 4 bytes left for tempVal */
            printf("temp is %f\r\n", tempVal);
            
            if((ble.gap().getState()).advertising == 1) {
                ble.gap().stopAdvertising();
                ble.gap().clearAdvertisingPayload();
            }
            /* Setup advertising. */
            ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_LIMITED_DISCOVERABLE);
            ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::UNKNOWN);
            ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)ADV_INFO, sizeof(ADV_INFO));
            ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
            ble.gap().setAdvertisingInterval(500);
    
            ble.gap().startAdvertising();
            
            fNewTempValue = 0;
        }
        ble.waitForEvent();
    }
}
