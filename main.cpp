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
#include "toolchain.h"
#include "ble/BLE.h"
#include "TMP_nrf51/TMP_nrf51.h"

static Ticker     ticker;
static TMP_nrf51  tempSensor;
static bool       triggerTempValueUpdate = false;
static DigitalOut alivenessLED(LED1, 1);

struct ApplicationData_t {
    uint16_t                     applicationSpecificId; /* An ID used to identify temperature value in the manufacture specific AD data field */
    TMP_nrf51::TempSensorValue_t tmpSensorValue;        /* User defined application data */
} PACKED;

void periodicCallback(void)
{
    alivenessLED = !alivenessLED;  /* Do blinky on LED1 while we're waiting for BLE events. */

    /* Note that the periodicCallback() executes in interrupt context, so it is safer to do
     * heavy-weight sensor polling from the main thread (where we should be able to block safely, if needed). */
    triggerTempValueUpdate = true;
}

void setupApplicationData(ApplicationData_t &appData)
{
    static const uint16_t APP_SPECIFIC_ID_TEST = 0xFEFE;

    appData.applicationSpecificId = APP_SPECIFIC_ID_TEST;
    appData.tmpSensorValue        = tempSensor.get();
}

/**
 * This function is called when the ble initialization process has failled
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    /* Setup advertising payload */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_THERMOMETER);
    ApplicationData_t appData;
    setupApplicationData(appData);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)&appData, sizeof(ApplicationData_t));

    /* Setup advertising parameters */
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(500);

    ble.gap().startAdvertising();
}

int main(void)
{
    ticker.attach(periodicCallback, 2); /* trigger sensor polling every 2 seconds */

    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }

    while (true) {
        if (triggerTempValueUpdate) {
            /* Do blocking calls or whatever hardware-specific action is
             * necessary to poll the sensor. */
            ApplicationData_t appData;
            setupApplicationData(appData);
            ble.gap().updateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *) &appData, sizeof(ApplicationData_t));

            triggerTempValueUpdate = false;
        }

        ble.waitForEvent();
    }
}
