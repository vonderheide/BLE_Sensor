/*
 * Copyright (c) Eric Tsai 2017
 *
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
 *
 *
 * Credit: started with the basic BLE Temperature Beacon code from mbed
 * Bluetooth Low Energy team
 * https://developer.mbed.org/teams/Bluetooth-Low-Energy/code/BLE_TemperatureBeacon/file/0a8bbb6dea16/main.cpp
 *
 * BLE sensor as Beacon advertisements.  Intended to function with specific BLE
 * observer. Tested on nRF51822 targets on mbed. keywords:  todo, tochange
 */

/*
 * Updated 2022 by Jens Vonderheide to use the BTHome (https://bthome.io/)
 * advertising format
 *
 * Note that this makes it incompatible with the corresponding BLE gateway.
 *
 * Additional changes:
 * - Removed the "birthday" spoof mechanism
 * - Some refactoring
 */

#include <cstdint>
#include <vector>
extern "C" {
#include "nrf_ecb.h" //required to call the ecb functions for encryption
}

#include "ble/BLE.h"
#include "mbed.h"
#include "toolchain.h"

/*******************************************************************************************
 * START tochange: items that may need customization depending on sensors,
 * hardware, and desired behavior
 *******************************************************************************************/

// enables serial output for debug, consumes ~1mA when idle
#define MyDebugEnb 1

// number of seconds between periodic I/O status re-transmits
const uint16_t Periodic_Update_Seconds = 20;

// local advertised name
const static char DEVICE_NAME[] = "REED1";

// hardware interrupt pins, selected based on hardware
// Syntax:  Pin "P0.4" on nRF51822 documentation is mbed "p4".
// InterruptIn is pulled-up.  GND the pin to activate.
InterruptIn button1(p0); // nRF51822 P0.0
InterruptIn button2(p1); // nRF51822 P0.1

/******************************************************************************************
 * END tochange
 *******************************************************************************************/

#if MyDebugEnb
// if you see ~1mA consumption during sleep, that's because MyDebugEnb==1, it's
// enabled.
Serial device(p9, p11); // nRF51822 uart :  TX=p9.  RX=p11
#endif

// used to stop advertising after X seconds
static Ticker Tic_Stop_Adv;

// debounce I/O
static Ticker Tic_Debounce;

// transmit sensor data on a periodic basis outside I/O events
static Ticker Tic_Periodic;

enum MainLoopAction {
  action_none,
  action_start_announce,
  action_stop_announce
};

static MainLoopAction NextAction = action_none;

void debounce_Callback(void) {
  Tic_Debounce.detach();
  NextAction = action_start_announce;
}

void buttonToggledCallback(void) { Tic_Debounce.attach(debounce_Callback, 1); }

void stop_adv_Callback(void) { NextAction = action_stop_announce; }

/* ****************************************
 * Decides what actions need to be performed on periodic basis
 *******************************************/
void periodic_Callback(void) { NextAction = action_start_announce; }

/* ****************************************
 * Read battery voltage using bandgap reference
 * shunt Vdd to ADC, thanks to Marcelo Salazar's notes here:
 * https://developer.mbed.org/users/MarceloSalazar/notebook/measuring-battery-voltage-with-nordic-nrf51x/
 *******************************************/
float read_bat_volt(void) {
  // 10 bit resolution, route Vdd as analog input, set ADC ref to VBG band gap
  // disable analog pin select "PSEL" because we're using Vdd as analog input
  // no external voltage reference
  NRF_ADC->CONFIG =
      (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  NRF_ADC->TASKS_START = 1;

  // while loop doesn't actually loop until reading comlete, use a wait.
  while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) ==
         ADC_BUSY_BUSY_Busy) {
  };
  wait_ms(1);

  // disable ADC to lower bat consumption
  NRF_ADC->TASKS_STOP = 1;

  uint16_t adcResult = (uint16_t)NRF_ADC->RESULT;
  return (adcResult * 3.6) / 1024.0;
}

void setup_button(InterruptIn &button, bool enable) {
  if (enable) {
    button.fall(buttonToggledCallback);
    button.rise(buttonToggledCallback);
    button.mode(PullUp);
  } else {
    button.fall(NULL);
    button.rise(NULL);
    button.mode(PullNone);
  }
}

void setup_buttons(bool listen_button1, bool listen_button2) {
  setup_button(button1, listen_button1);
  setup_button(button2, listen_button2);
}

uint8_t read_sensor() {
  // set both pins to pull-up, so they're not floating when we read state
  button1.mode(PullUp);
  button2.mode(PullUp);

  // expect either button1 or button2 is grounded, b/c using SPDT reed switch
  // the "common" pin on the reed switch should be on GND
  uint8_t button1_state = button1.read();
  uint8_t button2_state = button2.read();

  // let's just update the pins on every wake.  Insurance against const drain.
  // if state == 0, pin is grounded.  Unset interrupt and float pin, set the
  // other pin for ISR
  uint8_t magnet_near = 0;
  if ((button1_state == 0) && (button2_state == 1)) {
    magnet_near = 1;
    setup_buttons(false, true);
#if MyDebugEnb
    device.printf("=== button 1!\r\n");
#endif
  } else if ((button1_state == 1) && (button2_state == 0)) {
    magnet_near = 0;
    setup_buttons(true, false);
#if MyDebugEnb
    device.printf("=== button 2!\r\n");
#endif
  } else {
    // odd state, shouldn't happen, suck battery and pullup both pins
    magnet_near = 2;
    setup_buttons(true, true);
#if MyDebugEnb
    device.printf("no buttons!!\r\n");
#endif
  } // end odd state

  return magnet_near;
}

std::vector<uint8_t> buildBtHomePayload(bool isOpened, float batteryVoltage) {
  std::vector<uint8_t> result;

  // unencrypted
  result.push_back(0x1c);
  result.push_back(0x18);

  // sensor payload, object id "opening"
  result.push_back(0x02);
  result.push_back(0x11);
  result.push_back(isOpened ? 1 : 0);

  // battery voltage payload
  uint16_t voltage = (uint16_t)(batteryVoltage * 1000);
  result.push_back(0x03);
  result.push_back(0x0c);
  result.push_back(voltage & 0xff);
  result.push_back((voltage >> 8) & 0xff);

  return result;
}

void setupAnnouncement(BLE &ble, const std::vector<uint8_t> &btHomePayload) {
  ble.gap().clearAdvertisingPayload();
  ble.gap().accumulateAdvertisingPayload(
      GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME,
      sizeof(DEVICE_NAME));
  ble.gap().accumulateAdvertisingPayload(
      GapAdvertisingData::BREDR_NOT_SUPPORTED |
      GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  ble.gap().setAdvertisingType(
      GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
  ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::SERVICE_DATA,
                                         &btHomePayload[0],
                                         btHomePayload.size());
  ble.gap().setAdvertisingInterval(900);
}

void startAnnouncement(BLE &ble, uint8_t magnet_near) {
  Tic_Periodic.attach(periodic_Callback, Periodic_Update_Seconds);

  // read and convert battery voltage
  float bat_reading = read_bat_volt();
#if MyDebugEnb
  device.printf("bat reading: %f \r\n", bat_reading);
#endif

  // disable ADC to save power
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;

  bool isOpened = magnet_near == 1;
  std::vector<uint8_t> btHomePayload =
      buildBtHomePayload(isOpened, bat_reading);

#if MyDebugEnb
  device.printf("BTHome payload:\r\n");
  for (int i = 0; i < btHomePayload.size(); ++i) {
    device.printf("%d: 0x%02x\r\n", i, btHomePayload[i]);
  }
#endif

  setupAnnouncement(ble, btHomePayload);
  ble.gap().startAdvertising();
  Tic_Stop_Adv.attach(stop_adv_Callback, 3);
}

/* ****************************************
 *
 * Main Loop
 *
 *******************************************/
int main(void) {

#if MyDebugEnb
  device.baud(9600);
  device.printf("started sensor node 36 ");
  device.printf("\r\n");
#endif

  BLE &ble = BLE::Instance();
  ble.init();

  /* SpinWait for initialization to complete. This is necessary because the
   * BLE object is used in the main loop below. */
  while (ble.hasInitialized() == false) {
  }

  // every X seconds, sends period update, up to 1800 (30 minutes)
  Tic_Periodic.attach(
      periodic_Callback,
      Periodic_Update_Seconds); // send updated I/O every x seconds

  while (true) {
    uint8_t magnet_near = read_sensor();

    if (NextAction == action_start_announce) {
      startAnnouncement(ble, magnet_near);
      NextAction = action_none;
    }

    if (NextAction == action_stop_announce) {
      ble.gap().stopAdvertising();
      Tic_Stop_Adv.detach();
      NextAction = action_none;
    }

    ble.waitForEvent();
  }
} // end main
