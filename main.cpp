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
extern "C" {
#include "nrf_ecb.h" //required to call the ecb functions for encryption
}

#include "TMP_nrf51/TMP_nrf51.h"
#include "ble/BLE.h"
#include "mbed.h"
#include "toolchain.h"

/*******************************************************************************************
 * START tochange: items that may need customization depending on sensors,
 * hardware, and desired behavior
 *******************************************************************************************/

// enables serial output for debug, consumes ~1mA when idle
#define MyDebugEnb 0

// number of seconds between periodic I/O status re-transmits
const uint16_t Periodic_Update_Seconds = 20;

// local advertised name
const static char DEVICE_NAME[] = "BLE_REED";

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

// flag to indicate event is hardware interrupt
static bool Flag_Update_IO = false;
// flag to indicate event is periodic callback
static bool Flag_Periodic_Call = false;
// flag to stop advertising
static bool Flag_Detach_Adv_Tic = false;

// Advertisement Data
// note:  AdvData[] holds bytes [5] to byte [30] of entire advertising data. The
// user content part after ADV flag and header
static uint8_t
    AdvData[] =
        {
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; // 26 Bytes manufacturer
                                                     // specific data
char buffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};    // hold I/O reading json
char bat_volt_char[6] = {0, 0, 0, 0, 0, 0}; // hold json for battery reading
uint8_t Adv_First_Section[10]; // holds the first several bytes with a pattern
                               // indicating this sensor is "one of ours"
uint8_t mac_reverse[6] = {0x0, 0x0, 0x0,
                          0x0, 0x0, 0x0}; // mac address for this module

static uint8_t key[16] = {0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4,
                          0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4};
// 26 bytes adv data
static uint8_t encrypted[26] = {
    0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x2, 0x3, 0x3, 0x3, 0x4, 0x4,
    0x4, 0x5, 0x5, 0x5, 0x6, 0x6, 0x6, 0x7, 0x7, 0x7, 0x8, 0x8}; /* Example of
                                                                    hex data */
// static uint8_t key_buf[16] =
// {0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4};
static uint8_t key_buf[16] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x1,
                              0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x1, 0x2};
static uint8_t src_buf[16] = {0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4,
                              0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4};
static uint8_t des_buf[16] = {0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4,
                              0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4};

void debounce_Callback(void) {
  Tic_Debounce.detach();
  Flag_Update_IO = true; // start advertising
  /* Note that the buttonPressedCallback() executes in interrupt context, so it
   * is safer to access BLE device API from the main thread. */
}

// ISR for I/O interrupt
void buttonPressedCallback(void) { Tic_Debounce.attach(debounce_Callback, 1); }

// ISR for I/O interrupt
void buttonReleasedCallback(void) { Tic_Debounce.attach(debounce_Callback, 1); }

void stop_adv_Callback(void) {
  // stops advertising after X seconds
  /* Note that the Callback() executes in interrupt context, so it is safer to
   * do heavy-weight sensor polling from the main thread (where we should be
   * able to block safely, if needed). */
  Flag_Detach_Adv_Tic = true;
}

/* ****************************************
 * Decides what actions need to be performed on periodic basis
 *******************************************/
void periodic_Callback(void) {
  Flag_Update_IO = true;
  Flag_Periodic_Call = true;
}

/**
 * This function is called when the ble initialization process has failled
 */
void onBleInitError(BLE &ble, ble_error_t error) {
  /* Initialization error handling should go here */
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params) {
  BLE &ble = params->ble;
  ble_error_t error = params->error;

  if (error != BLE_ERROR_NONE) {
    /* In case of error, forward the error handling to onBleInitError */
    onBleInitError(ble, error);
    return;
  }

  /* Ensure that it is the default instance of BLE */
  if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
    return;
  }

  /* Set device name characteristic data */
  ble.gap().setDeviceName((const uint8_t *)DEVICE_NAME);

  /* Setup advertising payload */
  // set modes "no EDR", "discoverable" for beacon type advertisements
  ble.gap().accumulateAdvertisingPayload(
      GapAdvertisingData::BREDR_NOT_SUPPORTED |
      GapAdvertisingData::LE_GENERAL_DISCOVERABLE);

  // from GAP example
  /* Sacrifice 2B of 31B to AdvType overhead, rest goes to AdvData array you
   * define */
  ble.gap().accumulateAdvertisingPayload(
      GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, AdvData, sizeof(AdvData));

  /* Setup advertising parameters:  not connectable */
  ble.gap().setAdvertisingType(
      GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
  ble.gap().setAdvertisingInterval(900); // one advertisment every 300ms.  Self
                                         // tickers, so you don't have to worry.
}

/* ****************************************
 * Read battery voltage using bandgap reference
 * shunt Vdd to ADC, thanks to Marcelo Salazar's notes here:
 * https://developer.mbed.org/users/MarceloSalazar/notebook/measuring-battery-voltage-with-nordic-nrf51x/
 *******************************************/
uint16_t read_bat_volt(void) {
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

  return (uint16_t)NRF_ADC->RESULT;
}

/* ****************************************
 * Pattern scheme indicating "one of ours"
 * generate first part of ADV data so that observer can recognize it as "one of
 *ours". use specific schema to decide how we're recognizing our sensor ADV
 *******************************************/
void hash_first_section(uint8_t *dest, const uint8_t *mac_addr,
                        const char *bat_volt_str) {
  dest[0] = mac_addr[3];
  dest[1] = mac_addr[2];
  dest[2] = mac_addr[1];
  dest[3] = mac_addr[0];
  dest[4] = bat_volt_str[0];
  dest[5] = bat_volt_str[1];
  dest[6] = bat_volt_str[2];
  dest[7] = bat_volt_str[3];
  dest[8] = 0x10;
  dest[9] = 0x11;
#if MyDebugEnb

  device.printf("hash array: ");
  for (int i = 0; i < 10; i++) {
    device.printf("%x ", dest[i]);
  }
  device.printf("\r\n");
#endif
}

uint8_t read_sensor() 
{
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

      button1.fall(NULL);     // disable interrupt
      button1.rise(NULL);     // disable interrupt
      button1.mode(PullNone); // float pin to save battery

      button2.fall(buttonReleasedCallback); // enable interrupt
      button2.rise(buttonReleasedCallback); // enable interrupt
      button2.mode(PullUp);                 // pull up on pin to get interrupt
#if MyDebugEnb
      device.printf("=== button 1!  %d seconds=== \r\n", seconds_Old);
#endif
    } else if ((button1_state == 1) && (button2_state == 0)) {
      magnet_near = 0;

      button1.fall(buttonReleasedCallback); // enable interrupt
      button1.rise(buttonReleasedCallback); // enable interrupt
      button1.mode(PullUp);                 // pull up on pin to get interrupt

      // button2.disable_irq() //don't know if disables IRQ on port or pin
      button2.fall(NULL);     // disable interrupt
      button2.rise(NULL);     // disable interrupt
      button2.mode(PullNone); // float pin to save battery
#if MyDebugEnb
      device.printf("=== button 2! === %d seconds\r\n", seconds_Old);
#endif
    }    // end if button1
    else // odd state, shouldn't happen, suck battery and pullup both pins
    {
      magnet_near = 2;

      button1.fall(buttonReleasedCallback); // disable interrupt
      button1.rise(buttonReleasedCallback); // disable interrupt
      button1.mode(PullUp);                 // float pin to save battery

      button2.fall(buttonReleasedCallback); // disable interrupt
      button2.rise(buttonReleasedCallback); // disable interrupt
      button2.mode(PullUp);                 // float pin to save battery
#if MyDebugEnb
      device.printf("no buttons!! %d seconds\r\n", seconds_Old);
#endif
    } // end odd state

    return magnet_near;
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
  ble.init(bleInitComplete);

  float bat_reading; // hold battery voltage reading (Vbg/Vcc)

  /* SpinWait for initialization to complete. This is necessary because the
   * BLE object is used in the main loop below. */
  while (ble.hasInitialized() == false) {
  }

  // every X seconds, sends period update, up to 1800 (30 minutes)
  Tic_Periodic.attach(
      periodic_Callback,
      Periodic_Update_Seconds); // send updated I/O every x seconds

  ble.getAddress(0, mac_reverse); // last byte of MAC (as shown on phone app) is
                                  // at mac[0], not mac[6];
#if MyDebugEnb
  device.printf("mac = ");
  for (int i = 0; i < 6; i++) // prints out MAC address in reverse order; opps.
  {
    device.printf("%x:", mac_reverse[i]);
  }
  device.printf("\r\n");
#endif
  while (true) { // Main Loop
    uint8_t magnet_near = read_sensor();

    if (Flag_Update_IO) {
      /* Do blocking calls or whatever hardware-specific action is
       * necessary to poll the sensor. */

      // call attach again on periodic update to reset ticker
      // next periodic updates happens Perioidc_Update_Seconds after I/O events
      Tic_Periodic.attach(periodic_Callback, Periodic_Update_Seconds);

      // read and convert battery voltage
      bat_reading = (float)read_bat_volt();
      bat_reading = (bat_reading * 3.6) / 1024.0;
#if MyDebugEnb
      device.printf("bat reading: %f \r\n", bat_reading);
#endif
      // write battery voltage
      uint8_t total_chars;
      memset(&bat_volt_char[0], 0, sizeof(bat_volt_char)); // clear out buffer
      // convert battery voltage float value to string reprsentation to 2
      // decimal places, and save the size of string.
      total_chars = sprintf(bat_volt_char, "%.2f", bat_reading);

      // disable ADC to save power
      NRF_ADC->TASKS_STOP = 1;
      NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled; // disable to shutdown ADC &
                                                    // lower bat consumption

#if MyDebugEnb
      device.printf("char buff: %c%c%c%c \r\n", bat_volt_char[0],
                    bat_volt_char[1], bat_volt_char[2], bat_volt_char[3]);
      device.printf("num chars: %d \r\n", total_chars);
#endif

      // Generate "First Section" for ADV_Data so gateway will recognize our
      // advertisement pattern
      hash_first_section(Adv_First_Section, mac_reverse, bat_volt_char);

      /* ****************************************
       * start writing out ADVData array
       * todo: this is easy to write but hard to read.  Maybe make it easy to
       *read and hard to write?
       ******************************************/
      memset(&AdvData[0], 0, sizeof(AdvData));
      uint8_t JSON_loc = 0; // AdvData[0]

      AdvData[0] = Adv_First_Section[0]; //"our device" flag, MAC[3]
      JSON_loc++;                        // JSON_loc == 1
      AdvData[1] = Adv_First_Section[1]; //"out device" flag, MAC[2]...
      JSON_loc++;                        // JSON_loc == 2
      AdvData[2] = Adv_First_Section[2];
      JSON_loc++; // JSON_loc == 3
      AdvData[3] = Adv_First_Section[3];
      JSON_loc++; // JSON_loc == 4
      AdvData[4] = Adv_First_Section[4];
      JSON_loc++; // JSON_loc == 5
      AdvData[5] = Adv_First_Section[5];
      JSON_loc++; // JSON_loc == 6
      AdvData[6] = Adv_First_Section[6];
      JSON_loc++;
      AdvData[7] = Adv_First_Section[7];
      JSON_loc++;
      AdvData[8] = Adv_First_Section[8];
      JSON_loc++;
      AdvData[9] = Adv_First_Section[9];
      JSON_loc++;

#if MyDebugEnb
      device.printf("ADV first 10 array: ");
      for (int i = 0; i < 10; i++) {
        device.printf("%x ", AdvData[i]);
      }
      device.printf("\r\n");
#endif

      JSON_loc = 10;
      // Start of encrypted user data

      AdvData[10] = 0x0;
      JSON_loc++;
      AdvData[11] = 0x0;
      JSON_loc++;

      AdvData[12] = 0x0;
      JSON_loc++;

      // start of jason data
      //"mag":
      JSON_loc = 13;
      AdvData[JSON_loc] = 0x22; // ADV_Data[13] = "
      JSON_loc++;               // 14

      AdvData[JSON_loc] = 0x6d; // ADV_Data[14] = m
      JSON_loc++;               // 15

      AdvData[JSON_loc] = 0x61; // ADV_Data[15] = a
      JSON_loc++;               // 16

      AdvData[JSON_loc] = 0x67; // ADV_Data[16] = g
      JSON_loc++;               // 17

      // for periodic calls, we want to add an extra mqtt level "p", using "/p"
      // to delineate between MQTT publishes from real world I/O interrupts vs
      // timer interrupts
      if (Flag_Periodic_Call) {
        AdvData[JSON_loc] = 0x2f; // ADV_Data[17] = /
        JSON_loc++;               // 18
        AdvData[JSON_loc] = 0x70; // ADV_Data[18] =p
        JSON_loc++;               // 19
      }

      AdvData[JSON_loc] = 0x22; // ADV_Data[17 or 19] = "
      JSON_loc++;               // 20

      AdvData[JSON_loc] = 0x3a; // ADV_Data[18 or 20] = :
      JSON_loc++;               // 21

      // convert magnet variable to string, for magnet sensor, this is easy
      // since we only have 1 or 0, but this also works for analog values
      memset(&buffer[0], 0, sizeof(buffer)); // clear out buffer
      total_chars = sprintf(buffer, "%d",
                            magnet_near); // returns total number of characters,
                                          // which is 1 character.
      for (int i = 0; i < total_chars; i++) {
        AdvData[JSON_loc] = buffer[i];
        JSON_loc++; // 23
      }             // JSON_loc left at location of next character

      // AdvData[JSON_loc] = 0x0;    //since AdvData was cleared to start with,
      // we don't need to null term

      /*********************
       * start encrypting last 16 bytes of ADV_Data
       *********************/
      for (int i = 0; i < 16; i++) {
        src_buf[i] =
            AdvData[i + 10]; // start of encrypted section is at AdvData[10]
      }
      nrf_ecb_init();
      nrf_ecb_set_key(key_buf);
      bool successful_ecb = nrf_ecb_crypt(des_buf, src_buf);
#if MyDebugEnb
      device.printf("success ecb = %d \r\n", successful_ecb);
      device.printf(
          "src_buf: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \r\n",
          src_buf[0], src_buf[1], src_buf[2], src_buf[3], src_buf[4],
          src_buf[5], src_buf[6], src_buf[7], src_buf[8], src_buf[9],
          src_buf[10], src_buf[11], src_buf[12], src_buf[13], src_buf[14],
          src_buf[15]);
      device.printf(
          "des_buf: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \r\n",
          des_buf[0], des_buf[1], des_buf[2], des_buf[3], des_buf[4],
          des_buf[5], des_buf[6], des_buf[7], des_buf[8], des_buf[9],
          des_buf[10], des_buf[11], des_buf[12], des_buf[13], des_buf[14],
          des_buf[15]);
#endif
      for (int i = 0; i < 16;
           i++) // replace last 16 bytes with encrypted 16 bytes
      {
        AdvData[i + 10] = des_buf[i];
      }

      // set payload for advertisement to our custom manufactured data.  First 5
      // bytes is BLE standard, last 26 bytes is our array
      ble.gap().accumulateAdvertisingPayload(
          GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, AdvData,
          sizeof(AdvData));

      Flag_Update_IO = false;
      Flag_Periodic_Call = false;

      ble.gap().startAdvertising();
      Tic_Stop_Adv.attach(
          stop_adv_Callback,
          3); /* trigger turn off advertisement after X seconds */

    } // end Flag_Update_IO

    if (Flag_Detach_Adv_Tic == true) // ticker callback flag to stop advertising
    {
      ble.gap()
          .stopAdvertising(); // may be safer to execute BLE operations in main
      Tic_Stop_Adv.detach();
      Flag_Detach_Adv_Tic = false;
    }

    ble.waitForEvent(); // sleeps until interrupt form ticker or I/O
  }                     // end forever while
} // end main
