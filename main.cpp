/* 
Eric Tsai

7/29/2017:  added clock algorithm to corresond with gateway spoof checking
 
 */

//required to call the ecb functions
extern "C"
{
   #include "nrf_ecb.h"
}
 
#include "mbed.h"
#include "toolchain.h"
#include "ble/BLE.h"
#include "TMP_nrf51/TMP_nrf51.h"

//comment out when done with debug uart, else eats batteries
#define MyDebugEnb 0

//Pin "P0.4" on nRF51822 = mbed "p4".
//InterruptIn is pulled-up.  GND the pin to activate.

// waveshare board ******
//InterruptIn button1(p10);
//InterruptIn button2(p11);

// purple board ******
//InterruptIn button1(p23);
//InterruptIn button2(p24);

// universal
InterruptIn button1(p0);
InterruptIn button2(p1);

// Rigado BMD200 ******
//InterruptIn button1(p4);
//InterruptIn button2(p5);

//Serial device(p9, p11);  // tx, rx, purple board and Rigado
#if MyDebugEnb
// if you see ~1mA consumption during sleep, that's because uart is enabled.
Serial device(p9, p11);  //nRF51822 uart :  TX=p9.  RX=p11
#endif


static Timer myTimer;  //timed advertising
static Ticker tic_adv;   //stop adv
static Ticker tic_debounce; //debounce I/O
static Ticker tic_periodic; //regular updates
static Ticker tic_clock_reset; //resets clock
const uint16_t Periodicity = 180;   //clock periodicity used for spoof checking, should be 1800 seconds for 30minutes
//static TMP_nrf51  tempSensor;
static bool flag_update_io = false;
static bool flag_periodic_call = false;
static bool flag_detach_adv_tic = false;
static bool flag_set_debounce_tic = false;  //not used

//static DigitalOut alivenessLED(LED1, 1);
float mySensor = 2.0f;


/* Optional: Device Name, add for human read-ability */
const static char     DEVICE_NAME[] = "CUU";

/* You have up to 26 bytes of advertising data to use. */
/*
Advertisement

DECvolt:3.11111111,mag:1
AdvData[0-2] = Look for DEC on observer
AdvData[3] = beginning of data
*/
//full with nullls
static uint8_t AdvData[] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};   /* Example of hex data */
char buffer[10]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //battery analog reading
char bat_volt_char[6] = {0, 0, 0, 0, 0, 0};
uint8_t Adv_First_Section[10];
uint8_t mac_reverse[6] = {0x0,0x0,0x0,0x0,0x0,0x0};
uint8_t magnet_near=0;



static uint8_t key[16] = {0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4};
//26 bytes adv data
static uint8_t encrypted[26] = {0x0,0x0,0x0,0x1,0x1,0x1,0x2,0x2,0x2,0x3,0x3,0x3,0x4,0x4,0x4,0x5,0x5,0x5,0x6,0x6,0x6,0x7,0x7,0x7,0x8,0x8};   /* Example of hex data */
//static uint8_t key_buf[16] = {0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4};
static uint8_t key_buf[16] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x1, 0x2};
static uint8_t src_buf[16] = {0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4};
static uint8_t des_buf[16] = {0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4,0x1,0x2,0x3,0x4};

uint8_t Xmit_Cnt = 1;
//const static uint8_t AdvData[] = {"ChangeThisData"};         /* Example of character data */



/* **** NOT USED **** */
//16byte UUID loading happens here
//Look at <GapAdvertisingData.h> for rest of definition
struct ApplicationData_t {
    //Byte 0:  AppID High Byte
    //Byte 1:  AppID Low Byte
    //Byte 2:  sensor High Word
    //Byte 3:
    //Byte 4:
    //Byte 5:  sensor Low Byte
    
    
    //app ID is 16 bit, (0xFEFE)
    uint16_t    applicationSpecificId; /* An ID used to identify temperature value in the manufacture specific AD data field */
    
    //float = 32-bit.  
    //tsai: change this to uint32_t!!!
    TMP_nrf51::TempSensorValue_t tmpSensorValue;        /* this is a float (32-bit), user data */
} PACKED;



void debounce_Callback(void)
{
    tic_debounce.detach();
    flag_set_debounce_tic = false;      //not used
    flag_update_io = true;  //start advertising
    /* Note that the buttonPressedCallback() executes in interrupt context, so it is safer to access
     * BLE device API from the main thread. */
    //buttonState = PRESSED;
}

//----- button rising ---
void buttonPressedCallback(void)
{


    //flag_update_io = true;
    
    tic_debounce.attach(debounce_Callback, 1); //ok to attach multiple times, first one wins
    

    //buttonState = PRESSED;
    
    /*
    if (flag_set_debounce_tic == false)
    {
        flag_set_debounce_tic = true;
        
    }
    */
}

//----- button falling ---
void buttonReleasedCallback(void)
{
    

    //flag_update_io = true;
    
    tic_debounce.attach(debounce_Callback, 1);
    
    
  
}


void stop_adv_Callback(void)
{
    //stops advertising after X seconds
    /* Note that the Callback() executes in interrupt context, so it is safer to do
     * heavy-weight sensor polling from the main thread (where we should be able to block safely, if needed). */
    
    //tic_adv.detach();
    
    flag_detach_adv_tic = true;
    //ble.gap().stopAdvertising();
    

}





void periodic_Callback(void)
{
    flag_update_io = true;
    flag_periodic_call = true;
}

void clock_reset_Callback(void)
{
#if MyDebugEnb
    device.printf("===== reset timer =====");
    device.printf("\r\n");
#endif
    myTimer.reset();
};

void setupApplicationData(ApplicationData_t &appData)
{
    // two byte ID:  0xFEFE
    static const uint16_t APP_SPECIFIC_ID_TEST = 0xFEFE;        //2 byte application ID

    appData.applicationSpecificId = APP_SPECIFIC_ID_TEST;
    //appData.tmpSensorValue        = tempSensor.get();
    appData.tmpSensorValue        = mySensor;
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
    
    /* Set device name characteristic data */
    ble.gap().setDeviceName((const uint8_t *) DEVICE_NAME);

    /* Setup advertising payload */
    //set modes "no EDR", "discoverable" for beacon type advertisements
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    
    //set device profile in ADV/GATT
    //ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_THERMOMETER);
    
    //set advertising data (ADV data)
    //ApplicationData_t appData;
    //setupApplicationData(appData);
    //in /BLE_API/ble/GapAdvertisingData.h:  sets payload (uuid)
    //ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *)&appData, sizeof(ApplicationData_t));

    //from GAP example
    /* Sacrifice 2B of 31B to AdvType overhead, rest goes to AdvData array you define */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, AdvData, sizeof(AdvData));

    /* Setup advertising parameters:  not connectable */
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(300);  //one advertisment every 300ms.  Self tickers, so you don't have to worry.



    //don't start advertising on init.  Only advertise on pin interrupt.
    //ble.gap().startAdvertising();
}


//https://developer.mbed.org/users/MarceloSalazar/notebook/measuring-battery-voltage-with-nordic-nrf51x/
void my_analogin_init(void)
{
    
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      //(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      //(ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
}

uint16_t my_analogin_read_u16(void)
{
    //10 bit resolution, route Vdd as analog input, set ADC ref to VBG band gap
    //disable analog pin select "PSEL" because we're using Vdd as analog input
    //no external voltage reference
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      //(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      //(ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

    //NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
    //NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos;
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->TASKS_START = 1;
    
    
    //while loop doesn't actually loop until reading comlete, use a wait.
    while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};
    wait_ms(2);

    //save off RESULT before disabling.
    //uint16_t myresult = (uint16_t)NRF_ADC->RESULT;
    
    //disable ADC to lower bat consumption
    NRF_ADC->TASKS_STOP = 1;
    //NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;    //disable to shutdown ADC & lower bat consumption
    
    return (uint16_t)NRF_ADC->RESULT; // 10 bit
    //return myresult;
}

uint16_t my_analog_pin_read(void)
{

    //10 bit resolution, route PSEL pin as 1/3 input sel,
    //set ADC ref to VBG band gap
    //set AnalogInput4 as input pin (this is P0.03)
    //no external voltage reference
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                      //(ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                       //ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
    //set pin select to AnalogInput4 = pin 7 = p0.03 = AIN4
    //NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
    //NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos;
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->TASKS_START = 1;
    
    
    //while loop doesn't actually loop until reading comlete, use a wait.
    while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};
    wait_ms(2);

    //save off RESULT before disabling.
    //uint16_t myresult = (uint16_t)NRF_ADC->RESULT;
    
    //disable ADC to lower bat consumption
    //NRF_ADC->TASKS_STOP = 1;
    //NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;    //disable to shutdown ADC & lower bat consumption
    
    return (uint16_t)NRF_ADC->RESULT; // 10 bit
    //return myresult;
}


//hash_first_section(Adv_First_Section, mac_reverse, bat_reading);
void hash_first_section(uint8_t * dest, const uint8_t * mac_addr, const char * bat_volt_str)
{
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
    for (int i=0; i<10; i++)
    {
        device.printf("%x ", dest[i]);
    }
    device.printf("\r\n");
#endif
}


uint16_t clock_remainder = 0;

int main(void)
{

#if MyDebugEnb
    device.baud(9600);
    device.printf("started sensor node 36 ");
    device.printf("\r\n");
#endif

    
    
    //Timer myTimer;  //moved to global
    myTimer.start();


    AdvData[0] = 0x44;          //D
    AdvData[1] = 0x45;          //E
    AdvData[2] = 0x43;          //C
    AdvData[3] = 0x22;          //"
    AdvData[4] = 0x76;          //V volt
    AdvData[5] = 0x6f;          //o
    AdvData[6] = 0x22;          //"
    AdvData[7] = 0x3a;          //:
    AdvData[8] = 0x24;          //3     #
    AdvData[9] = 0x24;          //.     #
    AdvData[10] = 0x24;         //1     #
    AdvData[11] = 0x24;         //1     #
    AdvData[12] = 0x2c;         //,     
    AdvData[13] = 0x22;         //" mag
    AdvData[14] = 0x6d;         //a
    AdvData[15] = 0x22;         //"
    AdvData[16] = 0x3a;         //:
    AdvData[17] = 0x24;         //0 or 1, 30 or 31

    button1.fall(buttonPressedCallback);
    button1.rise(buttonReleasedCallback);
    button1.mode(PullNone);
    button1.fall(NULL);
    button1.rise(NULL); 

    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);
    
    //debug uart
    //device.baud(115200);
    float bat_reading;  //hold battery voltage reading (Vbg/Vcc)
    
    my_analogin_init();//routes band-gap to analog input

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized() == false) { /* spin loop */ }
    
    //every X seconds, sends period update, up to 1800 (30 minutes)
    tic_periodic.attach(periodic_Callback, 1600);
    tic_clock_reset.attach(clock_reset_Callback, Periodicity);
    
    //how to generate random number using die tempearature
    uint32_t myTempRand;
    uint32_t * p_temp;
    //get temperature for random number
    //SVCALL(SD_TEMP_GET, myTempRand, sd_temp_get(p_temp));
    //sd_temp_get(& myTempRand);


    ble.getAddress(0,mac_reverse);  //NOTE:  last byte of MAC (as shown on phone app) is at mac[0], not mac[6];
#if MyDebugEnb
    device.printf("mac = ");
    for (int i=0; i<6; i++) //prints out MAC address in reverse order; opps.
    {
        device.printf("%x:", mac_reverse[i]);
    }
    device.printf("\r\n");
#endif
    while (true) {
        //seconds counts to 35 minutes:  mySeconds=2142, then 63392 which should be 2450 but isn't.
        //00001000,01011110   -> 11110111,10100000
        uint16_t mySeconds =(uint16_t)(myTimer.read_ms()/1000); //problem:  mySeconds is only 2 byte
        //xmit_cnt++;
        //if (mySeconds > 1800)
        //{
        //    myTimer.reset();
            //clock_remainder = mySeconds - 1800;
        //}
        //mySeconds = mySeconds + clock_remainder;  l3kjl3
        
        //reading the ADV value, only goes up to 0-255;
        //reading the uart:  current time in seconds: -1782, goes negative.
        //need to be able to count 1800 seconds since that's the length of timer.
        
#if MyDebugEnb
        device.printf("current time in seconds: %d \r\n", mySeconds);
#endif
        //**** set which pin should be interrupt, set pullups ***
        
        //set both pins to pull-up, so they're not floating when we read state
        button1.mode(PullUp);
        button2.mode(PullUp);
        
        //wait_ms(300);   //contact settle
        

        //AdvData[12] is automatically CR?  why?
        
        //0x33 0x2E 0x33 0x32 0x13
        //   3    .    3    2   CR
        
        //expect either button1 or button2 is grounded, b/c using SPDT reed switch
        //the "common" pin on the reed switch should be on GND
        uint8_t button1_state = button1.read();
        uint8_t button2_state = button2.read();
        
        
        //let's just update the pins on every wake.  Insurance against const drain.
        //if state == 0, pin is grounded.  Unset interrupt and float pin
        //set the other input
        if ( (button1_state == 0) && (button2_state == 1) )
        {
            magnet_near = 1;
            //AdvData[4] = 0x11;  //dont' set ADV data directly.  Using json now, need spacing
            //button1.disable_irq() //don't know if disables IRQ on port or pin
            button1.fall(NULL);     //disable interrupt
            button1.rise(NULL);     //disable interrupt
            button1.mode(PullNone); //float pin to save battery
            
            //button2.disable_irq() //don't know if disables IRQ on port or pin
            button2.fall(buttonReleasedCallback);     //enable interrupt
            button2.rise(buttonReleasedCallback);     //enable interrupt
            button2.mode(PullUp); //pull up on pin to get interrupt
#if MyDebugEnb
        device.printf("=== button 1!  %d seconds=== \r\n", mySeconds);
#endif
        }
        else if ( (button1_state == 1) && (button2_state == 0) )       //assume other pin is open circuit
        {
            magnet_near = 0;
            //AdvData[4] = 0x22;    //dont' set ADV data directly.  Using json now, need spacing
            //button1.disable_irq() //don't know if disables IRQ on port or pin
            button1.fall(buttonReleasedCallback);     //enable interrupt
            button1.rise(buttonReleasedCallback);     //enable interrupt
            button1.mode(PullUp); //pull up on pin to get interrupt
            
            //button2.disable_irq() //don't know if disables IRQ on port or pin
            button2.fall(NULL);     //disable interrupt
            button2.rise(NULL);     //disable interrupt
            button2.mode(PullNone); //float pin to save battery
#if MyDebugEnb
        device.printf("=== button 2! === %d seconds\r\n", mySeconds);
#endif
        }    
        else    //odd state, shouldn't happen, suck battery and pullup both pins
        {
            magnet_near = 2;
            //AdvData[4] = 0x33;
            //button1.disable_irq() //don't know if disables IRQ on port or pin
            button1.fall(buttonReleasedCallback);     //disable interrupt
            button1.rise(buttonReleasedCallback);     //disable interrupt
            button1.mode(PullUp); //float pin to save battery
            
            //button2.disable_irq() //don't know if disables IRQ on port or pin
            button2.fall(buttonReleasedCallback);     //disable interrupt
            button2.rise(buttonReleasedCallback);     //disable interrupt
            button2.mode(PullUp); //float pin to save battery
#if MyDebugEnb
        device.printf("no buttons!! %d seconds\r\n", mySeconds);
#endif
        }         
        
        
        if (flag_update_io) {
            /* Do blocking calls or whatever hardware-specific action is
             * necessary to poll the sensor. */

            Xmit_Cnt++; //increment transmit counter when updating I/O


            //read battery voltage
            //analog reading consumes 940uA if not disabled
            
            
            bat_reading = (float)my_analogin_read_u16();    
            
            bat_reading = (bat_reading * 3.6) / 1024.0;
#if MyDebugEnb
            device.printf("bat reading: %f \r\n", bat_reading);
#endif

            //memset(&buffer[0], 0, sizeof(buffer));      //clear out buffer
            //sprintf (buffer, "%f.2", bat_reading);    //don't know what i'm doing
            //sprintf (buffer, "%.2f", bat_reading);
            //AdvData[8] = buffer[0]; //"3"=0x33
            //AdvData[9] = buffer[1]; //"."=0x2E
            //AdvData[10] = buffer[2];//"3"=0x33
            //AdvData[11] = buffer[3];//"2"=0x32
            
            //try this
            //https://developer.mbed.org/users/mbed_official/code/mbed-src/file/cb4253f91ada/targets/hal/TARGET_NORDIC/TARGET_NRF51822/analogin_api.c
            NRF_ADC->TASKS_STOP = 1;
            float analogreading;
            analogreading = (float)my_analog_pin_read();
            analogreading = (analogreading * 3.6) / 1024.0;
            #if MyDebugEnb
            device.printf("separate analog reading: %.02f \r\n", analogreading);
            #endif
            
            //disable ADC
            NRF_ADC->TASKS_STOP = 1;
            NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;    //disable to shutdown ADC & lower bat consumption



            //***********************************
            //form JSON string in ADV_DATA
            //1)  volts starts at AdvData[8]
            


            //write battery voltage
            uint8_t total_chars;
            memset(&bat_volt_char[0], 0, sizeof(bat_volt_char));      //clear out buffer
            total_chars = sprintf (bat_volt_char, "%.2f", bat_reading);    //returns total number of characters

#if MyDebugEnb
            device.printf("char buff: %c%c%c%c \r\n", bat_volt_char[0], bat_volt_char[1], bat_volt_char[2], bat_volt_char[3]);
            device.printf("num chars: %d \r\n", total_chars);
#endif



            //memset(&Adv_First_Section[0], 0, sizeof(Adv_First_Section));  //not needed to reset
            hash_first_section(Adv_First_Section, mac_reverse, bat_volt_char);
            

            
            //------------------------------------------------------------------
            //start writing out ADVData array
            //------------------------------------------------------------------

            memset(&AdvData[0], 0, sizeof(AdvData));
            uint8_t JSON_loc=0; //AdvData[0]

            AdvData[0] = Adv_First_Section[0];          //AdvData[0] = manf ID 01
            JSON_loc++; //1
            AdvData[1] = Adv_First_Section[1];          //AdvData[1] = manf ID 02
            JSON_loc++; //2
            AdvData[2] = Adv_First_Section[2];          //AdvData[2] = reserved
            JSON_loc++; //3...
            AdvData[3] = Adv_First_Section[3];          //AdvData[3] = reserved
            JSON_loc++;
            AdvData[4] = Adv_First_Section[4];          //AdvData[4] = voltage 01
            JSON_loc++;
            AdvData[5] = Adv_First_Section[5];          //AdvData[5] = voltage 02
            JSON_loc++;
            AdvData[6] = Adv_First_Section[6];          //AdvData[6] = voltage 03
            JSON_loc++;
            AdvData[7] = Adv_First_Section[7];          //AdvData[7] = voltage 04
            JSON_loc++;
            AdvData[8] = Adv_First_Section[8];          //AdvData[8] = reserved
            JSON_loc++;
            AdvData[9] = Adv_First_Section[9];          //AdvData[9] = reserved
            JSON_loc++;

#if MyDebugEnb
    
            device.printf("ADV first 10 array: ");
            for (int i=0; i<10; i++)
            {
                device.printf("%x ", AdvData[i]);
            }
            device.printf("\r\n");
#endif


            JSON_loc = 10;
            //start of encrypted data
            //AdvData[10]
            AdvData[10] = mySeconds & 0xFF;           //reserved for timer
            JSON_loc++;
            AdvData[11] = (mySeconds >> 8) & 0xFF;           //reserved for timer
            JSON_loc++;
            AdvData[12] = Xmit_Cnt;
            JSON_loc++;
            //start of jason data
            JSON_loc = 13;
            AdvData[JSON_loc] = 0x22;       //" start mag   pos=13
            JSON_loc++; //14
            
            AdvData[JSON_loc] = 0x6d;       //m,    pos=14
            JSON_loc++; //15
            
            AdvData[JSON_loc] = 0x61;       //a,    pos=15
            JSON_loc++; //16
            
            AdvData[JSON_loc] = 0x67;       //g,    pos=16
            JSON_loc++; //17
            
            if (flag_periodic_call)
            {
                //AdvData[JSON_loc] = 0x2f;       // "/"
                //JSON_loc++;
                AdvData[JSON_loc] = 0x2f;       // "/"  pos=17
                JSON_loc++;//pos=18
                AdvData[JSON_loc] = 0x70;       // "p"  poes=18
                JSON_loc++;//pos=19
            }//end if period call
            
            AdvData[JSON_loc] = 0x22;       //"     pos = 17 or 19
            JSON_loc++; //20

            AdvData[JSON_loc] = 0x3a;       //:     pos = 18 or 20
            JSON_loc++; //22
            
            //prep magnet location (1 or 0) for char[]
            memset(&buffer[0], 0, sizeof(buffer));      //clear out buffer
            //magnet_near is an integer
            total_chars = sprintf (buffer, "%d", magnet_near);    //returns total number of characters, which is 1 character.
            for (int i=0; i < total_chars; i++)
            {
                AdvData[JSON_loc] = buffer[i];
                JSON_loc++; //23
            } //JSON_loc left at location of next character
            
            //MUST null terminate for JSON to read correctly, else get intermittent JSON parse errors at gateway
            //happens when string is shorter than last string, get trash left overs
            //not really needed after clearning AdvData at start.
            
            
            //AdvData[JSON_loc] = 0x0;    //null terminate here

            ApplicationData_t appData;
            setupApplicationData(appData);
            
            
            for (int i=0; i<16; i++)
            {
                src_buf[i] = AdvData[i+10]; //start of encrypted section is at AdvData[10]
            }

            //nrf_ecb_set_key(key_buf);
            nrf_ecb_init();
            nrf_ecb_set_key(key_buf);
            bool successful_ecb = nrf_ecb_crypt(des_buf, src_buf);
#if MyDebugEnb
            device.printf("success ecb = %d \r\n", successful_ecb);
            device.printf("src_buf: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \r\n", src_buf[0], src_buf[1], src_buf[2], src_buf[3], src_buf[4], src_buf[5], src_buf[6], src_buf[7], src_buf[8], src_buf[9], src_buf[10], src_buf[11], src_buf[12], src_buf[13], src_buf[14], src_buf[15]);
            device.printf("des_buf: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \r\n", des_buf[0], des_buf[1], des_buf[2], des_buf[3], des_buf[4], des_buf[5], des_buf[6], des_buf[7], des_buf[8], des_buf[9], des_buf[10], des_buf[11], des_buf[12], des_buf[13], des_buf[14], des_buf[15]);
#endif
            for (int i=0; i<16; i++)
            {
                AdvData[i+10] = des_buf[i];
            }
            
            //ble.gap().updateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (uint8_t *) &appData, sizeof(ApplicationData_t));
            ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, AdvData, sizeof(AdvData));
            
            flag_update_io = false;
            flag_periodic_call = false;
            
            //GAP::AddressType_t *myType;
            //GAP::Address_t myAddress
            //ble_error_t getAddress(Gap::AddressType_t *typeP, Gap::Address_t address)
            //ble.gap().getAddress(myType, myAddress);
            //ble.gap().getAddress(Gap::AddressType_t *typeP, Gap::Address_t address);

            

            ble.gap().startAdvertising();
            tic_adv.attach(stop_adv_Callback, 2); /* trigger turn off advertisement after X seconds */

            /*
            Timer myTimer;  //timed advertising
            myTimer.start();
            uint32_t duration = myTimer.read_ms();
            
            //do this as a ticker instead of keeping processor on
            while (duration < 15000)            //advertise for 1000 ms
            {
                duration = myTimer.read_ms();   //read miliseconds
            }
            myTimer.stop();
            ble.gap().stopAdvertising();
            */
        
        }//end flag_update_io
        
        
        /*
        if (flag_set_debounce_tic == true)
        {
            tic_debounce.attach();
            //flag_set_debounce_tic = false;
            
        }
        */
        
        //if (trigger_Detach_ADV_Tick == false)
        //{  
        //}
        if (flag_detach_adv_tic == true)    //Stop Advertising
        {
            ble.gap().stopAdvertising();    //may be safer to execute BLE operations in main
            tic_adv.detach();
            flag_detach_adv_tic = false;
        }
        //device.printf("Input Voltage: %f\n\r",bat_reading);
        
        ble.waitForEvent(); //sleeps until interrupt
        

    }//end forever while
}//end main
