#include "BluetoothSerial.h"

// HID stuff
#include <BLEDevice.h>
// #include <BleKeyboard.h>
#include <BLEHIDDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <HIDKeyboardTypes.h>
//

#include "driver/i2c.h"
#include "SPI.h"

// state machine for collecting the data from certain sensors depending on what is attached
// add a connection test that tests SPI connections on bootup
// code for sensors and BMS

// // Check if Bluetooth is available
// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// // Check Serial Port Profile
// #if !defined(CONFIG_BT_SPP_ENABLED)
// #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
// #endif

// BluetoothSerial SerialBT;

BLEHIDDevice *hid;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

const int detect = 21;
const int button_test = 13;
// #define INPUT(size) 0x81, size

enum State
{
   IDLE,
   DRILL,
   VICE
};

class I2CAttachment
{
private:
   // TODO: do I need to set memory here for initialization of memory block before assigning values?
   i2c_config_t *config_i;

public:
   // could take in speed? depending on attachment we might need higher speeds, maybe implement later
   I2CAttachment()
   {
      // config_i = new i2c_config_t;
      // initialize i2c pins
      config_i->mode = I2C_MODE_MASTER;
      config_i->sda_io_num = 17;
      config_i->scl_io_num = 16;
      config_i->sda_pullup_en = true;
      config_i->scl_pullup_en = true;
      config_i->master.clk_speed = 100000;
      i2c_param_config(I2C_NUM_1, config_i);
      i2c_driver_install(I2C_NUM_1, config_i->mode, 0, 0, 0);
   }

   // function for starting a transaction

   // function for ending a transaction

   // function for reading data and sending over bluetoooth?

   ~I2CAttachment()
   {
      i2c_driver_delete(I2C_NUM_1);
      // TODO: if I dont use 'new' to create this do I need delete? should I use new?
      delete config_i;
   }
};

class SPIAttachment
{
private:
   SPIClass *config_s;

public:
   SPIAttachment()
   {
      // TODO: define spi pins, do I begin spi transaction here or should I make a begin transaction function once the class is initialized

      const int cs = 15;
      const int scl = 16;
      const int miso = 18;
      const int mosi = 17;

      // uses hardware for chip select
      config_s->setHwCs(true);
      // sets MSB first
      config_s->setBitOrder(MSBFIRST);
      // in this mode the clock is low when idel and data is taken on the rising edge of the clock
      config_s->setDataMode(SPI_MODE0);
      // TODO: 1 MHz for now, probably wont need, could have another constructor that takes in clock speed
      config_s->setFrequency(1000000);
   }
   // function for starting a transaction

   // function for ending a transaction

   // function for reading data and sending over bluetoooth?
   // - or should we do a function for reading data, then giving that data to a function that send it over bluetooth?

   // function for pulling certain cs pins low (activating the device)

   ~SPIAttachment()
   {
      // delete config_s;
   }
};

class Vice : public I2CAttachment
{
public:
   // function for reading potentiometers to activate specific haptics responses?

   //
};

class Drill : public SPIAttachment
{
public:
   // function for reading pwm signal?

   // function for setting current limit?
};

// // takes in the current state and checks if there is a new state
// State nextState(State current_state)
// {
//    int current_detect_v = analogRead(detect);

//    // converts to float
//    float voltage = current_detect_v * (3.3 / 4095.0);

//    // TODO: define voltage thresholds for each state

//    // TODO: if detect is an analog pin and I am doing voltage thresholds to determine state, is that really scalable?
//    switch (current_state)
//    {
//    // TODO: implement idle state
//    case IDLE:
//       if (voltage > /*voltage threshold for drill*/ &&voltage < /*voltage threshold for vice*/)
//       {
//          current_state = DRILL;
//       }
//       else if (voltage > /*voltage threshold for vice*/)
//       {
//          current_state = VICE;
//       }
//    // TODO: implement drill state
//    case DRILL:
//       if (voltage < /*voltage threshold for drill*/)
//       {
//          current_state = IDLE;
//       }
//    // TODO: implement vice state
//    case VICE:
//       // this should never happen
//       if (voltage < /*voltage threshold for vice*/)
//       {
//          current_state = IDLE;
//       }
//    }
//    return current_state;
// }

// TODO: add function for parsing IMU data

void setup()
{
   Serial.begin(115200);
   i2c_config_t *config_m;

   BLEDevice::init("ESP32 Keyboard");

   pServer = BLEDevice::createServer();
   pService = pServer->createService("c638083c-b74a-46df-ac04-f113eb352cef");

   pCharacteristic = pService->createCharacteristic(
       "488991d4-302b-4312-bc74-12d53139b35e",
       BLECharacteristic::PROPERTY_NOTIFY);

   pService->start();

   // i2c
   config_m->mode = I2C_MODE_MASTER;
   // TODO: these pins are probably different on the schematic
   config_m->sda_io_num = 25;
   config_m->scl_io_num = 26;
   config_m->sda_pullup_en = true;
   config_m->scl_pullup_en = true;
   config_m->master.clk_speed = 100000;

   i2c_param_config(I2C_NUM_0, config_m);
   i2c_driver_install(I2C_NUM_0, config_m->mode, 0, 0, 0);

   pinMode(button_test, INPUT_PULLUP);

   // state initialization
   State current_state = IDLE;

   hid = new BLEHIDDevice(pServer);
   if (hid == nullptr)
   {
      Serial.println("Failed to create BLEHIDDevice.");
      return;
   }

   // HID service
   hid->manufacturer()->setValue("Example");
   hid->pnp(0x01, 0x02E5, 0xABCD, 0x0110);
   hid->hidInfo(0x00, 0x01);

   // example report map from chat
   const uint8_t reportMap[] = {
       0x05, 0x01, // Usage Page (Generic Desktop Controls)
       0x09, 0x06, // Usage (Keyboard)
       0xA1, 0x01, // Collection (Application)
       0x05, 0x07, // Usage Page (Keyboard/Keypad)
       0x19, 0x00, // Usage Minimum (0)
       0x29, 0x65, // Usage Maximum (101)
       0x15, 0x00, // Logical Minimum (0)
       0x25, 0x01, // Logical Maximum (1)
       0x75, 0x08, // Report Size (8)
       0x95, 0x01, // Report Count (1)
       0x81, 0x02, // Input (Data, Variable, Absolute) — expanded directly
       0xC0        // End Collection
   };
   hid->reportMap((uint8_t *)reportMap, sizeof(reportMap));
   hid->startServices();

   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
   pAdvertising->addServiceUUID(hid->hidService()->getUUID());
   pAdvertising->setScanResponse(true);
   pAdvertising->start();

   Serial.println("BLE HID ready.");

   delay(1000);
}

void loop()
{
   // for HID device
   // - input events
   // hid->inputReport(0)->setValue(...);
   // send over bluetooth

   // if (Serial.available())
   // {
   //     SerialBT.write(Serial.read());
   // }
   // if (SerialBT.available())
   // {
   //     Serial.write(SerialBT.read());
   // }
   // Read the state of the BOOT button (active-low, so pressed is LOW)
   int buttonState = digitalRead(button_test);

   if (buttonState == LOW)
   {
      // Button is pressed, simulate keypress (e.g., 'a' key)
      uint8_t keypress[8] = {0}; // 8-byte report: 1st byte is modifier, 2nd is reserved, 3-8 are keycodes
      keypress[2] = 0x04;        // Keycode for 'a'
      hid->inputReport(0)->setValue(keypress, sizeof(keypress));
      hid->inputReport(0)->notify();

      Serial.println("Key 'a' pressed.");
      delay(200); // Simple debounce
   }
   else
   {
      // Button is not pressed, release the key
      uint8_t keyrelease[8] = {0}; // Empty report means no key is pressed
      hid->inputReport(0)->setValue(keyrelease, sizeof(keyrelease));
      hid->inputReport(0)->notify();

      Serial.println("Key released.");
   }

   delay(100);
}

/*
TODO list:
- look into different sensors to figure out how to start the programming for them
   - macros, functionality
- BMIC drivers
- HID working
- figure out dynamic pin configuration switching
- integrate haptics
   - figure out how different vibrations are actually started

- connect button to a specific pin, simulate a state, read the data and send stuff over bluetooth

*/