#include "BluetoothSerial.h"

// HID stuff
#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <HIDKeyboardTypes.h>
//

#include "driver/i2c.h"
#include "SPI.h"

// state machine for collecting the data from certain things at a time
// add a connection test
// code for sensors and BMS

String device_name = "ESP32-way-too-easy";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

// BluetoothSerial SerialBT;

BLEHIDDevice* hid;
BLEServer* pServer;

const int detect = 21;
const int boot = 0;
#define INPUT(size) 0x81, size

enum State {
    IDLE,
    DRILL,
    VICE
};

class I2CAttachment {
    private:
        // TODO: do I need to set memory here for initialization of memory block before assigning values?
        i2c_config_t *config_i;

    public:
    // could take in speed? depending on attachment we might need higher speeds, maybe implement later
        I2CAttachment() {
            // config_i = new i2c_config_t;
            //initialize i2c pins
            config_i->mode = I2C_MODE_MASTER;
            config_i->sda_io_num = 17;
            config_i->scl_io_num = 16;
            config_i->sda_pullup_en = true;
            config_i->scl_pullup_en = true;
            config_i->master.clk_speed = 100000;
            i2c_driver_install(I2C_NUM_1, config_i->mode, 0, 0, 0);
        }

        ~I2CAttachment() {
            i2c_driver_delete(I2C_NUM_1);
            // TODO: if I dont use 'new' to create this do I need delete? should I use new?
            delete config_i;
        }
};

class SPIAttachment {
    private:
        SPIClass *config_s;

    public:
        SPIAttachment() {
            // TODO: define spi pins, do I begin spi transaction here or should I make a begin transaction function once the class is initialized

            const int cs =  15;
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

        ~SPIAttachment() {
            // delete config_s;
        }
};

// // takes in the current state and checks if there is a new state
State nextState(State current_state) {
    int current_detect_v = analogRead(detect);

    // converts to float
    float voltage = current_detect_v * (3.3 / 4095.0);

    // TODO: define voltage thresholds for each state

    // TODO: if detect is an analog pin and I am doing voltage thresholds to determine state, is that really scalable?
    switch(current_state) {
        // TODO: implement idle state
        case IDLE:
            if (voltage > /*voltage threshold for drill*/ && voltage < /*voltage threshold for vice*/) {
                current_state = DRILL;
            }
            else if (voltage > /*voltage threshold for vice*/) {
                current_state = VICE;
            }
        // TODO: implement drill state
        case DRILL:
            if (voltage < /*voltage threshold for drill*/) {
                current_state = IDLE;
            }
        // TODO: implement vice state
        case VICE:
            // this should never happen
            if (voltage < /*voltage threshold for vice*/) {
                current_state = IDLE;
            }
    }
    return current_state;
}

void setup()
{
    i2c_config_t *config_m;
    config_m->mode = I2C_MODE_MASTER;
    // TODO: these pins are probably different on the schematic
    config_m->sda_io_num = 25;
    config_m->scl_io_num = 26;
    config_m->sda_pullup_en = true;
    config_m->scl_pullup_en = true;
    config_m->master.clk_speed = 100000;
    i2c_driver_install(I2C_NUM_1, config_m->mode, 0, 0, 0);

    // when setting up we are in the idle state
    State current_state = IDLE;

    Serial.begin(115200);

    BLEDevice::init("ESP32 Keyboard");
  
    pServer = BLEDevice::createServer();
    hid = new BLEHIDDevice(pServer);

    // HID service
    hid->manufacturer()->setValue("Example");
    hid->pnp(0x01, 0x02E5, 0xABCD, 0x0110);
    hid->hidInfo(0x00, 0x01);

    // example report map from chat
    const uint8_t reportMap[] = {
        USAGE_PAGE(1), 0x01,    // Generic Desktop Controls
        USAGE(1), 0x06,         // Keyboard
        COLLECTION(1), 0x01,    // Application
        USAGE_PAGE(1), 0x07,    // Keyboard/Keypad
        USAGE_MINIMUM(1), 0x00, // No modifiers
        USAGE_MAXIMUM(1), 0x65, // Up to 101 keys
        LOGICAL_MINIMUM(1), 0x00,
        LOGICAL_MAXIMUM(1), 0x01,
        REPORT_SIZE(1), 0x08,   // 1 byte for key press
        REPORT_COUNT(1), 0x01,  // 1 key at a time
        INPUT(1), 0x02,         // Data, Variable, Absolute
        END_COLLECTION(0)       // End collection
    };
    hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));
    hid->startServices();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    Serial.println("BLE HID ready.");



    // SerialBT.begin(device_name); // Bluetooth device name
    // SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
    // Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
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
    int buttonState = digitalRead(boot);

    if (buttonState == LOW) {
        // Button is pressed, simulate keypress (e.g., 'a' key)
        uint8_t keypress[8] = {0}; // 8-byte report: 1st byte is modifier, 2nd is reserved, 3-8 are keycodes
        keypress[2] = 0x04;        // Keycode for 'a'
        hid->inputReport(0)->setValue(keypress, sizeof(keypress));
        hid->inputReport(0)->notify();

        Serial.println("Key 'a' pressed.");
        delay(200); // Simple debounce
    } else {
        // Button is not pressed, release the key
        uint8_t keyrelease[8] = {0}; // Empty report means no key is pressed
        hid->inputReport(0)->setValue(keyrelease, sizeof(keyrelease));
        hid->inputReport(0)->notify();

        Serial.println("Key released.");
    }

    delay(100);
}