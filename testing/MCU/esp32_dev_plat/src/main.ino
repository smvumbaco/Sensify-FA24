#include "BluetoothSerial.h"
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

BluetoothSerial SerialBT;

const int detect = 21;

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

// takes in the current state and checks if there is a new state
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
    SerialBT.begin(device_name); // Bluetooth device name
    // SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop()
{

    if (Serial.available())
    {
        SerialBT.write(Serial.read());
    }
    if (SerialBT.available())
    {
        Serial.write(SerialBT.read());
    }
    delay(20);
}