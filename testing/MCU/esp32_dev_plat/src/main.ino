#include "BluetoothSerial.h"
#include "driver/i2c.h"
#include "SPI.h"

// state machine for collecting the data from certain things at a time
// add a connection test
// code for sensors and BMS

String device_name = "ESP32-yeah-buddy";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

void setup()
{
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