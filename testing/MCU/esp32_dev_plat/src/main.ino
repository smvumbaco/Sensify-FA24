#include "BluetoothSerial.h"
#include "driver/i2c.h"
#include "SPI.h"

// state machine for collecting the data from certain sensors depending on what is attached
// add a connection test that tests SPI connections on bootup
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

// constant pins
const int detect = 21;
const int sda_main = 23;
const int scl_main = 25;

// TODO: finish class definitions, ask: to make this highly customizable what class setup should I have
class Attachment
{
   // constructor
   Attachment()
   {
      // I2C configurations for (a)ttachment
      i2c_config_t config_a;
      config_a.mode = I2C_MODE_MASTER;
      config_a.sda_io_num = 17;
      config_a.scl_io_num = 16;
      // these two use the internal pullup resistors to
      config_a.sda_pullup_en = true;
      config_a.scl_pullup_en = true;
      // 100kHz for now, change later
      config_a.master.clk_speed = 100000;
      // I2C_NUM_1 is one of the two I2C buses that we can use
      i2c_param_config(I2C_NUM_1, &config_a);
      i2c_driver_install(I2C_NUM_1, config_a.mode, 0, 0, 0);
   }
};

BluetoothSerial SerialBT;

void setup()
{
   // We have two I2C busses, one in 12 pin connector and one meant for main board stuff
   // I2C configurations for (m)ain board
   i2c_config_t config_m;
   config_m.mode = I2C_MODE_MASTER;
   config_m.sda_io_num = sda_main;
   config_m.scl_io_num = scl_main;
   config_m.master.clk_speed = 100000;
   i2c_param_config(I2C_NUM_0, &config_m);
   i2c_driver_install(I2C_NUM_0, config_m.mode, 0, 0, 0);

   // need to configure interrupt pin for direction of motor
   // - instead of always tracking the motors state a change in state would trigger the interrupt and then send that through bluetooth
   // configure en pin
   // - when a device is enabled or disabled
   // configure SPI for drill (prob do this in the drill class)

   // configure detect
   pinMode(detect, INPUT);

   Serial.begin(115200);
   SerialBT.begin(device_name); // Bluetooth device name
   // SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
   Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop()
{
   // continue reading the detect pin to determine state (would this be an interrupt as well?)
   int current_detect_state = digitalRead(detect);

   // TODO: switch case or if statements for detect - for scalability purposes, if we are reading voltage levels, is it possible to go up to like 50 possible attachments?

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

/* State machine for reading from specific sensors (which will then mean we only write values that we are currently reading)
States:
 - setup state?
    - once something is attached when it is running, do we need to give it a sec to configure/test connections (could just implement test in drill and vice grips state)
 - drill
    - enter this state when voltage is read for drill state
    - reading from
 - vice grip
    - enter this state when detect pin measures voltage for vice grips state
    - reading from ADCs that the potentiometers are connected to (3 ADCs)
    - set interrupts for ADCs if getting whack readings so we can stop and restart each one mid run
 - detached (idle)
    - enter this state when detect pin reads 0 V
    - nothing attached to the MCU
    - nothing needs to be configured, read from, or sent over bluetooth


implementation
 - switch case for performing specific actions based on the detect pin reading
 - create instance of the specific attachments
 - ...


scalability
 - is switch case scalable/modular?
 - probably could do classes that represent each attachment that has the configuration of pins and new instance is created in a switch statement
 - do a class attachment, that is configured based on the number of pins needed?

 - could create two base attachment classes, then build classes off that for each attachment depending on communication protocol of different sensors
 - should we make a base class for attachments that only use I2C, only use SPI, or use both? This will allow for a high variability of sensors in future mechanism designs
 - can the state machine be for configuring pins? or is it just for reading from specific pins?

 - pins will stay the same, sensors being read from will change, but we will still need to create instances of attachments to know the data being sent?
   - Or can we figure that out through I2C scan?


Options
- attachment base class
- drill and vice class build off the base class - this would work if every attachment we have is required to use I2C,
- and anything additional can be configured in each additional attachment class

- three base classes - I2C attachemnts, SPI attachments, both
*/