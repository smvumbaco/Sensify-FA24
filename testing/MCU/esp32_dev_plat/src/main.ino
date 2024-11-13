#include "BluetoothSerial.h"

#include "driver/i2c.h"
#include "SPI.h"
#include <Arduino.h>
#include <BleGamepad.h>

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0

String device_name = "ESP32-BT-Serial";

BluetoothSerial SerialBT;

BleGamepad bleGamepad;

const int detect = 21;
const int button_test = 13;

enum State
{
   IDLE,
   DRILL,
   VICE
};

typedef struct
{
   uint8_t address;
   // length of data in bytes
   uint8_t length_of_data;
} PeripheralConfiguration;

// I eventually want to make this a non-global variable
State current_state;

/*
My idea is that you will never actually create an instance if the I2CAttachment or SPIAttachment class directly.
Instead you will create instances of the child classes (drill, vice), which build off the functions
of the parent class (which gives each child it's specialized functionality). You should then define the number
of sensors/peripheral devices you want and the addresses for those devices
- Note: You can create one function that completely handles the functionality of the state and usses the functions written
  for that attachment (this is optional, but would make it easy to read in the main function)

The reason I have the start transaction and end transaction functions not inside the read_and_send functions is because we do not want to keep
accessing the i2c bus then freeing it every time we read something from it. We want to instead continuously read and send over
bluetooth until the state changes, which will be done in the one function (that handles the state's functionality) mentioned above
*/
class I2CAttachment
{
private:
   // TODO: do I need to set memory here for initialization of memory block before assigning values?
   i2c_config_t config_i;

public:
   I2CAttachment(int sda, int scl, uint32_t clock_speed)
   {
      // config_i = new i2c_config_t;
      // initialize i2c pins
      config_i.mode = I2C_MODE_MASTER;
      config_i.sda_io_num = sda; // 17
      config_i.scl_io_num = scl; // 16
      config_i.sda_pullup_en = true;
      config_i.scl_pullup_en = true;
      config_i.master.clk_speed = clock_speed; // 100000
      i2c_param_config(I2C_NUM_1, &config_i);
      i2c_driver_install(I2C_NUM_1, config_i.mode, 0, 0, 0);
   }

   // function for reading data
   // This should only be called after start()
   void read_and_send_sensors(int sensor_count, PeripheralConfiguration *sensors)
   {
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      for (int i = 0; i < sensor_count; i++)
      {
         i2c_master_start(cmd);
         // read request
         i2c_master_write_byte(cmd, (sensors[i].address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

         // buffer for sensor data
         uint8_t sensor_data[sensors[i].length_of_data];
         i2c_master_read(cmd, sensor_data, sizeof(sensor_data), I2C_MASTER_LAST_NACK);

         i2c_master_stop(cmd);
         esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
         if (ret == ESP_OK)
         {
            // TODO: define this function
            send_data_over_bluetooth(sensor_data, sizeof(sensor_data));
         }
         else
         {
            printf("Failed to read data from sensor %d.\n", i);
         }

         cmd = i2c_cmd_link_create();
      }

      i2c_cmd_link_delete(cmd);
   }
   // TODO: function for sending over bluetoooth - use gamepad library - could add a button and define what buttons mean what in the specific attachments
   void send_data_over_bluetooth(uint8_t *sensor_data, int length)
   {
      if (SerialBT.available())
      {
         for (int i = 0; i < length; i++)
         {
            SerialBT.write(sensor_data[i]);
         }
      }
   }

   // for future attachments, you might need to write over I2C, so a function should be defined for that purpose

   ~I2CAttachment()
   {
      i2c_driver_delete(I2C_NUM_1);
      // TODO: if I dont use 'new' to create this do I need delete? should I use new?
      // delete config_i;
   }
};

class SPIAttachment
{
private:
   SPIClass config_s;

public:
   // 16, 18, 17
   SPIAttachment(int scl, int miso, int mosi)
   {
      config_s.begin(scl, miso, mosi, -1);
      // config_s.setHwCs(true);
   }
   // function for starting a transaction
   void start_transaction(uint32_t freq = 1000000, int8_t bit_o = MSBFIRST, int8_t mode = SPI_MODE0)
   {
      SPISettings settings(freq, bit_o, mode);
      config_s.beginTransaction(settings);
   }

   // function for ending a transaction
   void end_transaction()
   {
      config_s.endTransaction();
   }

   void select(int8_t cs_pin)
   {
      digitalWrite(cs_pin, LOW);
   }

   void deselect(int8_t cs_pin)
   {
      digitalWrite(cs_pin, HIGH);
   }

   // function for reading data
   // TODO: move the starting and stopping of transactions to the handle drill function
   void read_and_send(int8_t cs_pin, int sensor_count, PeripheralConfiguration *sensors)
   {
      // could make an array that stores the length of the data for each sensor
      for (int i = 0; i < sensor_count; i++)
      {
         digitalWrite(cs_pin, LOW);
         start_transaction();
         // TODO: this int8_t most likely needs to be changed, I could make an array here (I NEED LENGTH SOMEHOW)
         int8_t sensor_data = SPI.transfer(sensors[i].address);
         send_data_over_bluetooth(&sensor_data, sensor_count);

         SPI.endTransaction();
         digitalWrite(cs_pin, HIGH);
      }
   }

   // TODO: function that sends data over bluetooth
   // takes in the data pointer and the length of the data to send over bluetooth
   void send_data_over_bluetooth(int8_t *data, int length)
   {
      if (SerialBT.available())
      {
         for (int i = 0; i < length; i++)
         {
            SerialBT.write(data[i]);
         }
      }
   }

   ~SPIAttachment()
   {
      // delete config_s;
   }
};

// // takes in the current state and checks if there is a new state
void next_state(State *current_state)
{
   // TODO: this probably has an adc so I would need to change this
   int current_detect_v = analogRead(detect);

   // converts to float
   float voltage = current_detect_v * (3.3 / 4095.0);

   // voltage for drill is 3 or above, vice is ~1.8

   switch (*current_state)
   {
   case IDLE:
      if (voltage < 2 && voltage > 1.6)
      {
         *current_state = VICE;
      }
      else if (voltage > 2.8)
      {
         *current_state = DRILL;
      }
      break;
   case DRILL:
      if (voltage < 2.8)
      {
         *current_state = IDLE;
      }
      break;
   case VICE:
      if (voltage < 1.6)
      {
         *current_state = IDLE;
      }
      break;
   }
}

class Vice : public I2CAttachment
{
public:
   static const int num_of_sensors = 2;
   PeripheralConfiguration vice_sensors[num_of_sensors];

   Vice() : I2CAttachment(17, 16, 100000)
   {
      // TODO: change addresses when known, this is just an example
      vice_sensors[0] = {0x40, 1}; // address, byte length
      vice_sensors[1] = {0x41, 1};
   }
   // function for reading potentiometers to activate specific haptics responses?

   void handle_vice_state(State *state)
   {

      while (*state == VICE)
      {
         // listen to i2c bus and send over HID input
         read_and_send_sensors(num_of_sensors, vice_sensors);

         // change state function
         next_state(state);
      }
   }
};

class Drill : public SPIAttachment
{
private:
   int8_t cs_pin_drill = 15;
   static const int num_of_sensors = 2;
   PeripheralConfiguration drill_peripherals[num_of_sensors];

public:
   Drill() : SPIAttachment(16, 18, 17)
   {
      // TODO: change addresses when known, this is just an example

      drill_peripherals[0] = {0x40, 1};
      drill_peripherals[1] = {0x41, 1};
      // we only have one cs pin which is why it needs to be set as an ouput pin (we can activate or deactivate the reading from the device on this pin)
      pinMode(cs_pin_drill, OUTPUT);
   }

   // HARI:
   // function for reading pwm signal?

   // function for setting current limit?

   // use above functions in this handle drill state function to make it easy for the main function
   // - the main function will only need to call this function... maybe
   void handle_drill_state()
   {

      // pull cs pin low for things we want to listen for

      // while state is still drill:

      // listen for readings/update values using

      // send over BT

      // call change state function
      // end while loop
      // end transaction
   }
};

// TODO: add function for parsing IMU data

// pass in the current_state variable here by reference
void handle_idle_state(State *state)
{
   while (*state == IDLE)
   {
      if (bleGamepad.isConnected())
      {
         // This code below is for HID gamepad
         Serial.println("Press buttons 1, 32, 64 and 128. Set hat 1 to down right and hat 2 to up left");

         // Press buttons 5, 32, 64 and 128
         bleGamepad.press(BUTTON_5);
         bleGamepad.press(BUTTON_32);
         bleGamepad.press(BUTTON_64);
         bleGamepad.press(BUTTON_128);

         // Move all axes to max.
         bleGamepad.setLeftThumb(32767, 32767);  // or bleGamepad.setX(32767); and bleGamepad.setY(32767);
         bleGamepad.setRightThumb(32767, 32767); // or bleGamepad.setZ(32767); and bleGamepad.setRZ(32767);
         bleGamepad.setLeftTrigger(32767);       // or bleGamepad.setRX(32767);
         bleGamepad.setRightTrigger(32767);      // or bleGamepad.setRY(32767);
         bleGamepad.setSlider1(32767);
         bleGamepad.setSlider2(32767);

         // Set hat 1 to down right and hat 2 to up left (hats are otherwise centered by default)
         bleGamepad.setHat1(DPAD_DOWN_RIGHT); // or bleGamepad.setHat1(HAT_DOWN_RIGHT);
         bleGamepad.setHat2(DPAD_UP_LEFT);    // or bleGamepad.setHat2(HAT_UP_LEFT);
         // Or bleGamepad.setHats(DPAD_DOWN_RIGHT, DPAD_UP_LEFT);

         // Send the gamepad report
         bleGamepad.sendReport();
         delay(500);

         Serial.println("Release button 5 and 64. Move all axes to min. Set hat 1 and 2 to centred.");
         bleGamepad.release(BUTTON_5);
         bleGamepad.release(BUTTON_64);
         bleGamepad.setAxes(0, 0, 0, 0, 0, 0, 0, 0);
         bleGamepad.setHats(DPAD_CENTERED, HAT_CENTERED);
         bleGamepad.sendReport();
         delay(500);
      }

      // for regular bluetooth serial
      if (Serial.available())
      {
         SerialBT.write(Serial.read());
      }
      if (SerialBT.available())
      {
         Serial.write(SerialBT.read());
      }
      printf("In the IDLE state. Waiting for device... \n");

      delay(1000);
   }
}

void setup()
{
   Serial.begin(115200);
   Serial.println("Starting BLE work!");
   BleGamepadConfiguration bleGamepadConfig;
   bleGamepadConfig.setAutoReport(false); // This is true by default
   bleGamepadConfig.setButtonCount(128);
   bleGamepadConfig.setHatSwitchCount(2);
   bleGamepad.begin(&bleGamepadConfig);

   i2c_config_t config_m;

   // i2c
   config_m.mode = I2C_MODE_MASTER;
   // TODO: these pins are probably different
   config_m.sda_io_num = 23;
   config_m.scl_io_num = 22;
   config_m.sda_pullup_en = true;
   config_m.scl_pullup_en = true;
   config_m.master.clk_speed = 100000;

   i2c_param_config(I2C_NUM_0, &config_m);
   i2c_driver_install(I2C_NUM_0, config_m.mode, 0, 0, 0);

   pinMode(button_test, INPUT_PULLUP);

   // state initialization
   current_state = IDLE;

   Serial.print("done");

   delay(20);
}

void loop()
{
   // connect before doing anything

   // current_state = nextState(current_state);

   // Read the state of the button (active-low, so pressed is LOW)
   int buttonState = digitalRead(button_test);

   if (current_state == IDLE)
   {
      handle_idle_state(&current_state);
   }
   else if (current_state == DRILL)
   {
      // initialize drill class
      Drill drill;
      // start spi transaction
      drill.start_transaction();

      drill.handle_drill_state();
   }
   else if (current_state == VICE)
   {
      // initialize vice class
      Vice vice;
      // start i2c transaction
      vice.handle_vice_state(&current_state);
   }

   delay(100);
}

/*
TODO list:
- look into different sensors to figure out how data will be parsed and sent as gamepad inputs
   - IMU on main board
   - vice ADCs and interrupts for each potentiometer
   - drill PWM?
- BMIC configuration, drivers
- HID testing
- figure out dynamic pin configuration switching
- integrate haptics

- connect button to a specific pin, simulate a state, read the data and send stuff over bluetooth

*/