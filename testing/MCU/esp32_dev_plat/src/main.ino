#include "BluetoothSerial.h"

#include "driver/i2c.h"
#include "SPI.h"

// // Check if Bluetooth is available
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

const int detect = 21;
const int button_test = 13;
// #define INPUT(size) 0x81, size

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
} SensorConfiguration;

State current_state;

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
   // function for reading data
   void read_and_send_sensors(int sensor_count, SensorConfiguration *sensors)
   {
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();

      for (int i = 0; i < sensor_count; i++)
      {
         i2c_master_start(cmd);

         // read request
         i2c_master_write_byte(cmd, (sensors[i].address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

         // buffer for sensor data
         uint8_t sensor_data[2];
         i2c_master_read(cmd, sensor_data, sizeof(sensor_data), I2C_MASTER_LAST_NACK);

         i2c_master_stop(cmd);
         esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
         if (ret == ESP_OK)
         {
            // TODO: define this function
            send_data_over_bluetooth(sensor_data, sizeof(sensor_data), sensor_count);
         }
         else
         {
            printf("Failed to read data from sensor %d.\n", i);
         }

         cmd = i2c_cmd_link_create();
      }

      // Clean up
      i2c_cmd_link_delete(cmd);
   }
   // function for sending over bluetoooth?
   void send_data_over_bluetooth(uint8_t *sensor_data, int length, int sensor_count)
   {
      if (SerialBT.available())
      {
         for (int i = 0; i < sensor_count; i++)
         {
            SerialBT.write(sensor_data[i]);
         }
      }
   }

   ~I2CAttachment()
   {
      i2c_driver_delete(I2C_NUM_1);
      // TODO: if I dont use 'new' to create this do I need delete? should I use new?
      // delete config_i;
   }
};

class Vice : public I2CAttachment
{
public:
   Vice() : I2CAttachment()
   {
      // TODO: change addresses when known, this is just an example
      SensorConfiguration vice_sensors[] = {
          {0x40, 1},
          {0x41, 1}};

      int num_of_sensors = 2;
   }
   // function for reading potentiometers to activate specific haptics responses?

   void handle_vice_state()
   {
      // initialize vice class

      // while state is still vice:

      // listen for readings/update values using i2c

      // send over BT

      // call change state function
   }
};

class SPIAttachment
{
private:
   SPIClass *config_s;

public:
   SPIAttachment()
   {
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

   // function for reading data

   // function that sends data over bluetooth

   // function for pulling given cs pins low (activating the device)

   ~SPIAttachment()
   {
      // delete config_s;
   }
};

class Drill : public SPIAttachment
{
public:
   // function for reading pwm signal?

   // function for setting current limit?

   void handle_drill_state()
   {
      // initialize drill class
      // start spi transaction
      // pull cs pin low for things we want to listen for

      // while state is still drill:

      // listen for readings/update values using

      // send over BT

      // call change state function
      // end while loop
      // end transaction
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

// TODO: add function for parsing IMU data

// pass in the current_state variable here by reference
void handle_idle_state(State *state)
{
   while (*state == IDLE)
   {
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

      // next_state(state);
   }
}

void setup()
{
   Serial.begin(115200);
   SerialBT.begin(device_name);

   i2c_config_t config_m;

   // i2c
   config_m.mode = I2C_MODE_MASTER;
   // TODO: these pins are probably different on the schematic
   config_m.sda_io_num = 23;
   config_m.scl_io_num = 22;
   config_m.sda_pullup_en = true;
   config_m.scl_pullup_en = true;
   config_m.master.clk_speed = 100000;

   // i2c_param_config(I2C_NUM_0, &config_m);
   // i2c_driver_install(I2C_NUM_0, config_m.mode, 0, 0, 0);

   // pinMode(button_test, INPUT_PULLUP);

   // state initialization
   current_state = IDLE;

   Serial.print("done");

   delay(20);
}

void loop()
{
   // connect before doing anything
   if (SerialBT.hasClient())
   {
      Serial.println("Bluetooth client connected!");
   }
   else
   {
      Serial.print(".");
      delay(1000);
   }
   // current_state = nextState(current_state);

   // Read the state of the button (active-low, so pressed is LOW)
   int buttonState = digitalRead(button_test);

   if (current_state == IDLE)
   {
      handle_idle_state(&current_state);
   }
   // else if (*current_state == DRILL)
   // {
   //    handle_drill_state();
   // }
   // else if (*current_state == VICE)
   // {
   //    // start i2c transaction
   //    handle_vice_state();
   // }

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