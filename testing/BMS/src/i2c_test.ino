#include <driver/i2c.h>
#include <Arduino.h>

// TODO:
// - test in hardware
// - set up configuration registers
//    - nonvolatile block programming
//    - disable write protection
//    - nonvoltatile backup
//    - charging current
// - integrate with main.ino (threads?)
// - convert register readings into meaningful values (see Table 15 of MAX17320 datasheet)

#define SDA_PIN_TEST 21     // SDA pin for breadboard testing with dev kit
#define SCL_PIN_TEST 22     // SCL pin for breadboard testing with dev kit

#define SDA_PIN 23          // SDA pin
#define SCL_PIN 22          // SCL pin

#define I2C_PORT I2C_NUM_0  // i2c port 0

#define MAX17320_ADDR_LOWER_REGS 0x6C    // MAX17320 slave address for registers in range 0x000-0x0FF (first page)
#define MAX17320_ADDR_HIGHER_REGS 0x16    // MAX17320 slave address for registers in range 0x100-0x1FF (second page)
#define PAGE_ADDR_MASK (1 << 8)         // bitmask to determine if register is on first page or second page,
                                        // i.e. whether to use slave address 0x6C or 0x16

#define REG_MASK 0xFF   // bitmask to get actual memory address for a register

// Measurement registers
// #define REG_VCELL 0x01A     // lowest cell voltage
#define REG_CELL1 0x0D8     // CELL1 voltage
#define REG_CELL2 0x0D7     // CELL2 voltage
#define REG_BATT 0x0DA     // total battery pack voltage
#define REG_PACKP 0x0DB     // PACK+ voltage
// #define REG_AVGVCELL 0x019     // average cell voltage
// #define REG_AVGVCELL1 0x0D4     // average CELL1 voltage
// #define REG_AVGVCELL2 0x0D3     // average CELL2 voltage

// #define REG_TEMP 0x01B     // max of thermistors and die temp
#define REG_TEMP1 0x13A     // thermistor 1
// #define REG_AVGTEMP1 0x136     // avg temp of thermistor 1

#define REG_CURRENT 0x01C   // battery current
// #define REG_AVGCURRENT 0x01D   // average current


void i2c_scan() {
    Serial.println("Scanning I2C bus...");
    uint8_t address = MAX17320_ADDR_LOWER_REGS;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        Serial.printf("Device found at address: 0x%02X\n", address);
    } else if (ret == ESP_ERR_TIMEOUT) {
        Serial.printf("Timeout at address: 0x%02X\n", address);
    }
}

/**
 * Get the MAX17320 slave address based on the given register address.
 * If in the range 0x000-0x0FF, use address 0x6C.
 * Else, in the range 0x100-0x1FF, use address 0x16.
 * (see Tables 89 and 115 in MAX17320 datasheet for register map/slave addresses)
 * @param reg register address
 */
uint8_t max17320_get_slave_addr_from_reg_addr(uint16_t reg) {
    // If the register is on the second page...
    if (reg & PAGE_ADDR_MASK) {
        return MAX17320_ADDR_HIGHER_REGS;  // use slave address for second page
    }
    else {
        return MAX17320_ADDR_LOWER_REGS;    // otherwise use slave address for first page
    }
}

/**
 * Write to a register on the MAX17320 BMS IC
 * (see Figure 39 in MAX17320 datasheet for I2C write sequence protocol)
 * @param reg register to write to
 * @param value value to write
 */
void max17320_write_register(uint16_t reg, uint16_t value) {
    // determine the slave address
    uint8_t slave_addr = max17320_get_slave_addr_from_reg_addr(reg);

    // get the address of actual register in MAX17320 memory to write to (removes page bit)
    uint8_t reg_addr = reg && REG_MASK;

    // Data to write
    uint8_t data[3];
    data[0] = reg_addr;             // 1st byte = memory address of register
    data[1] = value & 0xFF;         // 2nd byte = 1st byte of value
    data[2] = (value >> 8) & 0xFF;  // 3rd byte = 2nd byte of value

    // Queue the commands
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();   // create command list
    i2c_master_start(cmd);                                                      // START
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);     // "write byte"
    i2c_master_write(cmd, data, 3, true);                                       // 3 data bytes
    i2c_master_stop(cmd);                                                       // STOP
    // Send out the queued commands
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);   // free command list

    if (ret == ESP_OK) {
        printf("Successfully wrote data to MAX17320 register %d.\n", reg);
    }
    else {
        printf("Failed to write data to MAX17320 register %d.\n", reg);
    }
}

/**
 * Read from a register on the MAX17320 BMS IC and send the data to the ESP32 over Bluetooth
 * (see Figure 40 in MAX17320 datasheet for I2C read sequence protocol)
 * @param reg register to read from
 */
void max17320_read_register(uint16_t reg) {
    // determine the slave address
    uint8_t slave_addr = max17320_get_slave_addr_from_reg_addr(reg);

    // get the address of actual register in MAX17320 memory to write to (removes page bit)
    uint8_t reg_addr = reg && REG_MASK;

    // Buffer to store data read
    uint8_t data[2] = {0};

    // Queue the commands
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();   // create command list
    i2c_master_start(cmd);                                                      // START
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);     // "write byte"
    i2c_master_write_byte(cmd, reg_addr, true);                                 // memory address of register
    i2c_master_start(cmd);                                                      // START (repeated)
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true);      // "read byte"
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);                        // Read 2 bytes
    i2c_master_stop(cmd);                                                       // STOP

    // Send out the queued commands
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);   // free command list

    if (ret == ESP_OK) {
        printf("Read %d from MAX17320 register %d.\n", (data[1] << 8 | data[0]), reg);
        // TODO: send data over bluetooth
        // send_data_over_bluetooth(slave_addr, data, sizeof(data));
    }
    else {
        printf("Failed to read data from MAX17320 register %d.\n", reg);
    }

}

void setup() {
    Serial.begin(115200);

    // Configure I2C master
    i2c_config_t config_m;
    config_m.mode = I2C_MODE_MASTER;
    config_m.sda_io_num = SDA_PIN_TEST;
    config_m.scl_io_num = SCL_PIN_TEST;
    config_m.sda_pullup_en = true;
    config_m.scl_pullup_en = true;
    config_m.master.clk_speed = 100000;     // Speed = 100kHz

    i2c_param_config(I2C_PORT, &config_m);
    i2c_driver_install(I2C_PORT, config_m.mode, 0, 0, 0);

}

void loop() {
  delay(100);
  i2c_scan();

//   examples
//   while(true) {
//     max17320_read_register(REG_CELL1);             // read voltage of CELL1
//     max17320_read_register(REG_CELL2);             // read voltage of CELL1
//     max17320_read_register(REG_BATT);             // read voltage of total pack
//     max17320_read_register(REG_CURRENT);            // read current
//     max17320_read_register(REG_TEMP1);             // read temperature
//     delay(1000);
//   }
}