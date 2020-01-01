#include "mbed.h"
#include "ADXL345_I2C.h"
#include "ADXL345.h"
I2C i2c(I2C_SDA , I2C_SCL);
// SPI device(SPI_MOSI, SPI_MISO, SPI_SCK);
// DigitalOut chip_select(SPI_CS);

const int addr7bit = 0x53;      // 7-bit I2C address
const int addr8bit = 0x53 << 1; // 8-bit I2C address, 0x90
ADXL345_I2C accelerometer_high(I2C_SDA, I2C_SCL, 0x1D);
ADXL345_I2C accelerometer_low(I2C_SDA, I2C_SCL, 0x53);

/*
    SPI_MOSI    = D11,
    SPI_MISO    = D12,
    SPI_SCK     = D13,
    SPI_CS      = D10,
*/
// ADXL345 accelerometer(D11, D12, D13, D10);

int readings_high[3] = {0, 0, 0};
int readings_low[3] = {0, 0, 0};

int offsets_high[3] = {0, 0, 0};
int offsets_low[3] = {0, 0, 0};

uint8_t rawReadings[6];

void calibration()
{
    int _sample_num = 0;
    printf("calibrate...\n");

    while (_sample_num < 200) {
        _sample_num++;
        accelerometer_high.getOutput(readings_high);
        accelerometer_low.getOutput(readings_low);

        for (int i = 0; i < 3; ++i) {
            offsets_high[i] += readings_high[i];
            offsets_low[i] += readings_low[i];
        }
        wait(0.0005);
    }

    for (int i = 0; i < 3; ++i) {
        offsets_high[i] /= _sample_num;
        offsets_low[i] /= _sample_num;
    }

    printf("Done calibration\n");
    _sample_num = 0;
}

int main() {
    // SPI device check
    // device.lock();
    // chip_select = 0;
    // int response = device.write(0xFF);
    // printf("Response: %d\n", response);
    // chip_select = 1;
    // device.unlock();


    // I2C device check
    
    printf("start\n");
    int ack;
    for(int i = 0; i < 256 ; i++) {
       ack = i2c.write(i, 0x00, 1);
        if (ack == 0) {
            printf("\tFound at %3d -- %3x\r\n", i, i);
        }
        wait(0.05);
    }
    printf("done\n");

    // char reg = 0x00;
    // char output;
    // ack = i2c.write(0xA6, &reg, 1);
    // printf("ACK = %d\n", ack);
    // ack = i2c.read(0xA7, &output, 1);
    // printf("ACK = %d\n", ack);
    // printf("Device id = %d\n", output);
    
     
    printf("Starting ADXL345 test...\n");
    wait_us(10000);
    printf("Device ID(HIGH) is: 0x%02x\n", accelerometer_high.getDeviceID());
    printf("Device ID(LOW) is: 0x%02x\n", accelerometer_low.getDeviceID());

    // printf("Device ID is: 0x%02x\n", accelerometer.getDevId());
    wait_us(10000);
    
    // These are here to test whether any of the initialization fails. It will print the failure
    accelerometer_high.setPowerControl(0x00);
    accelerometer_low.setPowerControl(0x00);

    //Full resolution, +/-16g, 4mg/LSB.
    wait_us(10000);
     
    accelerometer_high.setDataFormatControl(0x0B);
    accelerometer_low.setDataFormatControl(0x0B);
    // pc.printf("didn't set data format\n");
    wait_us(10000);    
     
    //3.2kHz data rate.
    accelerometer_high.setDataRate(ADXL345_3200HZ);
    accelerometer_low.setDataRate(ADXL345_3200HZ);
    wait_us(10000);
     
    //Measurement mode.
     
    accelerometer_high.setPowerControl(MeasurementMode); 
    accelerometer_low.setPowerControl(MeasurementMode); 
    calibration();   
 
    while (1) {
        // printf("%i, %i, %i\n", (int16_t)(readings[0]-offsets[0]), (int16_t)(readings[1]-offsets[1]), (int16_t)(readings[2]-offsets[2]));
     
        wait_us(100000);
         
        accelerometer_high.getOutput(readings_high);
        accelerometer_low.getOutput(readings_low);
        // accelerometer.getRawOutput(rawReadings);
         
        printf("HIGH %i, %i, %i        LOW %i, %i, %i\n", (int16_t)(readings_high[0]-offsets_high[0]), (int16_t)(readings_high[1]-offsets_high[1]), (int16_t)(readings_high[2]-offsets_high[2]),
        (int16_t)(readings_low[0]-offsets_low[0]), (int16_t)(readings_low[1]-offsets_low[1]), (int16_t)(readings_low[2]-offsets_low[2]));
        // printf("LOW %i, %i, %i\n", (int16_t)(readings_low[0]-offsets_low[0]), (int16_t)(readings_low[1]-offsets_low[1]), (int16_t)(readings_low[2]-offsets_low[2]));
     }


    //initialize
    /*
    char tx[2];
    tx[0] = 0x2C;
    tx[1] = 0x0E;
    int ret = i2c.write(addr8bit, tx, 2);
    printf("ACK = %d\n", ret);

    char cmd[1];
    char output[1];
    while (1) {
        cmd[0] = 0x01;
        // read and write takes the 8-bit version of the address.
        // set up configuration register (at 0x01)
        // i2c.write(addr8bit, cmd, 1);

        wait(0.5);

        // read 
        cmd[0] = 0x00;
        
        i2c.write(addr8bit, cmd, 1);
        ret = i2c.read( addr8bit | 0x1, output, 1);
        printf("ACK: %d\n", ret);
        printf("cmd[0] = %d\n", cmd[0]);
        
    }
    */
}