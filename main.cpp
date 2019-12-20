#include "mbed.h"
#include "ADXL345_I2C.h"
ADXL345_I2C accelerometer(PB_9, PB_8);
Serial pc(USBTX, USBRX);

// main() runs in its own thread in the OS
int main()
{
    pc.baud(115200);
    int readings[3] = {0, 0, 0};
     
    pc.printf("Starting ADXL345 test...\n");
    wait(.001);
    pc.printf("Device ID is: 0x%02x\n", accelerometer.getDeviceID());
    wait(.001);
    
    // These are here to test whether any of the initialization fails. It will print the failure
    accelerometer.setPowerControl(0x00);
    //Full resolution, +/-16g, 4mg/LSB.
    wait(.001);
     
    accelerometer.setDataFormatControl(0x0B);
    // pc.printf("didn't set data format\n");
    wait(.001);
     
    //3.2kHz data rate.
    accelerometer.setDataRate(ADXL345_3200HZ);
    wait(.001);
     
    //Measurement mode.
     
    accelerometer.setPowerControl(MeasurementMode);    
 
    while (1) {
        pc.printf("%i, %i, %i\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2]);
     
        wait(5);
         
        accelerometer.getOutput(readings);
         
        pc.printf("%i, %i, %i\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2]);
     }
}

