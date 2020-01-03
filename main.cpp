// #include "mbed.h"
#include "ADXL345_I2C.h"
#include "BSP.h"
// BLE
#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "gap/Gap.h"
#include "gap/AdvertisingDataParser.h"
#include "ble/services/HeartRateService.h"
#include "MyService.h"
#include "pretty_printer.h"

const static char DEVICE_NAME[] = "MySensor";
// I2C
I2C i2c(I2C_SDA , I2C_SCL);
#define TIMESTEP            0.05
#define SCALE_MULTIPLIER    0.045
ADXL345_I2C accelerometer_high(I2C_SDA, I2C_SCL, 0x1D);
ADXL345_I2C accelerometer_low(I2C_SDA, I2C_SCL, 0x53);


// int readings_high[3] = {0, 0, 0};
// int readings_low[3] = {0, 0, 0};

// int offsets_high[3] = {0, 0, 0};
// int offsets_low[3] = {0, 0, 0};

// uint8_t rawReadings[6];

// int16_t pDataXYZ[3] = {0};
// float pGyroDataXYZ[3] = {0};
// int   AccOffset[3] = {};
// float GyroOffset[3] = {};

// void calibration()
// {
//     int _sample_num = 0;
//     printf("calibrate...\n");

//     while (_sample_num < 500) {
//         _sample_num++;
//         accelerometer_high.getOutput(readings_high);
//         accelerometer_low.getOutput(readings_low);
//         BSP_GYRO_GetXYZ(pGyroDataXYZ);
//         BSP_ACCELERO_AccGetXYZ(pDataXYZ);

//         for (int i = 0; i < 3; ++i) {
//             offsets_high[i] += readings_high[i];
//             offsets_low[i] += readings_low[i];
//             GyroOffset[i] += pGyroDataXYZ[i];
//             AccOffset[i] += pDataXYZ[i];
//         }
//         wait(0.0005);
//     }

//     for (int i = 0; i < 3; ++i) {
//         offsets_high[i] /= _sample_num;
//         offsets_low[i] /= _sample_num;
//         GyroOffset[i] /= _sample_num;
//         AccOffset[i] /= _sample_num;
//     }

//     printf("Done calibration\n");
//     _sample_num = 0;
// }

// BLE
static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class Sensors {
#define SCALE_MULTIPLIER    0.045
#define TIMESTEP            0.0005
public:
    Sensors(events::EventQueue &event_queue) : 
    _event_queue(event_queue),
    accelerometer_high(I2C_SDA, I2C_SCL, 0x1D),
    accelerometer_low(I2C_SDA, I2C_SCL, 0x53) {
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
        BSP_ACCELERO_Init();    
        BSP_GYRO_Init();
        calibration();
        _event_queue.call_every(1, this, &Sensors::update);
    }
private: 
    // void getSensorData(int* readings_high, int* readings_low, int16_t* pDataXYZ, float* pGyroDataXYZ) {
    void getSensorData( uint8_t& _right, uint8_t& _jump, uint8_t& _attack) {
        accelerometer_high.getOutput(readings_high);
        accelerometer_low.getOutput(readings_low);
        BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        BSP_GYRO_GetXYZ(pGyroDataXYZ);
        for (int i = 0; i < 3; i++) {
            readings_high[i] = readings_high[i] - offsets_high[i];
            readings_low[i] = readings_low[i] - offsets_low[i];
            pDataXYZ[i] = pDataXYZ[i] - AccOffset[i];
            pGyroDataXYZ[i] = pGyroDataXYZ[i] - GyroOffset[i];
        } 
        // TODO transfer to right jump and attack here
        _right += 1;
        _jump += 1;
        _attack += 1;
    }
    void calibration()
    {
        int readings_high[3] = {};
        int readings_low[3] = {};
        int16_t pDataXYZ[3] = {};
        float pGyroDataXYZ[3] = {};

        int _sample_num = 0;
        printf("calibrate...\n");

        while (_sample_num < 500) {
            _sample_num++;
            accelerometer_high.getOutput(readings_high);
            accelerometer_low.getOutput(readings_low);
            BSP_GYRO_GetXYZ(pGyroDataXYZ);
            BSP_ACCELERO_AccGetXYZ(pDataXYZ);

            for (int i = 0; i < 3; ++i) {
                offsets_high[i] += readings_high[i];
                offsets_low[i] += readings_low[i];
                GyroOffset[i] += pGyroDataXYZ[i];
                AccOffset[i] += pDataXYZ[i];
            }
            wait(0.0005);
        }

        for (int i = 0; i < 3; ++i) {
            offsets_high[i] /= _sample_num;
            offsets_low[i] /= _sample_num;
            GyroOffset[i] /= _sample_num;
            AccOffset[i] /= _sample_num;
        }

        printf("Done calibration\n");
        _sample_num = 0;
    }
private:
    events::EventQueue &_event_queue;
    ADXL345_I2C accelerometer_high;
    ADXL345_I2C accelerometer_low;

    uint8_t rawReadings[6];

    int   AccOffset[3] = {};
    float GyroOffset[3] = {};
    int offsets_high[3] = {};
    int offsets_low[3] = {};
}

class MySensorDemo : ble::Gap::EventHandler {
public:
    MySensorDemo(BLE &ble, events::EventQueue &event_queue, uint8_t player, Sensors* mysensor) :
        _ble(ble),
        _event_queue(event_queue),
        _led1(LED1, 1),
        _connected(false),
        _uuid(GattService::UUID_MY_SENSOR_SERVICE),
        _service(ble, player),
        _adv_data_builder(_adv_buffer),
        _sensor(mysensor),
        _right(0),_angle(0),_send_count(0), _jump(0) { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &MySensorDemo::on_init_complete);

        _event_queue.call_every(500, this, &MySensorDemo::blink);
        _event_queue.call_every(1000, this, &MySensorDemo::update_sensor_value);

        // _event_queue.dispatch_forever();
    }
private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _adv_data_builder.setFlags();
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::GENERIC_HEART_RATE_SENSOR);
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&_uuid, 1));
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            printf("_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            printf("_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            printf("_ble.gap().startAdvertising() failed\r\n");
            return;
        }
    }

    void update_sensor_value() {
        // int readings_high[3] = {};
        // int readings_low[3] = {};
        // int16_t pDataXYZ[3] = {};
        // float pGyroDataXYZ[3] = {};
        uint8_t _right;
        uint8_t _jump;
        uint8_t _attack;
        if (_connected) {
            // Do blocking calls or whatever is necessary for sensor polling.
            // In our case, we simply update the HRM measgetSensorData
            // sensor get left&right, hit, jump
            // _sensor -> getSensorData(&_right, &_jump, &_attack);
            _right+=1;
            _jump+=1;
            _attack+=1;

            _service.updateInfo(_right, _jump, _attack);
            printf("%lld\n", ++_send_count);
        } 
    }

    void blink(void) {
        _led1 = !_led1;
    }


private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _connected = false;
    }

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }
    

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _led1;
    Sensors* _sensor;

    bool _connected;

    UUID _uuid;

    MyService _service;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;

    uint8_t _send_count;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

Sensors mysensor(event_queue);
void calibration() {
    event_queue.call(callback(&mysensor, &Sensors::calibration);
}

int main() {

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
    
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    MySensorDemo demo(ble, event_queue, 1, &mysensor);
    demo.start();

    event_queue.dispatch_forever();


    // BSP_GYRO_Init();
    // BSP_ACCELERO_Init();
     
    // printf("Starting ADXL345 test...\n");
    // wait_us(10000);
    // printf("Device ID(HIGH) is: 0x%02x\n", accelerometer_high.getDeviceID());
    // printf("Device ID(LOW) is: 0x%02x\n", accelerometer_low.getDeviceID());

    // // printf("Device ID is: 0x%02x\n", accelerometer.getDevId());
    // wait_us(10000);
    
    // // These are here to test whether any of the initialization fails. It will print the failure
    // accelerometer_high.setPowerControl(0x00);
    // accelerometer_low.setPowerControl(0x00);

    // //Full resolution, +/-16g, 4mg/LSB.
    // wait_us(10000);
     
    // accelerometer_high.setDataFormatControl(0x0B);
    // accelerometer_low.setDataFormatControl(0x0B);
    // // pc.printf("didn't set data format\n");
    // wait_us(10000);    
     
    // //3.2kHz data rate.
    // accelerometer_high.setDataRate(ADXL345_3200HZ);
    // accelerometer_low.setDataRate(ADXL345_3200HZ);
    // wait_us(10000);
     
    // //Measurement mode.
     
    // accelerometer_high.setPowerControl(MeasurementMode); 
    // accelerometer_low.setPowerControl(MeasurementMode); 
    // calibration();   
 
    // while (1) {     
    //     wait_us(10000);
         
    //     accelerometer_high.getOutput(readings_high);
    //     accelerometer_low.getOutput(readings_low);
    //     BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    //     BSP_GYRO_GetXYZ(pGyroDataXYZ);
         
    //     printf("HIGH %i, %i, %i   LOW %i, %i, %i   ACC %d, %d, %d  Gyro %.2f, %.2f, %.2f \n", (int16_t)(readings_high[0]-offsets_high[0]), (int16_t)(readings_high[1]-offsets_high[1]), (int16_t)(readings_high[2]-offsets_high[2]),
    //     (int16_t)(readings_low[0]-offsets_low[0]), (int16_t)(readings_low[1]-offsets_low[1]), (int16_t)(readings_low[2]-offsets_low[2]), 
    //     pDataXYZ[0]-AccOffset[0], pDataXYZ[1]-AccOffset[1], pDataXYZ[2]-AccOffset[2], 
    //     (pGyroDataXYZ[0] - GyroOffset[0]) * SCALE_MULTIPLIER, (pGyroDataXYZ[1] - GyroOffset[1]) * SCALE_MULTIPLIER, (pGyroDataXYZ[2] - GyroOffset[2]) * SCALE_MULTIPLIER);
    //  }
}