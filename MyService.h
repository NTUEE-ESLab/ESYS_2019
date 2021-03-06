/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MY_BLE_SERVICE_H__
#define MY_BLE_SERVICE_H__

#include <mbed.h>
#include "ble/BLE.h"
#include "ble/GattCharacteristic.h"
#include "ble/GattService.h"

#if BLE_FEATURE_GATT_SERVER

extern Serial pc;
/**
 * BLE Heart Rate Service.
 *
 * @par purpose
 *
 * Fitness applications use the heart rate service to expose the heart
 * beat per minute measured by a heart rate sensor.
 *
 * Clients can read the intended location of the sensor and the last heart rate
 * value measured. Additionally, clients can subscribe to server initiated
 * updates of the heart rate value measured by the sensor. The service delivers
 * these updates to the subscribed client in a notification packet.
 *
 * The subscription mechanism is useful to save power; it avoids unecessary data
 * traffic between the client and the server, which may be induced by polling the
 * value of the heart rate measurement characteristic.
 *
 * @par usage
 *
 * When this class is instantiated, it adds a heart rate service in the GattServer.
 * The service contains the location of the sensor and the initial value measured
 * by the sensor.
 *
 * Application code can invoke updateHeartRate() when a new heart rate measurement
 * is acquired; this function updates the value of the heart rate measurement
 * characteristic and notifies the new value to subscribed clients.
 *
 * @note You can find specification of the heart rate service here:
 * https://www.bluetooth.com/specifications/gatt
 *
 * @attention The service does not expose information related to the sensor
 * contact, the accumulated energy expanded or the interbeat intervals.
 *
 * @attention The heart rate profile limits the number of instantiations of the
 * heart rate services to one.
 */
class MyService {
public:
    /**
     * Intended location of the heart rate sensor.
     */
    // enum BodySensorLocation {
    //     /**
    //      * Other location.
    //      */
    //     LOCATION_OTHER = 0,

    //     /**
    //      * Chest.
    //      */
    //     LOCATION_CHEST = 1,

    //     /**
    //      * Wrist.
    //      */
    //     LOCATION_WRIST = 2,

    //     /**
    //      * Finger.
    //      */
    //     LOCATION_FINGER,

    //     /**
    //      * Hand.
    //      */
    //     LOCATION_HAND,

    //     /**
    //      * Earlobe.
    //      */
    //     LOCATION_EAR_LOBE,

    //     /**
    //      * Foot.
    //      */
    //     LOCATION_FOOT,
    // };

public:
    /**
     * Construct and initialize a heart rate service.
     *
     * The construction process adds a GATT heart rate service in @p _ble
     * GattServer, sets the value of the heart rate measurement characteristic
     * to @p hrmCounter and the value of the body sensor location characteristic
     * to @p location.
     *
     * @param[in] _ble BLE device that hosts the heart rate service.
     * @param[in] hrmCounter Heart beats per minute measured by the heart rate
     * sensor.
     * @param[in] location Intended location of the heart rate sensor.
     */
    MyService(BLE &_ble, uint8_t player /*, BodySensorLocation location */) :
        ble(_ble),
        valueBytes(player),
        ch(
            // GattCharacteristic::UUID_HEART_RATE_MEASUREMENT_CHAR,
            GattCharacteristic::UUID_MY_SERVICE_CHAR,
            valueBytes.getPointer(),
            valueBytes.getNumValueBytes(),
            MyValueBytes::MAX_VALUE_BYTES,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
        )
        // hrmLocation(
        //     GattCharacteristic::UUID_BODY_SENSOR_LOCATION_CHAR,
        //     reinterpret_cast<uint8_t*>(&location)
        // )
    {
        setupService();
    }

    /**
     * Update the heart rate that the service exposes.
     *
     * The server sends a notification of the new value to clients that have
     * subscribed to updates of the heart rate measurement characteristic; clients
     * reading the heart rate measurement characteristic after the update obtain
     * the updated value.
     *
     * @param[in] hrmCounter Heart rate measured in BPM.
     *
     * @attention This function must be called in the execution context of the
     * BLE stack.
     */
    // void updateHeartRate(uint16_t hrmCounter) {
    //     valueBytes.updateHeartRate(hrmCounter);
    //     ble.gattServer().write(
    //         hrmRate.getValueHandle(),
    //         valueBytes.getPointer(),
    //         valueBytes.getNumValueBytes()
    //     );
    // }

    void updateInfo(uint8_t walk, uint8_t right, uint8_t jump, uint8_t attack) {
         valueBytes.updateInfo( walk, right, jump, attack);
         ble.gattServer().write(
             ch.getValueHandle(),
             valueBytes.getPointer(),
             valueBytes.getNumValueBytes()
         );
         
        //  pc.printf("%d, %d, %d\n", valueBytes.getPointer()[0], valueBytes.getPointer()[1], valueBytes.getPointer()[2]);
    }

protected:
    /**
     * Construct and add to the GattServer the heart rate service.
     */
    void setupService(void) {
        GattCharacteristic *charTable[] = {
            &ch
            // &hrmLocation
        };
        GattService myService(
            // GattService::UUID_HEART_RATE_SERVICE,
            GattService::UUID_MY_SENSOR_SERVICE,
            charTable,
            sizeof(charTable) / sizeof(GattCharacteristic*)
        );

        ble.gattServer().addService(myService);
    }

protected:
    /*
     * Heart rate measurement value.
     */
    struct MyValueBytes {
        /* 1 byte for the Flags, and up to two bytes for heart rate value. */
        static const unsigned MAX_VALUE_BYTES = 6;
        // static const unsigned FLAGS_BYTE_INDEX = 0;

        // static const unsigned VALUE_FORMAT_BITNUM = 0;
        // static const uint8_t  VALUE_FORMAT_FLAG = (1 << VALUE_FORMAT_BITNUM);

        MyValueBytes(uint8_t player) : valueBytes()
        {
            updateInfo(0, 0, 0, 0);
            // valueBytes[4] = player;
        }

        void updateInfo(uint8_t walk, uint8_t right, uint8_t up, uint8_t attack)
        {
            // if (hrmCounter <= 255) {
            //     valueBytes[FLAGS_BYTE_INDEX] &= ~VALUE_FORMAT_FLAG;
            //     valueBytes[FLAGS_BYTE_INDEX + 1] = hrmCounter;
            // } else {
            //     valueBytes[FLAGS_BYTE_INDEX] |= VALUE_FORMAT_FLAG;
            //     valueBytes[FLAGS_BYTE_INDEX + 1] = (uint8_t)(hrmCounter & 0xFF);
            //     valueBytes[FLAGS_BYTE_INDEX + 2] = (uint8_t)(hrmCounter >> 8);
            // }
            valueBytes[0] = walk;
            valueBytes[1] = right;
            valueBytes[2] = up;
            valueBytes[3] = attack;

            // for (int i = 0; i < 6; i++){
            //     valueBytes[i] = readings[i];
            // }
        }

        uint8_t *getPointer(void)
        {
            return valueBytes;
        }

        const uint8_t *getPointer(void) const
        {
            return valueBytes;
        }

        unsigned getNumValueBytes(void) const
        {

            return MAX_VALUE_BYTES;
        }

    private:
        uint8_t valueBytes[MAX_VALUE_BYTES];
    };

protected:
    BLE &ble;
    MyValueBytes valueBytes;
    GattCharacteristic ch;
    // ReadOnlyGattCharacteristic<uint8_t> hrmLocation;
};

#endif // BLE_FEATURE_GATT_SERVER

#endif /* #ifndef MY_BLE_SERVICE_H__ */