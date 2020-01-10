# 2019-STM Fit

## Motivation

## Implementation
![](https://i.imgur.com/86JFX5c.png)
### STM32
Two ADXL345 are connected to STM32L475 through I2C.
#### Walk Detection
We tie one ADXL345 to leg, which can be use to detect the player's walking action. Instead of using raw sensor data, we use a sliding window to calculate the standard value a certain amount of data. After several tests, we decide to monitor the value of the sum of the standard value `(getStd(buffer_high_x) + getStd(buffer_high_y) + getStd(buffer_high_z))`. If the value become higher than the walk threshold, then we would send 1 through BLE, else 0.

#### Attack Detection
We tie one ADXL345 to arm, which can be used to detect the player's waving action. Similar to walk detection, if the the value of the sum of the standard value `(getStd(buffer_low_x) + getStd(buffer_low_y) + getStd(buffer_low_z))` is higher than the attack threshold, then we would send 1 through BLE, else 0.
  
#### Jump Detection
The STM32L475 is tied to player's waist, and we use the internal accelerometer to detect the jump motion. Because jump detection is easy to be detected as walking, we use another formula `buffer_stm[buffer_p] = sqrt(pow((float)pDataXYZ[0],2)+pow((float)pDataXYZ[1],2)+pow((float)pDataXYZ[2],2))` and then use its standard value to judge it.                 
#### Direction
We use the gyroscope in the STM32 to detect the change of angle. The gyroscope in stm32 will return the angular velocity. Since we need the absolute angle, we do integral (summation, actually). First we have to re-calibrate the direction, and set the initial value to 1. If the angle is higher than 1000, then we consider the direction right. Else if the angle is smaller than -1000, we consider it left.

#### Data Transmission
We implement our custom GATT Service and GATT Read Characteristic, so that the sensor data can be transfered through the BLE protocol. In this project, byte array which contain _walk, _attack, _jump, and _direction would be transmitted to PC.
#### Key mapping
We use pynput to control the I/O of keyboard according to the data pc received.

## Setup
Windows 
Mbed Studio Mbed-OS5

### Libraries
* For BSP, should import libraries: `http://os.mbed.com/teams/ST/code/BSP_B-L475E-IOT01/`
* For BLE, should import libraries: `https://github.com/ARMmbed/cordio-ble-x-nucleo-idb0xa1/#811f3fea7aa8083c0bbf378e1b51a8b131d7efcc`
* For BLE, 
### Configuration
* For STM32
    * .mbed
* For BLE 
    * mbed_app.json
    * module.json

## How to run?
1. Open Windows
2. `git clone https://github.com/NTUEE-ESLab/ESYS_2019.git`
3. Build ESYS_2019 mbed program into STM32L475
4. Open Linux 
5. ```
    pip install bluepy 
    pip install pygame
    ```
6. `git clone https://github.com/NTUEE-ESLab/ESYS_2019.git`
7. to connect PC and STM32L475`cd ESYS_2019/pc_code & sudo python3 ble_keymapping.py`
8. `git clone https://github.com/mx0c/super-mario-python.git`
9. `python3 super-mario-python/main.py` to start game

## Screenshots and Demo video
* [demo video](https://drive.google.com/file/d/1jrIsBItb10eqMPvhacGpMeTStFNeKTL9/view?fbclid=IwAR1LEsNIuGHfFwhYCHltQwz8dBGymkLxZEkyJ-KoSGBGhTAmd26P39NTxR4)
* ![](https://i.imgur.com/TENwqAd.png)
* ![](https://i.imgur.com/qrfhor5.jpg)
* ![](https://i.imgur.com/FXODEY9.jpg)


## References
* https://os.mbed.com/teams/mbed-os-examples/code/mbed-os-example-ble-HeartRate/
* https://os.mbed.com/docs/mbed-os/v5.15/apis/i2c.html
