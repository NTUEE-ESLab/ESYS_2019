# 2019-STM Fit

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
