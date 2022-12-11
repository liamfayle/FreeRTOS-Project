# FreeRTOS "Security System" Project

## Components
- [STM32F429I-discovery MCU](https://www.st.com/en/evaluation-tools/32f429idiscovery.html "STM32F429I-discovery MCU")
- [PIR Motion Sensor](https://www.adafruit.com/product/189 "PIR Motion Sensor")
- [Arducam Mini Module Camera Shield 5MP SPI Digital Camera](https://www.uctronics.com/arducam-mini-module-camera-shield-5mp-plus-ov5642-camera-module-for-arduino-uno-mega-2560-board.html "Arducam Mini Module Camera Shield 5MP SPI Digital Camera")

## Interfacing
#### PIR Motion Sensor Pins
- GPIO (PC8)
- 5V Power
- GND

#### Arducam Pins
*SPI*
- SCK  (PE2)
- CS   (PE3)
- MISO (PE5)
- MOSI (PE6)

*I2C*
- SDA (PC9)
- SCL (PA8)

#### PIR sensor & Arducam Interfaced with STM32
![interface (2)](https://user-images.githubusercontent.com/74878922/206932834-b29e62d7-7b57-4e78-b113-75d3cbb3f212.jpg)


## Synchronization
![sync1](https://user-images.githubusercontent.com/74878922/206932840-ffc6c851-b245-416a-a534-fbf7673af3ef.jpg)
![sync2](https://user-images.githubusercontent.com/74878922/206932843-930d4d11-53dc-4ebd-9875-a7acd9091059.png)


## System Performance
#### Idle
![not detected](https://user-images.githubusercontent.com/74878922/206932847-5686aa22-02c5-4376-af40-a1bb005a3570.jpg)

#### Motion Detected
![detected](https://user-images.githubusercontent.com/74878922/206932859-8299e9d4-420c-48ad-88cb-f0a3807fcf52.jpg)
