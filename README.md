# Smart-Glass

This is course project for ELEC 3300 (Introduction to Embedded System).

The project is a prototype of the smart google glass.

## Components
- MCU (STM32F103VETx)
- OLED screen (ssd1306)
- Wifi Module (esp8266)
- Camera (ov7725)
- GPS module (NEO-6M)
- Motion tracking device (MPU6050)
- Temperature & humidity (DHT11)
- SD card
- Ultrasonic sensor (HC-SR04)

## Features
- Real-time information on the display about latitude and longitude, current and future weather, temperature and humidity, acceleration, orientation and sweat concentration on lcd
- Buzzer when the user does not drive safely (high acceleration)
- Capture image and save to the SD card

## Project Video
Click on the following image

[<img src="https://github.com/gwmdunda/Smart-Glass/blob/main/screen_display.jpg" width="50%" height="50%">](https://drive.google.com/file/d/1x_GMsANLkR4j9o9LxzmRGyDjC9QR84xx/view)

## Setup the software
We use HAL library and Keil for IDE

Make sure the following packs are available in case Keil IDE is used
- ARM::CMSIS
- Keil::ARM_Compiler
- Keil::MDK-Middleware
- Keil::STM32F1xx_DFP

Open Keil editor. Project -> Open project and select USART.uvprojx under Project/RVMDK directory.

## Hardware Setup
[<img src="https://github.com/gwmdunda/Smart-Glass/blob/main/overview.jpg" width="50%" height="50%">]
