# Smart Greenhouse

An embedded Smart Greenhouse system built with Arduino/C++ that automatically monitors and reacts to environmental conditions such as temperature, humidity, soil moisture, and greenhouse access.

What sets this project apart is the use of **low-level AVR programming concepts**, not just standard Arduino functions. The system directly works with **hardware registers, timers, PWM, and interrupts** to control sensors and actuators efficiently.

## Key Features

- **Automatic irrigation** based on soil moisture readings
- **PWM-controlled fan** that adjusts its speed according to temperature and humidity
- **DHT11 monitoring** for temperature and humidity
- **Ultrasonic access detection** with buzzer alert
- **TM1637 display** for temperature and humidity
- **16x2 I2C LCD** for soil moisture and watering status
- Push-button control handled through an **external interrupt**

## Technical Highlights

The project makes use of several embedded systems concepts:

- Direct manipulation of AVR registers such as `DDRx`, `PORTx`, and `PINx`
- **Timer1 interrupts** for periodic sensor readings
- **Timer2 Fast PWM** for fan speed control
- **INT0 external interrupt** for button input
- Interrupt Service Routines (`ISR`)
- I2C communication
- Sensor and actuator integration

The soil moisture sensor is also powered only when a measurement is required, reducing unnecessary continuous operation.

## Hardware

- Arduino
- DHT11 temperature & humidity sensor
- Soil moisture sensor
- Water pump
- DC fan
- Ultrasonic sensor
- Buzzer
- TM1637 4-digit display
- 16x2 I2C LCD

## Project Structure

```text
Smart-Greenhouse/
├── Smart-Greenhouse.ino
├── Photos_Video_Project/
└── README.md
