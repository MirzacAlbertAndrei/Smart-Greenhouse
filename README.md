# Smart Greenhouse

An embedded Smart Greenhouse system built with Arduino/C++ that automatically monitors and controls environmental conditions such as temperature, humidity, soil moisture, and greenhouse access.

The project focuses on low-level embedded programming concepts, including **direct register manipulation, hardware timers, PWM, external interrupts, and sensor/actuator control**.

## Features

### Temperature & Humidity Monitoring
- Reads temperature and humidity using a **DHT11 sensor**
- Displays temperature on a **4-digit TM1637 7-segment display**
- Allows the user to temporarily display humidity using a push button
- Handles sensor read errors and reports system information through Serial

### Automatic Irrigation
- Periodically measures soil moisture
- Powers the soil sensor only while taking measurements
- Automatically activates a water pump when moisture falls below a configured threshold
- Displays soil condition and watering status on a **16x2 I2C LCD**

### Automatic Fan Control
Fan speed is adjusted according to greenhouse temperature and humidity.

The fan supports multiple operating states:

- **Off** under normal conditions
- **Low speed** when temperature or humidity begins to increase
- **Higher speed** when environmental conditions exceed the upper threshold

Fan speed is controlled through hardware **PWM using Timer2**.

### Greenhouse Access Detection
- Uses an ultrasonic sensor to monitor the greenhouse door/opening
- Detects when the measured distance exceeds a configured threshold
- Activates a buzzer as an alert

### Interrupt-Based Display Control
A push button connected to the external interrupt pin allows the displayed environmental value to be changed without continuously polling the button inside the main loop.

---

## Embedded Systems Concepts

A major goal of this project was to implement functionality using the microcontroller's hardware peripherals rather than relying only on high-level Arduino functions.

### Direct Register Manipulation

GPIO configuration and control are performed using AVR registers such as:

```cpp
DDRB
DDRD
PORTB
PORTD
PINB
```

This is used for components including:

- water pump
- soil sensor power
- ultrasonic sensor
- buzzer
- push button
- fan

### Timer1 – Periodic Sensor Reading

**Timer1** operates in CTC mode and generates a periodic interrupt.

```cpp
ISR(TIMER1_COMPA_vect)
```

The interrupt controls when the DHT11 temperature and humidity sensor should be read without placing the timing logic directly inside the main program flow.

### Timer2 – Fan PWM

**Timer2** is configured in Fast PWM mode.

The PWM duty cycle is controlled through:

```cpp
OCR2B
```

This allows the fan speed to change depending on the current environmental conditions.

### External Interrupt INT0

The push button uses the ATmega external interrupt system:

```cpp
ISR(INT0_vect)
```

The interrupt toggles the display between temperature and humidity.

A software debounce interval is also used to prevent multiple detections from a single button press.

---

## System Architecture

```text
                    +----------------+
                    |     DHT11      |
                    | Temp / Humidity|
                    +-------+--------+
                            |
                            v
                     +-------------+
                     |             |
       Button ------>|             |------> TM1637 Display
                     |             |
 Soil Moisture ----->|   Arduino   |------> Water Pump
                     |             |
 Ultrasonic Sensor ->|             |------> Buzzer
                     |             |
                     |             |------> PWM Fan
                     +------+------+
                            |
                            v
                      16x2 I2C LCD
```

---

## Hardware Components

| Component | Purpose |
| --- | --- |
| DHT11 | Temperature and humidity monitoring |
| Soil Moisture Sensor | Measures soil moisture level |
| Water Pump | Automatic irrigation |
| DC Fan | Temperature and humidity regulation |
| Ultrasonic Sensor | Greenhouse opening/door detection |
| Buzzer | Access alert |
| TM1637 4-Digit Display | Temperature and humidity display |
| 16x2 I2C LCD | Soil moisture and irrigation status |
| Push Button | Switch between temperature and humidity display |

---

## Pin Configuration

| Component | Pin |
| --- | --- |
| Push Button | D2 / INT0 |
| Fan | D3 / OC2B |
| TM1637 DIO | D4 |
| TM1637 CLK | D5 |
| Water Pump | D6 |
| DHT11 | D7 |
| Soil Sensor Power | D8 |
| Buzzer | D10 |
| Ultrasonic Echo | D11 |
| Ultrasonic Trigger | D12 |
| Soil Moisture Signal | A0 |

---

## Control Logic

### Fan

The fan operates according to the measured temperature and humidity:

```text
Temperature > 26°C OR Humidity > 85%
                |
                v
          Higher fan speed

Temperature > 22°C OR Humidity > 65%
                |
                v
            Low fan speed

Otherwise
                |
                v
              Fan OFF
```

### Irrigation

Soil moisture is checked every **30 seconds**.

```text
Read Soil Moisture
        |
        v
Moisture < Threshold?
      /     \
    Yes      No
     |        |
     v        v
Start Pump   Healthy
     |
     v
Water for 6 seconds
     |
     v
Stop Pump
```

The soil sensor is powered only during measurements to reduce unnecessary continuous operation.

---

## Software Structure

The main loop remains focused on the greenhouse tasks:

```cpp
void loop() {
    soil_check();
    detect_object();
    currentTime = millis();
    show_temp_hum();
}
```

The application is divided into dedicated functions for:

- pin initialization
- timer configuration
- interrupt configuration
- temperature and humidity display
- fan control
- irrigation control
- soil monitoring
- ultrasonic detection

---

## Libraries

The project uses:

```cpp
#include <dht.h>
#include <TM1637Display.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
```

The AVR headers are used for low-level register and timing functionality, while the remaining libraries interface with the connected sensors and displays.

---

## Running the Project

1. Connect the sensors and actuators according to the pin configuration.
2. Install the required Arduino libraries:
   - DHT
   - TM1637Display
   - LiquidCrystal_I2C
3. Open:

```text
Smart-Greenhouse.ino
```

4. Compile and upload the sketch using the Arduino IDE.
5. Open the Serial Monitor at:

```text
9600 baud
```

The Serial Monitor displays sensor readings, fan state, soil moisture information, and irrigation activity.

---

## Project Structure

```text
Smart-Greenhouse/
├── Smart-Greenhouse.ino
├── Photos_Video_Project/
└── README.md
```

---

## Technical Highlights

This project demonstrates practical experience with:

- Embedded C/C++
- AVR register-level programming
- Hardware timers
- PWM motor control
- Interrupt Service Routines
- External interrupts
- Sensor integration
- Actuator control
- I2C communication
- Real-time environmental monitoring
- Automated control logic
