# Smart-Greenhouse
Smart Greenhouse using Arduino with Registers, PWM, Timers, and ISRs

This code implements a Smart Greenhouse system that uses various sensors and actuators to monitor and control environmental conditions inside the greenhouse. This utilizes Arduino's hardware features, such as interrupts, timers, and direct register manipulation, to achieve efficient control.

Key Components & Features:

1.DHT11 Temperature and Humidity Sensor:  
-Reads temperature and humidity data.
-Data is displayed on a 4-digit 7-segment display and can be toggled between temperature and humidity using a button.

2.Soil Moisture Sensor:  
-Reads soil moisture levels.
-If the soil moisture is below a threshold, it activates a water pump for a preset duration.

3.Fan Control:  
-A PWM-controlled fan is used to maintain optimal temperature and humidity.
-Fan speed is adjusted based on temperature and humidity readings.

4.Motion Detection:  
-Uses an ultrasonic sensor to detect whether the door is closed or open.
-If an object is detected, a buzzer and a LED are activated as an alert.

5.Button-Activated Display Toggle:  
-A button toggles the display to show either temperature or humidity on the 7-segment display.

6.Interrupts and Timers:  
-Timer1 is used to periodically read the DHT11 sensor data.
-Timer2 is used for controlling the motor (fan) speed via PWM.
-INT0 (external interrupt) handles button presses for toggling the display
