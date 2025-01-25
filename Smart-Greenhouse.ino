#include <dht.h>
#include <TM1637Display.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Button
#define BUTTON_PIN PD2 //2

//Control
volatile bool debounceLock = false;
uint32_t lastButtonPressTime = 0;

//Motor
#define MOTOR_PIN PD3 //3

//TM1637 - 4digit 7segment display
#define DIO PD4 //4
#define CLK PD5 //5
TM1637Display segm_display(CLK, DIO);

//Pump
#define PUMP_PIN PD6 //6
#define PUMP_DURATION 6000 //6 sec

//DHT11 - temperature & humidity sensor
dht DHT;
#define DHT11_PIN PD7 //7
#define DHT_READ_INTERVAL 1000 //1 sec

//Time control
volatile bool readDHTSensorFlag = false;
volatile bool showHumidityFlag = false;

//Soil moisture sensor
#define SENSOR_PWR_PIN PB0  //8  //For protection
#define SOIL_SENSOR_PIN A0
#define SOIL_THRESHOLD 250
#define SENSOR_CHECK_INTERVAL 30000 //30 sec

//Buzzer
#define BUZZER_PIN PB2 //10

//Motion sensor
#define TRIG_PIN PB4 //12
#define ECHO_PIN PB3 //11
#define DIST_THRESHOLD_LOW 15 //cm
uint8_t state_motion = 1;

//LiquidCrystal_I2C diplay
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Global variables
int8_t temperature = 0;
int8_t humidity = 0;
uint32_t lastCheckTime = 0;
uint32_t pumpStartTime = 0;
uint32_t currentTime = 0;
uint8_t read_dht = 0;


void setup() {
    Serial.begin(9600);
    Serial.println("System Initialized");

    init_pins();
    lcd.init();
    lcd.backlight();

    segm_display.setBrightness(7);

    Timer1_init();
    Timer2_init();

    Init_INT0();
}

void loop() {
    soil_check();
    detect_object();
    currentTime = millis();
    show_temp_hum();
}

ISR(TIMER1_COMPA_vect) {
    static uint32_t elapsedMillis = 0;
    elapsedMillis++;

    if (elapsedMillis >= DHT_READ_INTERVAL) {
        elapsedMillis = 0;
        readDHTSensorFlag = true;
    }
}

ISR(INT0_vect) {
    uint32_t currentMillis = millis();

    if (currentMillis - lastButtonPressTime > 500) {
        lastButtonPressTime = currentMillis;

        showHumidityFlag = !showHumidityFlag;
    }
}

void Timer1_init() {
    TCCR1A = 0;                 // Normal mode
    TCCR1B = (1 << WGM12);      // CTC mode (using OCR1A for compare match)
    OCR1A = 249;                // Timer1 compare match value for 1ms interrupt (16MHz / 64 / 1000 = 249)
    TIMSK1 |= (1 << OCIE1A);    // Enable Timer1 compare match interrupt
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64
}

void Timer2_init() {
    DDRD |= (1 << MOTOR_PIN);  // Set MOTOR_PIN as output (connected to OC2B)

    TCCR2A = (1 << WGM20) | (1 << WGM21);  // Fast PWM mode
    TCCR2A |= (1 << COM2B1);               // Non-inverting mode on OC2B
    TCCR2B = (1 << CS21);                  // prescaler 8 (16MHz / 8 = 2MHz)
    OCR2B = 0;                             // PWM duty cycle to 0 (fan off)
}

void Init_INT0() {
    EICRA |= (1 << ISC01) | (1 << ISC00);  // Trigger on falling edge
    EIMSK |= (1 << INT0);                  // Enable INT0 interrupt
    sei();                                 // Enable global interrupts //SREG bit 7
}

void init_pins() {
    DDRD &= ~(1 << BUTTON_PIN);
    PORTD |= (1 << BUTTON_PIN); //Pull-up

    DDRB |= (1 << SENSOR_PWR_PIN);
    DDRD |= (1 << PUMP_PIN);
    PORTB &= ~(1 << SENSOR_PWR_PIN);
    PORTD &= ~(1 << PUMP_PIN);


    DDRB |= (1 << TRIG_PIN) | (1 << BUZZER_PIN); // Set TRIG_PIN and BUZZER_PIN as output
    DDRB &= ~(1 << ECHO_PIN);                   // Set ECHO_PIN as input

    PORTB &= ~((1 << TRIG_PIN) | (1 << BUZZER_PIN));
}

void show_temp_hum() {
    if (readDHTSensorFlag) {
        readDHTSensorFlag = false;

        read_dht = DHT.read11(DHT11_PIN);

        if (read_dht == DHTLIB_OK) {
            temperature = DHT.temperature;
            displayTemperature();
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.println("°C");

            humidity = DHT.humidity;
            Serial.print("Humidity: ");
            Serial.print(humidity);
            Serial.println("%");

            fan_control();

        }
        else {
            segm_display.showNumberDec(-1);
            Serial.print("Error reading sensor. Code: ");
            Serial.println(read_dht);
        }
    }

    if (showHumidityFlag) {
        showHumidityFlag = false;

        read_dht = DHT.read11(DHT11_PIN);

        if (read_dht == DHTLIB_OK) {
            displayHumidity();
            delay(500);
        }
        else {
            segm_display.showNumberDec(-1);
        }
    }
}
void displayTemperature() {
    uint8_t segments[4];
    segments[0] = segm_display.encodeDigit((temperature / 10) % 10);
    segments[1] = segm_display.encodeDigit(temperature % 10);
    segments[2] = 0b01100011; // Degree symbol
    segments[3] = 0b00111001; // Letter 'C'

    segm_display.setSegments(segments, 4, 0);
}

void displayHumidity() {
    uint8_t segments[4];
    if (humidity < 100) {
        segments[0] = segm_display.encodeDigit(((int)humidity / 10) % 10);
        segments[1] = segm_display.encodeDigit((int)humidity % 10);
        segments[2] = 0;
    }
    else {
        segments[0] = segm_display.encodeDigit(((int)humidity / 100) % 10);
        segments[1] = segm_display.encodeDigit(((int)humidity / 10) % 10);
        segments[2] = segm_display.encodeDigit((int)humidity % 10);
    }
    segments[3] = 0b00111110; //Letter 'U'
    segm_display.setSegments(segments, 4, 0);
}

void fan_control() {
    if (temperature > 26 || humidity > 85) {
        OCR2B = 130;
        Serial.println("Fan is ON at HIGH speed.");
    }
    else if (temperature > 22 || humidity > 65) {
        OCR2B = 100;
        Serial.println("Fan is ON at LOW speed.");
    }
    else {
        OCR2B = 0;
        Serial.println("Fan is OFF.");
    }
}

void soil_check() {
    if (currentTime - lastCheckTime >= SENSOR_CHECK_INTERVAL) {
        lastCheckTime = currentTime;

        Serial.println("Checking soil moisture...");

        PORTB |= (1 << SENSOR_PWR_PIN);
        delay(2000); // Stabilization time
        uint8_t soilMoisture = analogRead(SOIL_SENSOR_PIN); //Registers
        PORTB &= ~(1 << SENSOR_PWR_PIN);

        if (soilMoisture == 0 || soilMoisture > 1023) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Sensor Error!");
            Serial.println("ERROR: Unable to read soil moisture.");
            return;
        }

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Moist: ");
        lcd.print(soilMoisture);
        Serial.print("Soil Moisture: ");
        Serial.println(soilMoisture);

        if (soilMoisture < SOIL_THRESHOLD) {
            lcd.setCursor(0, 1);
            lcd.print("Watering... ");
            Serial.println("Starting watering process...");

            pump_control(true);
            Serial.println("Pump is ON");

            pumpStartTime = currentTime;
            while (currentTime - pumpStartTime < PUMP_DURATION) {
                currentTime = millis();
            }
            pump_control(false);
            Serial.println("Pump stopped. Watering process complete.");
        }
        else {
            lcd.setCursor(0, 1);
            lcd.print("Healthy! ");
            Serial.println("Plant is healthy. No watering needed.");
        }
    }
}

void pump_control(bool state) {
    if (state) {
        PORTD |= (1 << PUMP_PIN);
    }
    else {
        PORTD &= ~(1 << PUMP_PIN);
    }
}

void detect_object(void) {
    PORTB |= (1 << TRIG_PIN);
    //delay(10); 
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    uint32_t count = 0;
    while (!(PINB & (1 << ECHO_PIN))) {  //Protects the code from being stuck forever if no echo is detected.
        if (count++ > 3000) {
            return;
        }
    }

    count = 0;
    while (PINB & (1 << ECHO_PIN)) {
        if (count++ > 3000) {
            return;
        }
    }

    uint32_t distance = (count / 2) * 0.0343;

    if (distance >= DIST_THRESHOLD_LOW ) {
        if (state_motion == 1) {
            PORTB |= (1 << BUZZER_PIN);
            
            //_delay_ms(5000);             
            state_motion = 0;
        }
    }
    else {
        state_motion = 1;
        PORTB &= ~(1 << BUZZER_PIN);
    }
}