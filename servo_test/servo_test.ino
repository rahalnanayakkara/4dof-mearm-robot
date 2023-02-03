/*
    Used to test the limits of the servo motor. Analog read from the 
    potentiometer is used to determine the duty cycle to be applied 
    to the servo motor. A LPF is used to smoothen the input from the 
    potentiometer.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//I2C pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

//Servo pins
#define SERVO4_PIN 4
#define SERVO4_OFFSET 0
#define SERVO4_MIN_POS 0
#define SERVO4_MAX_POS 180

//PWM Frequency of Servo
#define FREQUENCY 50

//Pin connected to potentiometer
#define ANALOG_PIN 35

//LPF constant
#define LAMBDA 0.2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct
{
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

RobotServo_t servo4 = {SERVO4_PIN, SERVO4_OFFSET, SERVO4_MIN_POS, SERVO4_MAX_POS};

int val = 0;

void setup() 
{
    Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);

    Serial.begin(115200);
    Serial.println();

    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
    yield();
}

void loop() 
{
    int new_val = analogRead(ANALOG_PIN)/6;
    val = LAMBDA*new_val + (1-LAMBDA)*val;
    Serial.println(val);

    pwm.setPWM(servo4.pin, 0, val);
    delay(10);
}
