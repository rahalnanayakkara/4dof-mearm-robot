/*
    Used to test the limits of the joint limits of the robot after assembly. 
    Left and Right motors are the ones that are tested. Analog read from the 
    potentiometer is used to determine the duty cycle to be applied to the 
    servo motor. A LPF is used to smoothen the input from the potentiometer.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//I2C pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

//Servo pins
#define LEFT_PIN 8
#define RIGHT_PIN 11

// Servo parameters
#define LEFT_OFFSET 50
#define RIGHT_OFFSET -10
#define SERVO4_MIN_POS 0
#define SERVO4_MAX_POS 180

//PWM Parameters of Servo
#define MIN_PWM 120
#define MAX_PWM 510
#define FREQUENCY 50

//Pin connected to potentiometer
#define ANALOG_PIN_LEFT 35
#define ANALOG_PIN_RIGHT 34

//LPF constant
#define LAMBDA 0.1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct
{
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

RobotServo_t left_servo = {LEFT_PIN, LEFT_OFFSET, SERVO4_MIN_POS, SERVO4_MAX_POS};
RobotServo_t right_servo = {RIGHT_PIN, RIGHT_OFFSET, SERVO4_MIN_POS, SERVO4_MAX_POS};

int left_val = 0;
int right_val = 0;

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
    int new_left_val = analogRead(ANALOG_PIN_LEFT)/20;
    left_val = new_left_val*LAMBDA + left_val*(1-LAMBDA);
    Serial.print(left_val);
    Serial.print("   ");

    int new_right_val = analogRead(ANALOG_PIN_RIGHT)/20;
    right_val = new_right_val*LAMBDA + right_val*(1-LAMBDA);
    Serial.println(right_val);

    writeServo(left_servo, left_val);
    writeServo(right_servo, right_val);
    delay(10);
}

void writeServo(RobotServo_t &servo, int angle)
{
    int pulse_width;
    angle = constrain(angle, servo.min_pos, servo.max_pos);
    pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
    pwm.setPWM(servo.pin, 0, pulse_width);
}
