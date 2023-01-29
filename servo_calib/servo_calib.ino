#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SERVO4_PIN 4
#define SERVO4_OFFSET 0
#define SERVO4_MIN_POS 0
#define SERVO4_MAX_POS 180

#define MIN_PWM 130
#define MAX_PWM 570
#define FREQUENCY 50

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

    pwm.setPWM(servo4.pin, 0, 115);

    // writeServo(servo4, 180); //Sets the position of the servo to 90º
    
    // delay(1000);
    // detachServo(servo4);
}

void loop() 
{}

void writeServo(RobotServo_t &servo, int angle)
{
    int pulse_width;
    angle = constrain(angle, servo.min_pos, servo.max_pos);
    Serial.print("Angle : ");
    Serial.println(angle);

    pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
    Serial.print("Pulse Width : ");
    Serial.println(pulse_width);

    pwm.setPWM(servo.pin, 0, pulse_width);
}

void detachServo(RobotServo_t &servo)
{
    pwm.setPWM(servo.pin, 0, 0);
}
