/*
Sets all servos to specified angles
*/

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SERVO_OFFSET 0
#define SERVO_MIN_POS 0
#define SERVO_MAX_POS 180

#define MIN_PWM 120
#define MAX_PWM 510
#define FREQUENCY 50

#define MIDDLE_PIN 4
#define LEFT_PIN 8
#define RIGHT_PIN 11
#define CLAW_PIN 9

// Angles
#define MIDDLE_ANGLE 90
#define LEFT_ANGLE 90
#define RIGHT_ANGLE 90
#define CLAW_ANGLE 90

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct
{
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

RobotServo_t middle_servo = {MIDDLE_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};
RobotServo_t left_servo = {LEFT_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};
RobotServo_t right_servo = {RIGHT_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};
RobotServo_t claw_servo = {CLAW_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};

void setup() 
{
    Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
    
    Serial.begin(115200);
    Serial.println();

    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
    yield();
    delay(2000);

    // Moves all servos to final position
    writeServo(middle_servo, MIDDLE_ANGLE);
    writeServo(left_servo, LEFT_ANGLE);
    writeServo(right_servo, RIGHT_ANGLE);
    writeServo(claw_servo, CLAW_ANGLE);
    Serial.println("Moving to position");
    delay(2000);

    // Detaches all servos
    detachServo(middle_servo);
    detachServo(left_servo);
    detachServo(right_servo);
    detachServo(claw_servo);
    Serial.println("Detached Servos");
}

void loop() {}

// Converts angle to pulse width
void writeServo(RobotServo_t &servo, int angle)
{
    int pulse_width;
    angle = constrain(angle, servo.min_pos, servo.max_pos);
    // Serial.print("Angle : ");
    // Serial.println(angle);

    pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
    // Serial.print("Pulse Width : ");
    // Serial.println(pulse_width);

    pwm.setPWM(servo.pin, 0, pulse_width);
}

// Detaches servos, allowing them to be spun freely
void detachServo(RobotServo_t &servo)
{
    pwm.setPWM(servo.pin, 0, 0);
}
