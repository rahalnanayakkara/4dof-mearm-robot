/*
    Continuously move angle from start to end point

    Last Edited - 01/02/2023
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

// Starting angles
#define MIDDLE_ANGLE_START 60
#define LEFT_ANGLE_START 120
#define RIGHT_ANGLE_START 150
#define CLAW_ANGLE_START 35

// Stopping angles
#define MIDDLE_ANGLE_STOP 120
#define LEFT_ANGLE_STOP 60
#define RIGHT_ANGLE_STOP 90
#define CLAW_ANGLE_STOP 80

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

    // Moves all servos to initial position
    writeServo(middle_servo, MIDDLE_ANGLE_START);
    writeServo(left_servo, LEFT_ANGLE_START);
    writeServo(right_servo, RIGHT_ANGLE_START);
    writeServo(claw_servo, CLAW_ANGLE_START);
    Serial.println("Moving to start position");
    delay(2000);

    int middle_angle = MIDDLE_ANGLE_START;
    int left_angle = LEFT_ANGLE_START;
    int right_angle = RIGHT_ANGLE_START;
    int claw_angle = CLAW_ANGLE_START;

    Serial.println("Begining Motion");
    while(1)
    {
        if (middle_angle<MIDDLE_ANGLE_STOP)
            middle_angle++;
        else if (middle_angle>MIDDLE_ANGLE_STOP)
            middle_angle--;
        
        if (left_angle<LEFT_ANGLE_STOP)
            left_angle++;
        else if (left_angle>LEFT_ANGLE_STOP)
            left_angle--;

        if (right_angle<RIGHT_ANGLE_STOP)
            right_angle++;
        else if (right_angle>RIGHT_ANGLE_STOP)
            right_angle--;
        
        if (claw_angle<CLAW_ANGLE_STOP)
            claw_angle++;
        else if (claw_angle>CLAW_ANGLE_STOP)
            claw_angle--;

        if ((middle_angle==MIDDLE_ANGLE_STOP) && (claw_angle==CLAW_ANGLE_STOP) && (left_angle==LEFT_ANGLE_STOP) && (right_angle==RIGHT_ANGLE_STOP))
            break;

        writeServo(middle_servo, middle_angle);
        writeServo(left_servo, left_angle);
        writeServo(right_servo, right_angle);
        writeServo(claw_servo, claw_angle);

        delay(20);
    }
    Serial.println("End position reached");

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
