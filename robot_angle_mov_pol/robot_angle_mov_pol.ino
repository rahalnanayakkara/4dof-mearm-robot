/*
    Continuously move angle from start to end point

    Last Edited - 01/02/2023
*/

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define square(x) x*x
#define cube(x) x*x*x

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

// Time
# define TIME 1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct
{
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

typedef struct
{
    double a;
    double b;
    double c;
    double d;
} CubicPol_t;


RobotServo_t middle_servo = {MIDDLE_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};
RobotServo_t left_servo = {LEFT_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};
RobotServo_t right_servo = {RIGHT_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};
RobotServo_t claw_servo = {CLAW_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};

CubicPol_t middle_pol = {0.0, 0.0, 0.0, 0.0};
CubicPol_t left_pol = {0.0, 0.0, 0.0, 0.0};
CubicPol_t right_pol = {0.0, 0.0, 0.0, 0.0};
CubicPol_t claw_pol = {0.0, 0.0, 0.0, 0.0};

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

    // Compute Trajectory Polynomials
    computePol(middle_pol, MIDDLE_ANGLE_START, MIDDLE_ANGLE_STOP, TIME);
    computePol(left_pol, LEFT_ANGLE_START, LEFT_ANGLE_STOP, TIME);
    computePol(right_pol, RIGHT_ANGLE_START, RIGHT_ANGLE_STOP, TIME);
    computePol(claw_pol, CLAW_ANGLE_START, CLAW_ANGLE_STOP, TIME);

    Serial.println("Begining Motion");
    
    unsigned long time_ms = 0;
    unsigned long base_time = millis();
    
    while(1)
    {
        time_ms = millis() - base_time;

        middle_angle = evaluatePol(middle_pol, time_ms);
        left_angle = evaluatePol(left_pol, time_ms);
        right_angle = evaluatePol(right_pol, time_ms);

        writeServo(middle_servo, middle_angle);
        writeServo(left_servo, left_angle);
        writeServo(right_servo, right_angle);

        if (time_ms>TIME*1000) 
            break;

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

void computePol(CubicPol_t &pol, int q0, int qT, int T)
{
    pol.a = (-2.0*(qT-q0)) / (cube(T));
    pol.b = (3.0*(qT-q0)) / (square(T));
    pol.c = 0.0;
    pol.d = 1.0 * q0;
}

double evaluatePol(CubicPol_t &pol, unsigned long time_ms)
{
    double t = (1.0 * time_ms) / 1000;
    return pol.a * (cube(t)) + pol.b * (square(t)) + pol.c * t + pol.d;
}

// Converts angle to pulse width
void writeServo(RobotServo_t &servo, int angle)
{
    int pulse_width;
    angle = constrain(angle, servo.min_pos, servo.max_pos);
    pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
    pwm.setPWM(servo.pin, 0, pulse_width);
}

// Detaches servos, allowing them to be spun freely
void detachServo(RobotServo_t &servo)
{
    pwm.setPWM(servo.pin, 0, 0);
}

// Prints Polynomial
void printPol(CubicPol_t pol)
{
    Serial.print(pol.a);
    Serial.print(" ");
    Serial.print(pol.b);
    Serial.print(" ");
    Serial.print(pol.c);
    Serial.print(" ");
    Serial.print(pol.d);
    Serial.println();
}
