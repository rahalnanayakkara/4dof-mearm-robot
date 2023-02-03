/*
    Follow cubic polynomial trajectories to reach a sequence of intermediate
    configurations starting from a home configuration and returns to home 
    configuration at the end.

    Last Edited - 02/02/2023
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

#define JOINTS 3

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


RobotServo_t robotServos[JOINTS] = {{MIDDLE_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS},
                                    {LEFT_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS},
                                    {RIGHT_PIN, SERVO_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS}};


// number of intermediate configurations excluding home configuration
# define STEPS 3

double q[STEPS+1][JOINTS] = {{90, 150, 60},
                             {150, 60, 90},
                             {90, 120, 150},
                             {30, 60, 90}};

double travel_time[STEPS+1] = {1.5, 1.5, 2, 1.5};

void setup() 
{
    Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
    
    Serial.begin(115200);
    Serial.println();

    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
    yield();
    delay(2000);

    // Moves all servos to home position
    for(int i=0; i<JOINTS; i++)
        writeServo(robotServos[i], (int)q[0][i]);

    Serial.println("Moving to home position");
    delay(2000);

    for(int j=0; j<STEPS; j++)
    {
        moveAbsJ(robotServos, q[j], q[j+1], travel_time[j]);
        delay(100);
    }
    moveAbsJ(robotServos, q[STEPS], q[0], travel_time[STEPS]);

    delay(2000);
    // Detaches all servos
    for(int i=0; i<JOINTS; i++)
        detachServo(robotServos[i]);
    
    Serial.println("Detached Servos");
}

void loop() {}

void moveAbsJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const double qT[JOINTS], double T)
{
    CubicPol_t robot_pol[JOINTS] = {{0.0, 0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0, 0.0}};
    
    double angle[JOINTS] = {0.0, 0.0, 0.0};
    
    for(int i=0; i<JOINTS; i++)
        computePol(robot_pol[i], q0[i], qT[i], T);

    Serial.println("Begining Motion");
    
    unsigned long time_ms = 0;
    unsigned long base_time = millis();

    while(1)
    {
        time_ms = millis() - base_time;

        for(int i=0; i<JOINTS; i++)
        {
            angle[i] = evaluatePol(robot_pol[i], time_ms);
            writeServo(robotServos[i], angle[i]);
        }

        if (time_ms>T*1000) 
            break;

        delay(20);
    }
    Serial.println("End position reached");
}

void computePol(CubicPol_t &pol, int q0, int qT, double T)
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
void writeServo(const RobotServo_t &servo, int angle)
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
