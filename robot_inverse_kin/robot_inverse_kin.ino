/*
    Given a sequence of points in cartesian coordinates, 
    calculates the inverse kinematics and follow a cubic polynomial
    trajectory between the points and returns home.

    Last Edited - 02/02/2023
*/

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define square(x) (x)*(x)
#define cube(x) (x)*(x)*(x)

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SERVO_MIN_POS 0
#define SERVO_MAX_POS 180

#define MIN_PWM 120
#define MAX_PWM 510
#define FREQUENCY 50

#define MIDDLE_PIN 4
#define LEFT_PIN 8
#define RIGHT_PIN 11

#define MIDDLE_OFFSET 0
#define LEFT_OFFSET 50
#define RIGHT_OFFSET -10

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

typedef struct
{
    double l0;
    double h1;
    double l1;
    double l2;
    double l3;
    double l3I;
    double l3O;
    double l4;
    double l5;
    double d5;
} RobotParams_t;

typedef struct
{
    double x;
    double y;
    double z;
} RobotPosition_t;

double q_current[JOINTS] = {0, 0, 0};
double q_target[JOINTS] = {0, 0, 0};

RobotServo_t robotServos[JOINTS] = {{MIDDLE_PIN, MIDDLE_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS},
                                    {LEFT_PIN, LEFT_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS},
                                    {RIGHT_PIN, RIGHT_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS}};

RobotParams_t params={0, 64, 15, 80, 80, 35, 35, 80, 65, 5};

// Initially set to home configuration
double q_home[JOINTS] = {90, 90, 90};

// number of intermediate points
#define POINTS 3

//Move sequence
RobotPosition_t p[POINTS] = {{0, -200, 30},
                            {150, 0, 80},
                            {0, 180, 100}};

void moveJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const RobotPosition_t target, const float T, const RobotParams_t &params);

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
    {
        q_current[i] = q_home[i];
        writeServo(robotServos[i], (int)q_current[i]);
    }

    Serial.println("Moving to home position");
    delay(2000);

    Serial.println("Begining moving sequence");
    for(int i=0; i<POINTS; i++)
    {
        moveJ(robotServos, q_current, p[i], 1.5, params);
        delay(100);
    }
    
    delay(100);
    moveAbsJ(robotServos, q_current, q_home, 2);
    Serial.println("Completed");

    delay(2000);
    // Detaches all servos
    for(int i=0; i<JOINTS; i++)
        detachServo(robotServos[i]);
    
    Serial.println("Detached Servos");
}

void loop() {}

void moveJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const RobotPosition_t target, const float T, const RobotParams_t &params)
{
    inverseKin(target, params, q_target);
    moveAbsJ(robotServos, q0, q_target, T);

    for(int j=0; j<JOINTS; j++)
        q_current[j] = q_target[j];
}

void inverseKin(const RobotPosition_t &target, const RobotParams_t &params, double *q)
{
    // Step 1
    double q1 = atan2(target.x-params.l0, -target.y);
    q1 += asin( params.d5 / sqrt(square((target.x-params.l0)) + square(target.y)) );

    double pwx = target.x - params.l5 * sin(q1) - params.l0;
    double pwy = target.y + params.l5 * cos(q1);
    double pwz = target.z;

    // Step 2
    double r = sqrt(square(pwx) + square(pwy)) - params.l1;
    double ze = pwz - params.h1;
    double alpha = atan2(ze, r);
    double s = sqrt(square(r) + square(ze));

    double q30 = PI - acos((square(params.l3) + square(params.l2) - square(s)) / (2*params.l2*params.l3) );

    double q2 = PI - alpha - acos((square(s) + square(params.l2) - square(params.l3)) / (2*s*params.l2) );

    // Step 3
    double e = sqrt(square(params.l3O) + square(params.l2) - 2*params.l2*params.l3O*cos(q30) );

    double psi = asin(params.l3O * sin(q30) / e);
    double phi = acos((square(e) + square(params.l3I) - square(params.l4)) / (2*e*params.l3I));

    double q3 = psi + phi + PI/2 - q2;

    q[0] = round(q1 * 180 / PI);
    q[1] = round(q3 * 180 / PI);
    q[2] = round(q2 * 180 / PI);

    Serial.print(q[0]);
    Serial.print("\t");
    Serial.print(q[1]);
    Serial.print("\t");
    Serial.println(q[2]);
}

// Moves robot from q0 configuration to qT within time T
void moveAbsJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const double qT[JOINTS], double T)
{
    CubicPol_t robot_pol[JOINTS] = {{0.0, 0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0, 0.0}};
    
    double angle[JOINTS] = {0.0, 0.0, 0.0};
    
    for(int i=0; i<JOINTS; i++)
        computePol(robot_pol[i], q0[i], qT[i], T);
    
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
}

// Compute the cubic polynomial trajectory given the path parameters
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
