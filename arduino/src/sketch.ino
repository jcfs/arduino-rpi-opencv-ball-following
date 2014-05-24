#include "Servo.h"
#include "Wire.h"
#include "PID_v1.h"
#include "I2Cdev.h"
#include "L298n.h"

#define DEBUG 1
#define RUNNING_TIME 8000
#define TOP_SPEED 255

#define SERVO_PAN   9
#define SERVO_TILT 10

// Motor Control pins
#define ENABLE_A 11
#define MOTOR_A1 8
#define MOTOR_A2 12

#define ENABLE_B 3
#define MOTOR_B1 5
#define MOTOR_B2 7


// Globals
L298n * l298n;
Servo panServo;
Servo tiltServo;

char serialBuffer[4];
int motor_a_speed = TOP_SPEED;
int motor_b_speed = TOP_SPEED;
unsigned char pan_servo_angle = 90;
unsigned char tilt_servo_angle = 180;
double pid_setpoint, pid_input, pid_output;

PID pid_controller(&pid_input, &pid_output, &pid_setpoint, 2, 0.1, 0.1, DIRECT);

void setup() {
    // begin coms
    Wire.begin();
    Serial.begin(115200);
   
    // setup servos 
    panServo.attach(SERVO_PAN);
    tiltServo.attach(SERVO_TILT);
    panServo.write(pan_servo_angle);
    tiltServo.write(tilt_servo_angle);

    // setup pid
    pid_controller.SetMode(AUTOMATIC);
    pid_controller.SetOutputLimits(-255, 255); 

    // setup motors
    l298n = new L298n(ENABLE_A, MOTOR_A1, MOTOR_A2, ENABLE_B, MOTOR_B1, MOTOR_B2);

    pid_setpoint = 0;
}

void loop() {
    if (Serial.available() >= 4) {
        Serial.readBytesUntil('#', serialBuffer, 4);

        if (serialBuffer[0] == 'S') {
            pan_servo_angle = serialBuffer[1];
            tilt_servo_angle =  serialBuffer[2];
            panServo.write(pan_servo_angle);
            tiltServo.write(tilt_servo_angle);
            serialBuffer[0] = '#';

            l298n->forwardMotorB();
            l298n->forwardMotorA();
        }
    }

    pid_input = 90 - pan_servo_angle;

    if (pid_controller.Compute()) {
        pid_output = abs(pid_output);

        if (pid_input > 0) {
            motor_b_speed = TOP_SPEED - abs(pid_output);
        } else {
            motor_a_speed = TOP_SPEED - abs(pid_output);
        }

        if (motor_a_speed < TOP_SPEED && motor_b_speed < TOP_SPEED) {
            if (motor_a_speed > motor_b_speed) {
                int increase = TOP_SPEED * motor_b_speed / motor_a_speed;
                motor_a_speed = TOP_SPEED;
                motor_b_speed += increase;
            } else {
                int increase = TOP_SPEED * motor_a_speed / motor_b_speed;
                motor_a_speed += increase;
                motor_b_speed = TOP_SPEED;
            }
        }

        motor_a_speed = constrain(motor_a_speed, 0, TOP_SPEED);
        motor_b_speed = constrain(motor_b_speed, 0, TOP_SPEED);

    } 

    Serial.print(motor_a_speed);
    Serial.print("-");
    Serial.println(motor_b_speed);

    l298n->setMotorASpeed(motor_a_speed);
    l298n->setMotorBSpeed(motor_b_speed);    
}
