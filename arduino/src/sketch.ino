#include "Servo.h"
#include "Wire.h"
#include "PID_v1.h"
#include "I2Cdev.h"
#include "L298n.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
MPU6050 mpu;

char serial_buffer[4];
int motor_a_speed = 40;
int motor_b_speed = 40;
unsigned char pan_servo_angle = 90;
unsigned char tilt_servo_angle = 180;
double pid_setpoint, pid_input, pid_output;
int turnAngle = 0;
int alreadyTurnedAngle = 0;
int initial_angle = 0;

PID pid_controller(&pid_input, &pid_output, &pid_setpoint, 2, 0.1, 0.1, DIRECT);

// GYRO

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

int state = 0;

void init_mpu() {
    mpu.initialize();
    mpu.dmpInitialize();

    mpu.setZAccelOffset(1788); 
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Yaw stablelizing
    delay(15000);
    empty_serial();
}

// Try to find another way to do this
void empty_serial() {
    while(Serial.available()) Serial.read();
}

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

    init_mpu();

    pid_setpoint = 0;
}

int read_angle() {
    VectorFloat gravity;   
    Quaternion q;
    float ypr[3];
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


        return ypr[0] * 180/M_PI;
    }
    return -1;
}

void loop() {
    if (state == 0) {
        if (Serial.available() >= 4) {
            Serial.readBytesUntil('#', serial_buffer, 4);

            if (serial_buffer[0] == 'S') {
                pan_servo_angle = serial_buffer[1];
                tilt_servo_angle =  serial_buffer[2];
                panServo.write(pan_servo_angle);
                tiltServo.write(tilt_servo_angle);
                serial_buffer[0] = '#';
            }
        }

        if (pan_servo_angle > 60 && pan_servo_angle < 120) {
            l298n->brakeMotors();
        } else {
            state = 1;
            turnAngle = 90-pan_servo_angle-10;
            if (turnAngle > 0) {
                l298n->backwardMotorA();
                l298n->forwardMotorB(); 
            } else {
                l298n->backwardMotorB();
                l298n->forwardMotorA();
            }
            alreadyTurnedAngle = 0;
            while((initial_angle = read_angle()) == -1);
        } 
    } else {
        if (abs(alreadyTurnedAngle) > abs(turnAngle)) {
            l298n->brakeMotors();
            state = 0;
            pan_servo_angle = 90;
            panServo.write(pan_servo_angle);
            Serial.write((unsigned char)90);
            empty_serial();
        } else {
            int current_angle = -1;
            while((current_angle = read_angle()) == -1);
            alreadyTurnedAngle = initial_angle - current_angle;
            motor_a_speed++;
            motor_b_speed++;  
            l298n->setMotorASpeed(constrain(motor_a_speed, 0, 255));
            l298n->setMotorBSpeed(constrain(motor_b_speed, 0, 255));    
        }

    }
}
