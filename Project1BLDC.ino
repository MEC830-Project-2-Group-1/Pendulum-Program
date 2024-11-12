#include <PID_v1_bc.h>
#include <Wire.h>
#include "MPU6050.h"
#include <Servo.h>
#include <math.h> // Include this for fabs()

// PID Parameters
double Kp = 2, Ki = 0, Kd = 0;
double PendulumAngle, MotorSpeed, DesiredAngle = 90;
PID ReactionPID(&PendulumAngle, &MotorSpeed, &DesiredAngle, Kp, Ki, Kd, REVERSE);

// BLDC Motors
Servo ESC1, ESC2;
float bldcSpeed;

// IMU Sensor (MPU6050)
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

void setup() {
    // Initialize PID
    ReactionPID.SetMode(AUTOMATIC);
    ReactionPID.SetOutputLimits(-255, 255);

    // Initialize BLDC motors
    ESC1.attach(9, 1000, 2000);
    ESC2.attach(10, 1000, 2000);

    // Initialize Serial and IMU
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();

    ESC1.write(180);
    ESC2.write(180);
    delay(5000);
    ESC1.write(0);
    ESC2.write(0);
    delay(2000);
}


void loop() {
    // Read IMU data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    data.Y = map(ay, -17000, 17000, 0, 180);

    // Calculate Pendulum Angle (using accelerometer data)
    PendulumAngle = data.Y;

    // Compute PID output
    ReactionPID.Compute();

    // Map PID output to BLDC speed
    bldcSpeed = map(MotorSpeed, -255, 255, -90, 90);

    // Control motors based on sign of MotorSpeed
    if (MotorSpeed < 0) {
        ESC1.write(fabs(bldcSpeed)); // Use fabs() instead of abs()
        ESC2.write(0);  // Stop the other motor
    } else {
        ESC2.write(fabs(bldcSpeed)); // Use fabs() instead of abs()
        ESC1.write(0);  // Stop the other motor
    }

    // Print the Pendulum Angle
    if (PendulumAngle < 0) {
        Serial.print("Pendulum angle: -");
        Serial.print(PendulumAngle);
    } else {
        Serial.print(" Pendulum Angle: ");
        Serial.print(PendulumAngle);
    }

    // Print debug information
    Serial.print("PID Speed: ");
    Serial.print(MotorSpeed);
    Serial.print(" BLDC Speed: ");
    Serial.println(bldcSpeed);

    // Non-blocking delay for smooth loop execution
    delay(500); // Adjust for responsiveness
}
