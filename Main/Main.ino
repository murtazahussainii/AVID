#include <SoftwareSerial.h>

#define MOTOR_PWM 5  // PWM pin for motor speed control
#define RX_PIN 10 // Arduino RX (Connects to ESP32 TX)
#define TX_PIN 11 // Arduino TX (Connects to ESP32 RX)

// Motor control pins
const int motor1Pin1 = 8;
const int motor1Pin2 = 7;
const int motor2Pin1 = 13;
const int motor2Pin2 = 12;

// Ultrasonic sensor 1 (Left Front)
#define ULTRA1_TRIG A0
#define ULTRA1_ECHO A1

SoftwareSerial mySerial(RX_PIN, TX_PIN); // RX, TX

void setup() {
    Serial.begin(9600);
    mySerial.begin(115200); // Start Software Serial to ESP32

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(ULTRA1_TRIG, OUTPUT);
    pinMode(ULTRA1_ECHO, INPUT);

    moveForward();
}

long readUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 50000);
    if (duration == 0) return 999;

    return duration * 0.034 / 2;
}

void moveForward() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Moving forward...");
}

void stopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 0);
    Serial.println("Stopping...");
}

void loop() {
    long leftDistance = readUltrasonicDistance(ULTRA1_TRIG, ULTRA1_ECHO);
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" cm");

    mySerial.println("Distance: " + String(leftDistance) + " cm"); // Send to ESP32

    if (leftDistance > 5) {
        moveForward();
    } else {
        stopMotors();
        delay(500);
        moveBackward();
        delay(1800);

        stopMotors();
        delay(3000);
        rotateLeft90();

        moveForward();
        delay(1000);
    }
}

void moveBackward() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Moving backward...");
}

void rotateLeft90() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Rotating LEFT 90 degrees...");
    delay(8000);
    stopMotors();
}
