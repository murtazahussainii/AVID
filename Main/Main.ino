#define MOTOR_PWM 5  // PWM pin for motor speed control

// Motor control pins
const int motor1Pin1 = 8;  // IN1 on L298N for Motor 1
const int motor1Pin2 = 7;  // IN2 on L298N for Motor 1
const int motor2Pin1 = 13; // IN1 on L298N for Motor 2
const int motor2Pin2 = 12; // IN2 on L298N for Motor 2

// Ultrasonic sensor 1 (Left Front)
#define ULTRA1_TRIG A0
#define ULTRA1_ECHO A1

void setup() {
    Serial.begin(9600);

    // Set motor control pins as outputs
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    // Set ultrasonic sensor 1 (Left Front)
    pinMode(ULTRA1_TRIG, OUTPUT);
    pinMode(ULTRA1_ECHO, INPUT);

    moveForward();
}

// Function to measure distance from an ultrasonic sensor
long readUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 50000);
    if (duration == 0) return 999;  // No response, return large value

    return duration * 0.034 / 2;  // Convert to cm
}

// Move forward
void moveForward() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Moving forward...");
}

// Stop motors
void stopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 0);
    Serial.println("Stopping...");
}

void loop() {
    // Read distance from ultrasonic sensor 1 (Left Front)
    long leftDistance = readUltrasonicDistance(ULTRA1_TRIG, ULTRA1_ECHO);
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" cm");

    // Move forward until very close to the wall
    if (leftDistance > 5) {  
        moveForward();
    } else {
        // Wall detected, move backward until there is enough space
        stopMotors();
        delay(500);
        moveBackward();
        delay(3000);

        // Rotate left 90 degrees (disable left tire, move right tire)
        stopMotors();
        delay(500);
        rotateLeft90();

        // Move forward 20 cm to clean the start of the next row
        moveForward();
        delay(1000);

        // Rotate another 90 degrees to complete 180-degree turn
        stopMotors();
        delay(500);
        rotateLeft90();
    }
}

// Move backward
void moveBackward() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Moving backward...");
}

// Rotate left 90 degrees by disabling the left tire and moving the right tire
void rotateLeft90() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Rotating LEFT 90 degrees...");
    delay(700);
    stopMotors();
}
