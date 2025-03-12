#define MOTOR_PWM 5  // PWM pin for motor speed control

// Motor control pins
const int motor1Pin1 = 8;  // IN1 on L298N for Motor 1
const int motor1Pin2 = 7;  // IN2 on L298N for Motor 1
const int motor2Pin1 = 13; // IN1 on L298N for Motor 2
const int motor2Pin2 = 12; // IN2 on L298N for Motor 2

// Ultrasonic sensor 1 (Left Front)
#define ULTRA1_TRIG A0
#define ULTRA1_ECHO A1

// Ultrasonic sensor 2 (Right Front)
#define ULTRA2_TRIG 3  
#define ULTRA2_ECHO 2  

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

    // Set ultrasonic sensor 2 (Right Front)
    pinMode(ULTRA2_TRIG, OUTPUT);
    pinMode(ULTRA2_ECHO, INPUT);

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

// Move backward
void moveBackward() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Moving backward...");
}

// Rotate in place (90 degrees) to the **right**
void rotateRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Rotating RIGHT...");
    delay(700);
    stopMotors();
}

// Rotate in place (90 degrees) to the **left**
void rotateLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM, 150);
    Serial.println("Rotating LEFT...");
    delay(700);
    stopMotors();
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
    // Read distances from both ultrasonic sensors
    long leftDistance = readUltrasonicDistance(ULTRA1_TRIG, ULTRA1_ECHO);
    long rightDistance = readUltrasonicDistance(ULTRA2_TRIG, ULTRA2_ECHO);

    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.print(" cm | Right Distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");

    // **FOV-Based Obstacle Avoidance**
    if (leftDistance < 15 && rightDistance < 15) {  
        // Both sensors detect an obstacle (blockage in front)
        stopMotors();
        delay(500);
        moveBackward();
        delay(1000);
        stopMotors();
        delay(500);
        rotateRight();  // Default turn direction
        moveForward();
    } 
    else if (leftDistance < 15) {  
        // Left sensor detects an obstacle, so turn right
        stopMotors();
        delay(500);
        rotateRight();
        moveForward();
    } 
    else if (rightDistance < 15) {  
        // Right sensor detects an obstacle, so turn left
        stopMotors();
        delay(500);
        rotateLeft();
        moveForward();
    }
}
