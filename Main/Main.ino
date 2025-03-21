#define MOTOR_PWM_1 5  // PWM pin for motor 1 speed control
#define MOTOR_PWM_2 6  // PWM pin for motor 2 speed control

// Motor control pins
const int motor1Pin1 = 8;  // IN1 on L298N for Motor 1
const int motor1Pin2 = 7;  // IN2 on L298N for Motor 1
const int motor2Pin1 = 13; // IN1 on L298N for Motor 2
const int motor2Pin2 = 12; // IN2 on L298N for Motor 2

// Ultrasonic sensor 1 (Left Front)
#define ULTRA1_TRIG A0
#define ULTRA1_ECHO A1

bool turnLeft = true; // Toggle turn direction

void setup() {
    Serial.begin(9600);

    // Set motor control pins as outputs
    pinMode(MOTOR_PWM_1, OUTPUT);
    pinMode(MOTOR_PWM_2, OUTPUT);
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
    analogWrite(MOTOR_PWM_1, 0); 
    analogWrite(MOTOR_PWM_2, 0); 
    Serial.println("Moving forward...");
}

// Stop motors
void stopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM_1, 0);
    analogWrite(MOTOR_PWM_2, 0);
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
        delay(1800);

        // Decide turn direction based on toggle
        stopMotors();
        if (turnLeft) {
            rotateLeft90();
        } else {
            rotateRight90();
        }
        turnLeft = !turnLeft; // Toggle turn direction

        // Move forward 20 cm to clean the start of the next row
        moveForward();
        delay(1000);
    }
}

// Move backward
void moveBackward() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(MOTOR_PWM_1, 145); // Adjusted speed for motor 1
    analogWrite(MOTOR_PWM_2, 155); // Adjusted speed for motor 2
    Serial.println("Moving backward...");
}

// Rotate left 90 degrees by disabling the left tire and moving the right tire
void rotateLeft90() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM_2, 155); // Ensure balanced speed in rotation
    Serial.println("Rotating LEFT 90 degrees...");
    delay(9000);
    stopMotors();
}

// Rotate right 90 degrees by disabling the right tire and moving the left tire
void rotateRight90() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(MOTOR_PWM_1, 145); // Ensure balanced speed in rotation
    Serial.println("Rotating RIGHT 90 degrees...");
    delay(9000);
    stopMotors();
}
