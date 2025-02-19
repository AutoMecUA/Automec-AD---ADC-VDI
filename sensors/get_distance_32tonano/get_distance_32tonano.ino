#include <Arduino.h>

// Define GPIO pins for ultrasonic sensors
const int frontLeftTrig = 2, frontLeftEcho = 3;
const int frontRightTrig = 4, frontRightEcho = 5;
const int backLeftTrig = 6, backLeftEcho = 7;
const int backRightTrig = 8, backRightEcho = 9;
const int sideLeftFrontTrig = 10, sideLeftFrontEcho = 11;
const int sideLeftBackTrig = 12, sideLeftBackEcho = 13;
const int sideRightFrontTrig = 14, sideRightFrontEcho = 15;
const int sideRightBackTrig = 16, sideRightBackEcho = 17;

// Function to get distance from ultrasonic sensor
long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;  // Convert to cm
    return distance;
}

void setup() {
    Serial.begin(115200); // Start USB serial communication
    
    // Configure pins
    int trigPins[] = {frontLeftTrig, frontRightTrig, backLeftTrig, backRightTrig, 
                      sideLeftFrontTrig, sideLeftBackTrig, sideRightFrontTrig, sideRightBackTrig};
    int echoPins[] = {frontLeftEcho, frontRightEcho, backLeftEcho, backRightEcho, 
                      sideLeftFrontEcho, sideLeftBackEcho, sideRightFrontEcho, sideRightBackEcho};
    
    for (int i = 0; i < 8; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
}

void loop() {
    long distances[8];
    
    // Read distances from all sensors
    distances[0] = getDistance(frontLeftTrig, frontLeftEcho);
    distances[1] = getDistance(frontRightTrig, frontRightEcho);
    distances[2] = getDistance(backLeftTrig, backLeftEcho);
    distances[3] = getDistance(backRightTrig, backRightEcho);
    distances[4] = getDistance(sideLeftFrontTrig, sideLeftFrontEcho);
    distances[5] = getDistance(sideLeftBackTrig, sideLeftBackEcho);
    distances[6] = getDistance(sideRightFrontTrig, sideRightFrontEcho);
    distances[7] = getDistance(sideRightBackTrig, sideRightBackEcho);
    
    // Send data via USB serial in JSON format
    Serial.print("{");
    Serial.print("\"front_left\":"); Serial.print(distances[0]); Serial.print(",");
    Serial.print("\"front_right\":"); Serial.print(distances[1]); Serial.print(",");
    Serial.print("\"back_left\":"); Serial.print(distances[2]); Serial.print(",");
    Serial.print("\"back_right\":"); Serial.print(distances[3]); Serial.print(",");
    Serial.print("\"side_left_front\":"); Serial.print(distances[4]); Serial.print(",");
    Serial.print("\"side_left_back\":"); Serial.print(distances[5]); Serial.print(",");
    Serial.print("\"side_right_front\":"); Serial.print(distances[6]); Serial.print(",");
    Serial.print("\"side_right_back\":"); Serial.print(distances[7]);
    Serial.println("}");
    
    delay(100); // Wait before next read
}
