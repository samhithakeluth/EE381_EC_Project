#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

// Initialize Bluetooth Serial (D10 = RX, D11 = TX)
SoftwareSerial BT(10, 11); // HC-05: TX to D10, RX to D11 (via voltage divider)

// Initialize MPU6050 with I2C Wire
MPU6050 mpu(Wire);

// Step detection variables
int stepCount = 0;
bool stepDetected = false;
const float threshold = 1.2; // Acceleration threshold for step
unsigned long lastStepTime = 0;
const unsigned long debounceTime = 300; // Time gap between steps

// Buzzer
const int buzzerPin = 8;

void setup() {
  Serial.begin(9600);     // Debug via Serial Monitor
  BT.begin(9600);         // Bluetooth output
  Wire.begin();           // Start I2C

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Initialize MPU6050
  if (mpu.begin() != 0) {
    Serial.println("MPU FAIL!");
    BT.println("MPU FAIL!");
    while (1); // Stop here
  }

  // Calibrate MPU6050
  mpu.calcOffsets(true, true); // true = print offsets to Serial

  Serial.println("Pedometer Ready");
  BT.println("Pedometer Ready");
  delay(1500);
}

void loop() {
  mpu.update(); // Read accelerometer data

  float az = mpu.getAccZ(); // Get vertical acceleration
  unsigned long now = millis();

  // Step detection logic
  if (az > threshold && !stepDetected && (now - lastStepTime > debounceTime)) {
    stepCount++;
    stepDetected = true;
    lastStepTime = now;

    // Optional buzzer every 25 steps
    if (stepCount % 25 == 0) {
      beep();
    }

    // Send data via Bluetooth and Serial
    BT.print("Steps: ");
    BT.println(stepCount);
    Serial.print("Steps: ");
    Serial.println(stepCount);
  }

  // Reset detection once below lower threshold
  if (az < 0.8) {
    stepDetected = false;
  }

  delay(100); // Polling delay
}

void beep() {
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);
}

