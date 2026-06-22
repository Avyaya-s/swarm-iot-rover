#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

Adafruit_MPU6050 mpu;

// ==========================================
// WI-FI & UDP CONFIGURATION
// ==========================================
const char* ssid     = "YOUR_WIFI_SSID";       // <-- replace with your SSID
const char* password = "YOUR_WIFI_PASSWORD";    // <-- replace with your password
const char* targetIP = "YOUR_LAPTOP_IP";        // <-- replace with your WSL2 host IP
const int   udpPort  = 5000;

WiFiUDP udp;

// ==========================================
// PIN CONFIGURATION
// ==========================================
const int enA_Left = 12;   const int in1_Left = 14;   const int in2_Left = 27;
const int enB_Right = 26;  const int in3_Right = 25;  const int in4_Right = 33;

const int trigLeft = 5;    const int echoLeft = 18;
const int trigFront = 19;  const int echoFront = 21;
const int trigRight = 22;  const int echoRight = 23;

const int I2C_SDA = 4;     const int I2C_SCL = 15;

// ==========================================
// TUNED NAVIGATION & SPEED CONSTANTS
// ==========================================
const int OBSTACLE_DISTANCE = 25;
const int CRITICAL_DISTANCE = 12;
const int CRUISE_SPEED = 125;
const int TURN_SPEED = 145;

unsigned long lastTime = 0;
float yawAngle = 0.0;
float gyroZOffset = 0.0;

void setup() {
  Serial.begin(115200);

  Serial.print("📡 Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  pinMode(enA_Left, OUTPUT); pinMode(in1_Left, OUTPUT); pinMode(in2_Left, OUTPUT);
  pinMode(enB_Right, OUTPUT); pinMode(in3_Right, OUTPUT); pinMode(in4_Right, OUTPUT);
  moveRobot(0, 0);

  pinMode(trigLeft, OUTPUT); pinMode(echoLeft, INPUT);
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(50);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("❌ MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("!!! DO NOT MOVE THE ROBOT !!! Calibrating gyro...");
  delay(1000);
  float totalGyroZ = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    totalGyroZ += g.gyro.z;
    delay(5);
  }
  gyroZOffset = totalGyroZ / samples;

  Serial.println("🚀 Robot Brain Fully Integrated. Launching...");
  delay(1000);
  lastTime = millis();
}

void loop() {
  updateYaw();

  long distLeft  = getDistance(trigLeft, echoLeft);   delay(20);
  long distFront = getDistance(trigFront, echoFront); delay(20);
  long distRight = getDistance(trigRight, echoRight); delay(20);

  char payload[50];
  snprintf(payload, sizeof(payload), "%.1f,%ld,%ld,%ld", yawAngle, distLeft, distFront, distRight);

  udp.beginPacket(targetIP, udpPort);
  udp.print(payload);
  udp.endPacket();

  Serial.print("Transmitting: "); Serial.println(payload);

  if (distFront > 0 && distFront < CRITICAL_DISTANCE) {
    smoothStop(CRUISE_SPEED);
    moveRobot(-130, -130);
    unsigned long backupStart = millis();
    while (millis() - backupStart < 600) {
      updateYaw();
      udp.beginPacket(targetIP, udpPort);
      udp.print(payload);
      udp.endPacket();
      delay(10);
    }
    moveRobot(0, 0);
  }
  else if (distFront > 0 && distFront < OBSTACLE_DISTANCE) {
    smoothStop(CRUISE_SPEED);
    if (distLeft > distRight) {
      turnDegrees(90.0);
    } else {
      turnDegrees(-90.0);
    }
  }
  else {
    moveRobot(CRUISE_SPEED, CRUISE_SPEED);
  }

  delay(10);
}

void smoothStop(int currentSpeed) {
  for (int speed = currentSpeed; speed >= 0; speed -= 25) {
    moveRobot(speed, speed);
    delay(20);
  }
  moveRobot(0, 0);
  delay(100);
}

void updateYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  float correctedGyroZ = g.gyro.z - gyroZOffset;
  if (abs(correctedGyroZ) < 0.03) { correctedGyroZ = 0.0; }
  yawAngle += (correctedGyroZ * 57.2958) * dt;
  if (yawAngle > 180)  yawAngle -= 360;
  if (yawAngle < -180) yawAngle += 360;
}

void turnDegrees(float deltaAngle) {
  updateYaw();
  float targetAngle = yawAngle + deltaAngle;
  if (targetAngle > 180)  targetAngle -= 360;
  if (targetAngle < -180) targetAngle += 360;
  while (true) {
    updateYaw();
    float error = targetAngle - yawAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    if (abs(error) < 6.0) { break; }
    if (error > 0) { moveRobot(-TURN_SPEED, TURN_SPEED); }
    else { moveRobot(TURN_SPEED, -TURN_SPEED); }
    delay(10);
  }
  moveRobot(0, 0);
  delay(300);
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 26000);
  long distance = duration * 0.0343 / 2;
  if (distance == 0 || distance > 400) { return 999; }
  return distance;
}

void moveRobot(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(in1_Left, HIGH);  digitalWrite(in2_Left, LOW); analogWrite(enA_Left, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(in1_Left, LOW);   digitalWrite(in2_Left, HIGH); analogWrite(enA_Left, abs(leftSpeed));
  } else {
    digitalWrite(in1_Left, LOW);   digitalWrite(in2_Left, LOW); analogWrite(enA_Left, 0);
  }
  if (rightSpeed > 0) {
    digitalWrite(in3_Right, HIGH); digitalWrite(in4_Right, LOW); analogWrite(enB_Right, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(in3_Right, LOW);  digitalWrite(in4_Right, HIGH); analogWrite(enB_Right, abs(rightSpeed));
  } else {
    digitalWrite(in3_Right, LOW);  digitalWrite(in4_Right, LOW); analogWrite(enB_Right, 0);
  }
}
