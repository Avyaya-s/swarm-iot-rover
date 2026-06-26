#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "Avyaya";
const char* password = "102030908070";
const char* targetIP = "172.27.142.68";
const int   udpPort  = 5000;

WiFiUDP udp;

const int enA_Left  = 12;  const int in1_Left  = 14;  const int in2_Left  = 27;
const int enB_Right = 26;  const int in3_Right = 25;  const int in4_Right = 33;

const int trigLeft  = 5;   const int echoLeft  = 18;
const int trigFront = 19;  const int echoFront = 21;
const int trigRight = 22;  const int echoRight = 23;

const int CRUISE_SPEED  = 110;
const int TURN_SPEED    = 115;
const int OBSTACLE_DIST = 25;

const unsigned long TIME_90DEG = 2000;  // tune this if turn is not exactly 90

float yawAngle = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(enA_Left,  OUTPUT); pinMode(in1_Left,  OUTPUT); pinMode(in2_Left,  OUTPUT);
  pinMode(enB_Right, OUTPUT); pinMode(in3_Right, OUTPUT); pinMode(in4_Right, OUTPUT);
  moveRobot(0, 0);

  pinMode(trigLeft,  OUTPUT); pinMode(echoLeft,  INPUT);
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
  Serial.println("Launching in 2 seconds...");
  delay(2000);
}

void loop() {
  long distLeft  = getDistance(trigLeft,  echoLeft);  delay(20);
  long distFront = getDistance(trigFront, echoFront); delay(20);
  long distRight = getDistance(trigRight, echoRight); delay(20);

  sendUDP(distLeft, distFront, distRight);

  Serial.print("Yaw: "); Serial.print(yawAngle, 1);
  Serial.print(" | L: "); Serial.print(distLeft);
  Serial.print("cm | F: "); Serial.print(distFront);
  Serial.print("cm | R: "); Serial.print(distRight);
  Serial.println("cm");

  // No obstacle — cruise forward
  if (distFront >= OBSTACLE_DIST || distFront == 999) {
    Serial.println("-> FORWARD");
    moveRobot(CRUISE_SPEED, CRUISE_SPEED);
    delay(10);
    return;
  }

  // Front obstacle — stop and assess
  Serial.println("WARNING: FRONT OBSTACLE");
  moveRobot(0, 0);
  delay(200);

  long sideLeft  = getDistance(trigLeft,  echoLeft);  delay(30);
  long sideRight = getDistance(trigRight, echoRight); delay(30);

  bool leftBlocked  = (sideLeft  < OBSTACLE_DIST && sideLeft  != 999);
  bool rightBlocked = (sideRight < OBSTACLE_DIST && sideRight != 999);

  Serial.print("Side check -> L: "); Serial.print(sideLeft);
  Serial.print("cm | R: "); Serial.print(sideRight);
  Serial.println("cm");

  if (!leftBlocked && !rightBlocked) {
    // Both free — go toward more open side
    if (sideLeft > sideRight) {
      Serial.println("-> BOTH FREE — more space LEFT, turning LEFT 90");
      turnLeft90();
    } else {
      Serial.println("-> BOTH FREE — more space RIGHT, turning RIGHT 90");
      turnRight90();
    }
  }
  else if (leftBlocked && !rightBlocked) {
    Serial.println("-> LEFT BLOCKED, RIGHT FREE — turning RIGHT 90");
    turnRight90();
  }
  else if (!leftBlocked && rightBlocked) {
    Serial.println("-> RIGHT BLOCKED, LEFT FREE — turning LEFT 90");
    turnLeft90();
  }
  else {
    Serial.println("-> BOTH BLOCKED — reversing 5 seconds");
    moveRobot(-CRUISE_SPEED, -CRUISE_SPEED);
    unsigned long revStart = millis();
    while (millis() - revStart < 5000) {
      long rL = getDistance(trigLeft,  echoLeft);  delay(20);
      long rF = getDistance(trigFront, echoFront); delay(20);
      long rR = getDistance(trigRight, echoRight); delay(20);
      sendUDP(rL, rF, rR);
      delay(10);
    }
    moveRobot(0, 0);
    Serial.println("-> STOPPED");
    while (true) delay(1000);
  }
}

void turnLeft90() {
  Serial.println("Turning LEFT 90deg");
  moveRobot(-TURN_SPEED, TURN_SPEED);
  streamUDP(TIME_90DEG);
  moveRobot(0, 0);
  delay(200);
  yawAngle += 90.0;
  if (yawAngle > 180) yawAngle -= 360;
  Serial.print("Yaw now: "); Serial.println(yawAngle);
}

void turnRight90() {
  Serial.println("Turning RIGHT 90deg");
  moveRobot(TURN_SPEED, -TURN_SPEED);
  streamUDP(TIME_90DEG);
  moveRobot(0, 0);
  delay(200);
  yawAngle -= 90.0;
  if (yawAngle < -180) yawAngle += 360;
  Serial.print("Yaw now: "); Serial.println(yawAngle);
}

void streamUDP(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    long tL = getDistance(trigLeft,  echoLeft);
    long tF = getDistance(trigFront, echoFront);
    long tR = getDistance(trigRight, echoRight);
    sendUDP(tL, tF, tR);
    delay(50);
  }
}

void sendUDP(long l, long f, long r) {
  char payload[50];
  snprintf(payload, sizeof(payload), "%.1f,%ld,%ld,%ld", yawAngle, l, f, r);
  udp.beginPacket(targetIP, udpPort);
  udp.print(payload);
  udp.endPacket();
  Serial.print("TX: "); Serial.println(payload);
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 26000);
  long distance = duration * 0.0343 / 2;
  if (duration == 0)  return 999;
  if (distance > 400) return 999;
  if (distance < 2)   return 2;
  return distance;
}

void moveRobot(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(in1_Left, HIGH); digitalWrite(in2_Left, LOW);
    analogWrite(enA_Left, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(in1_Left, LOW);  digitalWrite(in2_Left, HIGH);
    analogWrite(enA_Left, abs(leftSpeed));
  } else {
    digitalWrite(in1_Left, LOW);  digitalWrite(in2_Left, LOW);
    analogWrite(enA_Left, 0);
  }
  if (rightSpeed > 0) {
    digitalWrite(in3_Right, HIGH); digitalWrite(in4_Right, LOW);
    analogWrite(enB_Right, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(in3_Right, LOW);  digitalWrite(in4_Right, HIGH);
    analogWrite(enB_Right, abs(rightSpeed));
  } else {
    digitalWrite(in3_Right, LOW);  digitalWrite(in4_Right, LOW);
    analogWrite(enB_Right, 0);
  }
}