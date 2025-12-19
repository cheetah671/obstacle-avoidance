#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// ===== WiFi Configuration =====
const char* ssid = "hotspotdev";
const char* password = "hotspotdev";

// ===== Firebase Configuration =====
#define API_KEY ""
#define DATABASE_URL ""
                                                                                                                    
// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ===== Hardware Pins =====
const int motorA_IN1 = 2;
const int motorA_IN2 = 4;
const int motorB_IN3 = 12;
const int motorB_IN4 = 14;
const int motorA_ENA = 16;
const int motorB_ENB = 27;
const int trigPin = 5;
const int echoPin = 34;
const int servoPin = 32;

Servo myServo;

// ===== MPU6050 =====
Adafruit_MPU6050 mpu;
float yaw = 0.0;
float driftRate = 0.0;
unsigned long lastMPUUpdate = 0;

// ===== Stuck Detection Variables =====
float lastKnownHeading = 0.0;
unsigned long lastHeadingChangeTime = 0;
const unsigned long STUCK_TIMEOUT = 3000; // 3 seconds
bool isStuck = false;
const float HEADING_CHANGE_THRESHOLD = 2.0; // 2 degrees minimum change

// ===== Robot State & Control Variables (Firebase Driven) =====
String currentMode = "manual";
String lastProcessedCommand = "stop";
bool firebaseReady = false;

// ===== Speed Control (PWM) =====
int currentSpeed = 180; // Default to min
const int MIN_PWM = 180; // Minimum PWM for motors to move
const int MAX_PWM = 255;
unsigned long lastSpeedCheck = 0;
const unsigned long speedCheckInterval = 500; // Check speed every 500ms

// ===== Timing for Non-Blocking Operations =====
unsigned long lastSensorUpdate = 0;
unsigned long lastCommandCheck = 0;
unsigned long lastModeCheck = 0;
unsigned long lastHeartbeat = 0;
const unsigned long sensorUpdateInterval = 5000;
const unsigned long commandCheckInterval = 100;
const unsigned long modeCheckInterval = 2000;
const unsigned long heartbeatInterval = 10000;

// ===== Error Tracking =====
int sslErrorCount = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 30000;

// ===== AUTONOMOUS NAVIGATION - PARAMETERS & STRUCTURES ===== //
#define SAFE_DISTANCE 60.0  // Increased from 30 to allow scanning before getting too close
#define CRITICAL_DISTANCE 25.0  // Increased from 15 for better emergency detection
#define MAX_DISTANCE 400.0
#define SCAN_RESOLUTION 5
#define NUM_READINGS 3
#define control 250
#define control_fwd 300

#define NUM_CONES 9
#define CONE_WIDTH 20

// Scoring weights
#define WEIGHT_CLEARANCE 0.40
#define WEIGHT_FORWARD 0.30
#define WEIGHT_CONTINUITY 0.20
#define WEIGHT_SAFETY 0.10

// Hysteresis parameters
#define MIN_TURN_ANGLE 15
#define CONFIDENCE_THRESHOLD 1.2
#define MOMENTUM_BIAS 0.1

// State Variables for Autonomous Mode
int lastDirection = 90;
float lastScore = 0;

struct ConeData {
  int startAngle;
  int endAngle;
  int centerAngle;
  float minDistance;
  float avgDistance;
  int measurementCount;
  int clearNeighbors;
  float weightedScore;
  String name;
  bool isValley;
};

ConeData cones[NUM_CONES] = {
  {0,   19,  10,  MAX_DISTANCE, 0, 0, 0, 0, "Hard-Right"},
  {20,  39,  30,  MAX_DISTANCE, 0, 0, 0, 0, "Med-Right"},
  {40,  59,  50,  MAX_DISTANCE, 0, 0, 0, 0, "Soft-Right"},
  {60,  79,  70,  MAX_DISTANCE, 0, 0, 0, 0, "Front-Right"},
  {80,  99,  90,  MAX_DISTANCE, 0, 0, 0, 0, "Front"},
  {100, 119, 110, MAX_DISTANCE, 0, 0, 0, 0, "Front-Left"},
  {120, 139, 130, MAX_DISTANCE, 0, 0, 0, 0, "Soft-Left"},
  {140, 159, 150, MAX_DISTANCE, 0, 0, 0, 0, "Med-Left"},
  {160, 180, 170, MAX_DISTANCE, 0, 0, 0, 0, "Hard-Left"}
};

struct PolarSector {
  int angle;
  float distance;
  float obstacleStrength;
};

#define NUM_SECTORS 37
PolarSector polarHistogram[NUM_SECTORS];

struct Valley {
  int startCone;
  int endCone;
  float avgClearance;
  int width;
};

#define MAX_VALLEYS 5
Valley valleys[MAX_VALLEYS];
int valleyCount = 0;

// ===== IMPROVED MPU6050 Functions with Better Error Handling =====

void setupMPU6050() {
  Wire.begin(21, 22);
  Wire.setClock(100000); // 100kHz for stability
  
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found");
    return;
  }
  Serial.println("✅ MPU6050 found");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(2000);
  calibrateMPU();
  
  // Initialize stuck detection
  lastKnownHeading = yaw;
  lastHeadingChangeTime = millis();
}

void calibrateMPU() {
  Serial.println("🧭 Calibrating MPU6050... Keep robot still!");
  float sumZ = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      sumZ += g.gyro.z;
      validReadings++;
    }
    delay(10);
  }
  
  if (validReadings > 0) {
    driftRate = sumZ / validReadings;
  } else {
    driftRate = 0.0;
  }
  
  yaw = 0.0;
  lastMPUUpdate = millis();
  Serial.println("🧭 MPU6050 calibrated!");
  Serial.println("   Drift rate: " + String(driftRate, 6) + " rad/s");
  Serial.println("   Valid readings: " + String(validReadings) + "/100");
}

void updateHeading() {
  unsigned long now = millis();
  float dt = (now - lastMPUUpdate) / 1000.0;
  
  if (dt < 0.05) return; // CHANGED: Update max every 50ms (reduced from 10ms)
  
  sensors_event_t a, g, t;
  int retries = 0;
  bool success = false;
  
  while (!success && retries < 3) {
    success = mpu.getEvent(&a, &g, &t);
    if (!success) {
      retries++;
      if (retries < 3) {
        delay(10); // Small delay before retry
      }
    }
  }
  
  if (!success) {
    // Don't print error every time - causes serial spam
    static unsigned long lastErrorPrint = 0;
    if (now - lastErrorPrint > 1000) {
      Serial.println("⚠️ MPU read failed, skipping update");
      lastErrorPrint = now;
    }
    return; // Skip this update, will catch up next cycle
  }
  
  // Update heading with drift compensation
  float gyroZ = g.gyro.z - driftRate;
  yaw += gyroZ * dt * (180.0 / PI);
  
  // Normalize yaw to 0-360 range
  while (yaw < 0) yaw += 360;
  while (yaw >= 360) yaw -= 360;
  
  lastMPUUpdate = now;
  
  // Check for stuck condition
  checkStuckCondition();
}

void forceUpdateHeading() {
  sensors_event_t a, g, t;
  int retries = 0;
  bool success = false;
  
  while (!success && retries < 3) {
    success = mpu.getEvent(&a, &g, &t);
    if (!success) {
      retries++;
      delay(20); // Longer delay for force update
    }
  }
  
  if (success) {
    unsigned long now = millis();
    float dt = (now - lastMPUUpdate) / 1000.0;
    if (dt > 0) {
      float gyroZ = g.gyro.z - driftRate;
      yaw += gyroZ * dt * (180.0 / PI);
      
      while (yaw < 0) yaw += 360;
      while (yaw >= 360) yaw -= 360;
      
      lastMPUUpdate = now;
    }
  }
}

// Check if robot is stuck (no heading change for 3 seconds)
void checkStuckCondition() {
  float headingDiff = abs(yaw - lastKnownHeading);
  
  // Handle wrap-around at 0/360 degrees
  if (headingDiff > 180) {
    headingDiff = 360 - headingDiff;
  }
  
  // If heading changed significantly, reset timer
  if (headingDiff > HEADING_CHANGE_THRESHOLD) {
    lastKnownHeading = yaw;
    lastHeadingChangeTime = millis();
    
    // If we were stuck, we're not anymore
    if (isStuck) {
      Serial.println("✅ Robot is moving again!");
      isStuck = false;
    }
  }
  
  // Check if stuck for too long
  unsigned long timeSinceChange = millis() - lastHeadingChangeTime;
  
  if (timeSinceChange > STUCK_TIMEOUT && !isStuck && currentMode == "auto") {
    isStuck = true;
    Serial.println("\n🚨 STUCK DETECTED! 🚨");
    Serial.println("   No heading change for " + String(timeSinceChange / 1000.0, 1) + " seconds");
    Serial.println("   Current heading: " + String(yaw, 1) + "°");
    Serial.println("   Last known heading: " + String(lastKnownHeading, 1) + "°");
    handleStuckCondition();
  }
}

// Handle stuck condition by triggering a scan
void handleStuckCondition() {
  Serial.println("🔍 Triggering emergency scan due to stuck condition...");
  
  // Stop motors first
  auto_stopMotors();
  delay(200);
  
  // Perform comprehensive scan
  performAdaptiveScan();
  calculateConeStatistics();
  detectValleys();
  calculateWeightedScores();
  
  int bestCone = selectBestDirection();
  printNavigationAnalysis();
  
  // Try to execute decision
  if (!executeNavigationDecision(bestCone)) {
    Serial.println("🚨 CRITICAL: No path found! Executing emergency escape...");
    
    // Emergency escape sequence
    auto_moveBackward(500);
    delay(200);
    auto_turnRight(900 - control); // 90-degree turn
    delay(200);
    
    // Re-scan after escape
    performAdaptiveScan();
    calculateConeStatistics();
    detectValleys();
    calculateWeightedScores();
    bestCone = selectBestDirection();
    executeNavigationDecision(bestCone);
  }
  
  // Reset stuck flag and timer
  isStuck = false;
  lastKnownHeading = yaw;
  lastHeadingChangeTime = millis();
}

float getCurrentHeading() {
  return yaw;
}

// =================================== //
// ===== INITIALIZATION & SETUP ===== //
// =================================== //

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  Serial.println("🤖 Integrated Firebase & Autonomous Robot Starting...");
  
  setupHardware();
  setupMPU6050();
  connectToWiFi();
  setupServo(); // Initialize servo AFTER MPU and WiFi
  setupFirebase();
  
  Serial.println("✅ Robot ready for web control!");
}

void setupHardware() {
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);
  pinMode(motorA_ENA, OUTPUT);
  pinMode(motorB_ENB, OUTPUT);
  
  // Set initial motor speed using PWM
  analogWrite(motorA_ENA, currentSpeed);
  analogWrite(motorB_ENB, currentSpeed);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("✅ Hardware initialized (motors & sensors)");
}

void setupServo() {
  Serial.println("\n🔧 ===== SERVO INITIALIZATION =====");
  Serial.println("Servo pin: " + String(servoPin));
  
  // Allocate ONLY timer 2 for servo to avoid conflicts
  ESP32PWM::allocateTimer(2);
  Serial.println("✅ PWM Timer 2 allocated");
  
  // Configure servo parameters
  myServo.setPeriodHertz(50);    // Standard 50Hz
  Serial.println("✅ Servo frequency set to 50Hz");
  
  // Attach servo with standard pulse widths
  int servoChannel = myServo.attach(servoPin, 500, 2400);
  
  if (servoChannel == -1) {
    Serial.println("❌❌❌ SERVO ATTACHMENT FAILED! ❌❌❌");
    Serial.println("Check: 1) Servo power supply, 2) Pin 32 connection, 3) Servo is functional");
    return;
  }
  
  Serial.println("✅ Servo attached successfully (channel: " + String(servoChannel) + ")");
  
  // Power-on test sequence
  Serial.println("\n🔄 Starting servo test sequence...");
  
  Serial.println("  1) Moving to 90° (center)...");
  myServo.write(90);
  delay(1000);
  
  Serial.println("  2) Moving to 0° (right)...");
  myServo.write(0);
  delay(1000);
  
  Serial.println("  3) Moving to 180° (left)...");
  myServo.write(180);
  delay(1000);
  
  Serial.println("  4) Returning to 90° (center)...");
  myServo.write(90);
  delay(1000);
  
  Serial.println("✅ Servo test complete!");
  Serial.println("Servo is attached: " + String(myServo.attached() ? "YES" : "NO"));
  Serial.println("================================\n");
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("🔗 Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("✅ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi connection failed!");
  }
}

void setupFirebase() {
  Serial.println("🔥 Setting up Firebase...");
  fbdo.setBSSLBufferSize(2048, 512);
  fbdo.setResponseSize(1024);
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("✅ Firebase authentication successful");
  } else {
    Serial.printf("❌ Firebase signup failed: %s\n", config.signer.signupError.message.c_str());
  }
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  if (Firebase.ready()) {
    firebaseReady = true;
    Serial.printf("📊 Free heap after Firebase init: %d bytes\n", ESP.getFreeHeap());
    Firebase.RTDB.setString(&fbdo, "/control/mode", "manual");
    Firebase.RTDB.setString(&fbdo, "/control/command", "stop");
    Firebase.RTDB.setInt(&fbdo, "/control/speed", currentSpeed); // Initialize speed
    Firebase.RTDB.setBool(&fbdo, "/status/connected", true);
    Serial.println("✅ Firebase connected successfully");
    Serial.println("🏎️ Initial speed: " + String(currentSpeed) + " PWM");
  } else {
    Serial.println("❌ Firebase failed to initialize.");
    firebaseReady = false;
  }
}

// =================================== //
// =====       MAIN LOOP          ===== //
// =================================== //

// In main loop() - skip MPU updates in manual mode
void loop() {
  unsigned long currentTime = millis();
  
  // CHANGED: Only update heading in auto mode
  if (currentMode == "auto") {
    updateHeading();
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi disconnected! Stopping motors...");
    auto_stopMotors();
    firebaseReady = false;
    delay(1000);
    return;
  }
  
  if (!firebaseReady && (currentTime - lastReconnectAttempt > reconnectInterval)) {
    Serial.println("🔄 Attempting Firebase reconnection...");
    if (Firebase.ready()) {
        firebaseReady = true;
        sslErrorCount = 0;
        Serial.println("✅ Firebase reconnected");
    }
    lastReconnectAttempt = currentTime;
  }
  
  if (!firebaseReady) {
    delay(100);
    return;
  }
  
  handleFirebaseBackgroundTasks(currentTime);
  
  if (currentMode == "auto") {
    runAutonomousMode();
  } else if (currentMode == "manual") {
    if (currentTime - lastCommandCheck >= commandCheckInterval) {
      checkForNewManualCommand();
      lastCommandCheck = currentTime;
    }
  }

  delay(50);
}
// =================================== //
// =====    FIREBASE HANDLERS     ===== //
// =================================== //

void handleFirebaseBackgroundTasks(unsigned long currentTime) {
  if (currentTime - lastSensorUpdate >= sensorUpdateInterval) {
    updateSensorDataToFirebase();
    lastSensorUpdate = currentTime;
  }
  if (currentTime - lastHeartbeat >= heartbeatInterval) {
    sendHeartbeatToFirebase();
    lastHeartbeat = currentTime;
  }
  if (currentTime - lastModeCheck >= modeCheckInterval) {
    checkModeChangesFromFirebase();
    lastModeCheck = currentTime;
  }
  if (currentTime - lastSpeedCheck >= speedCheckInterval) {
    checkSpeedFromFirebase();
    lastSpeedCheck = currentTime;
  }
}

void checkSpeedFromFirebase() {
  if (Firebase.RTDB.getInt(&fbdo, "/control/speed")) {
    int newSpeed = fbdo.to<int>();
    
    // Validate speed range
    if (newSpeed == 0) {
      // Speed 0 means stop - but we'll handle this in movement functions
      currentSpeed = 0;
    } else if (newSpeed >= MIN_PWM && newSpeed <= MAX_PWM) {
      if (newSpeed != currentSpeed) {
        currentSpeed = newSpeed;
        // Apply new speed immediately to motors
        analogWrite(motorA_ENA, currentSpeed);
        analogWrite(motorB_ENB, currentSpeed);
        Serial.println("🏎️ Speed updated to: " + String(currentSpeed) + " PWM");
      }
    }
  }
}

void applyMotorSpeed() {
  // Apply current speed to both motor enable pins
  if (currentSpeed == 0) {
    // If speed is 0, stop the motors
    analogWrite(motorA_ENA, 0);
    analogWrite(motorB_ENB, 0);
  } else {
    analogWrite(motorA_ENA, currentSpeed);
    analogWrite(motorB_ENB, currentSpeed);
  }
}

void checkModeChangesFromFirebase() {
  if (Firebase.RTDB.getString(&fbdo, "/control/mode")) {
    String newMode = fbdo.to<String>();
    newMode.toLowerCase();
    
    if (newMode != currentMode && (newMode == "manual" || newMode == "auto")) {
      Serial.println("🔄 Mode changed via Firebase to: " + newMode);
      currentMode = newMode;
      auto_stopMotors();
      Firebase.RTDB.setString(&fbdo, "/control/command", "stop");
      lastProcessedCommand = "stop";
    }
  }
}

void checkForNewManualCommand() {
  if (Firebase.RTDB.getString(&fbdo, "/control/command")) {
    String command = fbdo.to<String>();
    if (command != lastProcessedCommand) {
      executeManualCommand(command);
      lastProcessedCommand = command;
    }
  }
}

void updateSensorDataToFirebase() {
  myServo.write(90);
  delay(150);
  float distance = getFilteredDistance();
  
  if (distance <= 2.0) {
    distance = 0.0;
    Serial.println("📊 Front Distance: TOO CLOSE (showing 0cm) sent to Firebase");
  } else {
    Serial.println("📊 Front Distance: " + String(distance, 1) + "cm sent to Firebase");
  }
  
  if (Firebase.RTDB.setFloat(&fbdo, "/sensors/distance_cm", distance)) {
    // Success
  } else {
    Serial.println("⚠️ Failed to send sensor data to Firebase.");
  }
}

void sendHeartbeatToFirebase() {
  if (Firebase.RTDB.setBool(&fbdo, "/status/connected", true)) {
    Firebase.RTDB.setString(&fbdo, "/status/current_mode", currentMode);
    Serial.println("💓 Heartbeat sent (mode: " + currentMode + ")");
  }
}

// =================================== //
// =====   MANUAL MODE LOGIC      ===== //
// =================================== //

void executeManualCommand(String command) {
  if (currentMode != "manual") return;
  
  Serial.println("📱 Manual command received: " + command);
  
  if (command == "forward") moveForward();
  else if (command == "backward") moveBackward();
  else if (command == "left") turnLeft();
  else if (command == "right") turnRight();
  else if (command == "stop") stopMotors();
  else Serial.println("❓ Unknown manual command: " + command);
}

void moveForward() {
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, HIGH); digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, LOW); digitalWrite(motorB_IN4, HIGH);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "manual_forward");
  Serial.println("⬆️ Forward at speed: " + String(currentSpeed));
}
void moveBackward() {
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, HIGH);
  digitalWrite(motorB_IN3, HIGH); digitalWrite(motorB_IN4, LOW);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "manual_backward");
  Serial.println("⬇️ Backward at speed: " + String(currentSpeed));
}
void turnLeft() {
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, HIGH); digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, HIGH); digitalWrite(motorB_IN4, LOW);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "manual_left");
  Serial.println("⬅️ Left at speed: " + String(currentSpeed));
}
void turnRight() {
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, HIGH);
  digitalWrite(motorB_IN3, LOW); digitalWrite(motorB_IN4, HIGH);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "manual_right");
  Serial.println("➡️ Right at speed: " + String(currentSpeed));
}
void stopMotors() {
  analogWrite(motorA_ENA, 0); // Disable motors completely
  analogWrite(motorB_ENB, 0);
  digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, LOW); digitalWrite(motorB_IN4, LOW);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "manual_stop");
  Serial.println("🛑 Motors stopped");
}

// =================================== //
// =====   AUTONOMOUS MODE LOGIC  ===== //
// =================================== //

void runAutonomousMode() {
  Serial.println("\n--- Running Autonomous Cycle ---");
  
  // Verify servo is working
  if (!myServo.attached()) {
    Serial.println("❌ ERROR: Servo not attached! Stopping autonomous mode.");
    auto_stopMotors();
    return;
  }
  
  myServo.write(90);
  delay(200); // Increased delay
  float frontDistance = getFilteredDistance();
  
  Serial.println("📏 Front distance: " + String(frontDistance) + "cm | Safe threshold: " + String(SAFE_DISTANCE) + "cm");
  
  // Emergency backup for extremely close obstacles
  if (frontDistance <= 2.0) {
    Serial.println("🚨 EXTREMELY CLOSE OBSTACLE DETECTED! Emergency backup!");
    Serial.println("Front distance: " + String(frontDistance) + "cm - Moving backward immediately");
    auto_moveBackward(300);
    delay(200);
    
    frontDistance = getFilteredDistance();
    Serial.println("After backup, front distance: " + String(frontDistance) + "cm");
    
    if (frontDistance <= CRITICAL_DISTANCE) {
      Serial.println("Still too close - additional backup and turn");
      auto_moveBackward(200);
      auto_turnRight(600);
    }
    return;
  }
  
  // Path is very clear - move forward confidently
  if (frontDistance > SAFE_DISTANCE * 2) {
    Serial.println("AUTO: Path very clear (" + String(frontDistance) + "cm). Moving forward.");
    auto_moveForward(300);
    lastDirection = 90;
    lastScore = 100;
  } 
  // Path is somewhat clear - move forward cautiously
  else if (frontDistance > SAFE_DISTANCE * 1.2) {
    Serial.println("AUTO: Path clear (" + String(frontDistance) + "cm). Cautious forward.");
    auto_moveForward(200);
    lastDirection = 90;
    lastScore = 70;
  } 
  // Obstacle detected - MUST SCAN
  else {
    Serial.println("\n╔═══════════════════════════════════════════════════════╗");
    Serial.println("║     OBSTACLE DETECTED - INITIATING SCAN               ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝");
    Serial.println("Front distance: " + String(frontDistance) + "cm (below " + String(SAFE_DISTANCE) + "cm threshold)");
    Serial.println("Servo attached: " + String(myServo.attached() ? "YES" : "NO"));
    
    auto_stopMotors();
    delay(300);
    
    performAdaptiveScan();
    calculateConeStatistics();
    detectValleys();
    calculateWeightedScores();
    
    int bestCone = selectBestDirection();
    printNavigationAnalysis();
    
    if (!executeNavigationDecision(bestCone)) {
      Serial.println("🚨 EMERGENCY: All paths blocked! Executing escape maneuver.");
      auto_moveBackward(400);
      auto_turnRight(1000);
    }
  }
}

// IMPROVED: Movement functions with reduced MPU polling

void auto_moveForward(int duration) {
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  delay(100);
  float startHeading = getCurrentHeading();
  
  Serial.println("🔼 AUTO: FORWARD for " + String(duration) + "ms at speed " + String(currentSpeed));
  Serial.println("🧭 Starting heading: " + String(startHeading, 1) + "°");
  
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, HIGH); digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, LOW); digitalWrite(motorB_IN4, HIGH);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "auto_forward");
  
  // CHANGED: Update heading every 50ms instead of 10ms
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    updateHeading();
    delay(100);
  }
  
  auto_stopMotors();
  delay(100);
  
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  float endHeading = getCurrentHeading();
  
  float headingDrift = endHeading - startHeading;
  if (headingDrift < -180) headingDrift += 360;
  if (headingDrift > 180) headingDrift -= 360;
  
  Serial.println("🧭 Forward complete - Heading drift: " + String(headingDrift, 1) + "° (now at " + String(endHeading, 1) + "°)");
}

void auto_moveBackward(int duration) {
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  delay(100);
  float startHeading = getCurrentHeading();
  
  Serial.println("🔽 AUTO: BACKWARD for " + String(duration) + "ms at speed " + String(currentSpeed));
  Serial.println("🧭 Starting heading: " + String(startHeading, 1) + "°");
  
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, HIGH);
  digitalWrite(motorB_IN3, HIGH); digitalWrite(motorB_IN4, LOW);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "auto_backward");
  
  // CHANGED: Update heading every 50ms instead of 10ms
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    updateHeading();
    delay(100);
  }
  
  auto_stopMotors();
  delay(100);
  
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  float endHeading = getCurrentHeading();
  
  float headingDrift = endHeading - startHeading;
  if (headingDrift < -180) headingDrift += 360;
  if (headingDrift > 180) headingDrift -= 360;
  
  Serial.println("🧭 Backward complete - Heading drift: " + String(headingDrift, 1) + "° (now at " + String(endHeading, 1) + "°)");
}

void auto_turnLeft(int duration) {
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  delay(100);
  float startHeading = getCurrentHeading();
  
  Serial.println("◀️ AUTO: LEFT turn for " + String(duration) + "ms at speed " + String(currentSpeed));
  Serial.println("🧭 Starting heading: " + String(startHeading, 1) + "°");
  
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, HIGH); digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, HIGH); digitalWrite(motorB_IN4, LOW);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "auto_left");
  
  // CHANGED: Update heading every 50ms instead of 10ms
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    updateHeading();
    delay(100);
  }
  
  auto_stopMotors();
  delay(150); // CHANGED: Longer settle time
  
  forceUpdateHeading();
  delay(100);
  forceUpdateHeading();
  float endHeading = getCurrentHeading();
  
  float turnAngle = endHeading - startHeading;
  if (turnAngle < -180) turnAngle += 360;
  if (turnAngle > 180) turnAngle -= 360;
  
  Serial.println("🧭 LEFT turn complete: " + String(turnAngle, 1) + "° turned (now at " + String(endHeading, 1) + "°)");
  
  lastKnownHeading = endHeading;
  lastHeadingChangeTime = millis();
}

void auto_turnRight(int duration) {
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  delay(100);
  float startHeading = getCurrentHeading();
  
  Serial.println("▶️ AUTO: RIGHT turn for " + String(duration) + "ms at speed " + String(currentSpeed));
  Serial.println("🧭 Starting heading: " + String(startHeading, 1) + "°");
  
  applyMotorSpeed(); // Apply current speed from website
  digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, HIGH);
  digitalWrite(motorB_IN3, LOW); digitalWrite(motorB_IN4, HIGH);
  Firebase.RTDB.setString(&fbdo, "/status/last_action", "auto_right");
  
  // CHANGED: Update heading every 50ms instead of 10ms
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    updateHeading();
    delay(100);
  }
  
  auto_stopMotors();
  delay(150); // CHANGED: Longer settle time
  
  forceUpdateHeading();
  delay(200);
  forceUpdateHeading();
  float endHeading = getCurrentHeading();
  
  float turnAngle = endHeading - startHeading;
  if (turnAngle < -180) turnAngle += 360;
  if (turnAngle > 180) turnAngle -= 360;
  
  Serial.println("🧭 RIGHT turn complete: " + String(turnAngle, 1) + "° turned (now at " + String(endHeading, 1) + "°)");
  
  lastKnownHeading = endHeading;
  lastHeadingChangeTime = millis();
}

void auto_stopMotors() {
  analogWrite(motorA_ENA, 0); // Disable motors completely
  analogWrite(motorB_ENB, 0);
  digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, LOW); digitalWrite(motorB_IN4, LOW);
}

// =================================== //
// =====   SENSOR & NAVIGATION    ===== //
// =================================== //

float getFilteredDistance() {
  float readings[NUM_READINGS];
  int invalidReadings = 0;
  
  for (int i = 0; i < NUM_READINGS; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    float duration = pulseIn(echoPin, HIGH, 30000);
    readings[i] = (duration * 0.0343) / 2;
    
    if (readings[i] == 0 || readings[i] > MAX_DISTANCE) {
      readings[i] = MAX_DISTANCE;
      invalidReadings++;
    }
    delay(5);
  }
  
  if (invalidReadings >= NUM_READINGS - 1) {
    Serial.println("⚠️ Sensor readings invalid - likely TOO CLOSE to obstacle!");
    return 1.0;
  }
  
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = i + 1; j < NUM_READINGS; j++) {
      if (readings[i] > readings[j]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  return readings[NUM_READINGS / 2];
}

void performAdaptiveScan() {
  Serial.println("\n🔍 ========== STARTING ADAPTIVE SCAN ==========");
  
  // Critical: Verify servo is attached before scanning
  if (!myServo.attached()) {
    Serial.println("❌❌❌ CRITICAL ERROR: Servo not attached! ❌❌❌");
    Serial.println("Attempting to re-initialize servo...");
    setupServo();
    
    if (!myServo.attached()) {
      Serial.println("❌ SERVO RE-INITIALIZATION FAILED!");
      Serial.println("ABORTING SCAN - Check servo hardware!");
      return;
    }
  }
  
  Serial.println("✅ Servo status: Attached");
  Serial.println("Scan resolution: " + String(SCAN_RESOLUTION) + "° steps");
  Serial.println("Scanning from 0° to 180°...");
  
  for (int i = 0; i < NUM_CONES; i++) {
    cones[i].minDistance = MAX_DISTANCE;
    cones[i].avgDistance = 0;
    cones[i].measurementCount = 0;
    cones[i].isValley = false;
  }
  
  // Start from 90° center position
  Serial.println("\n➡️ Moving to start position (90°)...");
  myServo.write(90);
  delay(800); // Longer delay to ensure servo reaches position
  
  int measurementCount = 0;
  Serial.println("");
  for (int angle = 0; angle <= 180; angle += SCAN_RESOLUTION) {
    Serial.print("📐 Servo -> " + String(angle) + "° ");
    myServo.write(angle);
    delay(150); // Increased delay for servo to physically move and stabilize
    
    float distance = getFilteredDistance();
    measurementCount++;
    
    Serial.println(" Distance: " + String(distance, 1) + "cm");
    
    // Print detailed output every 45 degrees
    if (angle % 45 == 0) {
      Serial.println("  ✓ Checkpoint at " + String(angle) + "°: " + String(distance, 1) + "cm");
    }
    
    int coneIdx = angle / CONE_WIDTH;
    if (coneIdx >= NUM_CONES) coneIdx = NUM_CONES - 1;
    
    if (distance < cones[coneIdx].minDistance) {
      cones[coneIdx].minDistance = distance;
    }
    cones[coneIdx].avgDistance += distance;
    cones[coneIdx].measurementCount++;
  }
  
  Serial.println("\n📊 Scan complete! Total measurements: " + String(measurementCount));
  Serial.println("Returning servo to center (90°)...");
  myServo.write(90);
  delay(500);
  Serial.println("🔍 ========== SCAN FINISHED ==========\n");
}

void calculateConeStatistics() {
  for (int i = 0; i < NUM_CONES; i++) {
    if (cones[i].measurementCount > 0) {
      cones[i].avgDistance /= cones[i].measurementCount;
    }
  }
}

void detectValleys() {
  valleyCount = 0;
  bool inValley = false;
  int valleyStart = -1;
  for (int i = 0; i < NUM_CONES; i++) {
    if (cones[i].minDistance > SAFE_DISTANCE) {
      if (!inValley) {
        inValley = true;
        valleyStart = i;
      }
      cones[i].isValley = true;
    } else {
      if (inValley) {
        if (valleyCount < MAX_VALLEYS) {
          valleys[valleyCount].startCone = valleyStart;
          valleys[valleyCount].endCone = i - 1;
          valleys[valleyCount].width = i - valleyStart;
          valleyCount++;
        }
        inValley = false;
      }
    }
  }
  if (inValley && valleyCount < MAX_VALLEYS) {
    valleys[valleyCount].startCone = valleyStart;
    valleys[valleyCount].endCone = NUM_CONES - 1;
    valleys[valleyCount].width = NUM_CONES - valleyStart;
    valleyCount++;
  }
}

void calculateWeightedScores() {
  for (int i = 0; i < NUM_CONES; i++) {
    if (cones[i].minDistance < SAFE_DISTANCE) {
        cones[i].weightedScore = 0;
        continue;
    }
    
    float score = 0;
    
    float clearanceScore = 100 * ((cones[i].minDistance - SAFE_DISTANCE) / (MAX_DISTANCE - SAFE_DISTANCE));
    if (clearanceScore > 100) clearanceScore = 100;
    score += clearanceScore * WEIGHT_CLEARANCE;

    score += (100 * (1.0 - abs(90.0 - cones[i].centerAngle) / 90.0)) * WEIGHT_FORWARD;

    float continuityScore = 0;
    if (cones[i].isValley) {
      for (int v = 0; v < valleyCount; v++) {
        if (i >= valleys[v].startCone && i <= valleys[v].endCone) {
          continuityScore = valleys[v].width * 15;
          if (continuityScore > 100) continuityScore = 100;
          break;
        }
      }
    }
    score += continuityScore * WEIGHT_CONTINUITY;

    float safetyScore = 40;
    if(cones[i].minDistance > SAFE_DISTANCE) safetyScore = 70;
    if(cones[i].minDistance > SAFE_DISTANCE*2) safetyScore = 90;
    score += safetyScore * WEIGHT_SAFETY;

    if (abs(cones[i].centerAngle - lastDirection) < 30) {
      score *= (1.0 + MOMENTUM_BIAS);
    }
    
    cones[i].weightedScore = score;
  }
}

int selectBestDirection() {
  int bestCone = -1;
  float bestScore = -1.0;
  for (int i = 0; i < NUM_CONES; i++) {
    if (cones[i].minDistance > SAFE_DISTANCE && cones[i].weightedScore > bestScore) {
      bestScore = cones[i].weightedScore;
      bestCone = i;
    }
  }

  if (bestCone == -1) {
    float leastDangerousDist = -1.0;
    for (int i = 0; i < NUM_CONES; i++) {
      if (cones[i].minDistance > leastDangerousDist) {
        leastDangerousDist = cones[i].minDistance;
        bestCone = i;
      }
    }
  }
  return bestCone;
}

bool executeNavigationDecision(int targetCone) {
  if (targetCone < 0 || targetCone >= NUM_CONES) return false;
  if (cones[targetCone].minDistance < CRITICAL_DISTANCE) return false;

  Serial.println("\n=== EXECUTING AUTO DECISION ===");
  Serial.print("Target: " + cones[targetCone].name + ", Score: " + String(cones[targetCone].weightedScore, 1));
  Serial.println(", Dist: " + String(cones[targetCone].minDistance, 1) + "cm");

  int targetAngle = cones[targetCone].centerAngle;
  int angleDiff = targetAngle - 90;
  lastDirection = targetAngle;
  lastScore = cones[targetCone].weightedScore;

  if (abs(angleDiff) < 15) { 
    auto_moveForward(500 - control); 
  } else if (angleDiff < -60) { 
    auto_turnRight(800 - control); auto_moveForward(400 - control_fwd); 
  } else if (angleDiff < -30) { 
    auto_turnRight(500 - control); auto_moveForward(400 - control_fwd); 
  } else if (angleDiff < 0) { 
    auto_turnRight(300 - control); auto_moveForward(400 - control_fwd); 
  } else if (angleDiff > 60) { 
    auto_turnLeft(800 - control); auto_moveForward(400 - control_fwd); 
  } else if (angleDiff > 30) { 
    auto_turnLeft(500 - control); auto_moveForward(400 - control_fwd); 
  } else { 
    auto_turnLeft(300 - control); auto_moveForward(400 - control_fwd); 
  }
  
  return true;
}

void printNavigationAnalysis() {
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║          NAVIGATION ANALYSIS REPORT                   ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println("🧭 Current Heading: " + String(yaw, 1) + "°");
  Serial.println("⏱️ Time since last heading change: " + String((millis() - lastHeadingChangeTime) / 1000.0, 1) + "s");
  Serial.println("");
  
  for (int i = 0; i < NUM_CONES; i++) {
    Serial.print(String(i) + ". " + cones[i].name + ": ");
    Serial.print("Min=" + String(cones[i].minDistance, 1) + "cm");
    Serial.print(", Score=" + String(cones[i].weightedScore, 1));
    
    if (cones[i].minDistance < SAFE_DISTANCE) Serial.print(" ❌[BLOCKED]");
    if (cones[i].isValley) Serial.print(" ✅[VALLEY]");
    Serial.println();
  }
  
  Serial.println("\n📊 Valleys detected: " + String(valleyCount));
  for (int i = 0; i < valleyCount; i++) {
    Serial.println("   Valley " + String(i+1) + ": Cones " + String(valleys[i].startCone) + 
                   " to " + String(valleys[i].endCone) + " (width: " + String(valleys[i].width) + ")");
  }
  Serial.println("════════════════════════════════════════════════════════\n");
}