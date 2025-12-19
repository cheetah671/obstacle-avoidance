#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// ===== HARDWARE PINS =====
#define motorA_IN1 2
#define motorA_IN2 4
#define motorB_IN3 12
#define motorB_IN4 14
#define motorA_ENA 16
#define motorB_ENB 27

// Encoder pins
#define ENCODER_LEFT_PIN 25
#define ENCODER_RIGHT_PIN 26

// Ultrasonic + Servo
#define TRIG_PIN 5
#define ECHO_PIN 34
#define SERVO_PIN 32

// ===== CALIBRATION - FROM EMPIRICAL TESTING =====
#define ROBOT_SPEED_MPS 0.318 

// Encoder calibration - recalculated from actual measurements
#define PULSES_PER_METER 670.0  // Recalibrated from average of 3 tests
#define MOTOR_BALANCE_FACTOR 0.21  // Left motor 21% stronger

// Using encoder only for position tracking
#define ENCODER_WEIGHT 1.00
#define VELOCITY_WEIGHT 0.00

// ===== NAVIGATION PARAMETERS =====
const float MAX_SPEED = 0.318;       // m/s - actual robot speed
const float MIN_SPEED = 0.12;        // m/s - minimum controllable
const float GOAL_TOLERANCE = 0.10;   // m - stop within 10cm (reduced for accuracy)
const int BASE_PWM = 200;            // PWM for MAX_SPEED
const int MIN_PWM = 180;             // Minimum to overcome friction

// ===== OBSTACLE AVOIDANCE =====
#define OBSTACLE_THRESHOLD 0.30      // Stop if obstacle < 40cm ahead
#define SIDESTEP_DISTANCE 0.12       // Move 25cm sideways (enough to clear obstacle)
#define SIDESTEP_ANGLE 45            // 45 degree sidestep
#define MAX_SIDESTEP_ATTEMPTS 3      // Try 3 times before giving up

// ===== GLOBALS =====
Adafruit_MPU6050 mpu;
Servo scanServo;

// State machine for obstacle avoidance
enum State {
  GOING_TO_GOAL,
  CHECKING_SIDES,
  SIDESTEPPING,
  CHECKING_PATH
};
State current_state = GOING_TO_GOAL;
int sidestep_attempts = 0;
float sidestep_start_x = 0;
float sidestep_start_y = 0;
int chosen_side = 0;  // -1=left, +1=right

// Odometry - sensor fusion
float robot_x = 0.0;
float robot_y = 0.0;
float robot_yaw = 0.0;  // radians
float gyro_drift = 0.0;

// Encoder tracking
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
long last_encoder_left = 0;
long last_encoder_right = 0;

// Distance tracking
float total_distance_encoder = 0.0;

float goal_x = 1.0;
float goal_y = 0.0;
bool goal_active = false;

unsigned long last_update = 0;

// ISR forward declarations
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();

// ===== ENCODER ISRs =====
void IRAM_ATTR leftEncoderISR() {
  encoder_left_count++;
}

void IRAM_ATTR rightEncoderISR() {
  encoder_right_count++;
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("  SIMPLE PATH PLANNING - GYRO BASED");
  Serial.println("========================================\n");
  
  // I2C and MPU6050
  Wire.begin(21, 22);
  Wire.setClock(50000);  // Reduce to 50kHz for stability
  
  // Suppress I2C error messages
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 FAILED!");
    while (1) delay(10);
  }
  Serial.println("✅ MPU6050 initialized");
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Calibrate gyro
  Serial.print("⏳ Calibrating gyro (stay still)... ");
  delay(1000);
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(10);
  }
  gyro_drift = sum / 100.0;
  Serial.println("Done");
  
  // Encoder pins
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightEncoderISR, RISING);
  Serial.println("✅ Encoders initialized");
  
  // Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("✅ Ultrasonic initialized");
  
  // Motor pins - setup BEFORE servo to avoid LEDC channel conflicts
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);
  
  // Motor PWM on channels 0 and 1
  ledcAttach(motorA_ENA, 5000, 8);
  ledcAttach(motorB_ENB, 5000, 8);
  
  stopMotors();
  
  // Servo for scanning - setup AFTER motors to avoid conflicts
  // ESP32Servo uses different timer, but attach after ledcAttach to be safe
  ESP32PWM::allocateTimer(2);  // Use timer 2 for servo (motors use 0,1)
  scanServo.setPeriodHertz(50);  // Standard 50Hz servo
  scanServo.attach(SERVO_PIN, 500, 2400);  // min/max pulse width in microseconds
  scanServo.write(90);  // Center
  delay(500);
  Serial.println("✅ Servo initialized");
  
  Serial.println("\n✅ Robot ready!");
  Serial.println("\nCommands:");
  Serial.println("  g X Y  - Set goal (e.g., 'g 1.0 0')");
  Serial.println("  s      - Start navigation");
  Serial.println("  r      - Reset position");
  Serial.println("  p      - Stop\n");
  
  last_update = millis();
}

// ===== MAIN LOOP =====
void loop() {
  handleCommands();
  
  if (goal_active) {
    updateOdometry();
    navigate();
  } else {
    stopMotors();
  }
  
  delay(100);  // Increased to 100ms to reduce I2C load
}

// ===== COMMAND HANDLER =====
void handleCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("g ")) {
      int space = cmd.indexOf(' ', 2);
      goal_x = cmd.substring(2, space).toFloat();
      goal_y = cmd.substring(space + 1).toFloat();
      Serial.print("✓ Goal: (");
      Serial.print(goal_x, 2);
      Serial.print(", ");
      Serial.print(goal_y, 2);
      Serial.println(") - Type 's' to start");
    }
    else if (cmd == "s") {
      goal_active = true;
      current_state = GOING_TO_GOAL;
      sidestep_attempts = 0;
      last_update = millis();
      Serial.println("▶ Starting navigation...\n");
    }
    else if (cmd == "r") {
      robot_x = 0.0;
      robot_y = 0.0;
      robot_yaw = 0.0;
      encoder_left_count = 0;
      encoder_right_count = 0;
      last_encoder_left = 0;
      last_encoder_right = 0;
      total_distance_encoder = 0.0;
      goal_active = false;
      current_state = GOING_TO_GOAL;
      sidestep_attempts = 0;
      stopMotors();
      last_update = millis();
      Serial.println("⟲ Position reset\n");
    }
    else if (cmd == "p") {
      goal_active = false;
      stopMotors();
      Serial.println("■ Stopped\n");
    }
  }
}

// ===== ULTRASONIC DISTANCE SENSOR =====
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return 5.0;  // No echo = far away
  
  float distance = duration * 0.034 / 2.0 / 100.0;  // Convert to meters
  if (distance < 0.05) return 5.0;  // Invalid reading
  
  return constrain(distance, 0.05, 5.0);
}

// ===== SCAN SPECIFIC ANGLE =====
float scanAngle(int angle) {
  scanServo.write(angle);
  delay(200);  // Wait for servo to settle
  return getDistance();
}

// ===== ODOMETRY UPDATE (ENCODER + MPU HEADING) =====
void updateOdometry() {
  static unsigned long last_mpu_read = 0;
  static unsigned long last_print = 0;
  unsigned long now = millis();
  float dt = (now - last_update) / 1000.0;
  
  if (dt <= 0 || dt > 0.5) {
    last_update = now;
    return;
  }
  
  // Calculate distance from encoders
  long delta_left = encoder_left_count - last_encoder_left;
  long delta_right = encoder_right_count - last_encoder_right;
  last_encoder_left = encoder_left_count;
  last_encoder_right = encoder_right_count;
  
  float distance_left = delta_left / PULSES_PER_METER;
  float distance_right = delta_right / PULSES_PER_METER;
  
  // For differential drive: average of both wheels = center point distance traveled
  // This is correct even when turning - it measures arc length at center
  float distance_encoder = (distance_left + distance_right) / 2.0;
  
  // Track totals for comparison
  total_distance_encoder += abs(distance_encoder);  // Total path length
  
  // Read MPU6050 for heading - CRITICAL for accurate position
  sensors_event_t a, g, temp;
  
  if (now - last_mpu_read >= 100) {  // Read gyro more frequently (50ms)
    bool success = mpu.getEvent(&a, &g, &temp);
    
    if (success) {
      last_mpu_read = now;
      float gyro_z = g.gyro.z - gyro_drift;
      
      // Update heading from gyroscope
      robot_yaw += gyro_z * dt;
      while (robot_yaw > PI) robot_yaw -= 2 * PI;
      while (robot_yaw < -PI) robot_yaw += 2 * PI;
    }
  }
  
  // Update position using encoder distance and MPU heading
  // This correctly tracks position even when curving
  robot_x += distance_encoder * cos(robot_yaw);
  robot_y += distance_encoder * sin(robot_yaw);
  
  // Calculate target info
  float dx_target = goal_x - robot_x;
  float dy_target = goal_y - robot_y;
  float dist_to_target = sqrt(dx_target*dx_target + dy_target*dy_target);
  float angle_to_target = atan2(dy_target, dx_target) * 57.3;  // degrees
  float heading_deg = robot_yaw * 57.3;
  
  // Print status every 500ms with TARGET INFO
  if (now - last_print >= 500) {
    last_print = now;
    
    Serial.println("----------------------------------------");
    Serial.print("Heading: ");
    Serial.print(heading_deg, 1);
    Serial.print(" deg | Encoders L=");
    Serial.print(encoder_left_count);
    Serial.print(" R=");
    Serial.println(encoder_right_count);
    
    Serial.print("Position: (");
    Serial.print(robot_x, 2);
    Serial.print(", ");
    Serial.print(robot_y, 2);
    Serial.print(") m | Path traveled: ");
    Serial.print(total_distance_encoder * 100, 1);
    Serial.println(" cm");
    
    Serial.print("TARGET: (");
    Serial.print(goal_x, 2);
    Serial.print(", ");
    Serial.print(goal_y, 2);
    Serial.print(") | DISTANCE: ");
    Serial.print(dist_to_target * 100, 1);
    Serial.print(" cm | Direction: ");
    Serial.print(angle_to_target, 1);
    Serial.println(" deg");
    Serial.println("----------------------------------------");
  }
  
  last_update = now;
}

// ===== HELPER: NAVIGATE TO GOAL (no obstacle check) =====
void navigateToGoal() {
  float dx = goal_x - robot_x;
  float dy = goal_y - robot_y;
  float dist = sqrt(dx * dx + dy * dy);
  float desired_angle = atan2(dy, dx);
  float heading_error = desired_angle - robot_yaw;
  
  while (heading_error > PI) heading_error -= 2 * PI;
  while (heading_error < -PI) heading_error += 2 * PI;
  
  int left_pwm, right_pwm;
  
  if (abs(heading_error) > 0.5) {  // ~30 degrees - turn in place
    if (heading_error > 0) {
      left_pwm = -180;
      right_pwm = 180;
    } else {
      left_pwm = 180;
      right_pwm = -180;
    }
  } else {
    int base = (dist < 0.5) ? MIN_PWM : BASE_PWM;
    int turn = constrain((int)(heading_error * 80), -30, 30);
    left_pwm = base - turn;
    right_pwm = base + turn;
  }
  
  setMotors(left_pwm, right_pwm);
}

// ===== NAVIGATION WITH OBSTACLE AVOIDANCE =====
void navigate() {
  static unsigned long last_print = 0;
  
  // Calculate distance to goal
  float dx = goal_x - robot_x;
  float dy = goal_y - robot_y;
  float dist = sqrt(dx * dx + dy * dy);
  
  // Check if goal reached
  if (dist < GOAL_TOLERANCE) {
    stopMotors();
    goal_active = false;
    Serial.println("\n========================================");
    Serial.println("GOAL REACHED!");
    Serial.println("========================================");
    Serial.print("Final Position: (");
    Serial.print(robot_x, 2);
    Serial.print(", ");
    Serial.print(robot_y, 2);
    Serial.print(")m | Error: ");
    Serial.print(dist * 100, 1);
    Serial.println("cm");
    Serial.println("========================================\n");
    return;
  }
  
  // State machine for obstacle avoidance
  switch (current_state) {
    
    // ========================================
    // STATE 1: GO STRAIGHT TO GOAL
    // ========================================
    case GOING_TO_GOAL: {
      // Check front obstacle every 500ms
      static unsigned long last_check = 0;
      if (millis() - last_check > 500) {
        scanServo.write(90);  // Ensure servo is centered
        delay(150);
        float front_dist = getDistance();
        last_check = millis();
        
        Serial.print("[FRONT] ");
        Serial.print(front_dist * 100, 1);
        Serial.print("cm (threshold=");
        Serial.print(OBSTACLE_THRESHOLD * 100, 1);
        Serial.println("cm)");
        
        if (front_dist < OBSTACLE_THRESHOLD) {
          Serial.println("*** OBSTACLE! Stopping to scan sides ***");
          stopMotors();
          delay(500);  // Longer pause before scanning
          current_state = CHECKING_SIDES;
          return;
        }
      }
      
      // No obstacle - drive toward goal
      navigateToGoal();
      break;
    }
    
    // ========================================
    // STATE 2: SCAN LEFT & RIGHT (60° each way)
    // ========================================
    case CHECKING_SIDES: {
      Serial.println("\n>>> SCANNING SIDES <<<");
      stopMotors();
      delay(500);  // Longer pause before scanning
      
      // Start from center (90°)
      scanServo.write(90);
      delay(600);
      
      // Scan RIGHT: 90° - 60° = 30°
      Serial.println("Scanning RIGHT (30°)...");
      scanServo.write(30);
      delay(800);  // Longer delay for servo to settle
      float right_dist = 0;
      for (int i = 0; i < 3; i++) {
        right_dist += getDistance();
        delay(100);
      }
      right_dist /= 3.0;
      Serial.print("  RIGHT: ");
      Serial.print(right_dist * 100, 1);
      Serial.println(" cm");
      delay(1000);  // Pause after reading
      
      // Scan LEFT: 90° + 60° = 150°
      scanServo.write(150);
      Serial.println("Scanning LEFT (150°)...");
      // scanServo.write(150);
      delay(1000);  // Longer delay for 120° sweep
      float left_dist = 0;
      for (int i = 0; i < 3; i++) {
        left_dist += getDistance();
        delay(100);
      }
      left_dist /= 3.0;
      Serial.print("  LEFT: ");
      Serial.print(left_dist * 100, 1);
      Serial.println(" cm");
      delay(300);  // Pause after reading
      
      // Return to center
      scanServo.write(90);
      delay(500);
      
      Serial.print("RESULT -> Right: ");
      Serial.print(right_dist * 100, 1);
      Serial.print("cm | Left: ");
      Serial.print(left_dist * 100, 1);
      Serial.println("cm");
      
      // Choose safer side (larger distance)
      if (right_dist >= left_dist && right_dist > 0.25) {
        chosen_side = 1;  // Sidestep RIGHT
        Serial.println("*** DECISION: Sidestep RIGHT ***");
      } else if (left_dist > 0.25) {
        chosen_side = -1;  // Sidestep LEFT
        Serial.println("*** DECISION: Sidestep LEFT ***");
      } else {
        // Both blocked - back up
        Serial.println("*** BOTH BLOCKED! Backing up ***");
        setMotors(-BASE_PWM, -BASE_PWM);
        delay(500);
        stopMotors();
        current_state = GOING_TO_GOAL;
        return;
      }
      
      sidestep_start_x = robot_x;
      sidestep_start_y = robot_y;
      current_state = SIDESTEPPING;
      break;
    }
    
    // ========================================
    // STATE 3: SIDESTEP DIAGONALLY
    // ========================================
    case SIDESTEPPING: {
      float dx_step = robot_x - sidestep_start_x;
      float dy_step = robot_y - sidestep_start_y;
      float dist_moved = sqrt(dx_step*dx_step + dy_step*dy_step);
      
      if (dist_moved >= SIDESTEP_DISTANCE) {
        Serial.print("Sidestepped ");
        Serial.print(dist_moved * 100, 1);
        Serial.println("cm - checking opposite side");
        
        stopMotors();
        delay(600);  // Longer pause after sidestep
        sidestep_attempts++;
        current_state = CHECKING_PATH;
      } else {
        // Move diagonally: curve in chosen direction
        // To curve LEFT: Right motor slower, Left motor faster
        // To curve RIGHT: Left motor slower, Right motor faster
        if (chosen_side < 0) {
          // Curve LEFT: slow RIGHT motor
          setMotors(BASE_PWM, MIN_PWM - 40);
        } else {
          // Curve RIGHT: slow LEFT motor
          setMotors(MIN_PWM - 40, BASE_PWM);
        }
      }
      break;
    }
    
    // ========================================
    // STATE 4: CHECK OPPOSITE DIRECTION (toward target)
    // If sidestepped RIGHT, check LEFT. If sidestepped LEFT, check RIGHT.
    // ========================================
    case CHECKING_PATH: {
      stopMotors();
      delay(500);  // Longer pause before checking
      
      // Scan OPPOSITE direction from sidestep
      // If we went RIGHT, target is on our LEFT
      // If we went LEFT, target is on our RIGHT
      int scan_angle;
      const char* direction_name;
      
      if (chosen_side > 0) {
        // Sidestepped RIGHT -> check LEFT (150°)
        scan_angle = 150;
        direction_name = "LEFT";
      } else {
        // Sidestepped LEFT -> check RIGHT (30°)
        scan_angle = 30;
        direction_name = "RIGHT";
      }
      
      Serial.print(">>> Checking ");
      Serial.print(direction_name);
      Serial.println(" (toward target) <<<");
      
      scanServo.write(scan_angle);
      delay(800);  // Longer delay for servo
      float check_dist = 0;
      for (int i = 0; i < 3; i++) {
        check_dist += getDistance();
        delay(100);
      }
      check_dist /= 3.0;
      scanServo.write(90);
      delay(500);
      
      Serial.print(direction_name);
      Serial.print(" distance: ");
      Serial.print(check_dist * 100, 1);
      Serial.println("cm");
      
      if (check_dist > OBSTACLE_THRESHOLD) {
        // Target direction is CLEAR! Turn toward goal and go
        Serial.print("*** ");
        Serial.print(direction_name);
        Serial.println(" CLEAR! Turning toward goal ***");
        
        // Turn to face the goal
        float desired_angle = atan2(goal_y - robot_y, goal_x - robot_x);
        
        unsigned long turn_start = millis();
        while (millis() - turn_start < 2000) {
          updateOdometry();
          
          float heading_error = desired_angle - robot_yaw;
          while (heading_error > PI) heading_error -= 2 * PI;
          while (heading_error < -PI) heading_error += 2 * PI;
          
          if (abs(heading_error) < 0.15) break;  // ~9 degrees
          
          if (heading_error > 0) {
            setMotors(-180, 180);  // Turn left
          } else {
            setMotors(180, -180);  // Turn right
          }
          delay(50);
        }
        stopMotors();
        delay(200);
        
        Serial.println("Now facing goal - resuming navigation");
        current_state = GOING_TO_GOAL;
        sidestep_attempts = 0;
        
      } else if (sidestep_attempts < MAX_SIDESTEP_ATTEMPTS) {
        Serial.print("Still blocked - sidestep again (#");
        Serial.print(sidestep_attempts + 1);
        Serial.println(")");
        
        sidestep_start_x = robot_x;
        sidestep_start_y = robot_y;
        current_state = SIDESTEPPING;
        
      } else {
        Serial.println("*** MAX ATTEMPTS - backing up, trying other side ***");
        setMotors(-BASE_PWM, -BASE_PWM);
        delay(600);
        stopMotors();
        delay(200);
        
        chosen_side = -chosen_side;  // Switch sides
        sidestep_attempts = 0;
        sidestep_start_x = robot_x;
        sidestep_start_y = robot_y;
        current_state = SIDESTEPPING;
      }
      break;
    }
  }
  
  // Print state only (position info is in updateOdometry)
  if (millis() - last_print > 500) {
    Serial.print(">>> STATE: ");
    switch (current_state) {
      case GOING_TO_GOAL: Serial.println("GOING_TO_GOAL"); break;
      case CHECKING_SIDES: Serial.println("CHECKING_SIDES"); break;
      case SIDESTEPPING: Serial.println("SIDESTEPPING"); break;
      case CHECKING_PATH: Serial.println("CHECKING_PATH"); break;
    }
    last_print = millis();
  }
}

// ===== MOTOR CONTROL =====
void setMotors(int left_pwm, int right_pwm) {
  int balanced_left_pwm = left_pwm;
  int balanced_right_pwm = right_pwm;
  
  if (right_pwm != 0) {
    // Boost right motor by balance factor, preserving sign
    int sign = (right_pwm > 0) ? 1 : -1;
    balanced_right_pwm = sign * (abs(right_pwm) * (1.0 + MOTOR_BALANCE_FACTOR));
  }
  
  // Left motor
  if (balanced_left_pwm >= 0) {
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    ledcWrite(motorA_ENA, abs(balanced_left_pwm));
  } else {
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, HIGH);
    ledcWrite(motorA_ENA, abs(balanced_left_pwm));
  }
  
  // Right motor
  if (balanced_right_pwm >= 0) {
    digitalWrite(motorB_IN3, LOW);
    digitalWrite(motorB_IN4, HIGH);
    ledcWrite(motorB_ENB, abs(balanced_right_pwm));
  } else {
    digitalWrite(motorB_IN3, HIGH);
    digitalWrite(motorB_IN4, LOW);
    ledcWrite(motorB_ENB, abs(balanced_right_pwm));
  }
}

void stopMotors() {
  ledcWrite(motorA_ENA, 0);
  ledcWrite(motorB_ENB, 0);
}