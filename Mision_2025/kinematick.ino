/*
  Kinematics control functions for Mecanum wheel robot
  Provides omnidirectional movement with heading control using ZX-IMU and accelerometer
*/

// --- Main Movement Functions ---

/**
 * @brief Move in specified direction while maintaining heading
 * @param direction Direction in radians (0=forward, PI/2=left, PI=backward, -PI/2=right)
 * @param speed Speed (0-100)
 * @param target_heading Target robot heading to maintain (radians)
 */
void moveInDirection(float direction, int speed, float target_heading) {
  // Store current values
  current_direction = direction;
  movement_speed = speed;
  
  // Calculate motor speeds for Mecanum wheel kinematics
  float sin_dir = sin(direction + PI/4.0f); // Rotate 45 degrees for mecanum
  float cos_dir = cos(direction + PI/4.0f);
  
  // Calculate base motor speeds for each wheel
  int motor1_speed = (int)(speed * sin_dir); // Front Left
  int motor2_speed = (int)(speed * cos_dir); // Back Left  
  int motor3_speed = (int)(speed * cos_dir); // Front Right
  int motor4_speed = (int)(speed * sin_dir); // Back Right
  
  // Maintain heading if enabled
  if (maintain_heading_enabled && readIMUHeadingSimple()) {
    // Use same PID algorithm as ZXimu.ino
    float currentYaw = getRelativeHeading(); // Get value in degrees
    float target_heading_deg = target_heading * 180.0f / PI; // Convert target to degrees
    head_error = target_heading_deg - currentYaw;
    
    // Limit error to -180 to 180 range
    while (head_error > 180.0f) head_error -= 360.0f;
    while (head_error < -180.0f) head_error += 360.0f;
    
    // Integral term (same as ZXimu.ino)
    head_i += head_error;
    head_i = constrain(head_i, -180, 180);
    
    // Derivative term (same as ZXimu.ino)  
    head_d = head_error - head_pError;
    
    // Calculate PID output (same as ZXimu.ino)
    float pid_output = (head_error * head_Kp) + 
                       (head_i * head_Ki) + 
                       (head_d * head_Kd);
    
    // Limit output
    pid_output = constrain(pid_output, -30, 30); // Reduce to avoid interfering with movement
    
    // Adjust motor speeds for rotation (same as ZXimu.ino)
    motor1_speed -= (int)pid_output;  // Front Left 
    motor2_speed -= (int)pid_output;  // Back Left
    motor3_speed += (int)pid_output;  // Front Right
    motor4_speed += (int)pid_output;  // Back Right
    
    // Save error for next iteration
    head_pError = head_error;
  }
  
  // Constrain motor speeds
  motor1_speed = constrain(motor1_speed, -100, 100);
  motor2_speed = constrain(motor2_speed, -100, 100);
  motor3_speed = constrain(motor3_speed, -100, 100);
  motor4_speed = constrain(motor4_speed, -100, 100);
  
  // Send commands to motors
  Motor(motor1_speed, motor2_speed, motor3_speed, motor4_speed);
}

// --- Common Direction Functions ---

/**
 * @brief Move forward (0°)
 * @param speed Speed (0-100)
 */
void moveForward(int speed) {
  moveInDirection(NORTH, speed, 0.0f); // Maintain robot heading at 0°
}

/**
 * @brief Move backward (180°)
 * @param speed Speed (0-100)
 */
void moveBackward(int speed) {
  moveInDirection(SOUTH, speed, 0.0f); // Maintain robot heading at 0°
}

/**
 * @brief Move left (90°)
 * @param speed Speed (0-100)
 */
void moveLeft(int speed) {
  moveInDirection(EAST, speed, 0.0f); // Maintain robot heading at 0°
}

/**
 * @brief Move right (-90°)
 * @param speed Speed (0-100)
 */
void moveRight(int speed) {
  moveInDirection(WEST, speed, 0.0f); // Maintain robot heading at 0°
}

/**
 * @brief Move at 45° angle (forward-right)
 * @param speed Speed (0-100)
 */
void move45Degrees(int speed) {
  moveInDirection(NORTHWEST, speed, 0.0f); // Maintain robot heading at 0°
}

/**
 * @brief Move at 135° angle (backward-left)
 * @param speed Speed (0-100)
 */
void move135Degrees(int speed) {
  moveInDirection(SOUTHEAST, speed, 0.0f); // Maintain robot heading at 0°
}

// --- Rotation Functions ---

/**
 * @brief Rotate robot to a specific angle
 * @param target_angle Target angle in radians
 */
void turnToAngle(float target_angle) {
  maintain_heading_enabled = false; // Temporarily disable heading hold
  
  // Use maintainHeading from ZXimu.ino
  maintainHeading(target_angle, 0);
  
  maintain_heading_enabled = true; // Re-enable
}

/**
 * @brief Rotate robot to popular angles
 */
void turnTo0Degrees() { turnToAngle(0.0f); }           // 0°
void turnTo45Degrees() { turnToAngle(PI/4.0f); }       // 45°
void turnTo90Degrees() { turnToAngle(PI/2.0f); }       // 90°
void turnTo135Degrees() { turnToAngle(3.0f*PI/4.0f); } // 135°
void turnTo180Degrees() { turnToAngle(PI); }           // 180°

// --- Control Settings Functions ---

/**
 * @brief Enable/disable heading hold
 * @param enable true=enable, false=disable
 */
void enableHeadingControl(bool enable) {
  maintain_heading_enabled = enable;
}

/**
 * @brief Set default movement speed
 * @param speed Speed (0-100)
 */
void setDefaultSpeed(int speed) {
  movement_speed = constrain(speed, 0, 100);
}

/**
 * @brief Stop all movement
 */
void stopMovement() {
  Motor_Stop();
}

// --- Test Functions ---

/**
 * @brief Test movement in various directions
 */
void testDirectionalMovement() {
  Serial.println("=== Testing Directional Movement ===");
  
  // Reset heading
  resetCurrentHeading();
  delay(1000);
  
  // Test each direction sequentially
  Serial.println("Moving Forward (0°)");
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    moveForward(50);
    printIMUData();
    delay(50);
  }
  
  Serial.println("Moving Right (-90°)");
  startTime = millis();
  while (millis() - startTime < 2000) {
    moveRight(50);
    printIMUData();
    delay(50);
  }
  
  Serial.println("Moving Backward (180°)");
  startTime = millis();
  while (millis() - startTime < 2000) {
    moveBackward(50);
    printIMUData();
    delay(50);
  }
  
  Serial.println("Moving Left (90°)");
  startTime = millis();
  while (millis() - startTime < 2000) {
    moveLeft(50);
    printIMUData();
    delay(50);
  }
  
  Serial.println("Moving 45° (NE)");
  startTime = millis();
  while (millis() - startTime < 2000) {
    move45Degrees(50);
    printIMUData();
    delay(50);
  }
  
  // Stop
  stopMovement();
  Serial.println("=== Test Complete ===");
}

/**
 * @brief Test movement in a specific direction for a duration
 * @param direction Direction in radians
 * @param speed Speed
 * @param duration Duration in milliseconds
 */
void moveToDirection(float direction, int speed, unsigned long duration) {
  Serial.print("Testing direction: ");
  Serial.print(direction * 180.0f / PI, 1);
  Serial.println("°");
  
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    moveInDirection(direction, speed, 0.0f); // Maintain heading at 0°
    printIMUData();
    delay(50);
  }
  
  stopMovement();
  delay(500);
}

/**
 * @brief Print current movement status
 */
void printMovementStatus() {
  Serial.print("Direction: ");
  Serial.print(current_direction * 180.0f / PI, 1);
  Serial.print("°\t");
  
  Serial.print("Speed: ");
  Serial.print(movement_speed);
  Serial.print("\t");
  
  Serial.print("Heading Control: ");
  Serial.println(maintain_heading_enabled ? "ON" : "OFF");
  
  // Show IMU and accelerometer data
  if (readIMUHeadingSimple()) {
    printIMUData();
  }
  
  measure_tilt_and_accel();
  Serial.print("Tilt - Pitch: ");
  Serial.print(pitch_deg, 1);
  Serial.print("° Roll: ");
  Serial.print(roll_deg, 1);
  Serial.println("°");
}
