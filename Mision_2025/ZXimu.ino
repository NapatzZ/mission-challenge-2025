/*
  ZX-IMU control functions including Yaw reading,
  heading maintenance, and PID control for Mecanum wheel robot
*/

// Note: All global variables moved to Mision_2025.ino

// --- IMU Reset and Calibration Functions ---

/**
 * @brief Initialize and reset ZX-IMU
 */
void setupIMU() {
  // Use baud rate from configuration
  Serial1.begin(IMU_BAUD_RATE);
  delay(100);
  
  Serial.println("IMU Initializing...");
  
  // Send reset commands via UART
  Serial1.write(0xA5); Serial1.write(0x54); delay(100);
  Serial1.write(0xA5); Serial1.write(0x55); delay(100);
  Serial1.write(0xA5); Serial1.write(0x52); delay(100);
  
  // Wait for IMU to reset
  delay(500);
  
  // Read initial values and set to 0
  for(int i = 0; i < 10; i++) {
    readIMUHeading();
    delay(10);
  }
  initialYaw = pvYaw;
  targetYaw = 0.0f;
  
  // Reset PID variables
  head_error = 0.0f;
  head_pError = 0.0f;
  head_i = 0.0f;
  head_d = 0.0f;
  
  Serial.println("IMU Setup Complete!");
}

/**
 * @brief Set the current angle as the initial heading (0 degrees)
 */
void setInitialHeading() {
  initialYaw = pvYaw;
  targetYaw = 0.0f;
  
  // Reset PID variables
  head_error = 0.0f;
  head_pError = 0.0f;
  head_i = 0.0f;
  head_d = 0.0f;
  
  Serial.println("Current heading set as initial (0 degrees)!");
}

// --- IMU Reading Functions ---

/**
 * @brief Read Yaw angle from ZX-IMU via UART
 * @return true if successful, false if failed
 */
bool readIMUHeading() {
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    
    // Look for header 0xAA
    if (rxCnt == 0) {
      if (data == 0xAA) {
        rxBuf[rxCnt++] = data;
      }
      // Skip if not 0xAA
      continue;
    }
    
    // Store subsequent bytes
    rxBuf[rxCnt++] = data;
    
    // Once 8 bytes received
    if (rxCnt == 8) {
      rxCnt = 0;
      
      // Check footer 0x55
      if (rxBuf[7] == 0x55) {
        // Calculate Yaw from byte 1 and 2
        int16_t rawYaw = (rxBuf[1] << 8) | rxBuf[2];
        pvYaw = rawYaw / 100.0f;
        return true;
      }
      // If footer incorrect, restart
    }
  }
  return false;
}

/**
 * @brief Non-blocking version of IMU reading function
 */
bool readIMUHeadingSimple() {
  static uint8_t buffer[8];
  static int bufIndex = 0;
  static bool foundHeader = false;
  
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    
    if (!foundHeader) {
      // Look for header
      if (data == 0xAA) {
        buffer[0] = data;
        bufIndex = 1;
        foundHeader = true;
      }
    } else {
      // Store next data
      buffer[bufIndex++] = data;
      
      if (bufIndex == 8) {
        foundHeader = false;
        bufIndex = 0;
        
        // Check footer
        if (buffer[7] == 0x55) {
          // Calculate Yaw
          int16_t rawYaw = (buffer[1] << 8) | buffer[2];
          pvYaw = rawYaw / 100.0f;
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * @brief Test function to display raw Serial1 data
 */
void testSerialData() {
  Serial.println("=== Testing Serial1 Raw Data ===");
  Serial.println("Listening for 10 seconds...");
  
  unsigned long startTime = millis();
  int byteCount = 0;
  
  while (millis() - startTime < 10000) {
    if (Serial1.available()) {
      uint8_t data = Serial1.read();
      Serial.print("Byte ");
      Serial.print(byteCount++);
      Serial.print(": 0x");
      Serial.print(data, HEX);
      Serial.print(" (");
      Serial.print(data);
      Serial.println(")");
    }
    delay(1);
  }
  
  Serial.print("Total bytes received: ");
  Serial.println(byteCount);
  Serial.println("=== Test Complete ===");
}

/**
 * @brief Get Yaw angle relative to the initial heading
 * @return Relative Yaw (-180 to 180 degrees)
 */
float getRelativeHeading() {
  float relativeYaw = pvYaw - initialYaw;
  
  // Constrain value between -180 and 180
  while (relativeYaw > 180.0f) relativeYaw -= 360.0f;
  while (relativeYaw < -180.0f) relativeYaw += 360.0f;
  
  return relativeYaw;
}

/**
 * @brief Reset and assign current heading as new initial heading
 */
void resetCurrentHeading() {
  // Read current value multiple times
  for(int i = 0; i < 5; i++) {
    readIMUHeading();
    delay(10);
  }
  
  // Set current as initial heading
  initialYaw = pvYaw;
  targetYaw = 0.0f;
  
  // Reset PID variables
  head_error = 0.0f;
  head_pError = 0.0f;
  head_i = 0.0f;
  head_d = 0.0f;
  
  Serial.print("Reset heading! Initial Yaw set to: ");
  Serial.print(initialYaw);
  Serial.println("°");
}

/**
 * @brief Convert radians to degrees
 * @param rad Angle in radians
 * @return Angle in degrees
 */
float radToDeg(float rad) {
  return rad * 180.0f / PI;
}

/**
 * @brief Convert degrees to radians  
 * @param deg Angle in degrees
 * @return Angle in radians
 */
float degToRad(float deg) {
  return deg * PI / 180.0f;
}

// --- Heading Control Functions ---

/**
 * @brief Set PID parameters for heading control
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void setPIDGains(float kp, float ki, float kd) {
  head_Kp = kp;
  head_Ki = ki;
  head_Kd = kd;
  
  Serial.println("PID Gains set");
}

/**
 * @brief Maintain heading while moving using PID control
 * @param targetTheta Target angle (radians)
 * @param speed Movement speed (0-100)
 */
void maintainHeading(float targetTheta, int speed) {
  // Convert target from radians to degrees
  targetYaw = radToDeg(targetTheta);
  
  // Calculate error from current adjusted yaw
  float currentYaw = getRelativeHeading();
  head_error = targetYaw - currentYaw;
  
  // Constrain error between -180 and 180
  while (head_error > 180.0f) head_error -= 360.0f;
  while (head_error < -180.0f) head_error += 360.0f;
  
  // Integral term
  head_i += head_error;
  head_i = constrain(head_i, -180, 180);  // Limit integral term
  
  // Derivative term
  head_d = head_error - head_pError;
  
  // Calculate PID output
  head_output = (head_error * head_Kp) + 
                (head_i * head_Ki) + 
                (head_d * head_Kd);
  
  // Limit output
  head_output = constrain(head_output, -50, 50);
  
  // Calculate motor speed for each Mecanum wheel to maintain heading
  int rotationSpeed = (int)head_output;
  
  // Drive Mecanum motors to rotate
  // Left wheels: forward, Right wheels: reverse
  Motor(-rotationSpeed, -rotationSpeed, rotationSpeed, rotationSpeed);
  
  // Save error for next iteration
  head_pError = head_error;
}

/**
 * @brief Maintain current heading while stationary
 */
void holdCurrentHeading() {
  maintainHeading(degToRad(targetYaw), 0);
}

// --- Testing Functions ---

/**
 * @brief Print IMU data to Serial Monitor
 */
void printIMUData() {
  Serial.print("Raw Yaw: ");
  Serial.print(pvYaw, 2);
  Serial.print("°\t");
  
  Serial.print("Relative Yaw: ");
  Serial.print(getRelativeHeading(), 2);
  Serial.print("°\t");
  
  Serial.print("Target: ");
  Serial.print(targetYaw, 2);
  Serial.print("°\t");
  
  Serial.print("Error: ");
  Serial.print(head_error, 2);
  Serial.print("°\t");
  
  Serial.print("PID Output: ");
  Serial.println(head_output, 2);
}
