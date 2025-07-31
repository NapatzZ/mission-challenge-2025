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



// --- Popular Direction Functions ---

/**
 * @brief Move forward (0°)
 * @param speed Speed (0-100)
 */
void moveForward(int speed) {
  moveInDirection(NORTH, speed, 0.0f); // Maintain robot heading at 0°
}

/**
 * @brief เคลื่อนที่ไปข้างหลัง (180°)
 * @param speed ความเร็ว (0-100)
 */
void moveBackward(int speed) {
  moveInDirection(SOUTH, speed, 0.0f); // รักษาหัวหุ่นยนต์ที่ 0°
}

/**
 * @brief เคลื่อนที่ไปซ้าย (90°)
 * @param speed ความเร็ว (0-100)
 */
void moveLeft(int speed) {
  moveInDirection(EAST, speed, 0.0f); // รักษาหัวหุ่นยนต์ที่ 0°
}

/**
 * @brief เคลื่อนที่ไปขวา (-90°)
 * @param speed ความเร็ว (0-100)
 */
void moveRight(int speed) {
  moveInDirection(WEST, speed, 0.0f); // รักษาหัวหุ่นยนต์ที่ 0°
}

/**
 * @brief เคลื่อนที่ไปมุม 45° (ข้างหน้าขวา)
 * @param speed ความเร็ว (0-100)
 */
void move45Degrees(int speed) {
  moveInDirection(NORTHWEST, speed, 0.0f); // รักษาหัวหุ่นยนต์ที่ 0°
}

/**
 * @brief เคลื่อนที่ไปมุม 135° (ข้างหลังซ้าย)
 * @param speed ความเร็ว (0-100)
 */
void move135Degrees(int speed) {
  moveInDirection(SOUTHEAST, speed, 0.0f); // รักษาหัวหุ่นยนต์ที่ 0°
}

// --- ฟังก์ชันการหมุน ---

/**
 * @brief หมุนหุ่นยนต์ไปมุมที่กำหนด
 * @param target_angle มุมเป้าหมายในหน่วยเรเดียน
 */
void turnToAngle(float target_angle) {
  maintain_heading_enabled = false; // ปิดการรักษา heading ชั่วคราว
  
  // ใช้ maintainHeading จาก ZXimu.ino
  maintainHeading(target_angle, 0);
  
  maintain_heading_enabled = true; // เปิดกลับ
}

/**
 * @brief หมุนหุ่นยนต์ไปมุมยอดฮิต
 */
void turnTo0Degrees() { turnToAngle(0.0f); }           // 0°
void turnTo45Degrees() { turnToAngle(PI/4.0f); }       // 45°
void turnTo90Degrees() { turnToAngle(PI/2.0f); }       // 90°
void turnTo135Degrees() { turnToAngle(3.0f*PI/4.0f); } // 135°
void turnTo180Degrees() { turnToAngle(PI); }           // 180°

// --- ฟังก์ชันควบคุมการตั้งค่า ---

/**
 * @brief เปิด/ปิดการรักษา heading
 * @param enable true=เปิด, false=ปิด
 */
void enableHeadingControl(bool enable) {
  maintain_heading_enabled = enable;
}

/**
 * @brief ตั้งค่าความเร็วเริ่มต้น
 * @param speed ความเร็ว (0-100)
 */
void setDefaultSpeed(int speed) {
  movement_speed = constrain(speed, 0, 100);
}

/**
 * @brief หยุดการเคลื่อนที่ทั้งหมด
 */
void stopMovement() {
  Motor_Stop();
}

// --- ฟังก์ชันทดสอบ ---

/**
 * @brief ทดสอบการเคลื่อนที่ในทิศทางต่างๆ
 */
void testDirectionalMovement() {
  Serial.println("=== Testing Directional Movement ===");
  
  // รีเซ็ต heading
  resetCurrentHeading();
  delay(1000);
  
  // ทดสอบทิศทางต่างๆ แบบต่อเนื่อง
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
  
  // หยุด
  stopMovement();
  Serial.println("=== Test Complete ===");
}

/**
 * @brief ทดสอบการเคลื่อนที่ทิศทางเดียวแบบต่อเนื่อง
 * @param direction ทิศทาง (เรเดียน)
 * @param speed ความเร็ว
 * @param duration ระยะเวลา (มิลลิวินาที)
 */
void moveToDirection(float direction, int speed, unsigned long duration) {
  Serial.print("Testing direction: ");
  Serial.print(direction * 180.0f / PI, 1);
  Serial.println("°");
  
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    moveInDirection(direction, speed, 0.0f); // รักษา heading ที่ 0°
    printIMUData();
    delay(50);
  }
  
  stopMovement();
  delay(500);
}

/**
 * @brief แสดงข้อมูลการเคลื่อนที่ปัจจุบัน
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
  
  // แสดงข้อมูล IMU และ accelerometer
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
