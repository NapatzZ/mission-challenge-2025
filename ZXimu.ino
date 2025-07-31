/*
  ไฟล์นี้ประกอบด้วยฟังก์ชันสำหรับการใช้งาน ZX-IMU 
  รวมถึงการอ่านค่า Yaw, การรักษา heading, และ PID control
  สำหรับหุ่นยนต์ล้อ Mecanum wheel
*/

// --- ตัวแปรสำหรับ IMU ---
uint8_t rxBuf[8], rxCnt = 0;
float pvYaw = 0.0f;        // ค่ามุมปัจจุบัน (องศา)
float initialYaw = 0.0f;   // ค่ามุมเริ่มต้น (องศา)
float targetYaw = 0.0f;    // ค่ามุมเป้าหมาย (องศา)

// --- ตัวแปรสำหรับ PID Control ---
float head_Kp = 1.0f;  // Proportional gain - ปรับได้
float head_Ki = 0.0f;  // Integral gain - ปรับได้  
float head_Kd = 0.1f;  // Derivative gain - ปรับได้

float head_error = 0.0f, head_pError = 0.0f;
float head_output = 0.0f, head_d = 0.0f, head_i = 0.0f;

// --- ฟังก์ชันสำหรับรีเซ็ตและปรับเทียบ IMU ---

/**
 * @brief ตั้งค่าเริ่มต้นและรีเซ็ต ZX-IMU
 */
void setupIMU() {
  // ลอง Baud rate ต่างๆ
  Serial1.begin(115200);  // ลองอันนี้ก่อน
  // Serial1.begin(9600);   // หากไม่ได้ลองอันนี้
  // Serial1.begin(38400);  // หรืออันนี้
  delay(100);
  
  Serial.println("IMU Initializing...");
  
  // ส่งคำสั่งรีเซ็ตผ่าน UART
  Serial1.write(0xA5); Serial1.write(0x54); delay(100);
  Serial1.write(0xA5); Serial1.write(0x55); delay(100);
  Serial1.write(0xA5); Serial1.write(0x52); delay(100);
  
  // รอให้ IMU รีเซ็ต
  delay(500);
  
  // อ่านค่าเริ่มต้นและตั้งเป็น 0
  for(int i = 0; i < 10; i++) {
    readIMUHeading();
    delay(10);
  }
  initialYaw = pvYaw;
  targetYaw = 0.0f;
  
  // รีเซ็ต PID variables
  head_error = 0.0f;
  head_pError = 0.0f;
  head_i = 0.0f;
  head_d = 0.0f;
  
  Serial.println("IMU Setup Complete!");
}

/**
 * @brief ตั้งค่ามุมปัจจุบันเป็นมุมเริ่มต้น (0 องศา)
 */
void setInitialHeading() {
  initialYaw = pvYaw;
  targetYaw = 0.0f;
  
  // รีเซ็ต PID variables
  head_error = 0.0f;
  head_pError = 0.0f;
  head_i = 0.0f;
  head_d = 0.0f;
  
  Serial.println("Current heading set as initial (0 degrees)!");
}

// --- ฟังก์ชันสำหรับอ่านค่าจาก IMU ---

/**
 * @brief อ่านค่ามุม Yaw จาก ZX-IMU ผ่าน UART
 * @return true หากอ่านข้อมูลสำเร็จ, false หากไม่สำเร็จ
 */
bool readIMUHeading() {
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    
    // หา header 0xAA
    if (rxCnt == 0) {
      if (data == 0xAA) {
        rxBuf[rxCnt++] = data;
      }
      // ถ้าไม่ใช่ 0xAA ให้ข้าม
      continue;
    }
    
    // เก็บข้อมูลต่อไป
    rxBuf[rxCnt++] = data;
    
    // เมื่อได้รับครบ 8 ไบต์
    if (rxCnt == 8) {
      rxCnt = 0;
      
      // ตรวจสอบ footer 0x55
      if (rxBuf[7] == 0x55) {
        // คำนวณค่ามุม Yaw - ใช้ byte 1 และ 2
        int16_t rawYaw = (rxBuf[1] << 8) | rxBuf[2];
        pvYaw = rawYaw / 100.0f;
        return true;
      }
      // ถ้า footer ผิด ให้เริ่มใหม่
    }
  }
  return false;
}

/**
 * @brief ฟังก์ชันอ่าน IMU แบบ non-blocking
 */
bool readIMUHeadingSimple() {
  static uint8_t buffer[8];
  static int bufIndex = 0;
  static bool foundHeader = false;
  
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    
    if (!foundHeader) {
      // ค้นหา header
      if (data == 0xAA) {
        buffer[0] = data;
        bufIndex = 1;
        foundHeader = true;
      }
    } else {
      // เก็บข้อมูลต่อไป
      buffer[bufIndex++] = data;
      
      if (bufIndex == 8) {
        foundHeader = false;
        bufIndex = 0;
        
        // ตรวจสอบ footer
        if (buffer[7] == 0x55) {
          // คำนวณค่ามุม Yaw
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
 * @brief ฟังก์ชันทดสอบการรับข้อมูล Raw จาก Serial1
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
 * @brief ได้ค่ามุม Yaw ที่ปรับตาม initial heading แล้ว
 * @return ค่ามุม Yaw ที่ปรับแล้ว (-180 ถึง 180 องศา)
 */
float getRelativeHeading() {
  float relativeYaw = pvYaw - initialYaw;
  
  // จำกัดค่าให้อยู่ในช่วง -180 ถึง 180
  while (relativeYaw > 180.0f) relativeYaw -= 360.0f;
  while (relativeYaw < -180.0f) relativeYaw += 360.0f;
  
  return relativeYaw;
}

/**
 * @brief รีเซ็ตและกำหนด initial heading ใหม่ทันที
 */
void resetCurrentHeading() {
  // อ่านค่าปัจจุบันหลายรอบ
  for(int i = 0; i < 5; i++) {
    readIMUHeading();
    delay(10);
  }
  
  // กำหนดค่าปัจจุบันเป็น initial heading
  initialYaw = pvYaw;
  targetYaw = 0.0f;
  
  // รีเซ็ต PID variables
  head_error = 0.0f;
  head_pError = 0.0f;
  head_i = 0.0f;
  head_d = 0.0f;
  
  Serial.print("Reset heading! Initial Yaw set to: ");
  Serial.print(initialYaw);
  Serial.println("°");
}

/**
 * @brief แปลงมุมจากเรเดียนเป็นองศา
 * @param rad มุมในหน่วยเรเดียน
 * @return มุมในหน่วยองศา
 */
float radToDeg(float rad) {
  return rad * 180.0f / PI;
}

/**
 * @brief แปลงมุมจากองศาเป็นเรเดียน  
 * @param deg มุมในหน่วยองศา
 * @return มุมในหน่วยเรเดียน
 */
float degToRad(float deg) {
  return deg * PI / 180.0f;
}

// --- ฟังก์ชันสำหรับการควบคุม Heading ---

/**
 * @brief ตั้งค่าพารามิเตอร์ PID สำหรับควบคุม heading
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
 * @brief รักษาทิศทางขณะเคลื่อนที่ด้วย PID control
 * @param targetTheta มุมเป้าหมาย (เรเดียน)
 * @param speed ความเร็วการเคลื่อนที่ (0-100)
 */
void maintainHeading(float targetTheta, int speed) {
  // แปลงเป้าหมายจากเรเดียนเป็นองศา
  targetYaw = radToDeg(targetTheta);
  
  // คำนวณ error จากค่าปัจจุบันที่ปรับแล้ว
  float currentYaw = getRelativeHeading();
  head_error = targetYaw - currentYaw;
  
  // จำกัด error ให้อยู่ในช่วง -180 ถึง 180
  while (head_error > 180.0f) head_error -= 360.0f;
  while (head_error < -180.0f) head_error += 360.0f;
  
  // Integral term
  head_i += head_error;
  head_i = constrain(head_i, -180, 180);  // จำกัดค่า integral
  
  // Derivative term
  head_d = head_error - head_pError;
  
  // คำนวณ output PID
  head_output = (head_error * head_Kp) + 
                (head_i * head_Ki) + 
                (head_d * head_Kd);
  
  // จำกัด output
  head_output = constrain(head_output, -50, 50);
  
  // คำนวณความเร็วมอเตอร์แต่ละตัว (Mecanum wheel)
  // สำหรับการรักษา heading โดยหมุนตัวหุ่นยนต์
  int rotationSpeed = (int)head_output;
  
  // ขับมอเตอร์ Mecanum wheel สำหรับการหมุน
  // ล้อซ้าย: หมุนไปข้างหน้า, ล้อขวา: หมุนย้อนกลับ
  Motor(-rotationSpeed, -rotationSpeed, rotationSpeed, rotationSpeed);
  
  // บันทึก error สำหรับรอบถัดไป
  head_pError = head_error;
}

/**
 * @brief รักษา heading ปัจจุบันขณะหยุดนิ่ง
 */
void holdCurrentHeading() {
  maintainHeading(degToRad(targetYaw), 0);
}

// --- ฟังก์ชันสำหรับการทดสอบ ---

/**
 * @brief แสดงข้อมูล IMU ผ่าง Serial Monitor
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
