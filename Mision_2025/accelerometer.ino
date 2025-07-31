/*
  Accelerometer functions for reading MC34X9 sensor
  and calculating tilt angles (pitch and roll)
*/



// --- I2C Communication Helper Functions ---

/**
 * @brief Write data to sensor register
 * @param reg Register address
 * @param value Value to write
 */
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * @brief Read 16-bit raw data from sensor register
 * @param reg Register address
 * @return 16-bit raw value
 */
int16_t readAccel(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDRESS, 2);
  int16_t lsb = Wire.read();
  int16_t msb = Wire.read();
  return (int16_t)(lsb | (msb << 8));
}

// --- ฟังก์ชันหลักสำหรับเซ็นเซอร์ ---

/**
 * @brief ตั้งค่าเริ่มต้นให้กับเซ็นเซอร์ MC34X9
 *        ต้องเรียกใช้ในฟังก์ชัน setup() ของโปรแกรมหลัก
 */
void setup_accelerometer() {
  Wire.begin();
  writeRegister(0x07, 0x01);  // ตั้งค่าเป็น Active mode
  writeRegister(0x20, 0x01);  // ตั้งค่าช่วงการวัดเป็น ±2g, ความละเอียด 16-bit
}

/**
 * @brief อ่านค่าความเร่งจากเซ็นเซอร์ทั้ง 3 แกน
 *        และแปลงเป็นหน่วย g (แรงโน้มถ่วง)
 *        ผลลัพธ์จะถูกเก็บในตัวแปร global: accelX_g, accelY_g, accelZ_g
 */
void read_acceleration() {
  accelX_g = readAccel(REG_OUT_X_LSB) * LSB_TO_G;
  accelY_g = readAccel(REG_OUT_Y_LSB) * LSB_TO_G;
  accelZ_g = readAccel(REG_OUT_Z_LSB) * LSB_TO_G;
}

/**
 * @brief คำนวณค่าความเอียง (Pitch และ Roll) จากข้อมูลความเร่ง
 *        ต้องเรียกใช้ read_acceleration() ก่อน
 *        ผลลัพธ์ (หน่วยองศา) จะถูกเก็บในตัวแปร global: pitch_deg, roll_deg
 */
void calculate_tilt() {
  // คำนวณ Roll (การเอียงรอบแกน X)
  roll_deg = atan2(accelY_g, accelZ_g) * 180.0 / M_PI;

  // คำนวณ Pitch (การเอียงรอบแกน Y)
  pitch_deg = atan2(-accelX_g, sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)) * 180.0 / M_PI;
}

/**
 * @brief ฟังก์ชันรวมสำหรับวัดค่าความเร่งและความเอียง
 *        เรียกใช้ read_acceleration() และ calculate_tilt() ตามลำดับ
 */
void measure_tilt_and_accel() {
    read_acceleration();
    calculate_tilt();
}

