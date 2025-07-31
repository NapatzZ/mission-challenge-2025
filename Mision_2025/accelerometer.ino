/*
  Accelerometer functions for reading MC34X9 sensor
  and calculating tilt angles (pitch and roll)
*/

// --- I2C Communication Helper Functions ---

/**
 * @brief Write data to a sensor register
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

// --- Main Functions for the Sensor ---

/**
 * @brief Initialize the MC34X9 sensor
 *        Must be called in the setup() function of the main program
 */
void setup_accelerometer() {
  Wire.begin();
  writeRegister(0x07, 0x01);  // Set to Active mode
  writeRegister(0x20, 0x01);  // Set measurement range to ±2g, 16-bit resolution
}

/**
 * @brief Read acceleration from all 3 axes of the sensor
 *        and convert to g units (gravity)
 *        The results will be stored in global variables: accelX_g, accelY_g, accelZ_g
 */
void read_acceleration() {
  accelX_g = readAccel(REG_OUT_X_LSB) * LSB_TO_G;
  accelY_g = readAccel(REG_OUT_Y_LSB) * LSB_TO_G;
  accelZ_g = readAccel(REG_OUT_Z_LSB) * LSB_TO_G;
}

/**
 * @brief Calculate tilt angles (Pitch and Roll) from acceleration data
 *        read_acceleration() must be called beforehand
 *        Results (in degrees) are stored in global variables: pitch_deg, roll_deg
 */
void calculate_tilt() {
  // Calculate Roll (tilt around X-axis)
  roll_deg = atan2(accelY_g, accelZ_g) * 180.0 / M_PI;

  // Calculate Pitch (tilt around Y-axis)
  pitch_deg = atan2(-accelX_g, sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)) * 180.0 / M_PI;
}

/**
 * @brief Combined function for measuring acceleration and tilt
 *        Calls read_acceleration() and calculate_tilt() in sequence
 */
void measure_tilt_and_accel() {
    read_acceleration();
    calculate_tilt();
}
