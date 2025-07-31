/*
  Additional motor control functions for enhanced robot movement
*/

// --- Extended Motor Control Functions ---

/**
 * @brief Set individual motor speed with direction
 * @param motor_id Motor number (1-4)
 * @param speed Speed and direction (-100 to 100)
 */
void setMotorSpeed(int motor_id, int speed) {
  speed = constrain(speed, -100, 100);
  motor(motor_id, speed);
}

/**
 * @brief Stop all motors immediately
 */
void emergencyStop() {
  Motor_Stop();
  Serial.println("Emergency stop activated!");
}

/**
 * @brief Gradual motor speed change for smooth movement
 * @param target_speeds Array of target speeds for motors 1-4
 * @param steps Number of steps for gradual change
 * @param step_delay Delay between steps (ms)
 */
void gradualSpeedChange(int target_speeds[4], int steps, int step_delay) {
  // Get current motor speeds (simplified - assume starting from 0)
  int current_speeds[4] = {0, 0, 0, 0};
  
  for (int step = 0; step <= steps; step++) {
    for (int i = 0; i < 4; i++) {
      int new_speed = map(step, 0, steps, current_speeds[i], target_speeds[i]);
      setMotorSpeed(i + 1, new_speed);
    }
    delay(step_delay);
  }
}