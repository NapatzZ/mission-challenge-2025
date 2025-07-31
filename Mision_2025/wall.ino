float getDistanceCM(int adcValue) {
  float voltage = adcValue * (3.3 / 4095.0);
  float distance = 27.86 / voltage;
  return distance;
}

void PID_wall(int Speed, float Kp, float Ki, float Kd, int mode) {
  while (1) {
    if (analogRead(1) > 500 && analogRead(0) > 1000) {
      Motor(60, 60, -60, -60);
    }
      else if (getDistanceCM(analogRead(0)) > 30) {
        Motor(25, 25, 55, 55);
      }
      else {
        float setpoint = mode == 0 ? analogRead(0) : analogRead(2);
        float error = getDistanceCM(setpoint) - 20;
        int output = (Kp * error) + ((error + Ki) * Ki) + (Kd * (error - last_error));
        last_error = error;
        int motorSpeedLeft = constrain(Speed - output, -100, 100);
        int motorSpeedRight = constrain(Speed + output, -100, 100);
        Motor(motorSpeedLeft, motorSpeedLeft, motorSpeedRight, motorSpeedRight);
        // Serial.print(motorSpeedLeft);
        // Serial.print("\t");
        // Serial.print(motorSpeedRight);
      }
    }
  }

  void M_motor(int sp) {
    for (int i = 10; i <= sp; i += 5) {
      Motor(i, i, i, i);  // เพิ่มค่าทีละ 10
      delay(500);         // เพิ่มหน่วงเวลาให้มอเตอร์มีเวลาหมุน (ถ้าจำเป็น)
    }
  }
