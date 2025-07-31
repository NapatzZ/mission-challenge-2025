void PID_Sensor(int Speed, float Kp, float Ki, float Kd) {
  while (1) {
    int error = Show_sensor() - 310;
    int output = (Kp * error) + ((error + Ki) * Ki) + (Kd * (error - last_error));
    last_error = error;
    int motorSpeedLeft = constrain(Speed + output, -100, 100);
    int motorSpeedRight = constrain(Speed - output, -100, 100);
    Motor(motorSpeedLeft, motorSpeedLeft, motorSpeedRight, motorSpeedRight);
    // Serial.print(motorSpeedLeft);
    // Serial.print("\t");
    // Serial.print(motorSpeedRight);
  }
}

void Motor(int sp1, int sp2, int sp3, int sp4) {
  motor(1, sp1);//front_left
  motor(2, sp2);//back_left
  motor(3, sp3);//front_right
  motor(4, sp4);//back_right
}

void Motor_Stop() {
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
  motor(4, 0);
}
