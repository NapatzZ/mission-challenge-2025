void Fan(int SpeedFan, int degree2, int degree3) {
  esc.writeMicroseconds(SpeedFan);
  servo(2, degree2);
  servo(3, degree3);
}