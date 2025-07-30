int readChannel(byte channel) {  // in setup
  if (channel > 7) return -1;

  digitalWrite(CS_PIN, LOW);

  // MCP3008 ต้องส่ง 3 ไบต์: Start bit + Single/Diff + Channel
  byte start_bit = 0x01;
  byte command = 0x80 | (channel << 4);  // 1000 XXXX (single-ended mode)

  SPI.transfer(start_bit);            // Start bit
  byte high = SPI.transfer(command);  // ส่งคำสั่ง + รับ MSB
  byte low = SPI.transfer(0x00);      // รับ LSB

  digitalWrite(CS_PIN, HIGH);

  int result = ((high & 0x03) << 8) | low;  // 10-bit result
  return result;
}

int Show_sensor() {
  int Sensor[8], Sum_Raw = 0, weight[8] = { 0, 100, 200, 300, 400, 500, 600, 700 };  //loop
  int Sum_Sum = 0, Sum_sensor = 0, Constan[8];
  for (int i = 0; i < 8; i++) {
    int READ = readChannel(i);
    Constan[i] = map(READ, 20, 960, 100, 0);
    Sensor[i] = (Constan[i] * weight[i]);
    Sum_sensor += Sensor[i];
    Sum_Raw += Constan[i];
  }
  if (Sum_Raw == 0) {
    return 350;  // Default center position if no sensors detect line
  }
  Sum_Sum = Sum_sensor / Sum_Raw;
  return Sum_Sum;
}
