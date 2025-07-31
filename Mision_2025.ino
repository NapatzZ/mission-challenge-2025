#include <POP32.h>
#include <SPI.h>
#include <Servo.h>  //esc set=1000 towork 1100-2000
#define CS_PIN PB0  // กำหนดขา Chip Select
#include <Wire.h>      // ไลบรารีสำหรับการสื่อสาร I2C
#include <math.h>      // ไลบรารีสำหรับฟังก์ชันทางคณิตศาสตร์เช่น atan2

// --- ค่าคงที่และตัวแปรสำหรับเซ็นเซอร์ ---
#define I2C_ADDRESS 0x6C    // ที่อยู่ I2C ของเซ็นเซอร์ MC34X9
#define REG_OUT_X_LSB 0x0D  // เรจิสเตอร์สำหรับอ่านค่าแกน X (LSB)
#define REG_OUT_Y_LSB 0x0F  // เรจิสเตอร์สำหรับอ่านค่าแกน Y (LSB)
#define REG_OUT_Z_LSB 0x11  // เรจิสเตอร์สำหรับอ่านค่าแกน Z (LSB)
#define LSB_TO_G 0.000061   // ค่าคงที่สำหรับแปลงจาก LSB เป็นหน่วย g (แรงโน้มถ่วง)

// ตัวแปรแบบ global สำหรับเก็บค่าความเร่งและความเอียง
float accelX_g, accelY_g, accelZ_g;
float pitch_deg, roll_deg;

float last_error = 0;
Servo esc;
int i=0;

void setup() {
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  esc.attach(PB10);  // pinservo
  esc.writeMicroseconds(1000);
  SPI.begin();  // PA5=SCK, PA6=MISO, PA7=MOSI (STM32 SPI1)
  waitAnykey_bmp();
  setup_accelerometer();
  beep(1000);
  setupIMU();

  testDirectionalMovement();

  // esc.writeMicroseconds(2000);
  
}


void loop() {

  // ลองใช้ฟังก์ชันใหม่ที่ sync ได้ดีกว่า
  if (readIMUHeadingSimple()) {  // <-- เปลี่ยนจาก readIMUHeading()
    maintainHeading(0, 40);
    printIMUData();
  }
  delay(50);

  // measure_tilt_and_accel();
  // Serial.print("Pitch: ");
  // Serial.print(pitch_deg);
  // Serial.print(" deg\t");

  // Serial.print("Roll: ");
  // Serial.print(roll_deg);
  // Serial.println(" deg");

  // Serial.print("Accel X: ");
  // Serial.print(accelX_g);
  // Serial.print(" g\t");

  // Serial.print("Y: ");
  // Serial.print(accelY_g);
  // Serial.print(" g\t");

  // Serial.print("Z: ");
  // Serial.print(accelZ_g);
  // Serial.println(" g");

  //delay(100); 
  //PID_wall(30, 2, 0, 0.5, 0);     // wall track
  // PID_Sensor(60,3,1.5,2.35);  // line track
}
