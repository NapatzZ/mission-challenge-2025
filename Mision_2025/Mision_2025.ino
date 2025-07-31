#include <POP32.h>
#include <SPI.h>
#include <Servo.h>  //esc set=1000 towork 1100-2000
#define CS_PIN PB0  // กำหนดขา Chip Select
#include <Wire.h>      // ไลบรารีสำหรับการสื่อสาร I2C
#include <math.h>      // ไลบรารีสำหรับฟังก์ชันทางคณิตศาสตร์เช่น atan2

// =============================================================================
// PROJECT CONFIGURATION 
// =============================================================================

// --- ค่าคงที่สำหรับเซ็นเซอร์ Accelerometer MC34X9 ---
#define I2C_ADDRESS 0x6C    // ที่อยู่ I2C ของเซ็นเซอร์ MC34X9
#define REG_OUT_X_LSB 0x0D  // เรจิสเตอร์สำหรับอ่านค่าแกน X (LSB)
#define REG_OUT_Y_LSB 0x0F  // เรจิสเตอร์สำหรับอ่านค่าแกน Y (LSB)
#define REG_OUT_Z_LSB 0x11  // เรจิสเตอร์สำหรับอ่านค่าแกน Z (LSB)
#define LSB_TO_G 0.000061   // ค่าคงที่สำหรับแปลงจาก LSB เป็นหน่วย g (แรงโน้มถ่วง)

// --- ค่าคงที่สำหรับทิศทางการเคลื่อนที่ (เรเดียน) ---
#define NORTH     0.0f          // 0°   - ไปข้างหน้า
#define NORTHEAST (PI/4.0f)     // 45°  - ไปข้างหน้าซ้าย
#define EAST      (PI/2.0f)     // 90°  - ไปซ้าย
#define SOUTHEAST (3.0f*PI/4.0f)// 135° - ไปข้างหลังซ้าย
#define SOUTH     PI            // 180° - ไปข้างหลัง
#define SOUTHWEST (-3.0f*PI/4.0f)// -135° - ไปข้างหลังขวา
#define WEST      (-PI/2.0f)    // -90° - ไปขวา
#define NORTHWEST (-PI/4.0f)    // -45° - ไปข้างหน้าขวา

// --- ค่าตั้งต้นของระบบ ---
#define DEFAULT_SPEED 50          // ความเร็วเริ่มต้น (0-100)
#define IMU_BAUD_RATE 115200     // Baud rate สำหรับ ZX-IMU
#define PID_UPDATE_RATE 50       // อัตราการอัปเดต PID (ms)

// --- การตั้งค่า PID สำหรับการรักษา heading ---
#define HEAD_KP 1.0f    // Proportional gain
#define HEAD_KI 0.0f    // Integral gain  
#define HEAD_KD 0.1f    // Derivative gain

// =============================================================================
// GLOBAL VARIABLES 
// =============================================================================

// ตัวแปรสำหรับเซ็นเซอร์ Accelerometer
float accelX_g, accelY_g, accelZ_g;
float pitch_deg, roll_deg;

// ตัวแปรสำหรับ ZX-IMU
uint8_t rxBuf[8], rxCnt = 0;
float pvYaw = 0.0f;        // ค่ามุมปัจจุบัน (องศา)
float initialYaw = 0.0f;   // ค่ามุมเริ่มต้น (องศา)
float targetYaw = 0.0f;    // ค่ามุมเป้าหมาย (องศา)

// ตัวแปรสำหรับ PID Control
float head_Kp = HEAD_KP;   // Proportional gain
float head_Ki = HEAD_KI;   // Integral gain  
float head_Kd = HEAD_KD;   // Derivative gain
float head_error = 0.0f, head_pError = 0.0f;
float head_output = 0.0f, head_d = 0.0f, head_i = 0.0f;

// ตัวแปรสำหรับการควบคุมการเคลื่อนที่
float current_direction = 0.0f;                // ทิศทางปัจจุบัน (เรเดียน)
int movement_speed = DEFAULT_SPEED;            // ความเร็วการเคลื่อนที่ (0-100)
bool maintain_heading_enabled = true;         // เปิด/ปิดการรักษา heading

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
