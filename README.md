# 🤖 Mecanum Robot Control System

ระบบควบคุมหุ่นยนต์ล้อ Mecanum ด้วย POP32 พร้อมการรักษาทิศทาง (Heading Control) โดยใช้ ZX-IMU และ Accelerometer MC34X9

---

## 📋 สารบัญ

1. [ภาพรวมของระบบ](#ภาพรวมของระบบ)
2. [ฮาร์ดแวร์ที่ใช้](#ฮาร์ดแวร์ที่ใช้)
3. [โครงสร้างไฟล์](#โครงสร้างไฟล์)
4. [การติดตั้งและการตั้งค่า](#การติดตั้งและการตั้งค่า)
5. [วิธีการใช้งาน](#วิธีการใช้งาน)
6. [API Reference](#api-reference)
7. [ตัวอย่างการใช้งาน](#ตัวอย่างการใช้งาน)
8. [หลักการทำงาน](#หลักการทำงาน)
9. [การแก้ปัญหา](#การแก้ปัญหา)

---

## 🔍 ภาพรวมของระบบ

ระบบนี้ออกแบบมาเพื่อควบคุมหุ่นยนต์ล้อ Mecanum ให้สามารถ:

- **เคลื่อนที่ในทิศทางใดก็ได้** (omnidirectional movement)
- **รักษาทิศทางหัวหุ่นยนต์** ขณะเคลื่อนที่ด้วย PID Control
- **วัดค่าความเอียง** ของหุ่นยนต์ด้วย Accelerometer
- **ทำงานแบบ Real-time** ด้วยการอัปเดต 20Hz (ทุก 50ms)

### ✨ ฟีเจอร์หลัก

- 🎯 **การรักษา Heading แบบอัตโนมัติ** - หุ่นยนต์จะรักษาทิศทางหัวไว้ตามที่กำหนด
- 🧭 **การเคลื่อนที่ 8 ทิศทาง** - รองรับการเคลื่อนที่แบบ omnidirectional
- 📐 **การวัดความเอียง** - รองรับการวัด Pitch และ Roll
- 🔧 **ปรับแต่งได้ง่าย** - สามารถปรับค่า PID และความเร็วได้
- 🧪 **ฟังก์ชันทดสอบ** - มีฟังก์ชันทดสอบการทำงานครบถ้วน

---

## 🛠 ฮาร์ดแวร์ที่ใช้

### หลัก
- **POP32** - บอร์ดควบคุมหลัก (STM32F103)
- **ZX-IMU** - เซ็นเซอร์วัดมุม Yaw (UART)
- **MC34X9** - เซ็นเซอร์ Accelerometer (I2C)
- **Mecanum Wheels (4 ล้อ)** - ล้อสำหรับการเคลื่อนที่แบบ omnidirectional

### การต่อสาย

#### ZX-IMU (UART)
```
ZX-IMU    ->    POP32
TX        ->    RX1 (PA10)
RX        ->    TX1 (PA9) 
VCC       ->    3.3V
GND       ->    GND
```

#### MC34X9 (I2C)
```
MC34X9    ->    POP32
SCL       ->    SCL (I2C)
SDA       ->    SDA (I2C)
VCC       ->    3.3V
GND       ->    GND
```

#### Mecanum Motors
```
Motor 1: Front Left  -> motor(1)
Motor 2: Back Left   -> motor(2)  
Motor 3: Front Right -> motor(3)
Motor 4: Back Right  -> motor(4)
```

---

## 📁 โครงสร้างไฟล์

```
Mision_2025/
├── Mision_2025.ino     # ไฟล์หลัก - การตั้งค่าและ main loop
├── kinematick.ino      # ระบบควบคุมการเคลื่อนที่ (เดิมชื่อ ball_zone.ino)
├── ZXimu.ino          # ระบบ ZX-IMU และ PID heading control
├── accelerometer.ino   # ระบบ Accelerometer และการวัดความเอียง (เดิมชื่อ sensor_fusion.ino)
├── PID.ino            # ฟังก์ชันพื้นฐานสำหรับมอเตอร์
├── motor.ino          # ฟังก์ชันควบคุมมอเตอร์เพิ่มเติม
├── wall.ino           # ระบบติดตามผนัง
├── irSensor.ino       # เซ็นเซอร์ IR และ ADC (เดิมชื่อ sensor.ino)
├── fan.ino            # ระบบพัดลม
├── state.ino          # การจัดการ state ของระบบ
└── README.md          # คู่มือการใช้งาน
```

### รายละเอียดไฟล์

#### 🎮 `Mision_2025.ino` (Main File)
- การตั้งค่าหลักของโปรเจค
- การกำหนดค่าคงที่และตัวแปร global
- ฟังก์ชัน `setup()` และ `loop()` หลัก

#### 🚀 `kinematick.ino` (Kinematics & Movement Control)
- ฟังก์ชันการเคลื่อนที่ทุกทิศทาง
- ระบบรักษา heading ขณะเคลื่อนที่
- ฟังก์ชันทดสอบการเคลื่อนที่
- คำนวณ kinematics สำหรับล้อ Mecanum

#### 🧭 `ZXimu.ino` (IMU & Heading Control)
- การอ่านค่าจาก ZX-IMU
- ระบบ PID สำหรับรักษา heading
- การรีเซ็ตและปรับเทียบ IMU

#### 📐 `accelerometer.ino` (Accelerometer & Tilt Sensing)
- การอ่านค่าจาก MC34X9
- การคำนวณ Pitch และ Roll
- การวัดความเร่งใน 3 แกน

#### 🔧 `motor.ino` (Motor Control)
- ฟังก์ชันควบคุมมอเตอร์เพิ่มเติม
- การจัดการความเร็วและทิศทาง

#### 📡 `irSensor.ino` (IR Sensors & ADC)
- การอ่านค่าจากเซ็นเซอร์ IR
- การใช้งาน ADC ผ่าน SPI

---

## ⚙️ การติดตั้งและการตั้งค่า

### 1. การติดตั้ง Libraries
```cpp
#include <POP32.h>     // สำหรับ POP32
#include <Wire.h>      // สำหรับ I2C
#include <math.h>      // สำหรับฟังก์ชันคณิตศาสตร์
```

### 2. การกำหนดค่าใน `Mision_2025.ino`

```cpp
// ความเร็วเริ่มต้น
#define DEFAULT_SPEED 50          // 0-100

// การตั้งค่า PID
#define HEAD_KP 1.0f    // Proportional gain
#define HEAD_KI 0.0f    // Integral gain  
#define HEAD_KD 0.1f    // Derivative gain

// อัตราการอัปเดต
#define PID_UPDATE_RATE 50       // มิลลิวินาที
```

### 3. การเริ่มต้นระบบ

```cpp
void setup() {
  Serial.begin(115200);
  
  // เริ่มต้นเซ็นเซอร์
  setup_accelerometer();   // MC34X9
  setupIMU();             // ZX-IMU
  
  // รีเซ็ต heading
  resetCurrentHeading();
}
```

---

## 🎮 วิธีการใช้งาน

### การเคลื่อนที่พื้นฐาน

#### 1. การเคลื่อนที่ในทิศทางหลัก
```cpp
// เคลื่อนที่ไปข้างหน้าด้วยความเร็ว 60
moveForward(60);

// เคลื่อนที่ไปข้างหลังด้วยความเร็ว 50  
moveBackward(50);

// เคลื่อนที่ไปซ้าย
moveLeft(40);

// เคลื่อนที่ไปขวา
moveRight(45);
```

#### 2. การเคลื่อนที่แบบเฉียง
```cpp
// เคลื่อนที่ไปมุม 45 องศา (ข้างหน้าขวา)
move45Degrees(50);

// เคลื่อนที่ไปมุม 135 องศา (ข้างหลังซ้าย)  
move135Degrees(50);
```

#### 3. การเคลื่อนที่ในทิศทางกำหนดเอง
```cpp
// เคลื่อนที่ไป 30 องศา พร้อมรักษา heading ที่ 0 องศา
float direction = 30.0f * PI / 180.0f;  // แปลงเป็นเรเดียน
moveInDirection(direction, 50, 0.0f);

// เคลื่อนที่ไป 120 องศา พร้อมรักษา heading ที่ 45 องศา
moveInDirection(2.0f*PI/3.0f, 60, PI/4.0f);
```

### การควบคุม Heading

#### 1. การหมุนไปมุมต่างๆ
```cpp
// หมุนไป 0 องศา
turnTo0Degrees();

// หมุนไป 90 องศา
turnTo90Degrees();

// หมุนไปมุมกำหนดเอง (60 องศา)
turnToAngle(PI/3.0f);
```

#### 2. การรักษา heading ขณะหยุดนิ่ง
```cpp
void loop() {
  if (readIMUHeadingSimple()) {
    maintainHeading(0.0f, 0);  // รักษา heading ที่ 0 องศา
    printIMUData();           // แสดงข้อมูล IMU
  }
  delay(50);
}
```

### การใช้งานแบบต่อเนื่อง

#### 1. เคลื่อนที่ในทิศทางเดียวเป็นเวลานาน
```cpp
// เคลื่อนที่ไปข้างหน้า 3 วินาที
moveToDirection(NORTH, 50, 3000);

// เคลื่อนที่ไป 45 องศา เป็นเวลา 2 วินาที
moveToDirection(PI/4.0f, 40, 2000);
```

#### 2. การทดสอบการเคลื่อนที่ทุกทิศทาง
```cpp
void loop() {
  testDirectionalMovement();  // ทดสอบทุกทิศทาง
  delay(5000);               // รอ 5 วินาที
}
```

---

## 📚 API Reference

### 🚀 Kinematics Functions (`kinematick.ino`)

#### การเคลื่อนที่พื้นฐาน
```cpp
void moveForward(int speed);           // เคลื่อนที่ไปข้างหน้า (0°)
void moveBackward(int speed);          // เคลื่อนที่ไปข้างหลัง (180°)
void moveLeft(int speed);              // เคลื่อนที่ไปซ้าย (90°)
void moveRight(int speed);             // เคลื่อนที่ไปขวา (-90°)
void move45Degrees(int speed);         // เคลื่อนที่ไปมุม 45°
void move135Degrees(int speed);        // เคลื่อนที่ไปมุม 135°
```

#### การเคลื่อนที่ขั้นสูง
```cpp
void moveInDirection(float direction, int speed, float target_heading);
// direction: ทิศทางในเรเดียน (0=หน้า, PI/2=ซ้าย, PI=หลัง, -PI/2=ขวา)
// speed: ความเร็ว (0-100)
// target_heading: มุมหัวหุ่นยนต์ที่ต้องการรักษา (เรเดียน)

void moveToDirection(float direction, int speed, unsigned long duration);
// duration: ระยะเวลาการเคลื่อนที่ (มิลลิวินาที)
```

#### การควบคุมการตั้งค่า
```cpp
void enableHeadingControl(bool enable);    // เปิด/ปิดการรักษา heading
void setDefaultSpeed(int speed);           // ตั้งค่าความเร็วเริ่มต้น
void stopMovement();                       // หยุดการเคลื่อนที่ทั้งหมด
```

#### การทดสอบ
```cpp
void testDirectionalMovement();            // ทดสอบการเคลื่อนที่ทุกทิศทาง
void printMovementStatus();                // แสดงสถานะการเคลื่อนที่
```

### 🧭 ZX-IMU Functions (`ZXimu.ino`)

#### การตั้งค่าและรีเซ็ต
```cpp
void setupIMU();                           // ตั้งค่าเริ่มต้น ZX-IMU
void resetCurrentHeading();                // รีเซ็ต heading ปัจจุบันเป็น 0°
void setInitialHeading();                  // ตั้งค่ามุมปัจจุบันเป็นมุมเริ่มต้น
```

#### การอ่านข้อมูล
```cpp
bool readIMUHeadingSimple();               // อ่านค่า heading (แนะนำ)
bool readIMUHeading();                     // อ่านค่า heading (debug mode)
float getRelativeHeading();                // ได้ค่ามุมที่ปรับตาม initial heading
```

#### การควบคุม Heading
```cpp
void maintainHeading(float targetTheta, int speed);  // รักษา heading
void holdCurrentHeading();                           // รักษา heading ปัจจุบัน
void setPIDGains(float kp, float ki, float kd);     // ตั้งค่า PID
```

#### การหมุน
```cpp
void turnToAngle(float target_angle);      // หมุนไปมุมที่กำหนด (เรเดียน)
void turnTo0Degrees();                     // หมุนไป 0°
void turnTo45Degrees();                    // หมุนไป 45°
void turnTo90Degrees();                    // หมุนไป 90°
void turnTo135Degrees();                   // หมุนไป 135°
void turnTo180Degrees();                   // หมุนไป 180°
```

#### การทดสอบ
```cpp
void testHeadingControl(unsigned long testDuration);  // ทดสอบการรักษา heading
void testRotationControl();                           // ทดสอบการหมุน
void printIMUData();                                  // แสดงข้อมูล IMU
```

### 📐 Accelerometer Functions (`accelerometer.ino`)

#### การตั้งค่า
```cpp
void setup_accelerometer();               // ตั้งค่าเริ่มต้น MC34X9
```

#### การอ่านข้อมูล
```cpp
void read_acceleration();                 // อ่านค่าความเร่ง 3 แกน
void calculate_tilt();                    // คำนวณ Pitch และ Roll
void measure_tilt_and_accel();           // อ่านและคำนวณทั้งหมด
```

### 🔧 Utility Functions (`PID.ino`)

```cpp
void Motor(int sp1, int sp2, int sp3, int sp4);  // ควบคุมมอเตอร์ 4 ตัว
void Motor_Stop();                               // หยุดมอเตอร์ทั้งหมด
```

---

## 💡 ตัวอย่างการใช้งาน

### ตัวอย่างที่ 1: การเคลื่อนที่พื้นฐาน

```cpp
void loop() {
  // เคลื่อนที่ไปข้างหน้า 2 วินาที
  moveToDirection(NORTH, 60, 2000);
  
  // เคลื่อนที่ไปขวา 2 วินาที  
  moveToDirection(WEST, 50, 2000);
  
  // เคลื่อนที่ไปข้างหลัง 2 วินาที
  moveToDirection(SOUTH, 60, 2000);
  
  // เคลื่อนที่ไปซ้าย 2 วินาที
  moveToDirection(EAST, 50, 2000);
  
  delay(1000);
}
```

### ตัวอย่างที่ 2: การเคลื่อนที่แบบเฉียงพร้อมรักษา heading

```cpp
void loop() {
  // เคลื่อนที่ไปมุม 30° พร้อมรักษาหัวที่ 0°
  float angle_30 = 30.0f * PI / 180.0f;
  moveToDirection(angle_30, 50, 3000);
  
  // เคลื่อนที่ไปมุม 120° พร้อมรักษาหัวที่ 45°  
  float angle_120 = 120.0f * PI / 180.0f;
  float heading_45 = 45.0f * PI / 180.0f;
  
  // หมุนหัวไป 45° ก่อน
  turnToAngle(heading_45);
  delay(1000);
  
  // จึงค่อยเคลื่อนที่
  moveInDirection(angle_120, 50, heading_45);
  delay(3000);
  
  stopMovement();
  delay(2000);
}
```

### ตัวอย่างที่ 3: การรักษา heading ขณะรับคำสั่งจาก Serial

```cpp
void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    
    switch(command) {
      case 'w': moveForward(60); break;      // ไปข้างหน้า
      case 's': moveBackward(60); break;     // ไปข้างหลัง  
      case 'a': moveLeft(60); break;         // ไปซ้าย
      case 'd': moveRight(60); break;        // ไปขวา
      case 'q': move45Degrees(60); break;    // ไปมุม 45°
      case 'e': move135Degrees(60); break;   // ไปมุม 135°
      case ' ': stopMovement(); break;       // หยุด
      case 'r': resetCurrentHeading(); break; // รีเซ็ต heading
    }
  }
  
  // แสดงสถานะ
  if (readIMUHeadingSimple()) {
    printIMUData();
  }
  
  delay(50);
}
```

### ตัวอย่างที่ 4: การทดสอบระบบ

```cpp
void setup() {
  Serial.begin(115200);
  setup_accelerometer();
  setupIMU();
  
  // รอให้ผู้ใช้กดปุ่ม
  waitAnykey_bmp();
  
  // ทดสอบการหมุน
  testRotationControl();
  delay(2000);
  
  // ทดสอบการเคลื่อนที่
  testDirectionalMovement();
}

void loop() {
  // รักษา heading ที่ 0°
  if (readIMUHeadingSimple()) {
    maintainHeading(0.0f, 0);
    printIMUData();
  }
  
  // แสดงข้อมูลความเอียง
  measure_tilt_and_accel();
  Serial.print("Pitch: ");
  Serial.print(pitch_deg, 1);
  Serial.print("° Roll: ");
  Serial.print(roll_deg, 1);
  Serial.println("°");
  
  delay(100);
}
```

---

## 🔬 หลักการทำงาน

### การควบคุมล้อ Mecanum

ล้อ Mecanum ทำงานโดยการหมุนล้อแต่ละตัวด้วยความเร็วที่แตกต่างกัน:

```
การเคลื่อนที่ไปข้างหน้า:
Motor1(+) Motor2(+) Motor3(+) Motor4(+)

การเคลื่อนที่ไปซ้าย:  
Motor1(+) Motor2(-) Motor3(-) Motor4(+)

การหมุนขวา:
Motor1(-) Motor2(-) Motor3(+) Motor4(+)
```

สูตรการคำนวณความเร็วมอเตอร์:
```cpp
float sin_dir = sin(direction + PI/4.0f);
float cos_dir = cos(direction + PI/4.0f);

motor1_speed = speed * sin_dir;  // Front Left
motor2_speed = speed * cos_dir;  // Back Left  
motor3_speed = speed * cos_dir;  // Front Right
motor4_speed = speed * sin_dir;  // Back Right
```

### การรักษา Heading ด้วย PID Control

ระบบใช้ PID Controller เพื่อรักษาทิศทางหัวหุ่นยนต์:

```cpp
// คำนวณ Error
head_error = target_heading - current_heading;

// PID Terms
P = head_error * Kp;                    // Proportional
I = integral_error * Ki;                // Integral  
D = (head_error - previous_error) * Kd; // Derivative

// PID Output
pid_output = P + I + D;

// ปรับค่ามอเตอร์เพื่อหมุน
motor1_speed -= pid_output;  // Front Left
motor2_speed -= pid_output;  // Back Left
motor3_speed += pid_output;  // Front Right
motor4_speed += pid_output;  // Back Right
```

### ระบบ Sensor Fusion

รวมข้อมูลจาก 2 เซ็นเซอร์:

1. **ZX-IMU**: วัดมุม Yaw (การหมุนรอบแกน Z)
2. **MC34X9**: วัดความเร่ง 3 แกน และคำนวณ Pitch/Roll

```cpp
// คำนวณมุมเอียง
Roll = atan2(accelY, accelZ) * 180/PI;
Pitch = atan2(-accelX, sqrt(accelY² + accelZ²)) * 180/PI;
```

---

## 🛠 การแก้ปัญหา

### ปัญหาที่พบบ่อย

#### 1. IMU ไม่อ่านค่า
```
อาการ: Raw Yaw ไม่อัปเดต
แก้ไข:
- ตรวจสายต่อ UART (TX->RX1, RX->TX1)
- ตรวจ Baud rate (115200)
- เรียกใช้ testSerialData() เพื่อดูข้อมูลดิบ
```

#### 2. หุ่นยนต์ไม่รักษา heading
```
อาการ: หุ่นยนต์เดินถูกทิศทางแต่หัวหมุน
แก้ไข:
- เช็คว่า maintain_heading_enabled = true
- ปรับค่า PID: เพิ่ม Kp หรือเพิ่ม Kd
- เรียกใช้ resetCurrentHeading() ก่อนใช้งาน
```

#### 3. การเคลื่อนที่ไม่ถูกทิศทาง
```
อาการ: สั่งไปหน้าแต่ไปข้าง
แก้ไข:
- ตรวจการต่อสายมอเตอร์
- ตรวจทิศทางการหมุนของมอเตอร์
- ปรับลำดับมอเตอร์ใน Motor() function
```

#### 4. Accelerometer ไม่ทำงาน
```
อาการ: Pitch/Roll เป็น 0 ตลอด
แก้ไข:
- ตรวจสาย I2C (SDA, SCL)
- ตรวจ I2C Address (0x6C)
- เรียกใช้ setup_accelerometer() ใน setup()
```

### การ Debug

#### ใช้ Serial Monitor เพื่อดูข้อมูล:
```cpp
void debugSystem() {
  Serial.println("=== System Debug ===");
  
  // Debug IMU
  if (readIMUHeadingSimple()) {
    printIMUData();
  } else {
    Serial.println("IMU not responding");
  }
  
  // Debug Accelerometer  
  measure_tilt_and_accel();
  Serial.print("Accel: X=");
  Serial.print(accelX_g, 3);
  Serial.print(" Y=");
  Serial.print(accelY_g, 3);
  Serial.print(" Z=");
  Serial.println(accelZ_g, 3);
  
  // Debug Movement
  printMovementStatus();
}
```

#### การทดสอบระบบทีละส่วน:
```cpp
// ทดสอบ IMU อย่างเดียว
void testIMUOnly() {
  while(1) {
    if (readIMUHeadingSimple()) {
      maintainHeading(0, 0);  // รักษา heading ที่ 0°
      printIMUData();
    }
    delay(50);
  }
}

// ทดสอบการเคลื่อนที่อย่างเดียว (ไม่รักษา heading)
void testMovementOnly() {
  enableHeadingControl(false);  // ปิดการรักษา heading
  
  moveForward(50);
  delay(2000);
  
  moveRight(50);
  delay(2000);
  
  stopMovement();
}
```

---

## 📋 การปรับแต่งค่าต่างๆ

### ค่า PID สำหรับการรักษา Heading

```cpp
// ในไฟล์ Mision_2025.ino
#define HEAD_KP 1.0f    // เพิ่มถ้าต้องการตอบสนองเร็วขึ้น
#define HEAD_KI 0.0f    // เพิ่มถ้ามี steady-state error
#define HEAD_KD 0.1f    // เพิ่มถ้ามี overshoot

// หรือปรับขณะ runtime
setPIDGains(1.5f, 0.1f, 0.2f);
```

### ความเร็วและการตอบสนอง

```cpp
#define DEFAULT_SPEED 50         // ความเร็วเริ่มต้น
#define PID_UPDATE_RATE 50       // ความถี่การอัปเดต PID (ms)
```

### ค่าจำกัดต่างๆ

```cpp
// ใน ball_zone.ino - ฟังก์ชัน moveInDirection()
pid_output = constrain(pid_output, -30, 30);  // จำกัด PID output
motor_speed = constrain(motor_speed, -100, 100); // จำกัดความเร็วมอเตอร์
```

---

## 📞 การสนับสนุน

หากมีปัญหาหรือข้อสงสัย:

1. ตรวจสอบ [การแก้ปัญหา](#การแก้ปัญหา) ก่อน
2. ใช้ฟังก์ชัน debug เพื่อตรวจสอบระบบ
3. ตรวจสอบการต่อสายและการตั้งค่า

---

**POP32 Mecanum Robot**

---
