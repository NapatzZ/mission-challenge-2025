#include <POP32.h>
#include <SPI.h>
#include <Servo.h>  //esc set=1000 towork 1100-2000
#define CS_PIN PB0  // กำหนดขา Chip Select

// PID Motor Control Variables
const int pulsesPerRev = 24;      // Encoder ให้ 24 พัลส์ต่อรอบ
int setpoint = 150;               // ตั้งเป้า RPM
float Kp = 0.01, Ki = 0.0, Kd = 0.0;

// Adaptive parameters สำหรับความเร็วต่างๆ
const int MIN_PWM = 15;           // PWM ต่ำสุดที่มอเตอร์เริ่มหมุน (dead zone compensation)
const int MAX_PWM = 100;          // PWM สูงสุด
int adaptiveSampleTime = 100;     // Sampling time ที่ปรับได้

unsigned long lastTime = 0;
float integral = 0;
float lastError = 0;
int lastEncoderState = 0;
int tickCount = 0;

// สำหรับ smooth RPM calculation
float rpmFilter = 0;              // Low-pass filtered RPM
const float FILTER_ALPHA = 0.3;   // Filter coefficient (0-1, ยิ่งต่ำยิ่งเรียบ)

// Auto Tuning Variables
bool autoTuning = false;
bool tuningComplete = false;
int tuningStep = 0;
unsigned long tuningStartTime = 0;
float maxRPM = 0;
float minRPM = 999;
float steadyStateRPM = 0;
float overshoot = 0;
unsigned long settlingTime = 0;
float rpmHistory[100];
int historyIndex = 0;
bool firstPeak = true;
unsigned long peakTime = 0;

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
  
  // เริ่มต้นค่า encoder
  lastEncoderState = in(3);
  lastTime = millis();
  
  // เริ่ม Auto Tuning
  startAutoTuning();
  
  // esc.writeMicroseconds(2000);
  
}

void startAutoTuning() {
  autoTuning = true;
  tuningComplete = false;
  tuningStep = 0;
  tuningStartTime = millis();
  maxRPM = 0;
  minRPM = 999;
  historyIndex = 0;
  firstPeak = true;
  
  // เริ่มด้วย P-only control
  Kp = 1.0;
  Ki = 0.0;
  Kd = 0.0;
  
  Serial.println("=== AUTO TUNING STARTED ===");
  Serial.println("Step 1: Finding Ultimate Gain...");
}

void autoTune() {
  unsigned long currentTime = millis() - tuningStartTime;
  
  if (tuningStep == 0) {
    // Step 1: หา Ultimate Gain (Ku) โดยเพิ่ม Kp จนกว่าจะ oscillate
    if (currentTime > 5000) { // ทุก 5 วินาที
      if (checkOscillation()) {
        Serial.print("Ultimate Gain found: Ku = ");
        Serial.println(Kp);
        tuningStep = 1;
        tuningStartTime = millis();
        resetTuningData();
      } else if (Kp < 10.0) {
        Kp += 0.5; // เพิ่ม Kp
        Serial.print("Increasing Kp to: ");
        Serial.println(Kp);
        tuningStartTime = millis();
        resetTuningData();
      } else {
        // ถ้า Kp มากเกินไปแล้วยังไม่ oscillate
        tuningStep = 2;
        tuningStartTime = millis();
      }
    }
  }
  else if (tuningStep == 1) {
    // Step 2: หา Ultimate Period (Tu) จาก oscillation
    if (currentTime > 10000) { // สังเกต 10 วินาที
      float Tu = calculateOscillationPeriod();
      if (Tu > 0) {
        Serial.print("Ultimate Period found: Tu = ");
        Serial.print(Tu);
        Serial.println(" seconds");
        
        // Apply Ziegler-Nichols PID tuning rules
        float Ku = Kp;
        Kp = 0.6 * Ku;
        Ki = (2.0 * Kp) / Tu;
        Kd = (Kp * Tu) / 8.0;
        
        Serial.println("=== ZIEGLER-NICHOLS TUNING COMPLETE ===");
        Serial.print("Final Kp: "); Serial.println(Kp);
        Serial.print("Final Ki: "); Serial.println(Ki);
        Serial.print("Final Kd: "); Serial.println(Kd);
        
        tuningComplete = true;
        autoTuning = false;
      } else {
        tuningStep = 2; // ไปใช้วิธีอื่น
        tuningStartTime = millis();
      }
    }
  }
  else if (tuningStep == 2) {
    // Step 3: ใช้ Step Response Method แทน
    if (currentTime > 8000) { // สังเกต 8 วินาที
      analyzeStepResponse();
      tuningComplete = true;
      autoTuning = false;
    }
  }
}

bool checkOscillation() {
  if (historyIndex < 50) return false;
  
  // ตรวจสอบว่ามี oscillation หรือไม่โดยดูจาก variance
  float sum = 0, mean = 0, variance = 0;
  for (int i = historyIndex - 20; i < historyIndex; i++) {
    sum += rpmHistory[i % 100];
  }
  mean = sum / 20.0;
  
  for (int i = historyIndex - 20; i < historyIndex; i++) {
    variance += pow(rpmHistory[i % 100] - mean, 2);
  }
  variance /= 20.0;
  
  // ถ้า variance มากกว่า threshold แสดงว่ามี oscillation
  return (variance > 100); // ปรับค่าได้ตามต้องการ
}

float calculateOscillationPeriod() {
  if (historyIndex < 80) return 0;
  
  // หา period จากการข้าม mean value
  float sum = 0;
  for (int i = historyIndex - 40; i < historyIndex; i++) {
    sum += rpmHistory[i % 100];
  }
  float mean = sum / 40.0;
  
  int crossings = 0;
  unsigned long firstCrossing = 0, lastCrossing = 0;
  bool above = rpmHistory[(historyIndex - 40) % 100] > mean;
  
  for (int i = historyIndex - 39; i < historyIndex; i++) {
    bool currentAbove = rpmHistory[i % 100] > mean;
    if (currentAbove != above) {
      crossings++;
      if (crossings == 1) firstCrossing = (i - (historyIndex - 40)) * 100; // แต่ละ sample ห่าง 100ms
      lastCrossing = (i - (historyIndex - 40)) * 100;
      above = currentAbove;
    }
  }
  
  if (crossings >= 4) { // อย่างน้อย 2 รอบ
    return (lastCrossing - firstCrossing) / 1000.0 / (crossings / 2.0); // convert to seconds
  }
  return 0;
}

void analyzeStepResponse() {
  // วิเคราะห์การตอบสนองและปรับค่า PID
  float finalValue = steadyStateRPM;
  float overshootPercent = ((maxRPM - finalValue) / finalValue) * 100;
  
  Serial.println("=== STEP RESPONSE ANALYSIS ===");
  Serial.print("Max RPM: "); Serial.println(maxRPM);
  Serial.print("Steady State RPM: "); Serial.println(finalValue);
  Serial.print("Overshoot: "); Serial.print(overshootPercent); Serial.println("%");
  
  // ปรับค่า PID ตาม overshoot
  if (overshootPercent > 20) {
    // Overshoot มากเกินไป ลด Kp
    Kp *= 0.7;
    Ki *= 0.5;
  } else if (overshootPercent < 5) {
    // Response ช้าเกินไป เพิ่ม Kp
    Kp *= 1.3;
    Ki *= 1.2;
  }
  
  // เพิ่ม Kd เล็กน้อยเพื่อลด overshoot
  Kd = Kp * 0.1;
  
  Serial.println("=== TUNING COMPLETE ===");
  Serial.print("Final Kp: "); Serial.println(Kp);
  Serial.print("Final Ki: "); Serial.println(Ki);
  Serial.print("Final Kd: "); Serial.println(Kd);
}

void resetTuningData() {
  maxRPM = 0;
  minRPM = 999;
  historyIndex = 0;
  integral = 0; // Reset integral เพื่อไม่ให้ windup
}

void loop() {
  // 1. นับ tick แบบ simple (detect rising edge) จาก in(3)
  int currentState = in(3);
  if (lastEncoderState == 0 && currentState == 1) {
    tickCount++;
  }
  lastEncoderState = currentState;

  // 2. Adaptive sampling time based on setpoint
  if (setpoint < 100) {
    adaptiveSampleTime = 200;  // ช้าลงสำหรับ RPM ต่ำ
  } else if (setpoint < 150) {
    adaptiveSampleTime = 150;
  } else {
    adaptiveSampleTime = 100;  // เร็วสำหรับ RPM สูง
  }
  
  if (millis() - lastTime >= adaptiveSampleTime) {
    float dt = (millis() - lastTime) / 1000.0;
    lastTime = millis();

    // --- คำนวณ RPM ---
    float rpm_raw = (tickCount / (float)pulsesPerRev) * (60.0 / dt);
    tickCount = 0;
    
    // ป้องกัน division by zero และค่าผิดปกติ
    if (rpm_raw < 0) rpm_raw = 0;
    if (rpm_raw > 1000) rpm_raw = rpmFilter; // ใช้ค่าเก่าถ้าค่าใหม่ผิดปกติ
    
    // Low-pass filter เพื่อทำให้ RPM เรียบขึ้น
    if (rpmFilter == 0) {
      rpmFilter = rpm_raw; // เริ่มต้น
    } else {
      rpmFilter = FILTER_ALPHA * rpm_raw + (1.0 - FILTER_ALPHA) * rpmFilter;
    }
    
    float rpm = rpmFilter;

    // เก็บข้อมูลสำหรับ auto tuning
    if (autoTuning) {
      rpmHistory[historyIndex % 100] = rpm;
      historyIndex++;
      
      if (rpm > maxRPM) maxRPM = rpm;
      if (rpm < minRPM && rpm > 0) minRPM = rpm;
      
      // คำนวณ steady state (เฉลี่ย 10 ค่าล่าสุด)
      if (historyIndex >= 10) {
        float sum = 0;
        for (int i = 0; i < 10; i++) {
          sum += rpmHistory[(historyIndex - 1 - i) % 100];
        }
        steadyStateRPM = sum / 10.0;
      }
      
      autoTune();
    }

    // --- Adaptive PID gains based on setpoint ---
    float adaptiveKp = Kp;
    float adaptiveKi = Ki;
    float adaptiveKd = Kd;
    
    if (setpoint < 100) {
      // สำหรับ RPM ต่ำ เพิ่ม Kp และลด Ki
      adaptiveKp = Kp * 2.0;
      adaptiveKi = Ki * 0.5;
    } else if (setpoint < 150) {
      adaptiveKp = Kp * 1.5;
      adaptiveKi = Ki * 0.8;
    }

    // --- PID ---
    float error = setpoint - rpm;
    integral += error * dt;
    
    // Anti-windup with adaptive limits
    float integralLimit = (setpoint < 100) ? 50 : 100;
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;
    
    float derivative = (error - lastError) / dt;
    float pTerm = adaptiveKp * error;
    float iTerm = adaptiveKi * integral;
    float dTerm = adaptiveKd * derivative;
    float output = pTerm + iTerm + dTerm;
    lastError = error;

    // --- Dead Zone Compensation & Motor Control ---
    int motorSpeed = 0;
    if (abs(output) < 0.1) {
      // ถ้า output เล็กมาก ให้หยุดมอเตอร์
      motorSpeed = 0;
    } else if (output > 0) {
      // Forward direction with dead zone compensation
      motorSpeed = constrain(map(output, 0, 100, MIN_PWM, MAX_PWM), MIN_PWM, MAX_PWM);
    } else {
      // Reverse direction with dead zone compensation  
      motorSpeed = constrain(map(output, -100, 0, -MAX_PWM, -MIN_PWM), -MAX_PWM, -MIN_PWM);
    }
    
    Motor(0, 0, 0, motorSpeed);

    // --- Debug สำหรับ Serial Plotter ---
    if (!autoTuning || tuningComplete) {
      Serial.print(setpoint);        // เส้นเป้าหมาย (สีแดง)
      Serial.print("\t");
      Serial.print(rpm);             // ค่า RPM filtered (สีน้ำเงิน)  
      Serial.print("\t");
      Serial.print(rpm_raw);         // ค่า RPM ดิบ (สีเขียว)
      Serial.print("\t");
      Serial.print(motorSpeed);      // ค่า Motor Speed (สีส้ม)
      Serial.print("\t");
      Serial.print(error);           // Error (สีม่วง)
      Serial.print("\t");
      Serial.println(output);        // PID Output (สีฟ้า)
    }
  }
  
  // อ่านค่าอื่นๆ ที่ต้องการ
  // Serial.println(getDistanceCM(analogRead(0)));
  // PID_wall(30, 2, 0, 0.5, 0);     // wall track
  // PID_Sensor(60,3,1.5,2.35);  // line track
}
