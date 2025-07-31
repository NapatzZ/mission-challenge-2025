#include <POP32.h>
#include <SPI.h>
#include <Servo.h>  // ESC: set = 1000 to start, 1100-2000 for operation
#define CS_PIN PB0  // Chip Select pin definition
#include <Wire.h>   // Library for I2C communication
#include <math.h>   // Math functions like atan2

// =============================================================================
// PROJECT CONFIGURATION 
// =============================================================================

// --- Constants for Accelerometer Sensor MC34X9 ---
#define I2C_ADDRESS 0x6C        // I2C address of MC34X9 sensor
#define REG_OUT_X_LSB 0x0D      // Register for reading X-axis (LSB)
#define REG_OUT_Y_LSB 0x0F      // Register for reading Y-axis (LSB)
#define REG_OUT_Z_LSB 0x11      // Register for reading Z-axis (LSB)
#define LSB_TO_G 0.000061       // Constant to convert LSB to g (gravity unit)

// --- Constants for movement direction (in radians) ---
#define NORTH     0.0f             // 0°   - forward
#define NORTHEAST (PI / 4.0f)      // 45°  - forward-left
#define EAST      (PI / 2.0f)      // 90°  - left
#define SOUTHEAST (3.0f * PI / 4.0f) // 135° - backward-left
#define SOUTH     PI               // 180° - backward
#define SOUTHWEST (-3.0f * PI / 4.0f) // -135° - backward-right
#define WEST      (-PI / 2.0f)     // -90° - right
#define NORTHWEST (-PI / 4.0f)     // -45° - forward-right

// --- Default system settings ---
#define DEFAULT_SPEED 50          // Default movement speed (0-100)
#define IMU_BAUD_RATE 115200      // Baud rate for ZX-IMU
#define PID_UPDATE_RATE 50        // PID update rate (ms)

// --- PID settings for maintaining heading ---
#define HEAD_KP 1.0f    // Proportional gain
#define HEAD_KI 0.0f    // Integral gain  
#define HEAD_KD 0.1f    // Derivative gain

// =============================================================================
// GLOBAL VARIABLES 
// =============================================================================

// Accelerometer sensor variables
float accelX_g, accelY_g, accelZ_g;
float pitch_deg, roll_deg;

// ZX-IMU variables
uint8_t rxBuf[8], rxCnt = 0;
float pvYaw = 0.0f;        // Current yaw (degrees)
float initialYaw = 0.0f;   // Initial yaw (degrees)
float targetYaw = 0.0f;    // Target yaw (degrees)

// PID control variables
float head_Kp = HEAD_KP;
float head_Ki = HEAD_KI;  
float head_Kd = HEAD_KD;
float head_error = 0.0f, head_pError = 0.0f;
float head_output = 0.0f, head_d = 0.0f, head_i = 0.0f;

// Movement control variables
float current_direction = 0.0f;          // Current direction (radians)
int movement_speed = DEFAULT_SPEED;     // Movement speed (0-100)
bool maintain_heading_enabled = true;   // Enable/disable heading correction

float last_error = 0;
Servo esc;
int i = 0;

void setup() {
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  esc.attach(PB10);  // Servo/ESC signal pin
  esc.writeMicroseconds(1000);  // Initialize ESC
  SPI.begin();  // PA5=SCK, PA6=MISO, PA7=MOSI (STM32 SPI1)
  
  waitAnykey_bmp();
  setup_accelerometer();
  beep(1000);
  setupIMU();

  motor(50, 50, -50, 50);  // Move motors
  sleep(10000);
  AO();                    // Stop all motors
  sleep(300);
  moveToDirection(0, 30, 10000);  // Move forward at speed 30 for 10 seconds

  // esc.writeMicroseconds(2000);  // Max speed test (optional)
}

void loop() {
  // Use improved function that syncs better
  if (readIMUHeadingSimple()) {  // <-- Changed from readIMUHeading()
    maintainHeading(0, 40);      // Try to maintain yaw = 0° at speed 40
    printIMUData();              // Print current IMU data
  }
  delay(50);

  // --- Optional Debug Section ---
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

  // PID_wall(30, 2, 0, 0.5, 0);     // Wall tracking (commented out)
  // PID_Sensor(60, 3, 1.5, 2.35);   // Line tracking (commented out)
}
