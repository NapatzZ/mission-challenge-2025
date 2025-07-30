#include <POP32.h>
#include <SPI.h>
#include <Servo.h>  //esc set=1000 towork 1100-2000
#define CS_PIN PB0  // กำหนดขา Chip Select

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
  
  // esc.writeMicroseconds(2000);
  
}


void loop() {
  PID_wall(30, 2, 0, 0.5, 0);     // wall track
  // PID_Sensor(60,3,1.5,2.35);  // line track
}
