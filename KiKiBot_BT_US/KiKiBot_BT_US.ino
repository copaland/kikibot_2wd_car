// KiKiBot 2WD Car control Bluetooth & Ultra Sound PWM
// Made by Kimson 
// 202500320 ok - 교육용 BT Name KiKiBot_xxx
// select 1 - BT Mode, 2 - Ultra Sens Mode
#include <BluetoothSerial.h>
#include "device_name.h"  // 블루투스 이름 자동 증가 헤더파일

//#define Reset_Count  // 카운트 초기화 끄고 PG
// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
//#if !defined(CONFIG_BT_SPP_ENABLED)
//#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
//#endif

// 핀 정의
#define LED_LEFT 4
#define LED_RIGHT 13
//KIKI-bot V1.2 DRV8837
#define IN1 33
#define IN2 25
#define ENA 32
#define IN3 27
#define IN4 14
#define ENB 26
#define TRIG_PIN 18
#define ECHO_PIN 19
#define KEY 5

#define SPEED_L 220      // 모터 속도 (200-255)
#define SPEED_R 220      // 모터 속도 (200-255)
#define MEASURE_INTERVAL 100 // 거리 측정 간격 (밀리초)

#define MIN_DISTANCE 25      // 최소 안전 거리 (cm)
#define TURN_DURATION 300   // 방향 전환 시간 (밀리초)

uint8_t MODE = 0;
//uint8_t newKey = 0;
//uint8_t oldKey = 0;
float distanceCm;

const int MOT_FREQ = 12; // 10~300kHz
const int RESOLUTION = 8; // 8-bit resolution

unsigned long previousButtonMillis = 0;
unsigned long previousLedMillis = 0;
const int buttonInterval = 20;    // 버튼 스캔 간격 (20ms)
const int ledInterval = 500;      // LED 깜빡임 간격 (200ms)

// 블루투스 관련 변수 device_name.h 자동증가 설정 
///String device_name = "KiKiBot-100"; //BT name 다르게 작성
BluetoothSerial SerialBT;

String btName;
bool isConnected = false;  // 연결 상태 플래그

//#define SCL 22 //I2C LCD
//#define SDA 21 //
//#define LFS1 23 //LINE FOLLOW SENSOR
//#define LFS2 13 //LINE FOLLOW SENSOR
//#define POT 34

float measureDistance();

void motorStop();
void led_blink(uint8_t m);
void goFront(void);
void goBack(void);
void goLeft(void);
void goRight(void);

void goFrontLeft(void);
void goFrontRight(void);
void goBackLeft(void);
void goBackRight(void);
void spinLeft(void);
void spinRight(void);
void motorTest(void);
void bt_action(void);
uint8_t key_scan(void);

void setup() 
{
  //Motor drv8837
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLUP);
  pinMode(KEY, INPUT_PULLUP);

  ledcAttach(IN1, MOT_FREQ, RESOLUTION);
  ledcAttach(IN2, MOT_FREQ, RESOLUTION);
  ledcAttach(IN3, MOT_FREQ, RESOLUTION);
  ledcAttach(IN4, MOT_FREQ, RESOLUTION);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

   // 시리얼 통신 초기화
  Serial.begin(115200);

  // 블루투스 카운트 초기화
  #ifdef Reset_Count
    resetBluetoothCounter();  // 카운트 초기화
  #else
  // 자동으로 증가된 블루투스 이름 가져오기(리셋해도 변경되지 않음)
  btName = getBluetoothName();
  SerialBT.begin(btName);
  Serial.print("Bluetooth enabled: ");
  Serial.println(btName);
  #endif
}

void loop() 
{  
  // 버튼 스캔
  MODE = key_scan();

  unsigned long currentMillis = millis();
  // 250ms마다 LED 깜빡임
  if (currentMillis - previousLedMillis >= ledInterval) {
      previousLedMillis = currentMillis;
      led_blink(MODE + 1); // MODE에 따라 LED 깜빡임 패턴 변경
  }

  // 모드에 따라 동작 실행
  switch (MODE) {
    case 0:
      //led_blink(1);
      bluetoothControl(); // 블루투스 제어
      break;
    case 1:
      //led_blink(2);
      obstacleAvoidance(); // 장애물 회피
      break;
  }

  delay(20);
}

// 블루투스 제어 함수
void bluetoothControl() {
  if(SerialBT.available()) {
    char command = SerialBT.read();
    //Serial.print("Received: ");
    Serial.println(command);

    switch (command) {
      case 'F': goFront(); break;
      case 'B': goBack(); break;
      case 'L': goLeft(); break;
      case 'R': goRight(); break;
      case 'U': spinLeft(); break;
      case 'V': spinRight(); break;
      case 'S': motorStop(); break;  
      default: break;
    }
  }
  if (!SerialBT.connected()) {
    SerialBT.begin(btName); // 자동 재연결 시도
    delay(500);
  }
}


// 장애물 회피 함수
void obstacleAvoidance() {
  distanceCm = measureDistance();
  //Serial.print("거리: ");
  //Serial.print(distanceCm);
  //Serial.println(" cm");
  if (distanceCm <= MIN_DISTANCE) {
    spinRight();//spinLeft();
    delay(TURN_DURATION);
  } else {
    goFront();
    delay(TURN_DURATION);
  }
}

uint8_t key_scan()
{
  static uint8_t mode_cnt=0;
  static uint8_t oldKey=0;
  
  uint8_t newKey = !digitalRead(KEY);

  if((newKey != oldKey) && newKey){ // if KEY pressed
    //motorStop();
    mode_cnt++;
    mode_cnt = mode_cnt % 2; //MODE 0:BT, MODE 1:Ultra Sens, MODE 2: motor test
    delay(1);
  }
  oldKey = newKey;
  return mode_cnt; //newKey;
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);//give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  float duration = pulseIn(ECHO_PIN, HIGH);
  float cm = duration * 0.034 / 2.0;     //m/s->cm/us (340*duration)/10000/2.0;
  return cm;
}

void led_blink(uint8_t m) {
  static bool ledState = LOW;
  ledState = !ledState;
    
  switch(m) {
    case 1: // 동시에 깜박임
      digitalWrite(LED_LEFT, ledState);
      digitalWrite(LED_RIGHT, ledState);
      break;
    case 2: // 번갈아 깜빡임
      digitalWrite(LED_LEFT, ledState);
      digitalWrite(LED_RIGHT, !ledState);
      break;
    //case 3:
    //  digitalWrite(LED_LEFT, HIGH);
    //  digitalWrite(LED_RIGHT, HIGH);
    //  break;
    default:
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
      break;
  }
}

// 모터 제어 함수들
void goFront() {
  ledcWrite(IN1, 0); ledcWrite(IN2, SPEED_L);
  ledcWrite(IN3, 0); ledcWrite(IN4, SPEED_R); 
}

void goBack() {
  ledcWrite(IN1, SPEED_L); ledcWrite(IN2, 0); 
  ledcWrite(IN3, SPEED_R); ledcWrite(IN4, 0); 
}

void goLeft() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 0);
  ledcWrite(IN3, 0); ledcWrite(IN4, SPEED_R);
}

void goRight() {
  ledcWrite(IN1, 0); ledcWrite(IN2, SPEED_L);
  ledcWrite(IN3, 0); ledcWrite(IN4, 0);
}

void spinLeft() {
  ledcWrite(IN1, SPEED_L); ledcWrite(IN2, 0); 
  ledcWrite(IN3, 0); ledcWrite(IN4, SPEED_R);
}

void spinRight() {
  ledcWrite(IN1, 0); ledcWrite(IN2, SPEED_L);
  ledcWrite(IN3, SPEED_R); ledcWrite(IN4, 0);
}

void motorStop() {
   ledcWrite(IN1, 0); ledcWrite(IN2, 0); 
   ledcWrite(IN3, 0); ledcWrite(IN4, 0);      
}
