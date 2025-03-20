// KiKiBot 2WD Car Bluetooth control program 용 
// Made by Kimson 
// 202500318 ok - 교육용 PWM
#include <Arduino.h>
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

uint8_t MODE = 0;
//uint8_t newKey = 0;
//uint8_t oldKey = 0;
//float distanceCm;

unsigned long previousButtonMillis = 0;
unsigned long previousLedMillis = 0;
const int buttonInterval = 20;    // 버튼 스캔 간격 (20ms)
const int ledInterval = 500;      // LED 깜빡임 간격 (200ms)
// 상수 정의
const int ULTRASONIC_THRESHOLD = 25; // cm
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

const int ledFrequency = 100; // 5kHz
const int ledResolution = 8; // 8-bit resolution

const int baseSpeedL = 150;  // 좌측 모터 기본 속도 (0~4095)
const int baseSpeedR = 150;  // 우측 모터 기본 속도 (0~4095)

uint8_t X;  // 회전 여부를 저장하는 변수

int measureDistance();

void stopMotors();
void led_blink(uint8_t m);
void goForward(void);
void goBack(void);
void goLeft(void);
void goRight(void);

void goForwardLeft(void);
void goForwardRight(void);
void goBackLeft(void);
void goBackRight(void);
void rotateLeft(void);
void rotateRight(void);
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

    // resolution 1-16 bits, freq limits depend on resolution, channel is automatically selected
  ledcAttach(IN1, ledFrequency, ledResolution);  // 12 kHz PWM, 8-bit resolution
  ledcAttach(IN2, ledFrequency, ledResolution);
  ledcAttach(IN3, ledFrequency, ledResolution);  // 12 kHz PWM, 8-bit resolution
  ledcAttach(IN4, ledFrequency, ledResolution);

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
      bluetoothControl(); // 블루투스 제어
      break;
    case 1:
      stopMotors(); // 모터 정지
      break;
    case 2:
      obstacleAvoidance(); // 장애물 회피
      switch (X) {
        case 1: goForward(); break;
        case 2: rotateRight(); break;  
      }
      break;
    case 3:
      stopMotors(); // 모터 정지
      break;            
  }
  delay(100);
}

// 블루투스 제어 함수
void bluetoothControl() {
  if(SerialBT.available()) {
    char command = SerialBT.read();
    //Serial.print("Received: ");
    Serial.println(command);

    switch (command) {
      case 'F': goForward(); break;
      case 'B': goBack(); break;
      case 'L': goLeft(); break;
      case 'R': goRight(); break;
      case 'U': rotateLeft(); break;
      case 'V': rotateRight(); break;
      case 'S': stopMotors(); break;
      //case 'H': goForwardLeft(); break;
      //case 'K': goForwardRight(); break;
      //case 'N': goBackLeft(); break;
      //case 'M': goBackRight(); break;      
      default: break;
    }
  }
  if (!SerialBT.connected()) {
    SerialBT.begin(btName); // 자동 재연결 시도
    delay(500);
  }
}

int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);//give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 타임아웃 추가 (30ms)
  int cm = (int)(duration * 0.034 / 2.0);     //m/s->cm/us (340*duration)/10000/2.0;
  return cm;
}

// 장애물 회피 함수
void obstacleAvoidance() {
  int distanceCm = measureDistance();
  //Serial.print("거리: ");
  Serial.println(distanceCm);
  //Serial.println(" cm");
  if (distanceCm <= ULTRASONIC_THRESHOLD) {
    X = 2; // 회전
  } else {
    X = 1; // 장애물 없으면 계속 직진
  }
  delay(100); // 측정 간격 확보(최소60ms 이상)
}

uint8_t key_scan()
{
  static uint8_t mode_cnt=0;
  static uint8_t oldKey=0;
  
  uint8_t newKey = !digitalRead(KEY);

  if((newKey != oldKey) && newKey){ // if KEY pressed
    //stopMotors();
    mode_cnt++;
    mode_cnt = mode_cnt % 4; //MODE 0:BT, MODE 1:Ultra Sens, MODE 2: motor test
    delay(1);
  }
  oldKey = newKey;
  return mode_cnt; //newKey;
}



void led_blink(uint8_t m) {
  static bool ledState = LOW;
  ledState = !ledState;
    
  switch(m) {
    case 1: // 동시에 깜박임
      digitalWrite(LED_LEFT, ledState);
      digitalWrite(LED_RIGHT, ledState);
      break;
    case 2: //계속 켜짐
      digitalWrite(LED_LEFT, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      break;
    case 3: // 번갈아 깜빡임
      digitalWrite(LED_LEFT, ledState);
      digitalWrite(LED_RIGHT, !ledState);
      break;
    case 4: // 
      digitalWrite(LED_LEFT, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      break;
    default:
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
      break;
  }
}

// 모터 제어 함수들
void goForward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(IN2, baseSpeedL);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(IN4, baseSpeedR); 
}

void goBack() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(IN1, baseSpeedL);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(IN3, baseSpeedR);
}

void goLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(IN4, baseSpeedR); 
}

void goRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(IN2, baseSpeedL);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
}

void rotateLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(IN1, baseSpeedL);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(IN4, baseSpeedR);
}

void rotateRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(IN2, baseSpeedL);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(IN3, baseSpeedR);
}

void stopMotors() {
   digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); 
   digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);       
}
