/*
20250320 KiKiBot 2WD Car
ESP32 ULTRA SOUND SENSOR 주행 OK
DIGITAL PWM
made by KiMSON
*/
#include <Arduino.h>

// 모터 핀 정의
#define IN1 33      // 모터 A 방향 제어 1
#define IN2 25      // 모터 A 방향 제어 2
#define ASLEEP 32   // 모터 A 슬립 모드 제어
#define IN3 27      // 모터 B 방향 제어 1
#define IN4 14      // 모터 B 방향 제어 2
#define BSLEEP 26   // 모터 B 슬립 모드 제어
#define TRIG_PIN 18 // 초음파 센서 트리거 핀
#define ECHO_PIN 19 // 초음파 센서 에코 핀
#define KEY 5       // 키 입력 핀
#define LED 4       // 상태 LED

// 상수 정의
#define MIN_DISTANCE 25      // 최소 안전 거리 (cm)
#define TURN_DURATION 300   // 방향 전환 시간 (밀리초)

#define MOTOR_LSPEED 220      // 모터 속도 (200-255)
#define MOTOR_RSPEED 220      // 모터 속도 (200-255)
#define MEASURE_INTERVAL 100 // 거리 측정 간격 (밀리초)

const int MOT_FREQ = 12; // 10~300kHz
const int MOT_RESOLUTION = 8; // 8-bit resolution

unsigned long lastMeasureTime = 0;
bool isTurning = false;
unsigned long lastTurnTime = 0;
bool isRunning = false;      // 로봇 동작 상태
uint8_t currentState = 0;    // 현재 상태 (STOPPED: 0, EXPLORING: 1)

uint8_t key_scan();
float getDistance();
void moveForward();
void spinTurn();
void stopMotors();

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
}

void moveForward() {
  // 왼쪽 모터 전진
  ledcWrite(IN1, 0); 
  ledcWrite(IN2, MOTOR_LSPEED);
  // 오른쪽 모터 전진
  ledcWrite(IN3, 0);
  ledcWrite(IN4, MOTOR_RSPEED); 
}

void spinTurn() {
  // 왼쪽 모터 후진, 오른쪽 모터 전진
  ledcWrite(IN1, 0); 
  ledcWrite(IN2, MOTOR_LSPEED);
  
  ledcWrite(IN3, MOTOR_RSPEED); 
  ledcWrite(IN4, 0); 
  
  //delay(TURN_DURATION);
}

void stopMotors() {
  // 모든 모터 정지
  ledcWrite(IN1, 0);//digitalWrite(IN1, LOW);
  ledcWrite(IN2, 0);//digitalWrite(IN2, LOW);
  ledcWrite(IN3, 0);//digitalWrite(IN3, LOW);
  ledcWrite(IN4, 0);//digitalWrite(IN4, LOW);
}

uint8_t key_scan() {
  static uint8_t mode_cnt = 0;
  static uint8_t oldKey = 0;
  uint8_t newKey = !digitalRead(KEY);
  
  if ((newKey != oldKey) && newKey) { // if KEY pressed
    mode_cnt++;
    mode_cnt = mode_cnt % 2; //MODE 0:stop, MODE 1:explore
    //currentState = (mode_cnt == 0) ? 0 : 1;
    delay(1);
  }
  oldKey = newKey;
  return mode_cnt;
}

void setup() {
  Serial.begin(115200);
  
  // 모터 핀 설정
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ASLEEP, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BSLEEP, OUTPUT);
  
  // 초음파 센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
   // 키 입력 핀 설정
  pinMode(KEY, INPUT);
  pinMode(LED, OUTPUT);

  ledcAttach(IN1, MOT_FREQ, MOT_RESOLUTION);
  ledcAttach(IN2, MOT_FREQ, MOT_RESOLUTION);
  ledcAttach(IN3, MOT_FREQ, MOT_RESOLUTION);
  ledcAttach(IN4, MOT_FREQ, MOT_RESOLUTION);

  // 슬립 모드 초기화
  digitalWrite(ASLEEP, HIGH);
  digitalWrite(BSLEEP, HIGH);

  // 초기 상태 LED 표시 (대기 모드)
  digitalWrite(LED, LOW);
  
}

void loop() {
  unsigned long currentTime = millis();
  
  // 키 입력 처리
  uint8_t mode = key_scan();
  if (mode != isRunning) {
    isRunning = mode;
    digitalWrite(LED, isRunning);  // LED로 현재 모드 표시
    if (!isRunning) {
      stopMotors();  // 대기 모드일 때 모터 정지
      //Serial.println("4");
    } 
  }

  // 초음파 모드일 때만 거리 측정 및 모터 제어
  if (isRunning) {
    // 100ms마다 거리 측정
    if (currentTime - lastMeasureTime >= MEASURE_INTERVAL) {
      lastMeasureTime = currentTime;
      
      float distance = getDistance();
      Serial.print("거리: ");
      Serial.println(distance);
      
      if (!isTurning) {
        if (distance <= MIN_DISTANCE) {
          isTurning = true;
          lastTurnTime = currentTime;
          // 가까운 거리일 때 스핀턴
          spinTurn(); 
          delay(TURN_DURATION);

        } else {
          moveForward();  // 20cm 이상이면 직진
          delay(TURN_DURATION);
        }
      } else {
        if (currentTime - lastTurnTime >= TURN_DURATION) {
          isTurning = false;
          moveForward();
          delay(TURN_DURATION);
          //Serial.println("3");
        }
      }
    }
  }
  
  delay(10);
}